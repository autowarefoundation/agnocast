use rlsf::Tlsf;
use std::{alloc::Layout, mem::MaybeUninit, ptr::NonNull, sync::Mutex};

use crate::{AgnocastSharedMemory, SharedMemoryAllocator};

const FLLEN: usize = 28; // The maximum block size is (32 << 28) - 1 = 8_589_934_591 (nearly 8GiB)
const SLLEN: usize = 64; // The worst-case internal fragmentation is ((32 << 28) / 64 - 2) = 134_217_726 (nearly 128MiB)
type FLBitmap = u32; // FLBitmap should contain at least FLLEN bits
type SLBitmap = u64; // SLBitmap should contain at least SLLEN bits
type TlsfType = Tlsf<'static, FLBitmap, SLBitmap, FLLEN, SLLEN>;

const POINTER_SIZE: usize = std::mem::size_of::<&usize>();
const POINTER_ALIGN: usize = std::mem::align_of::<&usize>();
const LAYOUT_ALIGN: usize = 1; // Minimun value that is a power of 2.

pub struct TLSFAllocator {
    inner: Mutex<TlsfType>,
}

unsafe impl SharedMemoryAllocator for TLSFAllocator {
    fn new(shm: &'static AgnocastSharedMemory) -> Self {
        let pool = unsafe {
            std::slice::from_raw_parts_mut(shm.as_ptr() as *mut MaybeUninit<u8>, shm.len())
        };
        let mut tlsf: TlsfType = Tlsf::new();
        tlsf.insert_free_block(pool);
        Self {
            inner: Mutex::new(tlsf),
        }
    }

    fn allocate(&self, layout: Layout) -> Option<NonNull<u8>> {
        // `alignment` must be at least `POINTER_ALIGN` to ensure that `aligned_ptr` is properly aligned to store a pointer.
        let alignment = layout.align().max(POINTER_ALIGN);
        let size = layout.size();
        let layout = Layout::from_size_align(POINTER_SIZE + size + alignment, LAYOUT_ALIGN).ok()?;

        // the original pointer returned by the internal allocator
        let mut tlsf = self.inner.lock().unwrap();
        let original_ptr = tlsf.allocate(layout)?;
        let original_addr = original_ptr.as_ptr() as usize;

        // the aligned pointer returned to the user
        //
        // It is our responsibility to satisfy alignment constraints.
        // We avoid using `Layout::align` because doing so requires us to remember the alignment.
        // This is because `Tlsf::{reallocate, deallocate}` functions require the same alignment.
        // See: https://docs.rs/rlsf/latest/rlsf/struct.Tlsf.html
        let aligned_addr = (original_addr + POINTER_SIZE + alignment - 1) & !(alignment - 1);

        // SAFETY: `aligned_addr` must be non-zero.
        debug_assert!(aligned_addr % alignment == 0 && aligned_addr != 0);
        let aligned_ptr = unsafe { NonNull::new_unchecked(aligned_addr as *mut u8) };

        // store the original pointer
        unsafe { *aligned_ptr.as_ptr().byte_sub(POINTER_SIZE).cast() = original_ptr };

        Some(aligned_ptr)
    }

    fn reallocate(&self, ptr: NonNull<u8>, new_layout: Layout) -> Option<NonNull<u8>> {
        // `alignment` must be at least `POINTER_ALIGN` to ensure that `aligned_ptr` is properly aligned to store a pointer.
        let alignment = new_layout.align().max(POINTER_ALIGN);
        let size = new_layout.size();
        // get the original pointer and compute the old aligned offset
        // SAFETY: `ptr` must have been allocated by `allocate`.
        let old_original_ptr: NonNull<u8> =
            unsafe { *ptr.as_ptr().byte_sub(POINTER_SIZE).cast() };
        let old_offset = ptr.as_ptr() as usize - old_original_ptr.as_ptr() as usize;

        // The new block must be large enough for both the final layout (metadata + user data +
        // alignment padding) and the memmove source (user data sitting at the old offset).
        // When the old alignment was larger than the new one (e.g. posix_memalign → realloc),
        // old_offset can exceed POINTER_SIZE + alignment, so we take the max.
        // NOTE: Without the max, the memmove below would read past the block boundary (UB).
        // This is not covered by tests because the OOB read happens within the contiguous
        // TLSF pool and doesn't corrupt the first old_size bytes that tests verify.
        let internal_size = (POINTER_SIZE + size + alignment).max(old_offset + size);
        let new_layout = Layout::from_size_align(internal_size, LAYOUT_ALIGN).ok()?;

        // the original pointer returned by the internal allocator
        let mut tlsf = self.inner.lock().unwrap();
        let new_original_ptr = unsafe { tlsf.reallocate(old_original_ptr, new_layout) }?;
        let new_original_addr = new_original_ptr.as_ptr() as usize;

        // the aligned pointer returned to the user
        //
        // It is our responsibility to satisfy alignment constraints.
        // We avoid using `Layout::align` because doing so requires us to remember the alignment.
        // This is because `Tlsf::{reallocate, deallocate}` functions require the same alignment.
        // See: https://docs.rs/rlsf/latest/rlsf/struct.Tlsf.html
        let new_aligned_addr =
            (new_original_addr + POINTER_SIZE + alignment - 1) & !(alignment - 1);

        // SAFETY: `new_aligned_addr` must be non-zero.
        debug_assert!(new_aligned_addr % alignment == 0 && new_aligned_addr != 0);
        let new_aligned_ptr = unsafe { NonNull::new_unchecked(new_aligned_addr as *mut u8) };

        // If the aligned offset changed after relocation, fix user data position.
        // rlsf's reallocate copies raw bytes at the block level, so user data
        // sits at the old offset in the new block. Shift it to the new offset.
        let new_offset = new_aligned_addr - new_original_addr;
        if old_offset != new_offset {
            unsafe {
                std::ptr::copy(
                    (new_original_addr + old_offset) as *const u8,
                    new_aligned_ptr.as_ptr(),
                    size,
                );
            }
        }

        // store the original pointer
        unsafe { *new_aligned_ptr.as_ptr().byte_sub(POINTER_SIZE).cast() = new_original_ptr };

        Some(new_aligned_ptr)
    }

    fn deallocate(&self, ptr: NonNull<u8>) {
        // get the original pointer
        // SAFETY: `ptr` must have been allocated by `allocate` or `reallocate`.
        let original_ptr = unsafe { *ptr.as_ptr().byte_sub(POINTER_SIZE).cast() };

        let mut tlsf = self.inner.lock().unwrap();
        unsafe { tlsf.deallocate(original_ptr, LAYOUT_ALIGN) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SharedMemoryAllocator;

    fn create_test_allocator() -> TLSFAllocator {
        let pool_size = 256 * 1024;
        let pool_ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                pool_size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };
        assert!(pool_ptr != libc::MAP_FAILED);

        // SAFETY: mmap'd memory lives until munmap; we intentionally leak it for 'static.
        let pool: &'static mut [MaybeUninit<u8>] = unsafe {
            std::slice::from_raw_parts_mut(pool_ptr as *mut MaybeUninit<u8>, pool_size)
        };
        let mut tlsf: TlsfType = Tlsf::new();
        tlsf.insert_free_block(pool);
        TLSFAllocator {
            inner: Mutex::new(tlsf),
        }
    }

    fn get_offset(ptr: NonNull<u8>) -> usize {
        let original_ptr: NonNull<u8> =
            unsafe { *ptr.as_ptr().byte_sub(POINTER_SIZE).cast() };
        ptr.as_ptr() as usize - original_ptr.as_ptr() as usize
    }

    /// Test that reallocate preserves user data when the aligned offset changes.
    ///
    /// The bug: rlsf's internal reallocate copies raw bytes at the block level.
    /// If the new block has a different base address alignment, the user-facing
    /// aligned offset differs, and the user sees shifted/corrupted data.
    ///
    /// This test allocates with alignment=256, then reallocates with alignment=16.
    /// It iterates with different padding sizes to find an allocation where the
    /// offsets genuinely differ, ensuring the memmove fix path is exercised.
    #[test]
    fn test_reallocate_with_offset_change() {
        let alloc = create_test_allocator();
        let old_size = 100;
        let new_size = 200;

        let mut offset_change_tested = false;

        for pad_size in 1..=128 {
            // Accumulate padding allocations to shift the TLSF free pointer,
            // changing the base address alignment of subsequent allocations.
            let _ = alloc.allocate(Layout::from_size_align(pad_size, 1).unwrap());

            let layout = Layout::from_size_align(old_size, 256).unwrap();
            let Some(ptr) = alloc.allocate(layout) else {
                break;
            };
            let old_offset = get_offset(ptr);

            // Write pattern
            unsafe {
                for i in 0..old_size {
                    *ptr.as_ptr().add(i) = i as u8;
                }
            }

            // Reallocate with smaller alignment (simulates realloc's MIN_ALIGN=16)
            let new_layout = Layout::from_size_align(new_size, 16).unwrap();
            let new_ptr = alloc.reallocate(ptr, new_layout).unwrap();
            let new_offset = get_offset(new_ptr);

            // Verify data preserved
            unsafe {
                for i in 0..old_size {
                    assert_eq!(
                        *new_ptr.as_ptr().add(i),
                        i as u8,
                        "Data corrupted at byte {} (old_offset={}, new_offset={})",
                        i,
                        old_offset,
                        new_offset
                    );
                }
            }

            if old_offset != new_offset {
                offset_change_tested = true;
            }

            alloc.deallocate(new_ptr);
        }

        assert!(
            offset_change_tested,
            "No allocation produced different offsets — test did not exercise the bug path"
        );
    }
}
