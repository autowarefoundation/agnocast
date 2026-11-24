use rlsf::Tlsf;
use std::{
    alloc::Layout,
    mem::MaybeUninit,
    ptr::NonNull,
    sync::Mutex,
};

use crate::{memory_stats, AgnocastSharedMemory, SharedMemoryAllocator};

const FLLEN: usize = 28; // The maximum block size is (32 << 28) - 1 = 8_589_934_591 (nearly 8GiB)
const SLLEN: usize = 64; // The worst-case internal fragmentation is ((32 << 28) / 64 - 2) = 134_217_726 (nearly 128MiB)
type FLBitmap = u32; // FLBitmap should contain at least FLLEN bits
type SLBitmap = u64; // SLBitmap should contain at least SLLEN bits
type TlsfType = Tlsf<'static, FLBitmap, SLBitmap, FLLEN, SLLEN>;

const POINTER_SIZE: usize = std::mem::size_of::<&usize>();
const POINTER_ALIGN: usize = std::mem::align_of::<&usize>();
const LAYOUT_ALIGN: usize = 1; // Minimun value that is a power of 2.

pub struct SizeTrackingAllocator {
    tlsf: TlsfType,
    /// Bytes currently allocated (usable size reported by TLSF, including our metadata/padding).
    current_size: usize,
    /// Peak value of `current_size` observed so far.
    max_size: usize,
    /// Lowest start address of any block allocated so far (`usize::MAX` before the first allocation).
    lowest_addr: usize,
    /// Highest end address (exclusive) of any block allocated so far.
    highest_end: usize,
}
impl SizeTrackingAllocator {
    fn new() -> Self {
        Self {
            tlsf: Tlsf::new(),
            current_size: 0,
            max_size: 0,
            lowest_addr: usize::MAX,
            highest_end: 0,
        }
    }

    /// Peak span of the pool that has ever been used: from the lowest block start to the highest block end.
    /// This is the allocated memory plus external fragmentation, so it is never smaller than `max_size`.
    fn high_water_mark(&self) -> usize {
        self.highest_end.saturating_sub(self.lowest_addr)
    }

    fn publish(&self) {
        memory_stats::publish(self.current_size, self.max_size, self.high_water_mark());
    }

    fn record_alloc(&mut self, addr: usize, size: usize) {
        self.current_size += size;
        self.max_size = self.max_size.max(self.current_size);
        self.lowest_addr = self.lowest_addr.min(addr);
        self.highest_end = self.highest_end.max(addr + size);
    }

    fn record_free(&mut self, size: usize) {
        self.current_size -= size;
    }

    /// A block of `size` usable bytes starting at `addr` was allocated.
    fn on_allocate(&mut self, addr: usize, size: usize) {
        self.record_alloc(addr, size);
        self.publish();
    }

    /// A block of `size` usable bytes was freed.
    fn on_deallocate(&mut self, size: usize) {
        self.record_free(size);
        self.publish();
    }

    /// A block was moved or resized. Publishes once so readers never see the intermediate state
    /// where the old block is already gone.
    fn on_reallocate(&mut self, old_size: usize, new_addr: usize, new_size: usize) {
        self.record_free(old_size);
        self.record_alloc(new_addr, new_size);
        self.publish();
    }
}

pub struct TLSFAllocator {
    inner: Mutex<SizeTrackingAllocator>,
}

unsafe impl SharedMemoryAllocator for TLSFAllocator {
    fn new(shm: &'static AgnocastSharedMemory) -> Self {
        let pool = unsafe {
            std::slice::from_raw_parts_mut(shm.as_ptr() as *mut MaybeUninit<u8>, shm.len())
        };

        let mut size_tracking_allocator = SizeTrackingAllocator::new();

        size_tracking_allocator.tlsf.insert_free_block(pool);
        Self {
            inner: Mutex::new(size_tracking_allocator),
        }
    }

    fn allocate(&self, layout: Layout) -> Option<NonNull<u8>> {
        // `alignment` must be at least `POINTER_ALIGN` to ensure that `aligned_ptr` is properly aligned to store a pointer.
        let alignment = layout.align().max(POINTER_ALIGN);
        let size = layout.size();
        let layout = Layout::from_size_align(POINTER_SIZE + size + alignment, LAYOUT_ALIGN).ok()?;

        let mut allocator = self.inner.lock().unwrap();
        // the original pointer returned by the internal allocator
        let original_ptr = allocator.tlsf.allocate(layout)?;
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
        let allocated_size = unsafe { TlsfType::allocation_usable_size(original_ptr) };
        allocator.on_allocate(original_addr, allocated_size);

        Some(aligned_ptr)
    }

    fn reallocate(&self, ptr: NonNull<u8>, new_layout: Layout) -> Option<NonNull<u8>> {
        // `alignment` must be at least `POINTER_ALIGN` to ensure that `aligned_ptr` is properly aligned to store a pointer.
        let alignment = new_layout.align().max(POINTER_ALIGN);
        let size = new_layout.size();
        // get the original pointer and compute the old aligned offset
        // SAFETY: `ptr` must have been allocated by `allocate`.
        let old_original_ptr: NonNull<u8> = unsafe { *ptr.as_ptr().byte_sub(POINTER_SIZE).cast() };
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
        let mut allocator = self.inner.lock().unwrap();
        let old_size: usize;
        unsafe {
            old_size = TlsfType::allocation_usable_size(old_original_ptr);
        }

        let new_original_ptr = unsafe { allocator.tlsf.reallocate(old_original_ptr, new_layout) }?;
        let new_original_addr = new_original_ptr.as_ptr() as usize;

        let new_size: usize;
        unsafe {
            new_size = TlsfType::allocation_usable_size(new_original_ptr);
        }
        allocator.on_reallocate(old_size, new_original_addr, new_size);

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

        let mut allocator = self.inner.lock().unwrap();
        let deallocate_size: usize;
        unsafe {
            deallocate_size = TlsfType::allocation_usable_size(original_ptr);
        }
        allocator.on_deallocate(deallocate_size);
        unsafe { allocator.tlsf.deallocate(original_ptr, LAYOUT_ALIGN) }
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
        let pool: &'static mut [MaybeUninit<u8>] =
            unsafe { std::slice::from_raw_parts_mut(pool_ptr as *mut MaybeUninit<u8>, pool_size) };
        let mut size_tracking_allocator = SizeTrackingAllocator::new();
        size_tracking_allocator.tlsf.insert_free_block(pool);

        TLSFAllocator {
            inner: Mutex::new(size_tracking_allocator),
        }
    }

    fn get_offset(ptr: NonNull<u8>) -> usize {
        let original_ptr: NonNull<u8> = unsafe { *ptr.as_ptr().byte_sub(POINTER_SIZE).cast() };
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

    fn sizes(alloc: &TLSFAllocator) -> (usize, usize) {
        let inner = alloc.inner.lock().unwrap();
        (inner.current_size, inner.max_size)
    }

    #[test]
    fn test_size_tracking() {
        let alloc = create_test_allocator();
        assert_eq!(sizes(&alloc), (0, 0));

        let a = alloc.allocate(Layout::from_size_align(1024, 1).unwrap()).unwrap();
        let (cur_a, max_a) = sizes(&alloc);
        assert!(cur_a >= 1024);
        assert_eq!(max_a, cur_a);

        let b = alloc.allocate(Layout::from_size_align(4096, 1).unwrap()).unwrap();
        let (cur_ab, max_ab) = sizes(&alloc);
        assert!(cur_ab >= cur_a + 4096);
        assert_eq!(max_ab, cur_ab);

        // Freeing lowers the current size but keeps the peak.
        alloc.deallocate(b);
        assert_eq!(sizes(&alloc), (cur_a, max_ab));

        // Growing via reallocate updates both current and peak.
        let a = alloc.reallocate(a, Layout::from_size_align(16 * 1024, 1).unwrap()).unwrap();
        let (cur_r, max_r) = sizes(&alloc);
        assert!(cur_r >= 16 * 1024);
        assert_eq!(max_r, max_ab.max(cur_r));

        alloc.deallocate(a);
        assert_eq!(sizes(&alloc), (0, max_r));
    }

    fn high_water_mark(alloc: &TLSFAllocator) -> usize {
        alloc.inner.lock().unwrap().high_water_mark()
    }

    fn block_range(ptr: NonNull<u8>) -> (usize, usize) {
        let original_ptr: NonNull<u8> = unsafe { *ptr.as_ptr().byte_sub(POINTER_SIZE).cast() };
        let start = original_ptr.as_ptr() as usize;
        (start, start + unsafe { TlsfType::allocation_usable_size(original_ptr) })
    }

    #[test]
    fn test_high_water_mark() {
        let alloc = create_test_allocator();
        assert_eq!(high_water_mark(&alloc), 0);

        let layout = Layout::from_size_align(1024, 1).unwrap();
        let a = alloc.allocate(layout).unwrap();
        let b = alloc.allocate(layout).unwrap();
        let c = alloc.allocate(layout).unwrap();
        let ranges = [block_range(a), block_range(b), block_range(c)];
        let lowest = ranges.iter().map(|r| r.0).min().unwrap();
        let highest = ranges.iter().map(|r| r.1).max().unwrap();
        assert_eq!(high_water_mark(&alloc), highest - lowest);
        assert!(high_water_mark(&alloc) >= sizes(&alloc).1);

        // Freeing a block in the middle leaves a hole: the span is unchanged although less is allocated.
        let span = high_water_mark(&alloc);
        alloc.deallocate(b);
        assert_eq!(high_water_mark(&alloc), span);
        assert!(span > sizes(&alloc).0);

        // A block that does not fit into the hole extends the span.
        let big = alloc.allocate(Layout::from_size_align(8 * 1024, 1).unwrap()).unwrap();
        let (big_start, big_end) = block_range(big);
        assert_eq!(high_water_mark(&alloc), highest.max(big_end) - lowest.min(big_start));
        assert!(high_water_mark(&alloc) > span);

        // It is a peak: freeing everything keeps it.
        let span = high_water_mark(&alloc);
        for ptr in [a, c, big] {
            alloc.deallocate(ptr);
        }
        assert_eq!(sizes(&alloc).0, 0);
        assert_eq!(high_water_mark(&alloc), span);
        assert!(span >= sizes(&alloc).1);
    }
}
