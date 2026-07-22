//! LD_PRELOAD hook that routes a CUDA publisher's `cudaMalloc`/`cudaFree` to the
//! Agnocast GPU shared-memory pool.
//!
//! Only `cudaMalloc` and `cudaFree` are hooked in this first version. The real
//! CUDA functions are resolved via `RTLD_NEXT`; the Agnocast proxy C ABI (in
//! `libagnocast_cuda.so`) is resolved lazily via `RTLD_DEFAULT` and, when absent,
//! the hook falls back transparently to the real CUDA functions.

use std::ffi::CStr;
use std::os::raw::{c_char, c_int, c_void};
use std::sync::OnceLock;

/// `cudaError_t` is an `int` enum; `cudaSuccess` is 0.
const CUDA_SUCCESS: c_int = 0;

/// Version string, queryable via `dlsym`/`nm -D` on the shared library.
#[no_mangle]
pub extern "C" fn agnocast_cuda_heaphook_get_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr() as *const c_char
}

// ---------------------------------------------------------------------------
// Original CUDA runtime functions (resolved via RTLD_NEXT).
// ---------------------------------------------------------------------------
type CudaMallocFn = unsafe extern "C" fn(*mut *mut c_void, usize) -> c_int;
type CudaFreeFn = unsafe extern "C" fn(*mut c_void) -> c_int;

static ORIGINAL_CUDA_MALLOC: OnceLock<CudaMallocFn> = OnceLock::new();
static ORIGINAL_CUDA_FREE: OnceLock<CudaFreeFn> = OnceLock::new();

unsafe fn resolve_next(name: &[u8]) -> *mut c_void {
    let symbol = CStr::from_bytes_with_nul(name).unwrap();
    libc::dlsym(libc::RTLD_NEXT, symbol.as_ptr())
}

fn original_cuda_malloc() -> CudaMallocFn {
    *ORIGINAL_CUDA_MALLOC.get_or_init(|| unsafe {
        let ptr = resolve_next(b"cudaMalloc\0");
        assert!(
            !ptr.is_null(),
            "agnocast_cuda_heaphook: cudaMalloc not found via RTLD_NEXT"
        );
        std::mem::transmute::<*mut c_void, CudaMallocFn>(ptr)
    })
}

fn original_cuda_free() -> CudaFreeFn {
    *ORIGINAL_CUDA_FREE.get_or_init(|| unsafe {
        let ptr = resolve_next(b"cudaFree\0");
        assert!(
            !ptr.is_null(),
            "agnocast_cuda_heaphook: cudaFree not found via RTLD_NEXT"
        );
        std::mem::transmute::<*mut c_void, CudaFreeFn>(ptr)
    })
}

// ---------------------------------------------------------------------------
// Agnocast proxy C ABI (resolved via RTLD_DEFAULT; optional).
// ---------------------------------------------------------------------------
type InPublishWindowFn = unsafe extern "C" fn() -> c_int;
type PoolAllocateFn = unsafe extern "C" fn(usize, *mut *mut c_void) -> c_int;
type PoolFreeFn = unsafe extern "C" fn(*mut c_void) -> c_int;

struct ProxyApi {
    in_publish_window: InPublishWindowFn,
    pool_allocate: PoolAllocateFn,
    pool_free: PoolFreeFn,
}

static PROXY_API: OnceLock<Option<ProxyApi>> = OnceLock::new();

unsafe fn resolve_default(name: &[u8]) -> Option<*mut c_void> {
    let symbol = CStr::from_bytes_with_nul(name).unwrap();
    let ptr = libc::dlsym(libc::RTLD_DEFAULT, symbol.as_ptr());
    if ptr.is_null() {
        None
    } else {
        Some(ptr)
    }
}

/// Returns the proxy API if all three symbols are present in the process (i.e.
/// `libagnocast_cuda.so` is loaded), else `None`.
fn proxy_api() -> Option<&'static ProxyApi> {
    PROXY_API
        .get_or_init(|| unsafe {
            let in_window = resolve_default(b"agnocast_cuda_in_publish_window\0");
            let allocate = resolve_default(b"agnocast_cuda_pool_allocate\0");
            let free = resolve_default(b"agnocast_cuda_pool_free\0");
            match (in_window, allocate, free) {
                (Some(w), Some(a), Some(f)) => Some(ProxyApi {
                    in_publish_window: std::mem::transmute::<*mut c_void, InPublishWindowFn>(w),
                    pool_allocate: std::mem::transmute::<*mut c_void, PoolAllocateFn>(a),
                    pool_free: std::mem::transmute::<*mut c_void, PoolFreeFn>(f),
                }),
                _ => None,
            }
        })
        .as_ref()
}

// ---------------------------------------------------------------------------
// Routing decisions (pure, unit-tested).
// ---------------------------------------------------------------------------
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum MallocRoute {
    /// Serve from the Agnocast pool.
    Pool,
    /// Call the real cudaMalloc.
    Original,
}

/// A cudaMalloc goes to the pool only when the proxy is available AND the calling
/// thread is inside a publisher's borrow..publish window.
pub(crate) fn decide_malloc_route(in_publish_window: bool, pool_available: bool) -> MallocRoute {
    if pool_available && in_publish_window {
        MallocRoute::Pool
    } else {
        MallocRoute::Original
    }
}

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum FreeRoute {
    /// The pool recognized and freed the pointer.
    PoolHandled,
    /// Not a pooled pointer: call the real cudaFree.
    Original,
}

pub(crate) fn decide_free_route(pool_freed: bool) -> FreeRoute {
    if pool_freed {
        FreeRoute::PoolHandled
    } else {
        FreeRoute::Original
    }
}

// ---------------------------------------------------------------------------
// Hooks.
// ---------------------------------------------------------------------------

/// # Safety
/// Matches the CUDA Runtime `cudaMalloc` contract; `dev_ptr` must be a valid
/// pointer to a `void*`.
#[no_mangle]
pub unsafe extern "C" fn cudaMalloc(dev_ptr: *mut *mut c_void, size: usize) -> c_int {
    let api = proxy_api();
    let in_window = api.map_or(false, |a| (a.in_publish_window)() != 0);

    if decide_malloc_route(in_window, api.is_some()) == MallocRoute::Pool {
        let api = api.unwrap();
        let mut pooled_ptr: *mut c_void = std::ptr::null_mut();
        if (api.pool_allocate)(size, &mut pooled_ptr as *mut *mut c_void) == 1 {
            if !dev_ptr.is_null() {
                *dev_ptr = pooled_ptr;
            }
            return CUDA_SUCCESS;
        }
        // Pool could not satisfy the request: fall back to the real cudaMalloc so
        // publishing still works (just not zero-copy this time).
    }

    original_cuda_malloc()(dev_ptr, size)
}

/// # Safety
/// Matches the CUDA Runtime `cudaFree` contract.
#[no_mangle]
pub unsafe extern "C" fn cudaFree(dev_ptr: *mut c_void) -> c_int {
    if !dev_ptr.is_null() {
        if let Some(api) = proxy_api() {
            if decide_free_route((api.pool_free)(dev_ptr) == 1) == FreeRoute::PoolHandled {
                return CUDA_SUCCESS;
            }
        }
    }
    original_cuda_free()(dev_ptr)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn malloc_routes_to_pool_only_in_window_and_when_available() {
        assert_eq!(decide_malloc_route(true, true), MallocRoute::Pool);
        assert_eq!(decide_malloc_route(false, true), MallocRoute::Original);
        assert_eq!(decide_malloc_route(true, false), MallocRoute::Original);
        assert_eq!(decide_malloc_route(false, false), MallocRoute::Original);
    }

    #[test]
    fn free_routes_to_pool_only_when_pool_owns_pointer() {
        assert_eq!(decide_free_route(true), FreeRoute::PoolHandled);
        assert_eq!(decide_free_route(false), FreeRoute::Original);
    }

    #[test]
    fn version_is_non_empty() {
        let ptr = agnocast_cuda_heaphook_get_version();
        let version = unsafe { CStr::from_ptr(ptr) };
        assert!(!version.to_bytes().is_empty());
    }
}
