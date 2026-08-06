//! LD_PRELOAD hook that routes a CUDA publisher's device allocations to the Agnocast
//! GPU shared-memory pool, and detects the one stream configuration that would break
//! Agnocast's GPU-side ordering silently.
//!
//! Hooked:
//!
//! * Runtime API — `cudaMalloc` / `cudaFree`, `cudaMallocAsync` / `cudaFreeAsync`.
//! * Driver API — `cuMemAlloc(_v2)` / `cuMemFree(_v2)`, `cuMemAllocAsync` /
//!   `cuMemFreeAsync`. Application writers may use the driver API directly, and a
//!   publisher buffer allocated through it must reach the pool like any other.
//! * `cuMemCreate` — the VMM allocator cannot be served from a pool of plain
//!   allocations, so this is a pass-through that warns once inside a publish window.
//! * `cudaStreamCreateWithFlags` / `cuStreamCreate(WithPriority)` — used only to
//!   notice that this process created a *non-blocking* stream. A default (legacy or
//!   per-thread) stream does not order against such a stream, so a publisher that
//!   creates one and then publishes without declaring a stream would record its
//!   data-ready event while the kernel is still writing. Agnocast fails fast on that
//!   combination, and this hook is how it learns to.
//!
//! The real CUDA functions are resolved via `RTLD_NEXT`; the Agnocast proxy C ABI (in
//! `libagnocast_cuda.so`) is resolved lazily via `RTLD_DEFAULT` and, when absent, the
//! hook falls back transparently to the real CUDA functions.
//!
//! Known gap: an application that resolves driver entry points through
//! `cuGetProcAddress` (as libcudart does internally) bypasses symbol interposition
//! entirely. Such allocations are not pooled, and `publish()` reports them.

use std::cell::Cell;
use std::ffi::CStr;
use std::os::raw::{c_char, c_int, c_uint, c_void};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

/// `cudaError_t` is an `int` enum; `cudaSuccess` is 0. The driver API's `CUresult`
/// uses the same value for `CUDA_SUCCESS`.
const CUDA_SUCCESS: c_int = 0;

/// `cudaStreamNonBlocking` (runtime) and `CU_STREAM_NON_BLOCKING` (driver) are both
/// `0x01`.
const STREAM_NON_BLOCKING: c_uint = 0x01;

/// Version string, queryable via `dlsym`/`nm -D` on the shared library.
#[no_mangle]
pub extern "C" fn agnocast_cuda_heaphook_get_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr() as *const c_char
}

// ---------------------------------------------------------------------------
// Re-entrancy guard.
//
// A hooked runtime function that falls back to the real implementation may internally
// reach a hooked *driver* function. Without a guard, that inner call would see the
// publish window still open and hand a pooled pointer to the CUDA runtime, corrupting
// its bookkeeping. While the guard is held, every hook is a pass-through.
// ---------------------------------------------------------------------------
thread_local! {
    static IN_ORIGINAL_CALL: Cell<bool> = const { Cell::new(false) };
}

fn in_original_call() -> bool {
    IN_ORIGINAL_CALL.with(|flag| flag.get())
}

/// Restores the guard to whatever it was on entry, so nested calls behave correctly: an
/// inner hook that passes through must not clear the flag for the outer call that is
/// still running. Restoring on drop also survives an unwind.
struct OriginalCallGuard {
    previous: bool,
}

impl OriginalCallGuard {
    fn enter() -> Self {
        let previous = IN_ORIGINAL_CALL.with(|flag| flag.replace(true));
        Self { previous }
    }
}

impl Drop for OriginalCallGuard {
    fn drop(&mut self) {
        IN_ORIGINAL_CALL.with(|flag| flag.set(self.previous));
    }
}

fn call_original<T>(body: impl FnOnce() -> T) -> T {
    let _guard = OriginalCallGuard::enter();
    body()
}

// ---------------------------------------------------------------------------
// Original CUDA functions (resolved via RTLD_NEXT).
// ---------------------------------------------------------------------------
type CudaMallocFn = unsafe extern "C" fn(*mut *mut c_void, usize) -> c_int;
type CudaFreeFn = unsafe extern "C" fn(*mut c_void) -> c_int;
type CudaMallocAsyncFn = unsafe extern "C" fn(*mut *mut c_void, usize, *mut c_void) -> c_int;
type CudaFreeAsyncFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> c_int;
type CudaStreamCreateWithFlagsFn = unsafe extern "C" fn(*mut *mut c_void, c_uint) -> c_int;
type CudaStreamCreateWithPriorityFn =
    unsafe extern "C" fn(*mut *mut c_void, c_uint, c_int) -> c_int;
type CudaStreamSynchronizeFn = unsafe extern "C" fn(*mut c_void) -> c_int;

/// `CUdeviceptr` is a 64-bit integer handle, not a pointer.
type CuMemAllocFn = unsafe extern "C" fn(*mut u64, usize) -> c_int;
type CuMemFreeFn = unsafe extern "C" fn(u64) -> c_int;
type CuMemAllocAsyncFn = unsafe extern "C" fn(*mut u64, usize, *mut c_void) -> c_int;
type CuMemFreeAsyncFn = unsafe extern "C" fn(u64, *mut c_void) -> c_int;
type CuMemCreateFn = unsafe extern "C" fn(*mut u64, usize, *const c_void, u64) -> c_int;
type CuStreamCreateFn = unsafe extern "C" fn(*mut *mut c_void, c_uint) -> c_int;
type CuStreamCreateWithPriorityFn = unsafe extern "C" fn(*mut *mut c_void, c_uint, c_int) -> c_int;
type CuStreamSynchronizeFn = unsafe extern "C" fn(*mut c_void) -> c_int;

unsafe fn resolve_next(name: &[u8]) -> *mut c_void {
    let symbol = CStr::from_bytes_with_nul(name).unwrap();
    libc::dlsym(libc::RTLD_NEXT, symbol.as_ptr())
}

/// Resolves a required symbol via `RTLD_NEXT`, panicking with a clear message if the
/// process does not provide it.
unsafe fn require_next<T>(name: &[u8]) -> T {
    let ptr = resolve_next(name);
    assert!(
        !ptr.is_null(),
        "agnocast_cuda_heaphook: {} not found via RTLD_NEXT",
        String::from_utf8_lossy(&name[..name.len() - 1])
    );
    std::mem::transmute_copy::<*mut c_void, T>(&ptr)
}

/// Resolves the first of `names` that exists via `RTLD_NEXT`. Driver-API entry points
/// carry an ABI suffix (`cuMemAlloc_v2`) that `cuda.h` hides behind a macro, so both
/// spellings must be tried.
unsafe fn resolve_next_any<T>(names: &[&[u8]]) -> Option<T> {
    for name in names {
        let ptr = resolve_next(name);
        if !ptr.is_null() {
            return Some(std::mem::transmute_copy::<*mut c_void, T>(&ptr));
        }
    }
    None
}

macro_rules! original {
    ($getter:ident, $cell:ident, $ty:ty, $name:literal) => {
        static $cell: OnceLock<$ty> = OnceLock::new();
        fn $getter() -> $ty {
            *$cell.get_or_init(|| unsafe { require_next(concat!($name, "\0").as_bytes()) })
        }
    };
}

original!(
    original_cuda_malloc,
    ORIGINAL_CUDA_MALLOC,
    CudaMallocFn,
    "cudaMalloc"
);
original!(
    original_cuda_free,
    ORIGINAL_CUDA_FREE,
    CudaFreeFn,
    "cudaFree"
);
original!(
    original_cuda_stream_create_with_flags,
    ORIGINAL_CUDA_STREAM_CREATE_WITH_FLAGS,
    CudaStreamCreateWithFlagsFn,
    "cudaStreamCreateWithFlags"
);

static ORIGINAL_CUDA_MALLOC_ASYNC: OnceLock<Option<CudaMallocAsyncFn>> = OnceLock::new();
static ORIGINAL_CUDA_STREAM_CREATE_WITH_PRIORITY: OnceLock<Option<CudaStreamCreateWithPriorityFn>> =
    OnceLock::new();
static ORIGINAL_CUDA_STREAM_SYNCHRONIZE: OnceLock<Option<CudaStreamSynchronizeFn>> =
    OnceLock::new();
static ORIGINAL_CU_STREAM_SYNCHRONIZE: OnceLock<Option<CuStreamSynchronizeFn>> = OnceLock::new();
static ORIGINAL_CUDA_FREE_ASYNC: OnceLock<Option<CudaFreeAsyncFn>> = OnceLock::new();
static ORIGINAL_CU_MEM_ALLOC: OnceLock<Option<CuMemAllocFn>> = OnceLock::new();
static ORIGINAL_CU_MEM_FREE: OnceLock<Option<CuMemFreeFn>> = OnceLock::new();
static ORIGINAL_CU_MEM_ALLOC_ASYNC: OnceLock<Option<CuMemAllocAsyncFn>> = OnceLock::new();
static ORIGINAL_CU_MEM_FREE_ASYNC: OnceLock<Option<CuMemFreeAsyncFn>> = OnceLock::new();
static ORIGINAL_CU_MEM_CREATE: OnceLock<Option<CuMemCreateFn>> = OnceLock::new();
static ORIGINAL_CU_STREAM_CREATE: OnceLock<Option<CuStreamCreateFn>> = OnceLock::new();
static ORIGINAL_CU_STREAM_CREATE_WITH_PRIORITY: OnceLock<Option<CuStreamCreateWithPriorityFn>> =
    OnceLock::new();

/// `CUDA_ERROR_NOT_INITIALIZED`. Returned when a driver entry point is interposed in a
/// process where libcuda is not actually present — which cannot normally happen, since
/// the caller had to reach our symbol through libcuda's own dependency chain.
const CUDA_ERROR_NOT_INITIALIZED: c_int = 3;

// ---------------------------------------------------------------------------
// Agnocast proxy C ABI (resolved via RTLD_DEFAULT; optional).
// ---------------------------------------------------------------------------
type InPublishWindowFn = unsafe extern "C" fn() -> c_int;
type PoolAllocateFn = unsafe extern "C" fn(usize, *mut *mut c_void) -> c_int;
type PoolFreeFn = unsafe extern "C" fn(*mut c_void) -> c_int;
type SlotIdFromPtrFn = unsafe extern "C" fn(*mut c_void, *mut u32) -> c_int;
type NoteNonBlockingStreamFn = unsafe extern "C" fn();

struct ProxyApi {
    in_publish_window: InPublishWindowFn,
    pool_allocate: PoolAllocateFn,
    pool_free: PoolFreeFn,
    /// Tells whether the pool owns a pointer WITHOUT freeing it, which the
    /// stream-ordered free path needs before it decides to synchronize.
    slot_id_from_ptr: SlotIdFromPtrFn,
    /// Absent in an older libagnocast_cuda; the fail-fast check then simply never
    /// fires, which is the pre-existing behaviour.
    note_non_blocking_stream: Option<NoteNonBlockingStreamFn>,
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

/// Returns the proxy API if the pool-routing symbols are present in the process (i.e.
/// `libagnocast_cuda.so` is loaded), else `None`.
fn proxy_api() -> Option<&'static ProxyApi> {
    PROXY_API
        .get_or_init(|| unsafe {
            let in_window = resolve_default(b"agnocast_cuda_in_publish_window\0");
            let allocate = resolve_default(b"agnocast_cuda_pool_allocate\0");
            let free = resolve_default(b"agnocast_cuda_pool_free\0");
            let slot_id = resolve_default(b"agnocast_cuda_slot_id_from_ptr\0");
            let note = resolve_default(b"agnocast_cuda_note_non_blocking_stream\0");
            match (in_window, allocate, free, slot_id) {
                (Some(w), Some(a), Some(f), Some(s)) => Some(ProxyApi {
                    in_publish_window: std::mem::transmute::<*mut c_void, InPublishWindowFn>(w),
                    pool_allocate: std::mem::transmute::<*mut c_void, PoolAllocateFn>(a),
                    pool_free: std::mem::transmute::<*mut c_void, PoolFreeFn>(f),
                    slot_id_from_ptr: std::mem::transmute::<*mut c_void, SlotIdFromPtrFn>(s),
                    note_non_blocking_stream: note
                        .map(|n| std::mem::transmute::<*mut c_void, NoteNonBlockingStreamFn>(n)),
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
    /// Call the real allocator.
    Original,
}

/// An allocation goes to the pool only when the proxy is available, the calling thread
/// is inside a publisher's borrow..publish window, and we are not already inside a
/// fallback call to a real CUDA allocator.
pub(crate) fn decide_malloc_route(
    in_publish_window: bool,
    pool_available: bool,
    reentrant: bool,
) -> MallocRoute {
    if pool_available && in_publish_window && !reentrant {
        MallocRoute::Pool
    } else {
        MallocRoute::Original
    }
}

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum FreeRoute {
    /// The pool recognized and freed the pointer.
    PoolHandled,
    /// Not a pooled pointer: call the real deallocator.
    Original,
}

pub(crate) fn decide_free_route(pool_freed: bool) -> FreeRoute {
    if pool_freed {
        FreeRoute::PoolHandled
    } else {
        FreeRoute::Original
    }
}

/// A stream created with the non-blocking flag is invisible to default-stream
/// ordering, which is what makes "non-blocking stream + no declared stream" a
/// correctness failure rather than a slowdown.
pub(crate) fn is_non_blocking_stream(flags: c_uint) -> bool {
    flags & STREAM_NON_BLOCKING != 0
}

/// Whether this process is in the publish window and the pool is available — the state
/// in which an unpoolable allocation (`cuMemCreate`) is worth warning about.
fn in_pool_window() -> bool {
    !in_original_call()
        && proxy_api().map_or(false, |api| unsafe { (api.in_publish_window)() != 0 })
}

fn note_non_blocking_stream() {
    if let Some(api) = proxy_api() {
        if let Some(note) = api.note_non_blocking_stream {
            unsafe { note() };
        }
    }
}

// ---------------------------------------------------------------------------
// Shared hook bodies.
// ---------------------------------------------------------------------------

/// Tries to serve `size` bytes from the pool. Returns the pooled pointer, or `None` to
/// fall back to the real allocator.
fn pooled_allocation(size: usize) -> Option<*mut c_void> {
    let api = proxy_api();
    let in_window = api.map_or(false, |a| unsafe { (a.in_publish_window)() } != 0);
    if decide_malloc_route(in_window, api.is_some(), in_original_call()) != MallocRoute::Pool {
        return None;
    }
    let api = api.unwrap();
    let mut pooled: *mut c_void = std::ptr::null_mut();
    if unsafe { (api.pool_allocate)(size, &mut pooled as *mut *mut c_void) } == 1 {
        Some(pooled)
    } else {
        // Pool exhausted or the size has no fitting class: publishing still works, just
        // not zero-copy this time.
        None
    }
}

/// Returns true if the pool owned `ptr` and freed it.
fn pooled_free(ptr: *mut c_void) -> bool {
    if ptr.is_null() || in_original_call() {
        return false;
    }
    match proxy_api() {
        Some(api) => {
            decide_free_route(unsafe { (api.pool_free)(ptr) } == 1) == FreeRoute::PoolHandled
        }
        None => false,
    }
}

/// Returns true if the Agnocast pool owns `ptr`, without freeing it.
fn is_pooled(ptr: *mut c_void) -> bool {
    if ptr.is_null() || in_original_call() {
        return false;
    }
    match proxy_api() {
        Some(api) => {
            let mut slot_id: u32 = 0;
            let rc = unsafe { (api.slot_id_from_ptr)(ptr, &mut slot_id as *mut u32) };
            rc == 1
        }
        None => false,
    }
}

/// Blocks until all work already enqueued on `stream` has completed, through the runtime
/// API. A null stream is the legacy default stream, which `cudaStreamSynchronize` handles.
fn synchronize_stream_runtime(stream: *mut c_void) {
    let original = *ORIGINAL_CUDA_STREAM_SYNCHRONIZE
        .get_or_init(|| unsafe { resolve_next_any(&[b"cudaStreamSynchronize\0"]) });
    if let Some(f) = original {
        call_original(|| unsafe { f(stream) });
    }
}

/// Same, through the driver API.
fn synchronize_stream_driver(stream: *mut c_void) {
    let original = *ORIGINAL_CU_STREAM_SYNCHRONIZE
        .get_or_init(|| unsafe { resolve_next_any(&[b"cuStreamSynchronize\0"]) });
    if let Some(f) = original {
        call_original(|| unsafe { f(stream) });
    }
}

/// Frees a pooled pointer for a *stream-ordered* deallocator (`cudaFreeAsync`,
/// `cuMemFreeAsync`).
///
/// Returning a slot to the pool is a synchronous, cross-process act: the daemon may hand
/// it to another publisher immediately, whose kernel would then overwrite bytes that work
/// already enqueued on `stream` is still reading or writing. The async free APIs promise
/// the memory stays valid for exactly that already-enqueued work, so the ordering has to
/// be honoured — and the pool has no way to defer a slot release. We therefore synchronize
/// the stream first.
///
/// The cost lands only on an explicit async free of a *pooled* message buffer, which is an
/// error/cleanup path: the normal lifetime ends with Agnocast's own reclaim after all
/// subscribers release. A non-pooled pointer is detected first and never pays for this.
///
/// Returns true if the pool owned the pointer and it was freed.
fn pooled_free_stream_ordered(ptr: *mut c_void, sync_stream: impl FnOnce()) -> bool {
    if !is_pooled(ptr) {
        return false;
    }
    sync_stream();
    pooled_free(ptr)
}

// ---------------------------------------------------------------------------
// Runtime API hooks.
// ---------------------------------------------------------------------------

/// # Safety
/// Matches the CUDA Runtime `cudaMalloc` contract; `dev_ptr` must be a valid
/// pointer to a `void*`.
#[no_mangle]
pub unsafe extern "C" fn cudaMalloc(dev_ptr: *mut *mut c_void, size: usize) -> c_int {
    if let Some(pooled) = pooled_allocation(size) {
        if !dev_ptr.is_null() {
            *dev_ptr = pooled;
        }
        return CUDA_SUCCESS;
    }
    call_original(|| original_cuda_malloc()(dev_ptr, size))
}

/// # Safety
/// Matches the CUDA Runtime `cudaFree` contract.
#[no_mangle]
pub unsafe extern "C" fn cudaFree(dev_ptr: *mut c_void) -> c_int {
    if pooled_free(dev_ptr) {
        return CUDA_SUCCESS;
    }
    call_original(|| original_cuda_free()(dev_ptr))
}

/// # Safety
/// Matches the CUDA Runtime `cudaMallocAsync` contract.
#[no_mangle]
pub unsafe extern "C" fn cudaMallocAsync(
    dev_ptr: *mut *mut c_void,
    size: usize,
    stream: *mut c_void,
) -> c_int {
    // A pooled pointer is valid immediately, which is strictly stronger than the
    // stream-ordered validity cudaMallocAsync promises.
    if let Some(pooled) = pooled_allocation(size) {
        if !dev_ptr.is_null() {
            *dev_ptr = pooled;
        }
        return CUDA_SUCCESS;
    }
    let original =
        *ORIGINAL_CUDA_MALLOC_ASYNC.get_or_init(|| resolve_next_any(&[b"cudaMallocAsync\0"]));
    match original {
        Some(f) => call_original(|| f(dev_ptr, size, stream)),
        None => CUDA_ERROR_NOT_INITIALIZED,
    }
}

/// # Safety
/// Matches the CUDA Runtime `cudaFreeAsync` contract.
#[no_mangle]
pub unsafe extern "C" fn cudaFreeAsync(dev_ptr: *mut c_void, stream: *mut c_void) -> c_int {
    if pooled_free_stream_ordered(dev_ptr, || synchronize_stream_runtime(stream)) {
        return CUDA_SUCCESS;
    }
    let original =
        *ORIGINAL_CUDA_FREE_ASYNC.get_or_init(|| resolve_next_any(&[b"cudaFreeAsync\0"]));
    match original {
        Some(f) => call_original(|| f(dev_ptr, stream)),
        None => CUDA_ERROR_NOT_INITIALIZED,
    }
}

/// # Safety
/// Matches the CUDA Runtime `cudaStreamCreateWithFlags` contract.
#[no_mangle]
pub unsafe extern "C" fn cudaStreamCreateWithFlags(
    stream: *mut *mut c_void,
    flags: c_uint,
) -> c_int {
    let result = call_original(|| original_cuda_stream_create_with_flags()(stream, flags));
    if result == CUDA_SUCCESS && is_non_blocking_stream(flags) {
        note_non_blocking_stream();
    }
    result
}

/// # Safety
/// Matches the CUDA Runtime `cudaStreamCreateWithPriority` contract.
///
/// Hooked for the same reason as `cudaStreamCreateWithFlags`: it accepts
/// `cudaStreamNonBlocking` too, and a node that creates its stream this way and then
/// publishes without declaring it must still be caught.
#[no_mangle]
pub unsafe extern "C" fn cudaStreamCreateWithPriority(
    stream: *mut *mut c_void,
    flags: c_uint,
    priority: c_int,
) -> c_int {
    let original = *ORIGINAL_CUDA_STREAM_CREATE_WITH_PRIORITY
        .get_or_init(|| resolve_next_any(&[b"cudaStreamCreateWithPriority\0"]));
    let result = match original {
        Some(f) => call_original(|| f(stream, flags, priority)),
        None => return CUDA_ERROR_NOT_INITIALIZED,
    };
    if result == CUDA_SUCCESS && is_non_blocking_stream(flags) {
        note_non_blocking_stream();
    }
    result
}

// ---------------------------------------------------------------------------
// Driver API hooks.
//
// Both the plain and `_v2` spellings are exported: `cuda.h` rewrites `cuMemAlloc` to
// `cuMemAlloc_v2` at compile time, so an application's undefined reference carries the
// suffix, while code that resolves the name dynamically may not.
// ---------------------------------------------------------------------------

unsafe fn cu_mem_alloc_impl(dptr: *mut u64, bytesize: usize) -> c_int {
    if let Some(pooled) = pooled_allocation(bytesize) {
        if !dptr.is_null() {
            *dptr = pooled as u64;
        }
        return CUDA_SUCCESS;
    }
    let original = *ORIGINAL_CU_MEM_ALLOC
        .get_or_init(|| resolve_next_any(&[b"cuMemAlloc_v2\0", b"cuMemAlloc\0"]));
    match original {
        Some(f) => call_original(|| f(dptr, bytesize)),
        None => CUDA_ERROR_NOT_INITIALIZED,
    }
}

unsafe fn cu_mem_free_impl(dptr: u64) -> c_int {
    if pooled_free(dptr as *mut c_void) {
        return CUDA_SUCCESS;
    }
    let original = *ORIGINAL_CU_MEM_FREE
        .get_or_init(|| resolve_next_any(&[b"cuMemFree_v2\0", b"cuMemFree\0"]));
    match original {
        Some(f) => call_original(|| f(dptr)),
        None => CUDA_ERROR_NOT_INITIALIZED,
    }
}

/// # Safety
/// Matches the CUDA Driver `cuMemAlloc` contract.
#[no_mangle]
pub unsafe extern "C" fn cuMemAlloc(dptr: *mut u64, bytesize: usize) -> c_int {
    cu_mem_alloc_impl(dptr, bytesize)
}

/// # Safety
/// Matches the CUDA Driver `cuMemAlloc_v2` contract.
#[no_mangle]
pub unsafe extern "C" fn cuMemAlloc_v2(dptr: *mut u64, bytesize: usize) -> c_int {
    cu_mem_alloc_impl(dptr, bytesize)
}

/// # Safety
/// Matches the CUDA Driver `cuMemFree` contract.
#[no_mangle]
pub unsafe extern "C" fn cuMemFree(dptr: u64) -> c_int {
    cu_mem_free_impl(dptr)
}

/// # Safety
/// Matches the CUDA Driver `cuMemFree_v2` contract.
#[no_mangle]
pub unsafe extern "C" fn cuMemFree_v2(dptr: u64) -> c_int {
    cu_mem_free_impl(dptr)
}

/// # Safety
/// Matches the CUDA Driver `cuMemAllocAsync` contract.
#[no_mangle]
pub unsafe extern "C" fn cuMemAllocAsync(
    dptr: *mut u64,
    bytesize: usize,
    stream: *mut c_void,
) -> c_int {
    if let Some(pooled) = pooled_allocation(bytesize) {
        if !dptr.is_null() {
            *dptr = pooled as u64;
        }
        return CUDA_SUCCESS;
    }
    let original =
        *ORIGINAL_CU_MEM_ALLOC_ASYNC.get_or_init(|| resolve_next_any(&[b"cuMemAllocAsync\0"]));
    match original {
        Some(f) => call_original(|| f(dptr, bytesize, stream)),
        None => CUDA_ERROR_NOT_INITIALIZED,
    }
}

/// # Safety
/// Matches the CUDA Driver `cuMemFreeAsync` contract.
#[no_mangle]
pub unsafe extern "C" fn cuMemFreeAsync(dptr: u64, stream: *mut c_void) -> c_int {
    if pooled_free_stream_ordered(dptr as *mut c_void, || synchronize_stream_driver(stream)) {
        return CUDA_SUCCESS;
    }
    let original =
        *ORIGINAL_CU_MEM_FREE_ASYNC.get_or_init(|| resolve_next_any(&[b"cuMemFreeAsync\0"]));
    match original {
        Some(f) => call_original(|| f(dptr, stream)),
        None => CUDA_ERROR_NOT_INITIALIZED,
    }
}

/// # Safety
/// Matches the CUDA Driver `cuMemCreate` contract.
///
/// Pass-through only: the VMM allocator returns an allocation *handle* that the caller
/// then maps at an address of its choosing, so it cannot be substituted with a pooled
/// pointer. Warn once when it happens inside a publish window, because `publish()`
/// would otherwise report only that the buffer "was not allocated from the GPU pool".
#[no_mangle]
pub unsafe extern "C" fn cuMemCreate(
    handle: *mut u64,
    size: usize,
    prop: *const c_void,
    flags: u64,
) -> c_int {
    static WARNED: AtomicBool = AtomicBool::new(false);
    if in_pool_window() && !WARNED.swap(true, Ordering::Relaxed) {
        eprintln!(
            "[agnocast_cuda_heaphook] cuMemCreate() was called inside a publisher's \
             borrow_loaned_message()..publish() window. Virtual-memory-management \
             allocations cannot be served from the Agnocast GPU pool, so this buffer will \
             not be shared zero-copy. Use cudaMalloc/cuMemAlloc for the message buffer."
        );
    }
    let original = *ORIGINAL_CU_MEM_CREATE.get_or_init(|| resolve_next_any(&[b"cuMemCreate\0"]));
    match original {
        Some(f) => call_original(|| f(handle, size, prop, flags)),
        None => CUDA_ERROR_NOT_INITIALIZED,
    }
}

/// # Safety
/// Matches the CUDA Driver `cuStreamCreate` contract.
#[no_mangle]
pub unsafe extern "C" fn cuStreamCreate(stream: *mut *mut c_void, flags: c_uint) -> c_int {
    let original =
        *ORIGINAL_CU_STREAM_CREATE.get_or_init(|| resolve_next_any(&[b"cuStreamCreate\0"]));
    let result = match original {
        Some(f) => call_original(|| f(stream, flags)),
        None => return CUDA_ERROR_NOT_INITIALIZED,
    };
    if result == CUDA_SUCCESS && is_non_blocking_stream(flags) {
        note_non_blocking_stream();
    }
    result
}

/// # Safety
/// Matches the CUDA Driver `cuStreamCreateWithPriority` contract.
#[no_mangle]
pub unsafe extern "C" fn cuStreamCreateWithPriority(
    stream: *mut *mut c_void,
    flags: c_uint,
    priority: c_int,
) -> c_int {
    let original = *ORIGINAL_CU_STREAM_CREATE_WITH_PRIORITY
        .get_or_init(|| resolve_next_any(&[b"cuStreamCreateWithPriority\0"]));
    let result = match original {
        Some(f) => call_original(|| f(stream, flags, priority)),
        None => return CUDA_ERROR_NOT_INITIALIZED,
    };
    if result == CUDA_SUCCESS && is_non_blocking_stream(flags) {
        note_non_blocking_stream();
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn malloc_routes_to_pool_only_in_window_and_when_available() {
        assert_eq!(decide_malloc_route(true, true, false), MallocRoute::Pool);
        assert_eq!(
            decide_malloc_route(false, true, false),
            MallocRoute::Original
        );
        assert_eq!(
            decide_malloc_route(true, false, false),
            MallocRoute::Original
        );
        assert_eq!(
            decide_malloc_route(false, false, false),
            MallocRoute::Original
        );
    }

    #[test]
    fn reentrant_calls_never_go_to_the_pool() {
        // Otherwise a fallback cudaMalloc that internally reaches cuMemAlloc would be
        // handed a pooled pointer, corrupting the runtime's own bookkeeping.
        assert_eq!(decide_malloc_route(true, true, true), MallocRoute::Original);
    }

    #[test]
    fn free_routes_to_pool_only_when_pool_owns_pointer() {
        assert_eq!(decide_free_route(true), FreeRoute::PoolHandled);
        assert_eq!(decide_free_route(false), FreeRoute::Original);
    }

    #[test]
    fn non_blocking_flag_is_detected_alongside_other_flags() {
        assert!(is_non_blocking_stream(0x01));
        assert!(is_non_blocking_stream(0x01 | 0x08));
        assert!(!is_non_blocking_stream(0x00));
        assert!(!is_non_blocking_stream(0x08));
    }

    #[test]
    fn reentrancy_guard_is_scoped_to_the_call() {
        assert!(!in_original_call());
        call_original(|| assert!(in_original_call()));
        assert!(!in_original_call());
    }

    #[test]
    fn reentrancy_guard_nests() {
        // An inner pass-through must not clear the flag for the outer call that is still
        // running: otherwise a second inner allocation would be served from the pool and
        // handed to the CUDA runtime for its own bookkeeping.
        call_original(|| {
            assert!(in_original_call());
            call_original(|| assert!(in_original_call()));
            assert!(in_original_call());
        });
        assert!(!in_original_call());
    }

    #[test]
    fn reentrancy_guard_is_restored_on_unwind() {
        let outcome = std::panic::catch_unwind(|| {
            call_original(|| panic!("boom"));
        });
        assert!(outcome.is_err());
        assert!(!in_original_call());
    }

    #[test]
    fn version_is_non_empty() {
        let ptr = agnocast_cuda_heaphook_get_version();
        let version = unsafe { CStr::from_ptr(ptr) };
        assert!(!version.to_bytes().is_empty());
    }
}
