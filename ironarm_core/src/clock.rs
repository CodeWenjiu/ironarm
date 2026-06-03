//! 单调时钟——由上层注入具体实现。
//!
//! core 不直接依赖 `std::time`，而是通过全局函数指针注入。
//! std 构建时注入 `Instant::now`，嵌入式构建时注入硬件定时器。

use core::sync::atomic::{AtomicPtr, Ordering};

/// 时钟函数类型：返回自某参考点以来的秒数（单调递增即可）。
type ClockFn = fn() -> f32;

static CLOCK: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

/// 注册全局时钟（由上层在启动时调用一次）。
///
/// # Safety
/// 必须在任何任务运行前调用，且 f 的生命周期必须覆盖整个程序。
pub fn set_clock(f: ClockFn) {
    CLOCK.store(f as *mut (), Ordering::Release);
}

/// 读取全局时钟。若未注册则返回 0.0（退化为透传模式）。
pub fn now_secs() -> f32 {
    let ptr = CLOCK.load(Ordering::Acquire);
    if ptr.is_null() {
        0.0
    } else {
        let f: ClockFn = unsafe { core::mem::transmute(ptr) };
        f()
    }
}
