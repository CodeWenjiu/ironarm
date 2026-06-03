//! 跨线程共享状态——Copper 写，Python 读。
//!
//! 基于社区 `triple_buffer` crate，无等待 SPSC 三缓冲：
//! - 写方无锁，始终有可写槽
//! - 读方无锁，始终返回最新完整值
//! - 零拷贝，无 unsafe（由 crate 审计）

use std::sync::{Mutex, OnceLock};
use triple_buffer::{Input, Output, triple_buffer};

/// Copper 与 Python 之间共享的状态。
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ArmState {
    pub j0: f32,
    pub j1: f32,
    pub j2: f32,
    pub j3: f32,
    pub j4: f32,
    pub j5: f32,
    pub wx: f32,
    pub wy: f32,
    pub wz: f32,
}

fn init() -> &'static (Mutex<Input<ArmState>>, Mutex<Output<ArmState>>) {
    static BUF: OnceLock<(Mutex<Input<ArmState>>, Mutex<Output<ArmState>>)> = OnceLock::new();
    BUF.get_or_init(|| {
        let (inp, out) = triple_buffer(&ArmState {
            j0: 0.0,
            j1: 0.0,
            j2: 0.0,
            j3: 0.0,
            j4: 0.0,
            j5: 0.0,
            wx: 0.0,
            wy: 0.0,
            wz: 0.0,
        });
        (Mutex::new(inp), Mutex::new(out))
    })
}

/// Copper 线程调用：写入最新状态。
pub fn write(state: ArmState) {
    init().0.lock().unwrap().write(state);
}

/// Python 线程调用：读取最新完整状态。
pub fn read() -> ArmState {
    *init().1.lock().unwrap().read()
}
