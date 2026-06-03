//! Copper 运行时入口。
//!
//! `#[copper_runtime]` 宏展开为主循环，读取 `copperconfig.ron`
//! 构建 DAG，按配置启动所有任务。

pub mod shared_state;
pub mod tasks;

use cu29::prelude::*;
use std::path::Path;
use std::sync::OnceLock;
use std::time::Instant;

#[copper_runtime(config = "copperconfig.ron")]
struct IronArmCli {}

/// 启动 Copper 运行时（阻塞，由独立线程调用）。
pub fn run_tui(logger_path: &Path, slab_size: Option<usize>) {
    // 注入 std 时钟：core 层的插值器通过此接口获取时间
    ironarm_core::clock::set_clock(|| {
        static START: OnceLock<Instant> = OnceLock::new();
        START.get_or_init(Instant::now).elapsed().as_secs_f32()
    });
    if let Some(parent) = logger_path.parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("无法创建日志目录");
        }
    }

    let mut application = IronArmCli::builder()
        .with_log_path(logger_path, slab_size)
        .expect("日志初始化失败")
        .build()
        .expect("应用构建失败");

    if let Err(e) = application.run() {
        debug!("应用已停止: {}.", e);
    }
}
