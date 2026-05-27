pub mod messages;
pub mod tasks;

use cu29::prelude::*;
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron")]
struct IronArmCli {}

/// 创建并运行 TUI 模式的 Copper 应用。
pub fn run_cli(logger_path: &Path, slab_size: Option<usize>) {
    if let Some(parent) = logger_path.parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }

    debug!("Logger created at {}.", logger_path);
    debug!("Creating application... ");

    let mut application = IronArmCli::builder()
        .with_log_path(logger_path, slab_size)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");

    debug!("Running... starting clock: {}.", application.clock().now());
    if let Err(e) = application.run() {
        debug!("Application stopped: {}.", e);
    }
    debug!("End of program: {}.", application.clock().now());
}
