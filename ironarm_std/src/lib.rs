pub mod ringbuf;
pub mod tasks;
pub mod trajectory;

use cu29::prelude::*;
use std::path::Path;

#[copper_runtime(config = "copperconfig.ron")]
struct IronArmCli {}

pub fn run_tui(logger_path: &Path, slab_size: Option<usize>) {
    if let Some(parent) = logger_path.parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }

    let mut application = IronArmCli::builder()
        .with_log_path(logger_path, slab_size)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");

    if let Err(e) = application.run() {
        debug!("Application stopped: {}.", e);
    }
}
