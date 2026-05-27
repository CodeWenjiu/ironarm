mod messages;
mod tasks;

use cu29::prelude::*;
use std::path::Path;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);
const APP_NAME: &str = env!("CARGO_PKG_NAME");

#[copper_runtime(config = "copperconfig.ron")]
struct IronArmCli {}

fn main() {
    let logger_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("Failed to get workspace root directory")
        .join("target")
        .join(format!("{APP_NAME}_log.copper"));

    if let Some(parent) = logger_path.parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }

    debug!("Logger created at {}.", &logger_path);
    debug!("Creating application... ");

    let mut application = IronArmCli::builder()
        .with_log_path(&logger_path, PREALLOCATED_STORAGE_SIZE)
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create application.");

    debug!("Running... starting clock: {}.", application.clock().now());
    if let Err(e) = application.run() {
        debug!("Application stopped: {}.", e);
    }
    debug!("End of program: {}.", application.clock().now());
}
