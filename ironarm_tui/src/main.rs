use std::path::Path;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);
const APP_NAME: &str = env!("CARGO_PKG_NAME");

fn main() {
    let logger_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("Failed to get workspace root directory")
        .join("target")
        .join(format!("{APP_NAME}_log.copper"));

    ironarm_std::run_tui(&logger_path, PREALLOCATED_STORAGE_SIZE);
}
