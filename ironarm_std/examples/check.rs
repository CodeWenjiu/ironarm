use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::thread;
use std::time::Duration;

fn main() {
    let count = Arc::new(AtomicU32::new(0));
    let c = count.clone();
    ironarm_std::tasks::state_sink::set_callback(Box::new(move |j0, j1, j2, j3, wx, wy, wz| {
        c.fetch_add(1, Ordering::Relaxed);
        println!(
            "CB #{n}: j=({j0:.3},{j1:.3},{j2:.3},{j3:.3}) wp=({wx:.2},{wy:.2},{wz:.2})",
            n = c.load(Ordering::Relaxed)
        );
    }));

    println!("Starting copper...");
    let logger_path = std::path::Path::new("target/copper_check.copper");
    let _ = thread::spawn(move || {
        ironarm_std::run_tui(logger_path, Some(1024 * 1024 * 10));
    });

    thread::sleep(Duration::from_secs(3));
    let n = count.load(Ordering::Relaxed);
    println!("After 3s: {n} callbacks");
    if n == 0 {
        println!("FAIL");
    } else {
        println!("OK ({:.0} Hz)", n as f32 / 3.0);
    }
}
