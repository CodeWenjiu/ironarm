use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::thread;
use std::time::Duration;

fn main() {
    let logger_path = std::path::Path::new("target/copper_check.copper");
    let t = thread::spawn(move || {
        ironarm_std::run_tui(logger_path, Some(1024 * 1024 * 10));
    });

    thread::sleep(Duration::from_millis(500));

    let count = Arc::new(AtomicU32::new(0));
    let c = count.clone();
    let start = std::time::Instant::now();

    while start.elapsed() < Duration::from_secs(3) {
        if ironarm_std::ringbuf::read().is_some() {
            c.fetch_add(1, Ordering::Relaxed);
        }
        thread::sleep(Duration::from_millis(1));
    }

    let n = count.load(Ordering::Relaxed);
    println!("After 3s: {n} states polled");
    if n == 0 {
        println!("FAIL");
    } else {
        println!("OK");
    }
}
