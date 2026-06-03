//! Lock-free shared state — copper writes, Python reads.
//! Uses a seqlock: writer bumps seq before/after write, reader
//! retries if seq changed mid-read.  Always returns latest value.
//! No allocation, no GIL, wait-free for writer.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, Ordering};

/// Data shared between copper and Python (44 bytes).
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

static SEQ: AtomicU64 = AtomicU64::new(0);

struct DataCell(UnsafeCell<MaybeUninit<ArmState>>);
unsafe impl Sync for DataCell {}

static DATA: DataCell = DataCell(UnsafeCell::new(MaybeUninit::uninit()));

/// Copper thread: write latest state.  Wait-free.
pub fn write(state: ArmState) {
    let seq = SEQ.load(Ordering::Relaxed).wrapping_add(1); // odd = writing
    SEQ.store(seq, Ordering::Release); // fence: subsequent writes stay below
    unsafe { (*DATA.0.get()).write(state) };
    SEQ.store(seq.wrapping_add(1), Ordering::Release); // even = done
}

/// Python thread: read latest state.  Returns None if no data written yet.
pub fn read() -> Option<ArmState> {
    loop {
        let seq1 = SEQ.load(Ordering::Acquire);
        if seq1 == 0 {
            return None; // never written
        }
        if seq1 & 1 != 0 {
            // writer in progress, spin briefly
            std::hint::spin_loop();
            continue;
        }
        let state = unsafe { (*DATA.0.get()).assume_init() };
        let seq2 = SEQ.load(Ordering::Acquire);
        if seq2 == seq1 {
            return Some(state);
        }
        // seq changed — writer updated mid-read, retry
    }
}
