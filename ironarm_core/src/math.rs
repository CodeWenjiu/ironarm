//! Math utilities for motion control.

/// Exponential smoothing: `current + smoothing * (target - current)`.
pub fn interpolate(current: f32, target: f32, smoothing: f32) -> f32 {
    current + smoothing * (target - current)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ramp() {
        let a = interpolate(0.0, 1.0, 0.2);
        assert!((a - 0.2).abs() < 0.01);
        let b = interpolate(a, 1.0, 0.2);
        assert!((b - 0.36).abs() < 0.01);
    }
}
