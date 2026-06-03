//! Trajectory types — map time t → CartesianWaypoint.
//!
//! Pure-math, `no_std` compatible.  Callers sample the trajectory at a
//! given time *t* (in seconds) to obtain a Cartesian waypoint.

use ironarm_core::messages::CartesianWaypoint;
use std::vec::Vec;

// ---------------------------------------------------------------------------
// Trajectory type
// ---------------------------------------------------------------------------

/// A trajectory maps time *t* (seconds) to a Cartesian waypoint.
#[derive(Debug, Clone)]
pub enum Trajectory {
    /// Horizontal circle at height *z*.
    Circle {
        cx: f32,
        cy: f32,
        r: f32,
        z: f32,
        period: f32,
    },
    /// Circle in an arbitrary plane (not parallel to any standard plane).
    /// `nx, ny, nz` is the plane normal (auto-normalised).
    TiltedCircle {
        cx: f32,
        cy: f32,
        cz: f32,
        nx: f32,
        ny: f32,
        nz: f32,
        r: f32,
        period: f32,
    },
    /// Linear interpolation from *start* to *end* over *duration* seconds.
    Linear {
        start: CartesianWaypoint,
        end: CartesianWaypoint,
        duration: f32,
    },
    /// Multi-waypoint trajectory.  Each entry is `(time_seconds, waypoint)`.
    /// Linear interpolation between consecutive entries.  If *looped*, wraps
    /// around; otherwise clamps at the last waypoint.
    Waypoints {
        points: Vec<(f32, CartesianWaypoint)>,
        looped: bool,
    },
}

impl Default for Trajectory {
    fn default() -> Self {
        Self::Circle {
            cx: 0.0,
            cy: 0.0,
            r: 0.0,
            z: 0.0,
            period: 1.0,
        }
    }
}

impl Trajectory {
    /// Sample the trajectory at time *t* (seconds).
    pub fn sample(&self, t: f32) -> CartesianWaypoint {
        match self {
            Self::Circle {
                cx,
                cy,
                r,
                z,
                period,
            } => {
                use core::f32::consts::PI;
                let phase = t * 2.0 * PI / period;
                CartesianWaypoint {
                    x: cx + r * phase.cos(),
                    y: cy + r * phase.sin(),
                    z: *z,
                }
            }
            Self::TiltedCircle {
                cx,
                cy,
                cz,
                nx,
                ny,
                nz,
                r,
                period,
            } => {
                use core::f32::consts::PI;
                let (u, v) = plane_basis(*nx, *ny, *nz);
                let phase = t * 2.0 * PI / period;
                CartesianWaypoint {
                    x: cx + r * (phase.cos() * u.0 + phase.sin() * v.0),
                    y: cy + r * (phase.cos() * u.1 + phase.sin() * v.1),
                    z: cz + r * (phase.cos() * u.2 + phase.sin() * v.2),
                }
            }
            Self::Linear {
                start,
                end,
                duration,
            } => {
                let frac = if *duration <= 0.0 {
                    1.0
                } else {
                    (t / duration).clamp(0.0, 1.0)
                };
                CartesianWaypoint {
                    x: lerp(start.x, end.x, frac),
                    y: lerp(start.y, end.y, frac),
                    z: lerp(start.z, end.z, frac),
                }
            }
            Self::Waypoints { points, looped } => sample_waypoints(points, t, *looped),
        }
    }
}

// ---------------------------------------------------------------------------
// Convenience constructors
// ---------------------------------------------------------------------------

/// Create a circular trajectory.
pub fn circle(cx: f32, cy: f32, r: f32, z: f32, period: f32) -> Trajectory {
    Trajectory::Circle {
        cx,
        cy,
        r,
        z,
        period,
    }
}

/// Create a linear trajectory.
pub fn linear(start: CartesianWaypoint, end: CartesianWaypoint, duration: f32) -> Trajectory {
    Trajectory::Linear {
        start,
        end,
        duration,
    }
}

/// Create a tilted-circle trajectory (plane defined by normal vector).
pub fn tilted_circle(
    cx: f32,
    cy: f32,
    cz: f32,
    nx: f32,
    ny: f32,
    nz: f32,
    r: f32,
    period: f32,
) -> Trajectory {
    let mag = f32::hypot(f32::hypot(nx, ny), nz);
    let (nx, ny, nz) = if mag > 0.0 {
        (nx / mag, ny / mag, nz / mag)
    } else {
        (0.0, 0.0, 1.0)
    };
    Trajectory::TiltedCircle {
        cx,
        cy,
        cz,
        nx,
        ny,
        nz,
        r,
        period,
    }
}

/// Create a waypoint trajectory.
pub fn waypoints(points: Vec<(f32, CartesianWaypoint)>, looped: bool) -> Trajectory {
    Trajectory::Waypoints { points, looped }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build two orthogonal unit vectors (u, v) spanning the plane with normal n.
fn plane_basis(nx: f32, ny: f32, nz: f32) -> ((f32, f32, f32), (f32, f32, f32)) {
    // Choose a reference vector not parallel to n
    let (rx, ry, rz) = if nx.abs() < 0.9 {
        (1.0, 0.0, 0.0)
    } else {
        (0.0, 1.0, 0.0)
    };
    // u = normalize(cross(r, n))
    let ux = ry * nz - rz * ny;
    let uy = rz * nx - rx * nz;
    let uz = rx * ny - ry * nx;
    let um = f32::hypot(f32::hypot(ux, uy), uz);
    let (ux, uy, uz) = if um > 0.0 {
        (ux / um, uy / um, uz / um)
    } else {
        (1.0, 0.0, 0.0)
    };
    // v = cross(n, u)
    let vx = ny * uz - nz * uy;
    let vy = nz * ux - nx * uz;
    let vz = nx * uy - ny * ux;
    ((ux, uy, uz), (vx, vy, vz))
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Linear interpolation through a sorted list of `(time, waypoint)` entries.
/// If *looped*, the trajectory wraps; otherwise it clamps at the last entry.
fn sample_waypoints(
    points: &[(f32, CartesianWaypoint)],
    t: f32,
    looped: bool,
) -> CartesianWaypoint {
    if points.is_empty() {
        return CartesianWaypoint {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
    }
    if points.len() == 1 {
        return points[0].1.clone();
    }

    let total_duration = points.last().unwrap().0;

    let t = if looped && total_duration > 0.0 {
        // wrap t into [0, total_duration)
        let wrapped = t % total_duration;
        if wrapped < 0.0 {
            wrapped + total_duration
        } else {
            wrapped
        }
    } else {
        t.clamp(0.0, total_duration)
    };

    // find the segment [points[i], points[i+1]) that contains t
    for i in 0..points.len().saturating_sub(1) {
        let t0 = points[i].0;
        let t1 = points[i + 1].0;
        if t >= t0 && t < t1 {
            let frac = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
            return CartesianWaypoint {
                x: lerp(points[i].1.x, points[i + 1].1.x, frac),
                y: lerp(points[i].1.y, points[i + 1].1.y, frac),
                z: lerp(points[i].1.z, points[i + 1].1.z, frac),
            };
        }
    }

    // t >= last time or no segment matched — return last waypoint
    points.last().unwrap().1.clone()
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circle_samples() {
        let traj = circle(0.0, 0.0, 1.0, 0.5, 4.0);
        let p0 = traj.sample(0.0);
        assert!((p0.x - 1.0).abs() < 0.01);
        assert!(p0.y.abs() < 0.01);
        assert!((p0.z - 0.5).abs() < 0.01);

        let p1 = traj.sample(1.0);
        assert!(p1.x.abs() < 0.01);
        assert!((p1.y - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_linear() {
        let traj = linear(
            CartesianWaypoint { x: 0.0, y: 0.0, z: 0.0 },
            CartesianWaypoint { x: 2.0, y: 4.0, z: 0.0 },
            2.0,
        );
        let p = traj.sample(1.0);
        assert!((p.x - 1.0).abs() < 0.01);
        assert!((p.y - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_waypoints() {
        let traj = waypoints(
            vec![
                (0.0, CartesianWaypoint { x: 0.0, y: 0.0, z: 0.0 }),
                (2.0, CartesianWaypoint { x: 2.0, y: 0.0, z: 0.0 }),
                (4.0, CartesianWaypoint { x: 2.0, y: 2.0, z: 0.0 }),
            ],
            false,
        );
        let p = traj.sample(1.0);
        assert!((p.x - 1.0).abs() < 0.01);
        assert!(p.y.abs() < 0.01);
    }

    #[test]
    fn test_waypoints_looped() {
        let traj = waypoints(
            vec![
                (0.0, CartesianWaypoint { x: 0.0, y: 0.0, z: 0.0 }),
                (2.0, CartesianWaypoint { x: 2.0, y: 0.0, z: 0.0 }),
            ],
            true,
        );
        let p = traj.sample(3.0);
        assert!((p.x - 1.0).abs() < 0.01);
        assert!(p.y.abs() < 0.01);
    }
}
