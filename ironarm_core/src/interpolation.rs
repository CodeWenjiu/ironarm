//! 通用插值算法——lerp、路径点采样、平面基向量。
//!
//! 不含具体轨迹定义（圆、∞ 字等），由上层通过 Rhai 脚本提供。

use crate::messages::CartesianWaypoint;

/// 线性插值。
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// 由法向量 n 构建平面的正交基 (u, v)。
pub fn plane_basis(nx: f32, ny: f32, nz: f32) -> ((f32, f32, f32), (f32, f32, f32)) {
    let (rx, ry, rz) = if nx.abs() < 0.9 {
        (1.0, 0.0, 0.0)
    } else {
        (0.0, 1.0, 0.0)
    };
    let ux = ry * nz - rz * ny;
    let uy = rz * nx - rx * nz;
    let uz = rx * ny - ry * nx;
    let um = f32::hypot(f32::hypot(ux, uy), uz);
    let (ux, uy, uz) = if um > 0.0 {
        (ux / um, uy / um, uz / um)
    } else {
        (1.0, 0.0, 0.0)
    };
    let vx = ny * uz - nz * uy;
    let vy = nz * ux - nx * uz;
    let vz = nx * uy - ny * ux;
    ((ux, uy, uz), (vx, vy, vz))
}

/// 在排序的 (时刻, 路径点) 列表中插值。若 looped 则循环。
pub fn sample_waypoints(
    points: &[(f32, CartesianWaypoint)],
    t: f32,
    looped: bool,
) -> CartesianWaypoint {
    if points.is_empty() {
        return CartesianWaypoint::default();
    }
    if points.len() == 1 {
        return points[0].1;
    }
    let total = points.last().unwrap().0;
    let t = if looped && total > 0.0 {
        let w = t % total;
        if w < 0.0 { w + total } else { w }
    } else {
        t.clamp(0.0, total)
    };
    for i in 0..points.len().saturating_sub(1) {
        let (t0, t1) = (points[i].0, points[i + 1].0);
        if t >= t0 && t < t1 {
            let f = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
            let a = &points[i].1;
            let b = &points[i + 1].1;
            return CartesianWaypoint {
                x: lerp(a.x, b.x, f),
                y: lerp(a.y, b.y, f),
                z: lerp(a.z, b.z, f),
                rx: lerp(a.rx, b.rx, f),
                ry: lerp(a.ry, b.ry, f),
                rz: lerp(a.rz, b.rz, f),
            };
        }
    }
    points.last().unwrap().1
}
