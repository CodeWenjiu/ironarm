//! 碰撞约束——配置解析 + 碰撞检测 + 解选择。

use alloc::format;
use alloc::vec::Vec;
use cu29::prelude::ComponentConfig;
use glam::{Mat3, Vec3};

use crate::ik_geo::{LinkOffsets, ScrewAxes};

// ---------------------------------------------------------------------------
// 类型 & 基础检测
// ---------------------------------------------------------------------------

pub type ObstacleSphere = [f32; 4];

pub fn joint_positions(h: &ScrewAxes, p: &LinkOffsets, q: &[f32; 6]) -> [Vec3; 7] {
    let mut pts = [Vec3::ZERO; 7];
    let mut r = Mat3::IDENTITY;
    let mut pos = p[0];
    pts[0] = pos;
    for i in 0..6 {
        r = r * Mat3::from_axis_angle(h[i], q[i]);
        pos += r * p[i + 1];
        pts[i + 1] = pos;
    }
    pts
}

pub fn is_collision_free(
    h: &ScrewAxes,
    p: &LinkOffsets,
    q: &[f32; 6],
    obstacles: &[ObstacleSphere],
    margin: f32,
) -> bool {
    let pts = joint_positions(h, p, q);
    for obs in obstacles {
        let r2 = (obs[3] + margin) * (obs[3] + margin);
        let obs_pos = Vec3::new(obs[0], obs[1], obs[2]);
        for pt in &pts {
            if (*pt - obs_pos).length_squared() < r2 {
                return false;
            }
        }
    }
    true
}

pub fn pick_safest(
    h: &ScrewAxes,
    p: &LinkOffsets,
    candidates: &[[f32; 6]],
    obstacles: &[ObstacleSphere],
    margin: f32,
) -> [f32; 6] {
    let valid: Vec<&[f32; 6]> = candidates
        .iter()
        .filter(|q| q.iter().all(|a| a.is_finite()))
        .collect();
    if valid.is_empty() {
        return [0.0; 6];
    }
    let mut best = valid[0];
    let mut best_dist = f32::MIN;
    for q in &valid {
        if is_collision_free(h, p, q, obstacles, margin) {
            let pts = joint_positions(h, p, q);
            let mut min_d2 = f32::MAX;
            for obs in obstacles {
                let obs_pos = Vec3::new(obs[0], obs[1], obs[2]);
                for pt in &pts {
                    min_d2 = min_d2.min((*pt - obs_pos).length_squared());
                }
            }
            if min_d2 > best_dist {
                best_dist = min_d2;
                best = q;
            }
        }
    }
    *best
}

// ---------------------------------------------------------------------------
// 配置解析
// ---------------------------------------------------------------------------

pub struct CollisionConfig {
    pub spheres: Vec<ObstacleSphere>,
    pub margin: f32,
}

impl Default for CollisionConfig {
    fn default() -> Self {
        Self {
            spheres: Vec::new(),
            margin: 0.05,
        }
    }
}

impl CollisionConfig {
    pub fn from_config(config: Option<&ComponentConfig>) -> Self {
        let cfg = config;
        let n: usize = cfg
            .and_then(|c| c.get::<f64>("obstacle_count").ok().flatten())
            .unwrap_or(0.0) as usize;
        let mut spheres = Vec::with_capacity(n);
        for i in 0..n {
            let prefix = format!("obs{i}");
            spheres.push([
                get_f32(cfg, &format!("{prefix}_x")),
                get_f32(cfg, &format!("{prefix}_y")),
                get_f32(cfg, &format!("{prefix}_z")),
                get_f32(cfg, &format!("{prefix}_r")),
            ]);
        }
        let margin = cfg
            .and_then(|c| c.get::<f64>("safety_margin").ok().flatten())
            .unwrap_or(0.05) as f32;
        Self { spheres, margin }
    }

    pub fn pick(&self, h: &ScrewAxes, p: &LinkOffsets, candidates: &[[f32; 6]]) -> [f32; 6] {
        if self.spheres.is_empty() {
            candidates
                .iter()
                .find(|s| s.iter().all(|a| a.is_finite()))
                .copied()
                .unwrap_or([0.0; 6])
        } else {
            pick_safest(h, p, candidates, &self.spheres, self.margin)
        }
    }
}

fn get_f32(cfg: Option<&ComponentConfig>, key: &str) -> f32 {
    cfg.and_then(|c| c.get::<f64>(key).ok().flatten())
        .unwrap_or(0.0) as f32
}

// ---------------------------------------------------------------------------
// 测试
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trajectory_avoidance() {
        let h = ironarm_model::SCREW_AXES;
        let p = ironarm_model::LINK_OFFSETS;
        let ground: [ObstacleSphere; 1] = [[0.0, 0.0, -10.0, 10.0]];
        let r_target = Mat3::IDENTITY;
        let mut violations = 0;
        let mut total = 0;
        for i in 0..20 {
            let phase = i as f32 * 2.0 * core::f32::consts::PI / 20.0;
            let (nx, ny, nz) = (0.5f32, 0.3, 0.8);
            let mag = f32::hypot(f32::hypot(nx, ny), nz);
            let (nx, ny, nz) = (nx / mag, ny / mag, nz / mag);
            let ru = (1.0f32, 0.0, 0.0);
            let v = (
                ny * ru.2 - nz * ru.1,
                nz * ru.0 - nx * ru.2,
                nx * ru.1 - ny * ru.0,
            );
            let um = f32::hypot(f32::hypot(ru.0, ru.1), ru.2);
            let un = (ru.0 / um, ru.1 / um, ru.2 / um);
            let pt = Vec3::new(
                -0.4 + 0.12 * (phase.cos() * un.0 + phase.sin() * v.0),
                0.12 * (phase.cos() * un.1 + phase.sin() * v.1),
                0.35 + 0.12 * (phase.cos() * un.2 + phase.sin() * v.2),
            );
            let sols = crate::ik_geo::solve_3p2i(&r_target, &pt, &h, &p);
            let q = pick_safest(&h, &p, &sols, &ground, 0.05);
            let pts = joint_positions(&h, &p, &q);
            total += 1;
            if pts.iter().any(|p| p.z < 0.0) {
                violations += 1;
            }
        }
        assert!(violations == 0, "{violations}/{total} ground penetrations");
    }
}
