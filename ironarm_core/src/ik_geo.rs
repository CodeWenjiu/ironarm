//! Pieper 型 6 自由度机械臂的解析逆运动学。
//!
//! 实现 IK-Geo 算法（https://arxiv.org/abs/2211.05737）中
//! "三平行轴 + 两相交轴" 的闭式解法。
//!
//! 目录结构：
//! - `ik_geo/types.rs`    — 类型定义（ScrewAxes, LinkOffsets）
//! - `ik_geo/vec.rs`      — 三维向量运算
//! - `ik_geo/mat.rs`      — 矩阵运算 & Rodrigues 公式
//! - `ik_geo/fk.rs`       — 正运动学
//! - `ik_geo/subprobs.rs` — 子问题 1/3/4
//! - `ik_geo/solver.rs`   — solve_3p2i 主求解器

mod fk;
pub(crate) mod mat;
mod solver;
mod subprobs;
mod types;
pub(crate) mod vec;

pub use fk::fk;
pub use solver::solve_3p2i;
pub use types::{LinkOffsets, ScrewAxes};
