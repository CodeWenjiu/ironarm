//! Pieper 型 6 自由度机械臂的解析逆运动学。
//!
//! 实现 IK-Geo 算法（https://arxiv.org/abs/2211.05737）中
//! "三平行轴 + 两相交轴" 的闭式解法。
//!
//! 向量/矩阵运算由 glam 提供（no_std + libm）。

mod fk;
mod math;
mod solver;
mod subprobs;
mod types;

pub use fk::fk;
pub use math::wrap_to_pi;
pub use solver::solve_3p2i;
pub use types::{LinkOffsets, ScrewAxes};
