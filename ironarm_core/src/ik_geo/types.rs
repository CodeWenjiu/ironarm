//! 运动学类型定义（基于 glam）。

/// 关节螺旋轴（零位形下基坐标系中的单位向量）。
pub type ScrewAxes = [glam::Vec3; 6];

/// 连杆偏移：p[i] = 关节 i 到关节 i+1 的向量（p[6] = 关节 6 → 工具法兰）。
pub type LinkOffsets = [glam::Vec3; 7];
