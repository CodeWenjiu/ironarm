//! Copper 任务模块。
//!
//! `JointInterpolator` 已移至 `ironarm_core`（引入时间接口后不再需要 std）。

pub mod motion_planner;
pub mod state_sink;

pub use motion_planner::MotionPlanner;
pub use state_sink::StateSink;

/// 监控器类型别名。
pub mod monitor {
    pub type AppMonitor = cu_consolemon::CuConsoleMon;
}
