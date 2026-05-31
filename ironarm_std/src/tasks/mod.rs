pub mod monitor {
    pub type AppMonitor = cu_consolemon::CuConsoleMon;
}

pub mod joint_interpolator;
pub mod motion_planner;

pub use joint_interpolator::JointInterpolator;
pub use motion_planner::MotionPlanner;
