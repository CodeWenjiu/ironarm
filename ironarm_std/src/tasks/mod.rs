pub mod monitor {
    pub type AppMonitor = cu_consolemon::CuConsoleMon;
}

pub mod motion_planner;
pub use motion_planner::MotionPlanner;
