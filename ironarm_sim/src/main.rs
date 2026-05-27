mod messages;
mod tasks;

use bevy::app::{App, AppExit};
use bevy::prelude::{
    DefaultPlugins, MessageReader, MessageWriter, PostUpdate,
    Res, ResMut, Resource, Time, Update,
};
use cu29::prelude::*;
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use ironarm_core::messages::JointCommand;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct IronArmSim {}

fn sim_callback(step: crate::default::SimStep) -> SimOverride {
    match step {
        crate::default::SimStep::CmdSrc(CuTaskCallbackState::Process(_input, output)) => {
            output.set_payload(JointCommand {
                target_angle: 0.5,
                target_velocity: 0.0,
                stiffness: 1.0,
            });
            SimOverride::ExecutedBySim
        }
        crate::default::SimStep::StateSink(CuTaskCallbackState::Process(input, _output)) => {
            let (j0, j1) = input;
            if let Some(s) = j0.payload() {
                eprintln!("Sink[sim][0]: angle={}", s.current_angle);
            }
            if let Some(s) = j1.payload() {
                eprintln!("Sink[sim][1]: angle={}", s.current_angle);
            }
            SimOverride::ExecutedBySim
        }
        _ => SimOverride::ExecuteByRuntime,
    }
}

#[derive(Resource)]
struct CopperApp {
    app: IronArmSim,
    clock_mock: RobotClockMock,
}

fn main() {
    let logger_path = std::env::temp_dir().join("ironarm_sim.copper");
    let (robot_clock, clock_mock) = RobotClock::mock();

    let mut copper = IronArmSim::builder()
        .with_clock(robot_clock.clone())
        .with_log_path(&logger_path, Some(1024 * 1024 * 10))
        .expect("Failed to setup logger.")
        .with_sim_callback(&mut sim_callback)
        .build()
        .expect("Failed to create sim runtime.");

    copper
        .start_all_tasks(&mut sim_callback)
        .expect("Failed to start all tasks.");

    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    app.insert_resource(CopperApp { app: copper, clock_mock });
    app.add_systems(Update, run_tick);
    app.add_systems(PostUpdate, stop_copper_on_exit);
    app.run();
}

fn run_tick(
    mut copper: ResMut<CopperApp>,
    time: Res<Time>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    copper.clock_mock.set_value(time.elapsed().as_nanos() as u64);
    if let Err(e) = copper.app.run_one_iteration(&mut sim_callback) {
        eprintln!("Sim stopped: {}", e);
        exit_writer.write(AppExit::Success);
    }
}

fn stop_copper_on_exit(
    mut exit_events: MessageReader<AppExit>,
    mut copper: ResMut<CopperApp>,
) {
    if exit_events.read().next().is_some() {
        copper
            .app
            .stop_all_tasks(&mut sim_callback)
            .expect("Failed to stop all tasks.");
    }
}
