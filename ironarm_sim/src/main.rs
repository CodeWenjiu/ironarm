mod messages;
mod tasks;

use bevy::app::{App, AppExit};
use bevy::prelude::{
    DefaultPlugins, Fixed, FixedUpdate, MessageReader, MessageWriter, PostUpdate, Res, ResMut,
    Resource,
};
use bevy::time::Time;
use cu29::prelude::*;
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use ironarm_core::messages::JointCommand;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct IronArmSim {}

fn noop_callback(_step: crate::default::SimStep) -> SimOverride {
    SimOverride::ExecuteByRuntime
}

fn set_msg_timing<T: CuMsgPayload>(clock: &RobotClock, msg: &mut CuMsg<T>) {
    let perf = cu29::curuntime::perf_now(clock);
    msg.tov = clock.now().into();
    msg.metadata.process_time.start = perf.into();
    msg.metadata.process_time.end = perf.into();
}

#[derive(Resource)]
struct CopperApp {
    app: IronArmSim,
    clock: RobotClock,
    clock_mock: RobotClockMock,
    last_tick: Option<u64>,
}

fn main() {
    let logger_path = std::env::temp_dir().join("ironarm_sim.copper");
    let (clock, clock_mock) = RobotClock::mock();

    let mut copper = IronArmSim::builder()
        .with_clock(clock.clone())
        .with_log_path(&logger_path, Some(1024 * 1024 * 10))
        .expect("Failed to setup logger.")
        .with_sim_callback(&mut noop_callback)
        .build()
        .expect("Failed to create sim runtime.");

    copper
        .start_all_tasks(&mut noop_callback)
        .expect("Failed to start all tasks.");

    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    app.insert_resource(CopperApp {
        app: copper,
        clock,
        clock_mock,
        last_tick: None,
    });
    app.add_systems(FixedUpdate, run_tick);
    app.add_systems(PostUpdate, stop_copper_on_exit);
    app.run();
}

fn run_tick(
    time: Res<Time<Fixed>>,
    mut copper: ResMut<CopperApp>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    let current_time = time.elapsed().as_nanos() as u64;
    if copper.last_tick == Some(current_time) {
        return;
    }
    copper.last_tick = Some(current_time);
    copper.clock_mock.set_value(current_time);

    let clock = copper.clock.clone();
    let mut cb = move |step: crate::default::SimStep| -> SimOverride {
        match step {
            crate::default::SimStep::CmdSrc(CuTaskCallbackState::Process(_input, output)) => {
                output.set_payload(JointCommand {
                    target_angle: 0.5,
                    target_velocity: 0.0,
                    stiffness: 1.0,
                });
                set_msg_timing(&clock, output);
                SimOverride::ExecutedBySim
            }
            crate::default::SimStep::StateSink(CuTaskCallbackState::Process(input, output)) => {
                let (j0, j1) = input;
                if let Some(s) = j0.payload() {
                    debug!("Sink[sim][0]: angle={:.3} rad", s.current_angle);
                }
                if let Some(s) = j1.payload() {
                    debug!("Sink[sim][1]: angle={:.3} rad", s.current_angle);
                }
                set_msg_timing(&clock, output);
                SimOverride::ExecutedBySim
            }
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    if let Err(e) = copper.app.run_one_iteration(&mut cb) {
        eprintln!("Sim stopped: {}", e);
        exit_writer.write(AppExit::Success);
    }
}

fn stop_copper_on_exit(mut exit_events: MessageReader<AppExit>, mut copper: ResMut<CopperApp>) {
    if exit_events.read().next().is_some() {
        copper
            .app
            .stop_all_tasks(&mut noop_callback)
            .expect("Failed to stop all tasks.");
    }
}
