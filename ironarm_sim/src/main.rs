mod arm_config;
mod messages;
mod motion;
mod tasks;
mod world;

use avian3d::prelude::*;
use bevy::app::{App, AppExit, PluginGroup};
use bevy::asset::{AssetApp, AssetPlugin};
use bevy::prelude::{
    DefaultPlugins, FixedUpdate, MessageReader, MessageWriter, PostUpdate, Query, Res, ResMut,
    Resource, Startup, Update,
};
use bevy::render::RenderPlugin;
use bevy::time::Time;
use cu29::prelude::*;
use cu29::simulation::{CuTaskCallbackState, SimOverride};
use ironarm_core::messages::JointCommand;

use crate::arm_config::{ArmConfig, ArmConfigLoader};
use crate::motion::RhaiMotion;
use crate::world::ArmEntities;

#[copper_runtime(config = "../ironarm_std/copperconfig.ron", sim_mode = true)]
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
    cmd_tick: u64,
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
    let render_plugin = RenderPlugin {
        render_creation: bevy::render::settings::WgpuSettings {
            backends: Some(bevy::render::settings::Backends::VULKAN),
            instance_flags:
                bevy::render::settings::InstanceFlags::ALLOW_UNDERLYING_NONCOMPLIANT_ADAPTER,
            ..Default::default()
        }
        .into(),
        ..Default::default()
    };
    app.add_plugins(DefaultPlugins.set(render_plugin).set(AssetPlugin {
        watch_for_changes_override: Some(cfg!(debug_assertions)),
        ..Default::default()
    }));
    app.add_plugins(PhysicsPlugins::default());
    app.init_asset::<ArmConfig>();
    app.register_asset_loader(ArmConfigLoader);
    app.insert_resource(Gravity::default());
    app.insert_resource(Time::<Physics>::default());
    app.insert_resource(world::CameraControl::default());
    app.add_systems(Startup, world::setup_world);
    app.add_systems(Update, world::spawn_arm);
    app.add_systems(Update, world::camera_control);
    app.add_systems(Update, reload_motion);

    match RhaiMotion::load() {
        Ok(m) => {
            app.insert_resource(m);
        }
        Err(e) => bevy::log::error!("[main] Failed to load motion script: {}", e),
    }
    app.insert_resource(CopperApp {
        app: copper,
        clock,
        clock_mock,
        last_tick: None,
        cmd_tick: 0,
    });
    app.add_systems(FixedUpdate, run_tick);
    app.add_systems(PostUpdate, stop_copper_on_exit);
    app.run();
}

fn run_tick(
    time: Res<Time<Physics>>,
    mut copper: ResMut<CopperApp>,
    mut exit_writer: MessageWriter<AppExit>,
    arm_entities: Option<Res<ArmEntities>>,
    mut revolute_joints: Query<&mut RevoluteJoint>,
    motion: Option<Res<RhaiMotion>>,
) {
    let current_time = time.elapsed().as_nanos() as u64;
    if copper.last_tick == Some(current_time) {
        return;
    }
    copper.last_tick = Some(current_time);
    copper.clock_mock.set_value(current_time);
    copper.cmd_tick += 1;

    let dt = time.delta_secs();
    let angles = if let Some(ref m) = motion {
        m.compute_angles(copper.cmd_tick, dt).unwrap_or([0.0; 2])
    } else {
        [0.0; 2]
    };

    let clock = copper.clock.clone();
    let mut cb = move |step: crate::default::SimStep| -> SimOverride {
        match step {
            crate::default::SimStep::CmdSrc0(CuTaskCallbackState::Process(_input, output)) => {
                output.set_payload(JointCommand {
                    target_angle: angles[0],
                    target_velocity: 0.0,
                    stiffness: 1.0,
                });
                set_msg_timing(&clock, output);
                SimOverride::ExecutedBySim
            }
            crate::default::SimStep::CmdSrc1(CuTaskCallbackState::Process(_input, output)) => {
                output.set_payload(JointCommand {
                    target_angle: angles[1],
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
        return;
    }

    if let Some(entities) = arm_entities.as_ref() {
        if let Ok(mut j0) = revolute_joints.get_mut(entities.joint0) {
            j0.motor.target_position = angles[0];
        }
        if let Ok(mut j1) = revolute_joints.get_mut(entities.joint1) {
            j1.motor.target_position = angles[1];
        }
    }
}

fn reload_motion(mut motion: Option<ResMut<RhaiMotion>>) {
    if let Some(ref mut m) = motion {
        m.try_reload();
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
