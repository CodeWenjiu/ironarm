use crate::arm_config::{ArmConfig, ArmConfigHandle, JointAxis};
use avian3d::prelude::*;
use bevy::color::palettes::css;
use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::prelude::*;

#[derive(Resource)]
pub struct CameraControl {
    pub zoom_sensitivity: f32,
    pub move_sensitivity: f32,
}

impl Default for CameraControl {
    fn default() -> Self {
        Self {
            zoom_sensitivity: 10.0,
            move_sensitivity: 0.1,
        }
    }
}

/// 生成相机、光源，加载机械臂配置
pub fn setup_world(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 3.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(2.0, 5.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let handle: Handle<ArmConfig> = asset_server.load("arm_config.ron");
    commands.insert_resource(ArmConfigHandle(handle));
}

#[derive(Resource, Clone)]
pub(crate) struct ArmEntities {
    pub(crate) base: Entity,
    pub(crate) link0: Entity,
    pub(crate) link1: Entity,
    pub(crate) joint0: Entity,
    pub(crate) joint1: Entity,
}

/// 机械臂热重载（debug）或一次性加载（release）。
pub fn spawn_arm(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    configs: Res<Assets<ArmConfig>>,
    handle: Res<ArmConfigHandle>,
    mut arm_entities: Local<Option<ArmEntities>>,
    mut last_cfg: Local<Option<ArmConfig>>,
) {
    let Some(cfg) = configs.get(&handle.0) else {
        return;
    };

    // release 模式下只加载一次，之后不重建
    #[cfg(not(debug_assertions))]
    if arm_entities.is_some() {
        return;
    }

    // debug: 首次加载或配置变更时重建（利用 PartialEq 比对）
    #[cfg(debug_assertions)]
    if let Some(ref last) = *last_cfg {
        if last == cfg {
            return;
        }
    }

    // 配置有变更或首次加载，保存快照并重建
    *last_cfg = Some(cfg.clone());

    if let Some(entities) = arm_entities.take() {
        commands.entity(entities.joint0).despawn();
        commands.entity(entities.joint1).despawn();
        commands.entity(entities.link0).despawn();
        commands.entity(entities.link1).despawn();
        commands.entity(entities.base).despawn();
        commands.remove_resource::<ArmEntities>();
    }

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(10.0, 0.2, 10.0))),
        MeshMaterial3d(materials.add(Color::Srgba(css::GRAY))),
        Transform::from_xyz(0.0, -0.6, 0.0),
        RigidBody::Static,
        Collider::cuboid(10.0, 0.2, 10.0),
    ));

    let base = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                cfg.base.size.0,
                cfg.base.size.1,
                cfg.base.size.2,
            ))),
            MeshMaterial3d(materials.add(Color::Srgba(css::DARK_GRAY))),
            Transform::from_xyz(cfg.base.center.0, cfg.base.center.1, cfg.base.center.2),
            RigidBody::Static,
        ))
        .id();

    let link0 = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                cfg.link0.size.0,
                cfg.link0.size.1,
                cfg.link0.size.2,
            ))),
            MeshMaterial3d(materials.add(Color::Srgba(css::STEEL_BLUE))),
            Transform::from_xyz(cfg.link0.center.0, cfg.link0.center.1, cfg.link0.center.2),
            RigidBody::Dynamic,
            LinearDamping(cfg.linear_damping),
            AngularDamping(cfg.angular_damping),
            Collider::cuboid(cfg.link0.size.0, cfg.link0.size.1, cfg.link0.size.2),
        ))
        .id();

    let link1 = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                cfg.link1.size.0,
                cfg.link1.size.1,
                cfg.link1.size.2,
            ))),
            MeshMaterial3d(materials.add(Color::Srgba(css::ORANGE))),
            Transform::from_xyz(cfg.link1.center.0, cfg.link1.center.1, cfg.link1.center.2),
            RigidBody::Dynamic,
            LinearDamping(cfg.linear_damping),
            AngularDamping(cfg.angular_damping),
            Collider::cuboid(cfg.link1.size.0, cfg.link1.size.1, cfg.link1.size.2),
        ))
        .id();

    let mut j0_revolute = RevoluteJoint::new(base, link0)
        .with_local_anchor1(Vec3::new(
            cfg.joint0.anchor1.0,
            cfg.joint0.anchor1.1,
            cfg.joint0.anchor1.2,
        ))
        .with_local_anchor2(Vec3::new(
            cfg.joint0.anchor2.0,
            cfg.joint0.anchor2.1,
            cfg.joint0.anchor2.2,
        ))
        .with_angle_limits(cfg.joint0.angle_limit_min, cfg.joint0.angle_limit_max)
        .with_motor(
            AngularMotor::new(MotorModel::SpringDamper {
                frequency: cfg.joint0.motor_frequency,
                damping_ratio: cfg.joint0.motor_damping_ratio,
            })
            .with_target_position(0.0),
        );
    if matches!(cfg.joint0.axis, JointAxis::Y) {
        j0_revolute = j0_revolute.with_hinge_axis(Vec3::Y);
    }
    let j0 = commands.spawn((j0_revolute,)).id();

    let mut j1 = RevoluteJoint::new(link0, link1)
        .with_local_anchor1(Vec3::new(
            cfg.joint1.anchor1.0,
            cfg.joint1.anchor1.1,
            cfg.joint1.anchor1.2,
        ))
        .with_local_anchor2(Vec3::new(
            cfg.joint1.anchor2.0,
            cfg.joint1.anchor2.1,
            cfg.joint1.anchor2.2,
        ))
        .with_angle_limits(cfg.joint1.angle_limit_min, cfg.joint1.angle_limit_max)
        .with_motor(
            AngularMotor::new(MotorModel::SpringDamper {
                frequency: cfg.joint1.motor_frequency,
                damping_ratio: cfg.joint1.motor_damping_ratio,
            })
            .with_target_position(0.0),
        );
    if matches!(cfg.joint1.axis, JointAxis::Z) {
        j1 = j1.with_hinge_axis(Vec3::Z);
    }
    let j1_entity = commands.spawn((j1,)).id();

    let entities = ArmEntities {
        base,
        link0,
        link1,
        joint0: j0,
        joint1: j1_entity,
    };
    commands.insert_resource(entities.clone());
    *arm_entities = Some(entities);
}

/// 相机控制：滚轮缩放、WASD/QE 平移
pub fn camera_control(
    ctrl: Res<CameraControl>,
    keys: Res<ButtonInput<KeyCode>>,
    mut scroll_evr: MessageReader<MouseWheel>,
    time: Res<Time>,
    mut query: Query<&mut Transform, With<Camera3d>>,
) {
    let mut cam = query.single_mut().expect("camera");
    let focal = Vec3::ZERO;

    // zoom
    let forward = cam.forward();
    for ev in scroll_evr.read() {
        let lines = match ev.unit {
            MouseScrollUnit::Line => ev.y,
            MouseScrollUnit::Pixel => ev.y / 16.0,
        };
        cam.translation += forward * lines * ctrl.zoom_sensitivity * time.delta_secs();
    }

    // keyboard movement
    let speed = ctrl.move_sensitivity;
    let fwd = if keys.pressed(KeyCode::KeyW) {
        cam.forward() * speed
    } else {
        Vec3::ZERO
    };
    let back = if keys.pressed(KeyCode::KeyS) {
        cam.back() * speed
    } else {
        Vec3::ZERO
    };
    let left = if keys.pressed(KeyCode::KeyA) {
        cam.left() * speed
    } else {
        Vec3::ZERO
    };
    let rgt = if keys.pressed(KeyCode::KeyD) {
        cam.right() * speed
    } else {
        Vec3::ZERO
    };
    let up = if keys.pressed(KeyCode::KeyQ) {
        Vec3::Y * speed
    } else {
        Vec3::ZERO
    };
    let down = if keys.pressed(KeyCode::KeyE) {
        Vec3::NEG_Y * speed
    } else {
        Vec3::ZERO
    };
    cam.translation += fwd + back + left + rgt + up + down;
    cam.look_at(focal, Vec3::Y);
}
