use crate::arm_config::{ArmConfig, JointAxis};
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

/// 生成相机、光源、地面、测试方块
fn s3(v: (f32, f32, f32)) -> Vec3 {
    Vec3::new(v.0, v.1, v.2)
}

/// 生成相机、光源、地面、机械臂
pub fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cfg = ArmConfig::load();

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 3.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(2.0, 5.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(10.0, 0.2, 10.0))),
        MeshMaterial3d(materials.add(Color::Srgba(css::GRAY))),
        Transform::from_xyz(0.0, -0.6, 0.0),
        RigidBody::Static,
        Collider::cuboid(10.0, 0.2, 10.0),
    ));

    // Mechanical arm
    let base = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                cfg.base.size.0,
                cfg.base.size.1,
                cfg.base.size.2,
            ))),
            MeshMaterial3d(materials.add(Color::Srgba(css::DARK_GRAY))),
            Transform::from_translation(s3(cfg.base.center)),
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
            Transform::from_translation(s3(cfg.link0.center)),
            RigidBody::Dynamic,
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
            Transform::from_translation(s3(cfg.link1.center)),
            RigidBody::Dynamic,
            Collider::cuboid(cfg.link1.size.0, cfg.link1.size.1, cfg.link1.size.2),
        ))
        .id();

    // Joint 0: base → link0
    commands.spawn((RevoluteJoint::new(base, link0)
        .with_local_anchor1(s3(cfg.joint0.anchor1))
        .with_local_anchor2(s3(cfg.joint0.anchor2)),));

    // Joint 1: link0 → link1
    let mut j1 = RevoluteJoint::new(link0, link1)
        .with_local_anchor1(s3(cfg.joint1.anchor1))
        .with_local_anchor2(s3(cfg.joint1.anchor2));
    if matches!(cfg.joint1.axis, JointAxis::Z) {
        j1 = j1.with_hinge_axis(Vec3::Z);
    }
    commands.spawn((j1,));
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
