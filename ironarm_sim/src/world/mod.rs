use avian3d::prelude::*;
use bevy::color::palettes::css;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::prelude::*;

#[derive(Resource)]
pub struct CameraControl {
    pub rotate_sensitivity: f32,
    pub zoom_sensitivity: f32,
    pub move_sensitivity: f32,
}

impl Default for CameraControl {
    fn default() -> Self {
        Self {
            rotate_sensitivity: 0.005,
            zoom_sensitivity: 10.0,
            move_sensitivity: 0.1,
        }
    }
}

/// 生成相机、光源、地面、测试方块
pub fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
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

    // Test cube
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 0.5))),
        MeshMaterial3d(materials.add(Color::Srgba(css::RED))),
        Transform::from_xyz(0.0, 2.0, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 0.5, 0.5),
    ));
}

/// 相机控制：中键拖拽旋转、滚轮缩放、WASD/QE 平移
pub fn camera_control(
    ctrl: Res<CameraControl>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut scroll_evr: MessageReader<MouseWheel>,
    mut mouse_motion: MessageReader<MouseMotion>,
    time: Res<Time>,
    mut query: Query<&mut Transform, With<Camera3d>>,
) {
    let mut cam = query.single_mut().expect("camera");
    let focal = Vec3::ZERO;
    let dir = cam.translation - focal;
    let radius = dir.length();

    // zoom
    let forward = cam.forward();
    for ev in scroll_evr.read() {
        let lines = match ev.unit {
            MouseScrollUnit::Line => ev.y,
            MouseScrollUnit::Pixel => ev.y / 16.0,
        };
        cam.translation += forward * lines * ctrl.zoom_sensitivity * time.delta_secs();
    }

    // orbit (middle button)
    if mouse_buttons.pressed(MouseButton::Middle) {
        for ev in mouse_motion.read() {
            let yaw = Quat::from_rotation_y(-ev.delta.x * ctrl.rotate_sensitivity);
            let pitch = Quat::from_rotation_x(-ev.delta.y * ctrl.rotate_sensitivity);
            cam.translation = focal + (yaw * pitch * dir).normalize() * radius;
            cam.look_at(focal, Vec3::Y);
        }
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
