// SPDX-License-Identifier: GPL-2.0-or-later
use bevy::prelude::*;
use bevy_drone::to_bevy_quat;
use tiny_rodriguez_std::{Pose3, Rotation3};

#[derive(Component)]
struct Drone {
    pose: Pose3,
    frames: u64,
}

#[derive(Component)]
struct Propeller {
    orientation: Rotation3,
    omega: [f64; 3],
}

#[derive(Component)]
struct Hud;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, (fly, spin_propellers, follow_camera, hud).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let body_mat = materials.add(Color::srgb_u8(40, 44, 52));
    let arm_mat = materials.add(Color::srgb_u8(70, 75, 85));
    let front_mat = materials.add(Color::srgb_u8(230, 50, 50));
    let rear_mat = materials.add(Color::srgb_u8(235, 235, 235));
    let ground_mat = materials.add(Color::srgb_u8(60, 90, 60));

    let body = meshes.add(Cuboid::new(0.3, 0.12, 0.42));
    let arm = meshes.add(Cuboid::new(1.6, 0.04, 0.06));
    let motor = meshes.add(Cylinder::new(0.07, 0.1));
    let blade = meshes.add(Cuboid::new(0.9, 0.01, 0.07));
    let nose = meshes.add(Sphere::new(0.05));

    // Sol
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(60.0, 60.0))),
        MeshMaterial3d(ground_mat),
    ));

    // Drone (le repère racine reçoit la pose SE(3))
    commands
        .spawn((
            Transform::default(),
            Visibility::default(),
            Drone {
                pose: Pose3::new(Rotation3::identity(), [0.0, 2.0, 0.0]),
                frames: 0,
            },
        ))
        .with_children(|d| {
            // Corps + "caméra" à l'avant
            d.spawn((Mesh3d(body), MeshMaterial3d(body_mat)));
            d.spawn((
                Mesh3d(nose),
                MeshMaterial3d(front_mat.clone()),
                Transform::from_xyz(0.0, -0.02, -0.23),
            ));

            // Deux bras en X
            for angle in [45.0_f32, -45.0] {
                d.spawn((
                    Mesh3d(arm.clone()),
                    MeshMaterial3d(arm_mat.clone()),
                    Transform::from_rotation(Quat::from_rotation_y(angle.to_radians())),
                ));
            }

            // Quatre moteurs + hélices (avant = -Z = rouge)
            let r = 0.566_f32;
            for (sx, sz) in [(1.0_f32, -1.0_f32), (-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                let mat = if sz < 0.0 { front_mat.clone() } else { rear_mat.clone() };
                d.spawn((
                    Mesh3d(motor.clone()),
                    MeshMaterial3d(body_mat_clone(&arm_mat)),
                    Transform::from_xyz(sx * r, 0.05, sz * r),
                ));
                // Hélices contrarotatives (sens alterné en damier)
                let dir = (sx * sz) as f64;
                d.spawn((
                    Transform::from_xyz(sx * r, 0.13, sz * r),
                    Visibility::default(),
                    Propeller {
                        orientation: Rotation3::identity(),
                        omega: [0.0, dir * 60.0, 0.0],
                    },
                ))
                .with_children(|p| {
                    p.spawn((Mesh3d(blade.clone()), MeshMaterial3d(mat.clone())));
                    p.spawn((
                        Mesh3d(blade.clone()),
                        MeshMaterial3d(mat.clone()),
                        Transform::from_rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2)),
                    ));
                });
            }
        });

    // Lumières
    commands.spawn((
        DirectionalLight { illuminance: 12_000.0, ..default() },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight { illuminance: 4_000.0, ..default() },
        Transform::from_xyz(-5.0, 6.0, -5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Caméra
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 4.0, 8.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
    ));

    // HUD
    commands.spawn((
        Text::new(""),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(8.0),
            left: Val::Px(8.0),
            ..default()
        },
        Hud,
    ));
}

fn body_mat_clone(h: &Handle<StandardMaterial>) -> Handle<StandardMaterial> {
    h.clone()
}

/// Vol : torseur constant dans le repère corps (avance + lacet) + oscillation verticale.
/// T <- T · exp(ξ dt), exact pour ξ constant.
fn fly(time: Res<Time>, mut query: Query<(&mut Transform, &mut Drone)>) {
    let dt = time.delta_secs_f64();
    let t = time.elapsed_secs_f64();
    for (mut transform, mut drone) in &mut query {
        // ξ = [vx, vy, vz, ωx, ωy, ωz]  (avance = -Z)
        let twist = [0.0, 0.6 * (0.7 * t).cos(), -2.0, 0.0, 0.7, 0.0];
        drone.pose = drone.pose.integrate(&twist, dt);

        drone.frames += 1;
        if drone.frames % 600 == 0 {
            drone.pose.rotation = drone.pose.rotation.renormalized();
        }

        let p = drone.pose.translation;
        transform.translation = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
        transform.rotation = to_bevy_quat(&drone.pose.rotation);
    }
}

fn spin_propellers(time: Res<Time>, mut query: Query<(&mut Transform, &mut Propeller)>) {
    let dt = time.delta_secs_f64();
    for (mut transform, mut prop) in &mut query {
        prop.orientation = prop.orientation.integrate(&prop.omega, dt);
        transform.rotation = to_bevy_quat(&prop.orientation);
    }
}

fn follow_camera(
    drones: Query<&Transform, (With<Drone>, Without<Camera3d>)>,
    mut cams: Query<&mut Transform, With<Camera3d>>,
) {
    let (Ok(d), Ok(mut cam)) = (drones.single(), cams.single_mut()) else {
        return;
    };
    // Caméra derrière-dessus, lissée
    let behind = d.translation + d.rotation * Vec3::new(0.0, 1.8, 5.0);
    cam.translation = cam.translation.lerp(behind, 0.05);
    cam.look_at(d.translation, Vec3::Y);
}

fn hud(drones: Query<&Drone>, mut texts: Query<&mut Text, With<Hud>>) {
    let (Ok(d), Ok(mut t)) = (drones.single(), texts.single_mut()) else {
        return;
    };
    let p = d.pose.translation;
    t.0 = format!(
        "|R^T R - I| = {:.2e}\npos = ({:.1}, {:.1}, {:.1})\nlacet = {:.2} rad",
        d.pose.rotation.orthogonality_error(),
        p[0], p[1], p[2],
        d.pose.rotation.angle()
    );
}