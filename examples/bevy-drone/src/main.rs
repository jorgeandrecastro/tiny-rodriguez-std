// SPDX-License-Identifier: GPL-2.0-or-later
use bevy::prelude::*;
use bevy_drone::to_bevy_quat;
use tiny_rodriguez_std::{Pose3, Rotation3};

#[derive(Resource, Default, PartialEq, Debug)]
enum FlightMode {
    #[default]
    Stationary,
    Looping { timer: f64 },
    Patrol,
}

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
        .init_resource::<FlightMode>()
        .add_systems(Startup, setup)
        .add_systems(Update, (keyboard_input_system, fly, spin_propellers, follow_camera, hud).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let body_mat = materials.add(Color::srgb_u8(35, 39, 46));
    let arm_mat = materials.add(Color::srgb_u8(70, 75, 85));
    let front_mat = materials.add(Color::srgb_u8(230, 50, 50));
    let rear_mat = materials.add(Color::srgb_u8(235, 235, 235));
    let ground_mat = materials.add(Color::srgb_u8(45, 70, 45));

    let body = meshes.add(Cuboid::new(0.3, 0.12, 0.42));
    let arm = meshes.add(Cuboid::new(1.6, 0.04, 0.06));
    let motor = meshes.add(Cylinder::new(0.07, 0.1));
    let blade = meshes.add(Cuboid::new(0.9, 0.01, 0.07));
    let nose = meshes.add(Sphere::new(0.05));

    // Sol
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(80.0, 80.0))),
        MeshMaterial3d(ground_mat),
    ));

    // Drone racine
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
            d.spawn((Mesh3d(body), MeshMaterial3d(body_mat)));
            d.spawn((
                Mesh3d(nose),
                MeshMaterial3d(front_mat.clone()),
                Transform::from_xyz(0.0, -0.02, -0.23),
            ));

            for angle in [45.0_f32, -45.0] {
                d.spawn((
                    Mesh3d(arm.clone()),
                    MeshMaterial3d(arm_mat.clone()),
                    Transform::from_rotation(Quat::from_rotation_y(angle.to_radians())),
                ));
            }

            let r = 0.566_f32;
            for (sx, sz) in [(1.0_f32, -1.0_f32), (-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                let mat = if sz < 0.0 { front_mat.clone() } else { rear_mat.clone() };
                d.spawn((
                    Mesh3d(motor.clone()),
                    MeshMaterial3d(body_mat_clone(&arm_mat)),
                    Transform::from_xyz(sx * r, 0.05, sz * r),
                ));
                let dir = (sx * sz) as f64;
                d.spawn((
                    Transform::from_xyz(sx * r, 0.13, sz * r),
                    Visibility::default(),
                    Propeller {
                        orientation: Rotation3::identity(),
                        omega: [0.0, dir * 80.0, 0.0],
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

    // Éclairage
    commands.spawn((
        DirectionalLight { illuminance: 15_000.0, ..default() },
        Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Caméra
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 7.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
    ));

    // HUD
    commands.spawn((
        Text::new(""),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        Hud,
    ));
}

fn body_mat_clone(h: &Handle<StandardMaterial>) -> Handle<StandardMaterial> {
    h.clone()
}

/// Gestion des entrées clavier pour changer de mode de vol
fn keyboard_input_system(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut flight_mode: ResMut<FlightMode>,
) {
    if keyboard_input.just_pressed(KeyCode::KeyL) {
        // Déclenche le looping (durée estimée à 3.0 secondes)
        *flight_mode = FlightMode::Looping { timer: 0.0 };
        println!("Commande clavier : Looping déclenché !");
    }
    if keyboard_input.just_pressed(KeyCode::KeyP) {
        // Active le mode patrouille
        *flight_mode = FlightMode::Patrol;
        println!("Commande clavier : Mode Patrouille activé.");
    }
    if keyboard_input.just_pressed(KeyCode::KeyS) {
        // Force le retour en stationnaire
        *flight_mode = FlightMode::Stationary;
        println!("Commande clavier : Mode Stationnaire.");
    }
}

/// Mouvement du drone en fonction du mode actif
fn fly(
    time: Res<Time>,
    mut flight_mode: ResMut<FlightMode>,
    mut query: Query<(&mut Transform, &mut Drone)>,
) {
    let dt = time.delta_secs_f64();
    let t = time.elapsed_secs_f64();

    for (mut transform, mut drone) in &mut query {
        let twist = match *flight_mode {
            FlightMode::Stationary => {
                // Légère oscillation naturelle de stationnaire pour faire vivre le drone
                let hover_vz = 0.2 * (3.0 * t).sin();
                [0.0, hover_vz, 0.0, 0.0, 0.0, 0.0]
            }
            FlightMode::Patrol => {
                // Avance en tournant doucement sur lui-même
                [-0.8, 0.0, -1.5, 0.0, 0.5, 0.0]
            }
            FlightMode::Looping { ref mut timer } => {
                *timer += dt;
                if *timer > 3.0 {
                    // Fin du looping, retour automatique au stationnaire
                    *flight_mode = FlightMode::Stationary;
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                } else {
                    // Exécution de la rotation du looping (tangage + avance)
                    [0.0, 0.5, -2.0, 0.0, 2.1, 0.0]
                }
            }
        };

        // Intégration exponentielle exacte via notre bibliothèque tiny-rodriguez-std
        drone.pose = drone.pose.integrate(&twist, dt);
        drone.frames += 1;

        if drone.frames % 500 == 0 {
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
    let target_cam_pos = d.translation + d.rotation * Vec3::new(0.0, 1.5, 4.5);
    cam.translation = cam.translation.lerp(target_cam_pos, 0.08);
    cam.look_at(d.translation + Vec3::Y * 0.5, Vec3::Y);
}

fn hud(
    flight_mode: Res<FlightMode>,
    drones: Query<&Drone>,
    mut texts: Query<&mut Text, With<Hud>>,
) {
    let (Ok(d), Ok(mut t)) = (drones.single(), texts.single_mut()) else {
        return;
    };
    let p = d.pose.translation;

    let mode_str = match *flight_mode {
        FlightMode::Stationary => "STATIONNAIRE (En attente...)",
        FlightMode::Patrol => "PATROUILLE (Appuyez sur S pour stop)",
        FlightMode::Looping { .. } => "ACROBATIE EN COURS (Looping 360)",
    };

    t.0 = format!(
        "=== TINY-RODRIGUEZ SDK ===\nMode Actif : {}\n[L] Looping | [P] Patrouille | [S] Stationnaire\nErreur d'orthogonalite : {:.2e}\nPosition : ({:.1}, {:.1}, {:.1})",
        mode_str,
        d.pose.rotation.orthogonality_error(),
        p[0], p[1], p[2]
    );
}