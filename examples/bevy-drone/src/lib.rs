// SPDX-License-Identifier: GPL-2.0-or-later
//! Passerelle `tiny-rodriguez-std` ⇄ Bevy, validée contre `glam` (moteur de Bevy).

use bevy::math::{Mat3, Quat};
use tiny_rodriguez_std::Rotation3;

/// Rotation → `Quat` Bevy.
pub fn to_bevy_quat(r: &Rotation3) -> Quat {
    let [x, y, z, w] = r.to_quaternion();
    Quat::from_xyzw(x as f32, y as f32, z as f32, w as f32).normalize()
}

/// `Quat` Bevy → rotation.
pub fn from_bevy_quat(q: Quat) -> Rotation3 {
    Rotation3::from_quaternion(&[q.x as f64, q.y as f64, q.z as f64, q.w as f64])
}

/// Rotation → `Mat3` Bevy (glam est en colonnes).
pub fn to_bevy_mat3(r: &Rotation3) -> Mat3 {
    let m = &r.matrix;
    Mat3::from_cols_array(&[
        m[0][0] as f32, m[1][0] as f32, m[2][0] as f32,
        m[0][1] as f32, m[1][1] as f32, m[2][1] as f32,
        m[0][2] as f32, m[1][2] as f32, m[2][2] as f32,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::Vec3;

    const V: [f64; 3] = [0.3, -1.0, 2.0];

    fn close(a: [f64; 3], b: Vec3, tol: f32) -> bool {
        (a[0] as f32 - b.x).abs() < tol && (a[1] as f32 - b.y).abs() < tol && (a[2] as f32 - b.z).abs() < tol
    }

    #[test]
    fn exp_matches_glam_axis_angle() {
        let axis = Vec3::new(1.0, 2.0, 3.0).normalize();
        let angle = 1.1_f32;
        let ours = Rotation3::exp(&[
            (axis.x * angle) as f64,
            (axis.y * angle) as f64,
            (axis.z * angle) as f64,
        ]);
        let theirs = Quat::from_axis_angle(axis, angle) * Vec3::new(V[0] as f32, V[1] as f32, V[2] as f32);
        assert!(close(ours.rotate(&V), theirs, 1e-5));
    }

    #[test]
    fn matrix_and_quaternion_agree_with_glam() {
        let r = Rotation3::exp(&[0.7, -0.3, 1.9]);
        let v = Vec3::new(V[0] as f32, V[1] as f32, V[2] as f32);
        let by_quat = to_bevy_quat(&r) * v;
        let by_mat = to_bevy_mat3(&r) * v;
        assert!(close(r.rotate(&V), by_quat, 1e-5));
        assert!(close(r.rotate(&V), by_mat, 1e-5));
    }

    #[test]
    fn quaternion_roundtrip_through_bevy() {
        let r = Rotation3::exp(&[2.0, 1.0, -0.5]);
        let back = from_bevy_quat(to_bevy_quat(&r));
        assert!(r.geodesic_distance(&back) < 1e-6);
    }

    #[test]
    fn integration_matches_glam_chain() {
        let omega = [0.5_f64, 1.2, 0.3];
        let dt = 1.0 / 60.0;
        let mut ours = Rotation3::identity();
        let mut q = Quat::IDENTITY;
        let w = Vec3::new(omega[0] as f32, omega[1] as f32, omega[2] as f32);
        for _ in 0..1000 {
            ours = ours.integrate(&omega, dt);
            q = (q * Quat::from_scaled_axis(w * dt as f32)).normalize();
        }
        let theirs = q * Vec3::new(V[0] as f32, V[1] as f32, V[2] as f32);
        assert!(close(ours.rotate(&V), theirs, 2e-3));
    }
}