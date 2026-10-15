// SPDX-License-Identifier: GPL-2.0-or-later
//! Tests par propriétés (proptest) pour SO(3).

mod common;

use common::{mat_diff, vec_diff};
use proptest::prelude::*;
use std::f64::consts::PI;
use tiny_rodriguez_std::linalg::{mat_mul, norm, scale, MAT3_IDENTITY};
use tiny_rodriguez_std::{Rotation3, Vec3};

/// Axe unitaire aléatoire.
fn unit_axis() -> impl Strategy<Value = Vec3> {
    (-1.0..=1.0f64, -1.0..=1.0f64, -1.0..=1.0f64).prop_filter_map("axe nul", |(x, y, z)| {
        let n = (x * x + y * y + z * z).sqrt();
        (n > 0.1).then(|| [x / n, y / n, z / n])
    })
}

/// Angle avec sur-représentation des zones délicates (0 et π).
fn angle() -> impl Strategy<Value = f64> {
    prop_oneof![
        3 => 0.0..PI,
        1 => Just(0.0),
        1 => 0.0..1e-3f64,
        1 => 1e-12..1e-8f64,
        2 => (PI - 1e-2)..=PI,
        1 => Just(PI),
    ]
}

/// Rotation de n'importe quel angle ∈ [0, π].
fn rotation() -> impl Strategy<Value = Rotation3> {
    (unit_axis(), angle()).prop_map(|(a, t)| Rotation3::exp(&scale(&a, t)))
}

/// Vecteur borné quelconque.
fn vec3(max: f64) -> impl Strategy<Value = Vec3> {
    (-max..=max, -max..=max, -max..=max).prop_map(|(x, y, z)| [x, y, z])
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2000))]

    #[test]
    fn exp_is_always_a_valid_rotation(a in unit_axis(), t in 0.0..50.0f64) {
        let r = Rotation3::exp(&scale(&a, t));
        prop_assert!(r.is_valid(1e-13), "θ = {t}, err = {}", r.orthogonality_error());
    }

    #[test]
    fn exp_log_exp_roundtrip(r in rotation()) {
        let r2 = Rotation3::exp(&r.log());
        prop_assert!(mat_diff(&r.matrix, &r2.matrix) < 1e-9,
            "θ = {}, diff = {}", r.angle(), mat_diff(&r.matrix, &r2.matrix));
    }

    #[test]
    fn log_exp_roundtrip_below_pi(a in unit_axis(), t in 0.0..(PI - 1e-2)) {
        let w = scale(&a, t);
        let w2 = Rotation3::exp(&w).log();
        prop_assert!(vec_diff(&w, &w2) < 1e-10, "θ = {t}");
    }

    #[test]
    fn log_norm_equals_angle_and_is_bounded(r in rotation()) {
        let n = norm(&r.log());
        prop_assert!(n <= PI + 1e-9);
        prop_assert!((n - r.angle()).abs() < 1e-9);
        prop_assert!((0.0..=PI).contains(&r.angle()));
    }

    #[test]
    fn inverse_and_composition(a in rotation(), b in rotation(), c in rotation()) {
        prop_assert!(mat_diff(&(a * a.inverse()).matrix, &MAT3_IDENTITY) < 1e-13);
        prop_assert!(mat_diff(&((a * b) * c).matrix, &(a * (b * c)).matrix) < 1e-13);
        prop_assert!((a * b).is_valid(1e-13));
    }

    #[test]
    fn rotation_preserves_norm_and_is_linear_with_matrix(r in rotation(), v in vec3(100.0)) {
        let rv = r * v;
        prop_assert!((norm(&rv) - norm(&v)).abs() < 1e-11);
        prop_assert!(vec_diff(&r.inverse().rotate(&rv), &v) < 1e-11);
    }

    #[test]
    fn quaternion_roundtrip_and_canonical_form(r in rotation()) {
        let q = r.to_quaternion();
        let nq = (q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]).sqrt();
        prop_assert!((nq - 1.0).abs() < 1e-13);
        prop_assert!(q[3] >= 0.0);
        let r2 = Rotation3::from_quaternion(&q);
        prop_assert!(mat_diff(&r.matrix, &r2.matrix) < 1e-12);
    }

    #[test]
    fn quaternion_scale_and_sign_do_not_matter(r in rotation(), s in 0.1..10.0f64) {
        let q = r.to_quaternion();
        let qa = Rotation3::from_quaternion(&q.map(|x| x * s));
        let qb = Rotation3::from_quaternion(&q.map(|x| -x));
        prop_assert!(mat_diff(&qa.matrix, &r.matrix) < 1e-12);
        prop_assert!(mat_diff(&qb.matrix, &r.matrix) < 1e-12);
    }

    #[test]
    fn geodesic_distance_is_a_metric(a in rotation(), b in rotation(), c in rotation()) {
        let (dab, dba) = (a.geodesic_distance(&b), b.geodesic_distance(&a));
        prop_assert!((dab - dba).abs() < 1e-9);
        prop_assert!(a.geodesic_distance(&a) < 1e-7);
        prop_assert!(a.geodesic_distance(&c) <= dab + b.geodesic_distance(&c) + 1e-9);
    }

    #[test]
    fn interpolation_endpoints_and_constant_speed(a in rotation(), b in rotation(), t in 0.0..=1.0f64) {
        prop_assume!(a.geodesic_distance(&b) < PI - 1e-2); // la géodésique est unique
        prop_assert!(mat_diff(&a.interpolate(&b, 0.0).matrix, &a.matrix) < 1e-12);
        prop_assert!(mat_diff(&a.interpolate(&b, 1.0).matrix, &b.matrix) < 1e-10);
        let d = a.geodesic_distance(&b);
        let m = a.interpolate(&b, t);
        prop_assert!((a.geodesic_distance(&m) - t * d).abs() < 1e-8);
    }

    #[test]
    fn world_and_body_integration_are_conjugate(r in rotation(), w in vec3(5.0), dt in 0.0..0.1f64) {
        // R · exp(ω dt) == exp((R ω) dt) · R
        let body = r.integrate(&w, dt);
        let world = r.integrate_world(&r.rotate(&w), dt);
        prop_assert!(mat_diff(&body.matrix, &world.matrix) < 1e-12);
    }

    #[test]
    fn integration_composes_additively_for_constant_omega(r in rotation(), w in vec3(5.0), dt in 0.0..0.05f64) {
        let two_steps = r.integrate(&w, dt).integrate(&w, dt);
        let one_step = r.integrate(&w, 2.0 * dt);
        prop_assert!(mat_diff(&two_steps.matrix, &one_step.matrix) < 1e-12);
    }

    #[test]
    fn from_axis_angle_ignores_axis_scale(a in unit_axis(), t in angle(), s in 0.01..100.0f64) {
        let r1 = Rotation3::from_axis_angle(&a, t);
        let r2 = Rotation3::from_axis_angle(&scale(&a, s), t);
        prop_assert!(mat_diff(&r1.matrix, &r2.matrix) < 1e-12);
        prop_assert!((r1.angle() - t).abs() < 1e-8);
    }

    #[test]
    fn jacobian_inverse_is_inverse(a in unit_axis(), t in 0.0..(2.0 * PI - 0.1)) {
        let w = scale(&a, t);
        let p = mat_mul(&Rotation3::left_jacobian(&w), &Rotation3::left_jacobian_inv(&w));
        prop_assert!(mat_diff(&p, &MAT3_IDENTITY) < 1e-10, "θ = {t}");
    }

    #[test]
    fn renormalization_repairs_perturbed_rotations(r in rotation(), eps in 1e-9..1e-4f64, i in 0usize..3, j in 0usize..3) {
        let mut m = r;
        m.matrix[i][j] += eps;
        let fixed = m.renormalized();
        prop_assert!(fixed.orthogonality_error() < 1e-12);
        prop_assert!(fixed.is_valid(1e-12));
        // la correction reste proche de la matrice d'origine
        prop_assert!(mat_diff(&fixed.matrix, &r.matrix) < 10.0 * eps);
    }
}

// ---------------------------------------------------------------------------
// Cas déterministes autour de θ = π (zone la plus délicate pour `log`)
// ---------------------------------------------------------------------------

#[test]
fn log_at_exactly_pi_for_many_axes() {
    let axes: [Vec3; 9] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
        [1.0, 0.0, -1.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, -2.0, 3.0],
        [-0.3, 0.2, -0.9],
    ];
    for axis in axes {
        for delta in [0.0, 1e-12, 1e-9, 1e-6, 1e-4, 5e-4, 2e-3] {
            let r = Rotation3::from_axis_angle(&axis, PI - delta);
            let l = r.log();
            assert!(
                (norm(&l) - (PI - delta)).abs() < 1e-8,
                "axe {axis:?}, δ = {delta}: ‖log‖ = {}",
                norm(&l)
            );
            let back = Rotation3::exp(&l);
            assert!(
                mat_diff(&r.matrix, &back.matrix) < 1e-9,
                "axe {axis:?}, δ = {delta}: diff = {}",
                mat_diff(&r.matrix, &back.matrix)
            );
        }
    }
}

#[test]
fn non_finite_inputs_never_panic() {
    let bad = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY];
    for b in bad {
        let r = Rotation3::exp(&[b, 0.0, 1.0]);
        let _ = (r.log(), r.angle(), r.to_quaternion(), r.renormalized());
        let _ = Rotation3::from_quaternion(&[b, 0.0, 0.0, 1.0]);
        let _ = Rotation3::from_axis_angle(&[b, 1.0, 0.0], 1.0);
        let _ = Rotation3::identity().integrate(&[b, 0.0, 0.0], 0.1);
        assert!(!r.is_valid(1e-9));
    }
}

#[test]
fn degenerate_inputs_give_identity() {
    assert_eq!(Rotation3::from_axis_angle(&[0.0; 3], 1.0), Rotation3::identity());
    assert_eq!(Rotation3::from_quaternion(&[0.0; 4]), Rotation3::identity());
    assert_eq!(Rotation3::exp(&[0.0; 3]), Rotation3::identity());
}