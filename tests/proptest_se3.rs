// SPDX-License-Identifier: GPL-2.0-or-later
//! Tests par propriétés (proptest) pour SE(3) et le traitement par lots.

mod common;

use common::{mat_diff, vec_diff};
use proptest::prelude::*;
use std::f64::consts::PI;
use tiny_rodriguez_std::linalg::{scale, MAT3_IDENTITY};
use tiny_rodriguez_std::{Pose3, Rotation3, Vec3};

fn unit_axis() -> impl Strategy<Value = Vec3> {
    (-1.0..=1.0f64, -1.0..=1.0f64, -1.0..=1.0f64).prop_filter_map("axe nul", |(x, y, z)| {
        let n = (x * x + y * y + z * z).sqrt();
        (n > 0.1).then(|| [x / n, y / n, z / n])
    })
}

fn vec3(max: f64) -> impl Strategy<Value = Vec3> {
    (-max..=max, -max..=max, -max..=max).prop_map(|(x, y, z)| [x, y, z])
}

/// Torseur [v, ω] avec ‖ω‖ < π − marge (log unique).
fn twist() -> impl Strategy<Value = [f64; 6]> {
    (vec3(10.0), unit_axis(), prop_oneof![0.0..(PI - 0.05), 0.0..1e-6f64]).prop_map(|(v, a, t)| {
        let w = scale(&a, t);
        [v[0], v[1], v[2], w[0], w[1], w[2]]
    })
}

fn pose() -> impl Strategy<Value = Pose3> {
    (unit_axis(), 0.0..PI, vec3(100.0))
        .prop_map(|(a, t, p)| Pose3::new(Rotation3::exp(&scale(&a, t)), p))
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2000))]

    #[test]
    fn se3_exp_log_roundtrip(xi in twist()) {
        let xi2 = Pose3::exp(&xi).log();
        prop_assert!(vec_diff(&xi, &xi2) < 1e-8, "diff = {}", vec_diff(&xi, &xi2));
    }

    #[test]
    fn se3_inverse_is_inverse(t in pose()) {
        let id = t.compose(&t.inverse());
        prop_assert!(mat_diff(&id.rotation.matrix, &MAT3_IDENTITY) < 1e-12);
        prop_assert!(vec_diff(&id.translation, &[0.0; 3]) < 1e-10);
    }

    #[test]
    fn se3_composition_is_associative(a in pose(), b in pose(), c in pose()) {
        let l = a.compose(&b).compose(&c);
        let r = a.compose(&b.compose(&c));
        prop_assert!(mat_diff(&l.rotation.matrix, &r.rotation.matrix) < 1e-12);
        prop_assert!(vec_diff(&l.translation, &r.translation) < 1e-9);
    }

    #[test]
    fn transform_point_is_consistent_with_composition(a in pose(), b in pose(), p in vec3(50.0)) {
        let direct = a.compose(&b).transform_point(&p);
        let chained = a.transform_point(&b.transform_point(&p));
        prop_assert!(vec_diff(&direct, &chained) < 1e-9);
    }

    #[test]
    fn transform_point_then_inverse_is_identity(t in pose(), p in vec3(50.0)) {
        let back = t.inverse().transform_point(&t.transform_point(&p));
        prop_assert!(vec_diff(&back, &p) < 1e-9);
    }

    #[test]
    fn transform_point_is_an_isometry(t in pose(), p in vec3(50.0), q in vec3(50.0)) {
        let d = |a: &Vec3, b: &Vec3| ((a[0]-b[0]).powi(2) + (a[1]-b[1]).powi(2) + (a[2]-b[2]).powi(2)).sqrt();
        let (tp, tq) = (t.transform_point(&p), t.transform_point(&q));
        prop_assert!((d(&tp, &tq) - d(&p, &q)).abs() < 1e-9);
    }

    #[test]
    fn se3_integration_composes_for_constant_twist(p in pose(), xi in twist(), dt in 0.0..0.05f64) {
        let two = p.integrate(&xi, dt).integrate(&xi, dt);
        let one = p.integrate(&xi, 2.0 * dt);
        prop_assert!(mat_diff(&two.rotation.matrix, &one.rotation.matrix) < 1e-10);
        prop_assert!(vec_diff(&two.translation, &one.translation) < 1e-9);
    }

    #[test]
    fn pure_translation_twist_moves_in_body_frame(r in unit_axis(), t in 0.0..PI, v in vec3(5.0), dt in 0.0..1.0f64) {
        let rot = Rotation3::exp(&scale(&r, t));
        let p = Pose3::new(rot, [1.0, 2.0, 3.0]).integrate(&[v[0], v[1], v[2], 0.0, 0.0, 0.0], dt);
        let expected = rot.rotate(&scale(&v, dt));
        let moved = [p.translation[0] - 1.0, p.translation[1] - 2.0, p.translation[2] - 3.0];
        prop_assert!(vec_diff(&moved, &expected) < 1e-12);
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// `integrate_batch`, `integrate_batch_in_place` et la boucle séquentielle donnent
    /// exactement les mêmes bits, avec ou sans Rayon, y compris au-delà du seuil de 256.
    #[test]
    fn batch_equals_sequential_bit_for_bit(
        n in prop_oneof![0usize..8, 250usize..270, 500usize..1200],
        seed in any::<u64>(),
        dt in 0.0..0.1f64,
    ) {
        let mut s = seed | 1;
        let mut next = || { s ^= s << 13; s ^= s >> 7; s ^= s << 17; (s >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0 };
        let rots: Vec<_> = (0..n).map(|_| Rotation3::exp(&[next() * 3.0, next() * 3.0, next() * 3.0])).collect();
        let oms: Vec<Vec3> = (0..n).map(|_| [next() * 5.0, next() * 5.0, next() * 5.0]).collect();

        let batch = Rotation3::integrate_batch(&rots, &oms, dt);
        let mut inplace = rots.clone();
        Rotation3::integrate_batch_in_place(&mut inplace, &oms, dt);
        prop_assert_eq!(batch.len(), n);
        for i in 0..n {
            let seq = rots[i].integrate(&oms[i], dt);
            prop_assert_eq!(batch[i], seq);
            prop_assert_eq!(inplace[i], seq);
        }

        let poses: Vec<_> = rots.iter().map(|r| Pose3::new(*r, [1.0, 0.0, -1.0])).collect();
        let tw: Vec<[f64; 6]> = oms.iter().map(|w| [w[0], w[1], w[2], w[2], w[0], w[1]]).collect();
        let pb = Pose3::integrate_batch(&poses, &tw, dt);
        let mut pi = poses.clone();
        Pose3::integrate_batch_in_place(&mut pi, &tw, dt);
        for i in 0..n {
            let seq = poses[i].integrate(&tw[i], dt);
            prop_assert_eq!(pb[i], seq);
            prop_assert_eq!(pi[i], seq);
        }
    }
}

#[test]
#[should_panic(expected = "même longueur")]
fn pose_batch_length_mismatch_panics() {
    let _ = Pose3::integrate_batch(&[Pose3::identity()], &[], 1.0);
}

#[test]
#[should_panic(expected = "même longueur")]
fn pose_batch_in_place_length_mismatch_panics() {
    Pose3::integrate_batch_in_place(&mut [Pose3::identity()], &[], 1.0);
}

#[test]
#[should_panic(expected = "même longueur")]
fn rotation_batch_in_place_length_mismatch_panics() {
    Rotation3::integrate_batch_in_place(&mut [Rotation3::identity(); 2], &[[0.0; 3]], 1.0);
}

#[test]
fn empty_batches_are_fine() {
    assert!(Rotation3::integrate_batch(&[], &[], 0.1).is_empty());
    assert!(Pose3::integrate_batch(&[], &[], 0.1).is_empty());
}