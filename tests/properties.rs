// SPDX-License-Identifier: GPL-2.0-or-later
use std::f64::consts::PI;
use tiny_rodriguez_std::linalg::{mat_mul, MAT3_IDENTITY};
use tiny_rodriguez_std::{Mat3, Pose3, Rotation3};

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn unit(&mut self) -> [f64; 3] {
        loop {
            let v = [self.next() * 2.0 - 1.0, self.next() * 2.0 - 1.0, self.next() * 2.0 - 1.0];
            let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            if n > 0.1 && n <= 1.0 {
                return [v[0] / n, v[1] / n, v[2] / n];
            }
        }
    }
}

fn mat_diff(a: &Mat3, b: &Mat3) -> f64 {
    let mut m: f64 = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            m = m.max((a[i][j] - b[i][j]).abs());
        }
    }
    m
}

fn vec_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max)
}

const ANGLES: [f64; 11] = [0.0, 1e-9, 1e-5, 1e-3, 0.1, 1.0, 2.0, 3.0, PI - 1e-4, PI - 1e-6, PI];

#[test]
fn exp_is_a_rotation() {
    let mut rng = Lcg(1);
    for &a in &ANGLES {
        let r = Rotation3::exp(&rng.unit().map(|x| x * a));
        assert!(r.is_valid(1e-14), "angle {a}");
    }
}

#[test]
fn exp_log_roundtrip() {
    let mut rng = Lcg(2);
    for &a in &ANGLES {
        for _ in 0..20 {
            let w = rng.unit().map(|x| x * a);
            let r = Rotation3::exp(&w);
            let r2 = Rotation3::exp(&r.log());
            assert!(mat_diff(&r.matrix, &r2.matrix) < 1e-12, "angle {a}");
            if a < 3.0 {
                assert!(vec_diff(&w, &r.log()) < 1e-10, "angle {a}");
            }
        }
    }
}

#[test]
fn hat_vee_roundtrip() {
    let w = [0.3, -1.2, 2.5];
    assert_eq!(Rotation3::vee(&Rotation3::hat(&w)), w);
}

#[test]
fn same_axis_rotations_add_angles() {
    let n = [0.0, 0.6, 0.8];
    let a = Rotation3::exp(&n.map(|x| x * 0.7));
    let b = Rotation3::exp(&n.map(|x| x * 1.1));
    let ab = Rotation3::exp(&n.map(|x| x * 1.8));
    assert!(mat_diff(&(a * b).matrix, &ab.matrix) < 1e-14);
}

#[test]
fn quaternion_roundtrip() {
    let mut rng = Lcg(3);
    for &a in &ANGLES {
        let r = Rotation3::exp(&rng.unit().map(|x| x * a));
        let r2 = Rotation3::from_quaternion(&r.to_quaternion());
        assert!(mat_diff(&r.matrix, &r2.matrix) < 1e-13, "angle {a}");
    }
}

#[test]
fn jacobian_times_inverse_is_identity() {
    let mut rng = Lcg(4);
    for &a in &[1e-6, 1e-3, 0.5, 2.0, 3.0] {
        let w = rng.unit().map(|x| x * a);
        let p = mat_mul(&Rotation3::left_jacobian(&w), &Rotation3::left_jacobian_inv(&w));
        assert!(mat_diff(&p, &MAT3_IDENTITY) < 1e-12, "angle {a}");
    }
}

#[test]
fn geodesic_interpolation_endpoints_and_midpoint() {
    let a = Rotation3::exp(&[0.2, -0.4, 0.1]);
    let b = Rotation3::exp(&[1.0, 0.5, -0.7]);
    assert!(mat_diff(&a.interpolate(&b, 0.0).matrix, &a.matrix) < 1e-14);
    assert!(mat_diff(&a.interpolate(&b, 1.0).matrix, &b.matrix) < 1e-13);
    let m = a.interpolate(&b, 0.5);
    let d = a.geodesic_distance(&b);
    assert!((a.geodesic_distance(&m) - d / 2.0).abs() < 1e-12);
    assert!((m.geodesic_distance(&b) - d / 2.0).abs() < 1e-12);
}

#[test]
fn constant_omega_integration_is_exact() {
    let w = [0.4, -0.2, 0.9];
    let (n, dt) = (1000, 1e-3);
    let mut r = Rotation3::identity();
    for _ in 0..n {
        r = r.integrate(&w, dt);
    }
    let exact = Rotation3::exp(&w.map(|x| x * n as f64 * dt));
    assert!(mat_diff(&r.matrix, &exact.matrix) < 1e-12);
}

#[test]
fn long_run_drift_is_bounded_and_renormalization_fixes_it() {
    let w = [0.5, 1.2, 0.3];
    let mut r = Rotation3::identity();
    for _ in 0..100_000 {
        r = r.integrate(&w, 1e-3);
    }
    assert!(r.orthogonality_error() < 1e-9);
    assert!(r.renormalized().orthogonality_error() < 1e-12);
}

#[test]
fn batch_matches_sequential_and_in_place() {
    let mut rng = Lcg(5);
    let rots: Vec<_> = (0..1000).map(|_| Rotation3::exp(&rng.unit().map(|x| x * 2.0))).collect();
    let oms: Vec<_> = (0..1000).map(|_| rng.unit()).collect();
    let batch = Rotation3::integrate_batch(&rots, &oms, 0.01);
    let mut inplace = rots.clone();
    Rotation3::integrate_batch_in_place(&mut inplace, &oms, 0.01);
    for i in 0..rots.len() {
        let seq = rots[i].integrate(&oms[i], 0.01);
        assert_eq!(batch[i], seq);
        assert_eq!(inplace[i], seq);
    }
}

#[test]
fn se3_exp_log_roundtrip() {
    let mut rng = Lcg(6);
    for &a in &[0.0, 1e-6, 0.3, 1.5, 3.0] {
        let w = rng.unit().map(|x| x * a);
        let xi = [0.5, -1.0, 2.0, w[0], w[1], w[2]];
        let xi2 = Pose3::exp(&xi).log();
        assert!(vec_diff(&xi, &xi2) < 1e-10, "angle {a}");
    }
}

#[test]
fn se3_pure_translation_and_inverse() {
    let p = Pose3::exp(&[1.0, 2.0, 3.0, 0.0, 0.0, 0.0]);
    assert_eq!(p.translation, [1.0, 2.0, 3.0]);

    let t = Pose3::new(Rotation3::exp(&[0.3, 0.2, -0.5]), [1.0, -2.0, 0.5]);
    let id = t.compose(&t.inverse());
    assert!(mat_diff(&id.rotation.matrix, &MAT3_IDENTITY) < 1e-14);
    assert!(vec_diff(&id.translation, &[0.0; 3]) < 1e-14);
}

#[test]
fn se3_constant_twist_integration_is_exact() {
    let xi = [0.3, 0.0, -0.2, 0.1, 0.4, -0.6];
    let (n, dt) = (500, 2e-3);
    let mut p = Pose3::identity();
    for _ in 0..n {
        p = p.integrate(&xi, dt);
    }
    let exact = Pose3::exp(&xi.map(|x| x * n as f64 * dt));
    assert!(mat_diff(&p.rotation.matrix, &exact.rotation.matrix) < 1e-12);
    assert!(vec_diff(&p.translation, &exact.translation) < 1e-12);
}