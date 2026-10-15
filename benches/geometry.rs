// SPDX-License-Identifier: GPL-2.0-or-later
//! Micro-benchmarks des opérations unitaires.
//!
//! ```bash
//! cargo bench --bench geometry
//! ```

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::f64::consts::PI;
use tiny_rodriguez_std::{Pose3, Rotation3};

fn bench_so3(c: &mut Criterion) {
    let mut g = c.benchmark_group("so3");

    // exp : régime Taylor, régime général, proche de π
    for (name, w) in [
        ("exp/small", [1e-3, 2e-3, -1e-3]),
        ("exp/medium", [0.3, -0.5, 0.8]),
        ("exp/near_pi", [PI - 1e-3, 0.0, 0.0]),
    ] {
        g.bench_function(name, |b| b.iter(|| Rotation3::exp(black_box(&w))));
    }

    // log : mêmes régimes (les trois branches de l'implémentation)
    for (name, w) in [
        ("log/small", [1e-3, 2e-3, -1e-3]),
        ("log/medium", [0.3, -0.5, 0.8]),
        ("log/near_pi", [PI - 1e-4, 0.0, 0.0]),
    ] {
        let r = Rotation3::exp(&w);
        g.bench_function(name, |b| b.iter(|| black_box(&r).log()));
    }

    let r = Rotation3::exp(&[0.3, -0.5, 0.8]);
    let s = Rotation3::exp(&[-0.2, 0.9, 0.1]);
    let w = [0.5, 1.2, 0.3];
    let v = [1.0, 2.0, 3.0];

    g.bench_function("compose", |b| b.iter(|| black_box(&r).compose(black_box(&s))));
    g.bench_function("rotate", |b| b.iter(|| black_box(&r).rotate(black_box(&v))));
    g.bench_function("integrate", |b| b.iter(|| black_box(&r).integrate(black_box(&w), 0.001)));
    g.bench_function("integrate_world", |b| {
        b.iter(|| black_box(&r).integrate_world(black_box(&w), 0.001))
    });
    g.bench_function("interpolate", |b| b.iter(|| black_box(&r).interpolate(black_box(&s), 0.5)));
    g.bench_function("to_quaternion", |b| b.iter(|| black_box(&r).to_quaternion()));
    g.bench_function("from_quaternion", |b| {
        let q = r.to_quaternion();
        b.iter(|| Rotation3::from_quaternion(black_box(&q)))
    });
    g.bench_function("renormalized", |b| b.iter(|| black_box(&r).renormalized()));
    g.bench_function("left_jacobian_inv", |b| {
        b.iter(|| Rotation3::left_jacobian_inv(black_box(&w)))
    });
    g.finish();
}

fn bench_se3(c: &mut Criterion) {
    let mut g = c.benchmark_group("se3");
    let xi = [0.3, -0.2, 0.5, 0.1, 0.4, -0.6];
    let p = Pose3::exp(&xi);
    let q = Pose3::exp(&[-0.1, 0.4, 0.2, 0.5, -0.3, 0.2]);

    g.bench_function("exp", |b| b.iter(|| Pose3::exp(black_box(&xi))));
    g.bench_function("log", |b| b.iter(|| black_box(&p).log()));
    g.bench_function("compose", |b| b.iter(|| black_box(&p).compose(black_box(&q))));
    g.bench_function("inverse", |b| b.iter(|| black_box(&p).inverse()));
    g.bench_function("integrate", |b| b.iter(|| black_box(&p).integrate(black_box(&xi), 0.001)));
    g.finish();
}

criterion_group!(benches, bench_so3, bench_se3);
criterion_main!(benches);