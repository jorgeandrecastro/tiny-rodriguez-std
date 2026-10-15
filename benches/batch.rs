// SPDX-License-Identifier: GPL-2.0-or-later
//! Lots : boucle séquentielle de référence contre `integrate_batch`.
//!
//! Sans la feature `rayon`, `integrate_batch` est séquentiel (les deux courbes doivent
//! alors coïncider). Avec la feature, la différence mesure le gain réel du parallélisme :
//!
//! ```bash
//! cargo bench --bench batch                    # séquentiel
//! cargo bench --bench batch --features rayon   # parallèle
//! ```

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use tiny_rodriguez_std::{Pose3, Rotation3};

const DT: f64 = 1.0 / 240.0;

fn data(n: usize) -> (Vec<Rotation3>, Vec<[f64; 3]>) {
    let rots = (0..n)
        .map(|i| {
            let f = i as f64 * 0.001;
            Rotation3::exp(&[f.sin(), f.cos(), 0.3 * f.sin()])
        })
        .collect();
    let oms = (0..n)
        .map(|i| {
            let f = i as f64 * 0.01;
            [0.5 + f.sin(), 1.2 * f.cos(), 0.3]
        })
        .collect();
    (rots, oms)
}

fn bench_rotations(c: &mut Criterion) {
    let mut g = c.benchmark_group("so3_batch");
    for n in [256usize, 1_024, 1_536, 2_048, 3_072, 4_096, 16_384, 65_536, 262_144] {
        let (rots, oms) = data(n);
        g.throughput(Throughput::Elements(n as u64));

        g.bench_with_input(BenchmarkId::new("sequential_reference", n), &n, |b, _| {
            b.iter(|| {
                let out: Vec<Rotation3> = rots
                    .iter()
                    .zip(oms.iter())
                    .map(|(r, w)| r.integrate(w, DT))
                    .collect();
                black_box(out)
            })
        });

        g.bench_with_input(BenchmarkId::new("integrate_batch", n), &n, |b, _| {
            b.iter(|| black_box(Rotation3::integrate_batch(&rots, &oms, DT)))
        });

        g.bench_with_input(BenchmarkId::new("integrate_batch_in_place", n), &n, |b, _| {
            let mut work = rots.clone();
            b.iter(|| {
                Rotation3::integrate_batch_in_place(&mut work, &oms, DT);
                black_box(&work);
            })
        });
    }
    g.finish();
}

fn bench_poses(c: &mut Criterion) {
    let mut g = c.benchmark_group("se3_batch");
    for n in [256usize, 1_024, 2_048, 4_096, 65_536] {
        let poses = vec![Pose3::identity(); n];
        let twists = vec![[1.0, 0.0, 0.0, 0.1, 0.4, 0.5]; n];
        g.throughput(Throughput::Elements(n as u64));

        g.bench_with_input(BenchmarkId::new("sequential_reference", n), &n, |b, _| {
            b.iter(|| {
                let out: Vec<Pose3> = poses
                    .iter()
                    .zip(twists.iter())
                    .map(|(p, t)| p.integrate(t, DT))
                    .collect();
                black_box(out)
            })
        });

        g.bench_with_input(BenchmarkId::new("integrate_batch", n), &n, |b, _| {
            b.iter(|| black_box(Pose3::integrate_batch(&poses, &twists, DT)))
        });
    }
    g.finish();
}

criterion_group!(benches, bench_rotations, bench_poses);
criterion_main!(benches);