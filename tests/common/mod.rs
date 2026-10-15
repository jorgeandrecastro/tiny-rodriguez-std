// SPDX-License-Identifier: GPL-2.0-or-later
#![allow(dead_code)]
//! Utilitaires partagés par les tests d'intégration.

use tiny_rodriguez_std::Mat3;

/// Plus grand écart absolu entre deux matrices.
pub fn mat_diff(a: &Mat3, b: &Mat3) -> f64 {
    let mut m: f64 = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            m = m.max((a[i][j] - b[i][j]).abs());
        }
    }
    m
}

/// Plus grand écart absolu entre deux tranches.
pub fn vec_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max)
}