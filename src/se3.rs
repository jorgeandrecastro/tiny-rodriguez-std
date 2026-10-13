// SPDX-License-Identifier: GPL-2.0-or-later
//! Groupe de Lie SE(3) : poses rigides (rotation + translation).
//!
//! Convention des torseurs : `ξ = [v₀, v₁, v₂, ω₀, ω₁, ω₂]` (vitesse linéaire d'abord).

use crate::linalg::{self, Vec3};
use crate::parallel;
use crate::so3::Rotation3;

/// Pose rigide 3D `(R, t)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose3 {
    /// Orientation.
    pub rotation: Rotation3,
    /// Position.
    pub translation: Vec3,
}

impl Default for Pose3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl Pose3 {
    /// Pose identité.
    pub const fn identity() -> Self {
        Self { rotation: Rotation3::identity(), translation: [0.0; 3] }
    }

    /// Construit une pose.
    pub const fn new(rotation: Rotation3, translation: Vec3) -> Self {
        Self { rotation, translation }
    }

    /// Exponentielle se(3) → SE(3) : `R = exp(ω)`, `t = J(ω)·v`.
    pub fn exp(xi: &[f64; 6]) -> Self {
        let v = [xi[0], xi[1], xi[2]];
        let w = [xi[3], xi[4], xi[5]];
        Self {
            rotation: Rotation3::exp(&w),
            translation: linalg::mat_vec(&Rotation3::left_jacobian(&w), &v),
        }
    }

    /// Logarithme SE(3) → se(3) : `ω = log R`, `v = J(ω)⁻¹·t`.
    pub fn log(&self) -> [f64; 6] {
        let w = self.rotation.log();
        let v = linalg::mat_vec(&Rotation3::left_jacobian_inv(&w), &self.translation);
        [v[0], v[1], v[2], w[0], w[1], w[2]]
    }

    /// Composition `self ∘ other`.
    #[inline]
    pub fn compose(&self, other: &Self) -> Self {
        let rt = self.rotation.rotate(&other.translation);
        Self {
            rotation: self.rotation.compose(&other.rotation),
            translation: [rt[0] + self.translation[0], rt[1] + self.translation[1], rt[2] + self.translation[2]],
        }
    }

    /// Inverse : `(Rᵀ, −Rᵀ t)`.
    pub fn inverse(&self) -> Self {
        let rinv = self.rotation.inverse();
        let t = rinv.rotate(&self.translation);
        Self { rotation: rinv, translation: [-t[0], -t[1], -t[2]] }
    }

    /// Transforme un point : `R p + t`.
    #[inline]
    pub fn transform_point(&self, p: &Vec3) -> Vec3 {
        let r = self.rotation.rotate(p);
        [r[0] + self.translation[0], r[1] + self.translation[1], r[2] + self.translation[2]]
    }

    /// Intégration exponentielle d'un torseur exprimé dans le repère corps :
    /// `T ← T · exp(ξ dt)`. Exacte pour ξ constant.
    #[inline]
    pub fn integrate(&self, twist_body: &[f64; 6], dt: f64) -> Self {
        let s = [
            twist_body[0] * dt, twist_body[1] * dt, twist_body[2] * dt,
            twist_body[3] * dt, twist_body[4] * dt, twist_body[5] * dt,
        ];
        self.compose(&Self::exp(&s))
    }

    /// Intègre un lot de poses (parallèle avec la feature `rayon`).
    ///
    /// # Panics
    /// Si les deux lots n'ont pas la même longueur.
    pub fn integrate_batch(poses: &[Self], twists: &[[f64; 6]], dt: f64) -> Vec<Self> {
        parallel::map_zip(poses, twists, |p, t| p.integrate(t, dt))
    }

    /// Variante en place de [`integrate_batch`](Self::integrate_batch).
    ///
    /// # Panics
    /// Si les deux lots n'ont pas la même longueur.
    pub fn integrate_batch_in_place(poses: &mut [Self], twists: &[[f64; 6]], dt: f64) {
        parallel::for_each_zip_mut(poses, twists, |p, t| *p = p.integrate(t, dt));
    }
}