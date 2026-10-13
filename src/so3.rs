// SPDX-License-Identifier: GPL-2.0-or-later
//! Groupe de Lie SO(3) : rotations 3D.

use crate::linalg::{self, Mat3, Vec3, MAT3_IDENTITY};
use crate::parallel;
use core::f64::consts::PI;
use core::ops::Mul;

/// Seuil sur θ² en dessous duquel on utilise les développements de Taylor.
const SMALL_THETA2: f64 = 1e-4;

/// Coefficients `(a, b, c)` avec
/// a = sin θ / θ, b = (1 − cos θ) / θ², c = (1 − a) / θ².
#[inline]
fn coefficients(theta2: f64) -> (f64, f64, f64) {
    if theta2 < SMALL_THETA2 {
        let t4 = theta2 * theta2;
        let t6 = t4 * theta2;
        (
            1.0 - theta2 / 6.0 + t4 / 120.0 - t6 / 5040.0,
            0.5 - theta2 / 24.0 + t4 / 720.0 - t6 / 40320.0,
            1.0 / 6.0 - theta2 / 120.0 + t4 / 5040.0 - t6 / 362880.0,
        )
    } else {
        let theta = theta2.sqrt();
        let hs = (0.5 * theta).sin();
        let a = theta.sin() / theta;
        (a, 2.0 * hs * hs / theta2, (1.0 - a) / theta2)
    }
}

/// Coefficient d = (1 − (θ/2)·cot(θ/2)) / θ² de l'inverse de la Jacobienne gauche.
#[inline]
fn inv_coefficient(theta2: f64) -> f64 {
    if theta2 < SMALL_THETA2 {
        let t4 = theta2 * theta2;
        1.0 / 12.0 + theta2 / 720.0 + t4 / 30240.0 + t4 * theta2 / 1_209_600.0
    } else {
        let theta = theta2.sqrt();
        (1.0 - 0.5 * theta / (0.5 * theta).tan()) / theta2
    }
}

/// K² = ω ωᵀ − |ω|² I, avec une diagonale sans annulation catastrophique.
#[inline]
fn hat_squared(w: &Vec3) -> Mat3 {
    let (xx, yy, zz) = (w[0] * w[0], w[1] * w[1], w[2] * w[2]);
    [
        [-(yy + zz), w[0] * w[1], w[0] * w[2]],
        [w[0] * w[1], -(xx + zz), w[1] * w[2]],
        [w[0] * w[2], w[1] * w[2], -(xx + yy)],
    ]
}

/// Retourne I + a·K + b·K².
#[inline]
fn id_plus(a: f64, k: &Mat3, b: f64, k2: &Mat3) -> Mat3 {
    let mut r = MAT3_IDENTITY;
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] += a * k[i][j] + b * k2[i][j];
        }
    }
    r
}

/// Rotation 3D, élément de SO(3), stockée sous forme de matrice 3×3.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotation3 {
    /// Matrice de rotation (stockage par lignes).
    pub matrix: Mat3,
}

impl Default for Rotation3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl Rotation3 {
    /// Rotation identité.
    pub const fn identity() -> Self {
        Self { matrix: MAT3_IDENTITY }
    }

    /// Opérateur `^` (hat) : vecteur 3D → matrice antisymétrique de so(3).
    #[inline]
    pub fn hat(omega: &Vec3) -> Mat3 {
        [
            [0.0, -omega[2], omega[1]],
            [omega[2], 0.0, -omega[0]],
            [-omega[1], omega[0], 0.0],
        ]
    }

    /// Opérateur `∨` (vee) : inverse de [`hat`](Self::hat).
    #[inline]
    pub fn vee(m: &Mat3) -> Vec3 {
        [m[2][1], m[0][2], m[1][0]]
    }

    /// Exponentielle de so(3) → SO(3) (formule de Rodrigues, stable pour θ → 0).
    pub fn exp(omega: &Vec3) -> Self {
        let theta2 = linalg::dot(omega, omega);
        let (a, b, _) = coefficients(theta2);
        let k = Self::hat(omega);
        let k2 = hat_squared(omega);
        Self { matrix: id_plus(a, &k, b, &k2) }
    }

    /// Rotation d'angle `angle` (rad) autour de `axis` (normalisé en interne).
    /// Un axe nul donne l'identité.
    pub fn from_axis_angle(axis: &Vec3, angle: f64) -> Self {
        let n = linalg::norm(axis);
        if n == 0.0 {
            return Self::identity();
        }
        Self::exp(&linalg::scale(axis, angle / n))
    }

    /// Angle ‖log R‖ ∈ [0, π] et vecteur `vee(R − Rᵀ)`.
    #[inline]
    fn angle_and_skew(&self) -> (f64, Vec3) {
        let m = &self.matrix;
        let v = [m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]];
        let s = 0.5 * linalg::norm(&v);
        let c = (0.5 * (m[0][0] + m[1][1] + m[2][2] - 1.0)).clamp(-1.0, 1.0);
        (s.atan2(c), v)
    }

    /// Angle de rotation ∈ [0, π].
    pub fn angle(&self) -> f64 {
        self.angle_and_skew().0
    }

    /// Logarithme SO(3) → so(3), de norme ≤ π.
    /// Robuste près de l'identité et près de θ = π.
    pub fn log(&self) -> Vec3 {
        let m = &self.matrix;
        let (theta, v) = self.angle_and_skew();

        if theta < 1e-4 {
            return linalg::scale(&v, 0.5 * (1.0 + theta * theta / 6.0));
        }
        if theta < PI - 1e-3 {
            return linalg::scale(&v, theta / (2.0 * theta.sin()));
        }

        // Voisinage de π : R ≈ c·I + (1−c)·n nᵀ ; on extrait n de la partie symétrique.
        let c = theta.cos();
        let denom = 1.0 - c;
        let mut k = 0;
        if m[1][1] > m[k][k] {
            k = 1;
        }
        if m[2][2] > m[k][k] {
            k = 2;
        }
        let nk = ((m[k][k] - c) / denom).max(0.0).sqrt();
        let mut n = [0.0; 3];
        n[k] = nk;
        for j in 0..3 {
            if j != k {
                n[j] = (m[k][j] + m[j][k]) / (2.0 * denom * nk);
            }
        }
        let nn = linalg::norm(&n);
        n = linalg::scale(&n, 1.0 / nn);
        if linalg::dot(&n, &v) < 0.0 {
            n = linalg::scale(&n, -1.0);
        }
        linalg::scale(&n, theta)
    }

    /// Jacobienne gauche J(ω) de SO(3).
    pub fn left_jacobian(omega: &Vec3) -> Mat3 {
        let (_, b, c) = coefficients(linalg::dot(omega, omega));
        id_plus(b, &Self::hat(omega), c, &hat_squared(omega))
    }

    /// Inverse de la Jacobienne gauche J(ω)⁻¹ (valable pour ‖ω‖ < 2π).
    pub fn left_jacobian_inv(omega: &Vec3) -> Mat3 {
        let d = inv_coefficient(linalg::dot(omega, omega));
        id_plus(-0.5, &Self::hat(omega), d, &hat_squared(omega))
    }

    /// Composition `self ∘ other` (appliquer `other` puis `self`).
    #[inline]
    pub fn compose(&self, other: &Self) -> Self {
        Self { matrix: linalg::mat_mul(&self.matrix, &other.matrix) }
    }

    /// Inverse (= transposée).
    #[inline]
    pub fn inverse(&self) -> Self {
        Self { matrix: linalg::transpose(&self.matrix) }
    }

    /// Applique la rotation à un vecteur.
    #[inline]
    pub fn rotate(&self, v: &Vec3) -> Vec3 {
        linalg::mat_vec(&self.matrix, v)
    }

    /// Distance géodésique (angle de la rotation relative) ∈ [0, π].
    pub fn geodesic_distance(&self, other: &Self) -> f64 {
        self.inverse().compose(other).angle()
    }

    /// Interpolation géodésique : `t = 0` → `self`, `t = 1` → `other`.
    pub fn interpolate(&self, other: &Self, t: f64) -> Self {
        let delta = self.inverse().compose(other).log();
        self.compose(&Self::exp(&linalg::scale(&delta, t)))
    }

    /// Intégration exponentielle, vitesse angulaire exprimée dans le **repère corps** :
    /// `R ← R · exp(ω dt)`. Exacte pour ω constant.
    #[inline]
    pub fn integrate(&self, omega_body: &Vec3, dt: f64) -> Self {
        self.compose(&Self::exp(&linalg::scale(omega_body, dt)))
    }

    /// Intégration exponentielle, vitesse angulaire dans le **repère monde** :
    /// `R ← exp(ω dt) · R`.
    #[inline]
    pub fn integrate_world(&self, omega_world: &Vec3, dt: f64) -> Self {
        Self::exp(&linalg::scale(omega_world, dt)).compose(self)
    }

    /// Intègre un lot de rotations (parallèle avec la feature `rayon`).
    ///
    /// # Panics
    /// Si `rotations` et `omegas` n'ont pas la même longueur.
    pub fn integrate_batch(rotations: &[Self], omegas: &[Vec3], dt: f64) -> Vec<Self> {
        parallel::map_zip(rotations, omegas, |r, w| r.integrate(w, dt))
    }

    /// Variante en place de [`integrate_batch`](Self::integrate_batch), sans allocation.
    ///
    /// # Panics
    /// Si `rotations` et `omegas` n'ont pas la même longueur.
    pub fn integrate_batch_in_place(rotations: &mut [Self], omegas: &[Vec3], dt: f64) {
        parallel::for_each_zip_mut(rotations, omegas, |r, w| *r = r.integrate(w, dt));
    }

    /// Erreur d'orthogonalité ‖RᵀR − I‖_F (dérive numérique).
    pub fn orthogonality_error(&self) -> f64 {
        let rtr = linalg::mat_mul(&linalg::transpose(&self.matrix), &self.matrix);
        let mut e = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                let d = rtr[i][j] - MAT3_IDENTITY[i][j];
                e += d * d;
            }
        }
        e.sqrt()
    }

    /// Vrai si la matrice est orthogonale (à `tol` près) de déterminant +1.
    pub fn is_valid(&self, tol: f64) -> bool {
        self.orthogonality_error() <= tol && (linalg::det(&self.matrix) - 1.0).abs() <= tol
    }

    /// Reprojette sur SO(3) par itération polaire (convergence quadratique).
    /// À appeler périodiquement lors d'intégrations très longues.
    pub fn renormalized(&self) -> Self {
        let mut r = self.matrix;
        for _ in 0..4 {
            let rtr = linalg::mat_mul(&linalg::transpose(&r), &r);
            let mut dev = 0.0;
            for i in 0..3 {
                for j in 0..3 {
                    let d = rtr[i][j] - MAT3_IDENTITY[i][j];
                    dev += d * d;
                }
            }
            if dev < 1e-32 {
                break;
            }
            let rrtr = linalg::mat_mul(&r, &rtr);
            for i in 0..3 {
                for j in 0..3 {
                    r[i][j] = 1.5 * r[i][j] - 0.5 * rrtr[i][j];
                }
            }
        }
        Self { matrix: r }
    }

    /// Quaternion unitaire `[x, y, z, w]` (convention Hamilton, w ≥ 0), méthode de Shepperd.
    pub fn to_quaternion(&self) -> [f64; 4] {
        let m = &self.matrix;
        let t = m[0][0] + m[1][1] + m[2][2];
        let mut q = if t > 0.0 {
            let s = (t + 1.0).sqrt() * 2.0;
            [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s]
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
            [0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]
        } else if m[1][1] > m[2][2] {
            let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
            [(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]
        } else {
            let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
            [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s]
        };
        let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        let sign = if q[3] < 0.0 { -1.0 } else { 1.0 };
        for x in &mut q {
            *x *= sign / n;
        }
        q
    }

    /// Construit la rotation depuis un quaternion `[x, y, z, w]` (normalisé en interne).
    /// Un quaternion nul donne l'identité.
    pub fn from_quaternion(q: &[f64; 4]) -> Self {
        let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        if n == 0.0 {
            return Self::identity();
        }
        let (x, y, z, w) = (q[0] / n, q[1] / n, q[2] / n, q[3] / n);
        let (xx, yy, zz) = (x * x, y * y, z * z);
        Self {
            matrix: [
                [1.0 - 2.0 * (yy + zz), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (xx + zz), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (xx + yy)],
            ],
        }
    }
}

impl Mul for Rotation3 {
    type Output = Rotation3;
    #[inline]
    fn mul(self, rhs: Rotation3) -> Rotation3 {
        self.compose(&rhs)
    }
}

impl Mul<Vec3> for Rotation3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, rhs: Vec3) -> Vec3 {
        self.rotate(&rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quarter_turn_about_z() {
        let r = Rotation3::exp(&[0.0, 0.0, PI / 2.0]);
        let v = r.rotate(&[1.0, 0.0, 0.0]);
        assert!(v[0].abs() < 1e-15 && (v[1] - 1.0).abs() < 1e-15 && v[2].abs() < 1e-15);
    }

    #[test]
    fn batch_integration() {
        let rotations = vec![Rotation3::identity(); 10];
        let omegas = vec![[0.0, 0.0, 0.1]; 10];
        let out = Rotation3::integrate_batch(&rotations, &omegas, 1.0);
        assert_eq!(out.len(), 10);
        assert_ne!(out[0], Rotation3::identity());
    }

    #[test]
    #[should_panic]
    fn batch_length_mismatch_panics() {
        let _ = Rotation3::integrate_batch(&[Rotation3::identity()], &[], 1.0);
    }
}