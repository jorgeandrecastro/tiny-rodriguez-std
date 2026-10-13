// SPDX-License-Identifier: GPL-2.0-or-later
//! # Tiny Rodriguez Std
//!
//! Géométrie différentielle minimale pour la robotique et les drones :
//!
//! * [`Rotation3`] : groupe de Lie SO(3) (exp/log, Jacobiennes, intégration exponentielle) ;
//! * [`Pose3`] : groupe de Lie SE(3) (pose rigide, torseurs) ;
//! * intégration par lots, parallélisée avec la feature `rayon`.
//!
//! Sans dépendance obligatoire, sans `unsafe`, sans allocation hors des API `Vec`.
//!
//! ```
//! use tiny_rodriguez_std::Rotation3;
//!
//! let r = Rotation3::identity().integrate(&[0.0, 0.0, 1.0], 0.5);
//! let w = r.log();
//! assert!((w[2] - 0.5).abs() < 1e-12);
//! ```

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod linalg;
mod parallel;
pub mod se3;
pub mod so3;

pub use linalg::{Mat3, Vec3};
pub use se3::Pose3;
pub use so3::Rotation3;