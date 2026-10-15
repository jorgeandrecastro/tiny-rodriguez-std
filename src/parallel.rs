// SPDX-License-Identifier: GPL-2.0-or-later
//! Primitives de traitement par lots (séquentielles ou Rayon).
//!
//! Avec la feature `rayon`, les lots plus petits que [`PAR_THRESHOLD`] restent
//! séquentiels : réveiller le pool de threads coûte plus cher que le calcul lui-même.

/// Taille de lot en dessous de laquelle on n'utilise pas Rayon.
/// Valeur provisoire à ajuster avec `cargo bench --bench batch --features rayon`.
#[cfg(feature = "rayon")]
pub(crate) const PAR_THRESHOLD: usize = 2048;

/// Granularité minimale par tâche Rayon (au-delà du seuil).
#[cfg(feature = "rayon")]
const MIN_LEN: usize = 256;

#[cfg(feature = "rayon")]
pub(crate) fn map_zip<A: Sync, B: Sync, O: Send>(
    a: &[A],
    b: &[B],
    f: impl Fn(&A, &B) -> O + Sync + Send,
) -> Vec<O> {
    assert_eq!(a.len(), b.len(), "les deux lots doivent avoir la même longueur");
    if a.len() < PAR_THRESHOLD {
        return a.iter().zip(b.iter()).map(|(x, y)| f(x, y)).collect();
    }
    use rayon::prelude::*;
    a.par_iter()
        .zip(b.par_iter())
        .with_min_len(MIN_LEN)
        .map(|(x, y)| f(x, y))
        .collect()
}

#[cfg(not(feature = "rayon"))]
pub(crate) fn map_zip<A, B, O>(a: &[A], b: &[B], f: impl Fn(&A, &B) -> O) -> Vec<O> {
    assert_eq!(a.len(), b.len(), "les deux lots doivent avoir la même longueur");
    a.iter().zip(b.iter()).map(|(x, y)| f(x, y)).collect()
}

#[cfg(feature = "rayon")]
pub(crate) fn for_each_zip_mut<A: Send, B: Sync>(
    a: &mut [A],
    b: &[B],
    f: impl Fn(&mut A, &B) + Sync + Send,
) {
    assert_eq!(a.len(), b.len(), "les deux lots doivent avoir la même longueur");
    if a.len() < PAR_THRESHOLD {
        a.iter_mut().zip(b.iter()).for_each(|(x, y)| f(x, y));
        return;
    }
    use rayon::prelude::*;
    a.par_iter_mut()
        .zip(b.par_iter())
        .with_min_len(MIN_LEN)
        .for_each(|(x, y)| f(x, y));
}

#[cfg(not(feature = "rayon"))]
pub(crate) fn for_each_zip_mut<A, B>(a: &mut [A], b: &[B], f: impl Fn(&mut A, &B)) {
    assert_eq!(a.len(), b.len(), "les deux lots doivent avoir la même longueur");
    a.iter_mut().zip(b.iter()).for_each(|(x, y)| f(x, y));
}