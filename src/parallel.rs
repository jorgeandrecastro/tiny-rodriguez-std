// SPDX-License-Identifier: GPL-2.0-or-later
//! Primitives de traitement par lots (séquentielles ou Rayon).

/// Granularité minimale par tâche Rayon : évite que le coût d'ordonnancement
/// dépasse celui du calcul (une intégration SO(3) ≈ quelques dizaines de ns).
#[cfg(feature = "rayon")]
const MIN_LEN: usize = 256;

#[cfg(feature = "rayon")]
pub(crate) fn map_zip<A: Sync, B: Sync, O: Send>(
    a: &[A],
    b: &[B],
    f: impl Fn(&A, &B) -> O + Sync + Send,
) -> Vec<O> {
    use rayon::prelude::*;
    assert_eq!(a.len(), b.len(), "les deux lots doivent avoir la même longueur");
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
    use rayon::prelude::*;
    assert_eq!(a.len(), b.len(), "les deux lots doivent avoir la même longueur");
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