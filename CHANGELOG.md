# Changelog

Toutes les modifications notables de ce projet sont consignées ici.

Le format suit [Keep a Changelog](https://keepachangelog.com/fr/1.1.0/) et le projet
suit le [versionnage sémantique](https://semver.org/lang/fr/). Tant que la version
majeure est 0, une version mineure peut contenir des changements incompatibles.

## [Non publié]

## [0.2.0] - 2026-10-14

### Ajouté

- **SE(3)** : type `Pose3` avec `identity`, `new`, `exp`, `log`, `compose`, `inverse`,
  `transform_point`, `integrate`, `integrate_batch` et `integrate_batch_in_place`.
  Convention des torseurs : `[v, ω]`, vitesse linéaire d'abord.
- **SO(3)** : `log` (robuste près de l'identité et de θ = π), `angle`, `vee`,
  `left_jacobian`, `left_jacobian_inv`, `from_axis_angle`, `compose`, `inverse`,
  `rotate`, `geodesic_distance`, `interpolate` (géodésique), `integrate_world`,
  `to_quaternion`, `from_quaternion`, `orthogonality_error`, `is_valid`, `renormalized`.
- Implémentations de `Default`, `Mul<Rotation3>` et `Mul<Vec3>` pour `Rotation3`.
- Module public `linalg` (`Vec3`, `Mat3`, `MAT3_IDENTITY`, `dot`, `cross`, `norm`,
  `scale`, `mat_mul`, `mat_vec`, `transpose`, `det`).
- `Rotation3::integrate_batch_in_place` : lot sans allocation.
- Seuil séquentiel pour la feature `rayon` : les lots de moins de 2048 éléments ne
  sont plus parallélisés.
- Benchmarks `criterion` (`benches/geometry.rs`, `benches/batch.rs`).
- Tests par propriétés `proptest` (`tests/proptest_so3.rs`, `tests/proptest_se3.rs`),
  dont l'égalité bit à bit des lots avec la boucle séquentielle.
- Exemple Bevy (`examples/bevy-drone`) : quadricoptère animé, avec tests de validation
  croisée contre `glam`.
- Intégration continue (format, clippy, tests avec et sans `rayon`).
- `README.md` complet et fichier `LICENSE` (GPL-2.0).

### Modifié

- **Incompatible** : `integrate_batch` panique désormais si les deux tranches n'ont pas
  la même longueur. En 0.1, le lot était tronqué silencieusement à la plus courte.
- `exp` utilise des développements de Taylor pour θ² < 10⁻⁴ au lieu de renvoyer
  l'identité sous 10⁻¹², ce qui évite une perte de précision pour les petits angles.
- `#![forbid(unsafe_code)]` et `#![warn(missing_docs)]` activés.
- Version minimale de Rust vérifiée : 1.75.

### Corrigé

- Avec `rayon`, un lot de 1 024 rotations était intégré 2,2 fois plus lentement qu'en
  séquentiel (granularité de 256 sans seuil séquentiel). Les petits lots restent
  maintenant séquentiels.

## [0.1.0]

### Ajouté

- Version initiale : `Rotation3` avec `identity`, `hat`, `exp` (formule de Rodrigues),
  `integrate` et `integrate_batch` (parallèle avec la feature `rayon`).

[Non publié]: https://github.com/jorgeandrecastro/tiny-rodriguez-std/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/jorgeandrecastro/tiny-rodriguez-std/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/jorgeandrecastro/tiny-rodriguez-std/releases/tag/v0.1.0