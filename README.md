# tiny-rodriguez-std

[![Licence: GPL v2+](https://img.shields.io/badge/licence-GPL--2.0--or--later-blue.svg)](LICENSE)
![Rust](https://img.shields.io/badge/rust-1.75%2B-orange.svg)
![unsafe](https://img.shields.io/badge/unsafe-forbidden-success.svg)

Bibliothèque minimale et rapide de **géométrie différentielle** pour la robotique et les drones :

- **SO(3)** : rotations 3D (`Rotation3`)
- **SE(3)** : poses rigides, rotation + translation (`Pose3`)
- **Intégration exponentielle** exacte pour une vitesse constante
- **Traitement par lots** parallélisable avec Rayon (feature optionnelle)

Sans dépendance obligatoire, sans `unsafe` (`#![forbid(unsafe_code)]`), sans allocation dans les opérations unitaires. Cible `std` et `f64`.

Historique des versions : voir [CHANGELOG.md](CHANGELOG.md).

---

## Sommaire

- [Installation](#installation)
- [Démarrage rapide](#démarrage-rapide)
- [Conventions](#conventions)
- [Référence de l'API](#référence-de-lapi)
  - [`Rotation3`](#rotation3-so3)
  - [`Pose3`](#pose3-se3)
  - [`linalg`](#linalg)
- [Feature `rayon`](#feature-rayon)
- [Performances](#performances)
- [Stabilité numérique](#stabilité-numérique)
- [Tests et validation](#tests-et-validation)
- [Exemple Bevy](#exemple-bevy)
- [Structure du projet](#structure-du-projet)
- [Feuille de route](#feuille-de-route)
- [Licence](#licence)

---

## Installation

```toml
[dependencies]
tiny-rodriguez-std = "0.2"

# avec le traitement par lots parallèle :
tiny-rodriguez-std = { version = "0.2", features = ["rayon"] }
```

Version minimale de Rust : **1.75** pour la bibliothèque (vérifiée), édition 2021. Pour lancer les tests et les benchmarks, utilisez une toolchain récente : les dépendances de développement (`proptest`, `criterion`) exigent une version plus récente que celle de la bibliothèque.

---

## Démarrage rapide

```rust
use tiny_rodriguez_std::{Pose3, Rotation3};

fn main() {
    // --- SO(3) ---
    // Vitesse angulaire dans le repère corps (rad/s), pas de temps 16 ms
    let r = Rotation3::identity().integrate(&[0.5, 1.2, 0.3], 0.016);
    let omega = r.log(); // retrouve ω·dt

    // Interpolation géodésique entre deux orientations
    let a = Rotation3::exp(&[0.2, -0.4, 0.1]);
    let b = Rotation3::exp(&[1.0, 0.5, -0.7]);
    let milieu = a.interpolate(&b, 0.5);

    // Rotation d'un vecteur, composition par opérateur *
    let v = (a * b) * [1.0, 0.0, 0.0];

    // --- SE(3) ---
    // Torseur [vx, vy, vz, ωx, ωy, ωz] dans le repère corps
    let mut pose = Pose3::identity();
    for _ in 0..1000 {
        pose = pose.integrate(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.5], 0.001);
    }
    let xi = pose.log(); // [v, ω] de la pose finale

    let _ = (omega, milieu, v, xi);
}
```

---

## Conventions

| Sujet | Convention |
|---|---|
| Stockage des matrices | Par **lignes** : `matrix[i][j]` = ligne `i`, colonne `j` |
| Quaternions | `[x, y, z, w]`, Hamilton, `w ≥ 0` en sortie |
| Torseurs (se(3)) | `ξ = [v₀, v₁, v₂, ω₀, ω₁, ω₂]` : vitesse **linéaire d'abord**, puis angulaire |
| `integrate` | Vitesse exprimée dans le **repère corps** : `R ← R · exp(ω dt)` |
| `integrate_world` | Vitesse exprimée dans le **repère monde** : `R ← exp(ω dt) · R` |
| Composition | `a.compose(&b)` = appliquer `b` puis `a` (équivaut à `a * b`) |
| Angles | Radians |
| Précision | `f64` |
| Entrées dégénérées | Axe nul ou quaternion nul → identité. `NaN` / infini : pas de panique, le résultat est `NaN`. |

Attention si vous interfacez avec d'autres bibliothèques : certaines utilisent `[ω, v]` pour les torseurs.

---

## Référence de l'API

### `Rotation3` (SO(3))

```rust
pub struct Rotation3 { pub matrix: Mat3 }
```

Implémente `Debug`, `Clone`, `Copy`, `PartialEq`, `Default` (identité), `Mul<Rotation3>` et `Mul<Vec3>`.

#### Construction

| Fonction | Description |
|---|---|
| `identity() -> Self` | Rotation identité (`const fn`). |
| `exp(omega: &Vec3) -> Self` | Exponentielle so(3) → SO(3) (formule de Rodrigues), stable pour θ → 0 grâce aux développements de Taylor. |
| `from_axis_angle(axis: &Vec3, angle: f64) -> Self` | Rotation d'angle `angle` autour de `axis` (normalisé en interne). Un axe nul donne l'identité. |
| `from_quaternion(q: &[f64; 4]) -> Self` | Depuis un quaternion `[x, y, z, w]` (normalisé en interne). Un quaternion nul donne l'identité. |

#### Algèbre de Lie

| Fonction | Description |
|---|---|
| `hat(omega: &Vec3) -> Mat3` | Opérateur `^` : vecteur → matrice antisymétrique. |
| `vee(m: &Mat3) -> Vec3` | Opérateur `∨` : inverse de `hat`. |
| `log(&self) -> Vec3` | Logarithme SO(3) → so(3), de norme ≤ π. Robuste près de l'identité et près de θ = π. |
| `angle(&self) -> f64` | Angle de rotation dans [0, π]. |
| `left_jacobian(omega: &Vec3) -> Mat3` | Jacobienne gauche `J(ω)`. |
| `left_jacobian_inv(omega: &Vec3) -> Mat3` | Inverse `J(ω)⁻¹`, valable pour `‖ω‖ < 2π`. |

#### Opérations de groupe

| Fonction | Description |
|---|---|
| `compose(&self, other: &Self) -> Self` | `self ∘ other`. |
| `inverse(&self) -> Self` | Inverse (transposée). |
| `rotate(&self, v: &Vec3) -> Vec3` | Applique la rotation à un vecteur. |
| `geodesic_distance(&self, other: &Self) -> f64` | Angle de la rotation relative, dans [0, π]. |
| `interpolate(&self, other: &Self, t: f64) -> Self` | Interpolation géodésique : `t = 0` → `self`, `t = 1` → `other`. |

#### Intégration

| Fonction | Description |
|---|---|
| `integrate(&self, omega_body: &Vec3, dt: f64) -> Self` | `R ← R · exp(ω dt)`, ω dans le repère corps. Exacte pour ω constant. |
| `integrate_world(&self, omega_world: &Vec3, dt: f64) -> Self` | `R ← exp(ω dt) · R`, ω dans le repère monde. |
| `integrate_batch(rotations: &[Self], omegas: &[Vec3], dt: f64) -> Vec<Self>` | Intègre un lot (parallèle avec `rayon`, voir le seuil). **Panique** si les longueurs diffèrent. |
| `integrate_batch_in_place(rotations: &mut [Self], omegas: &[Vec3], dt: f64)` | Variante en place, sans allocation. **Panique** si les longueurs diffèrent. |

#### Contrôle de la dérive numérique

| Fonction | Description |
|---|---|
| `orthogonality_error(&self) -> f64` | Erreur `‖RᵀR − I‖_F`. |
| `is_valid(&self, tol: f64) -> bool` | Vrai si `R` est orthogonale (à `tol` près) de déterminant +1. |
| `renormalized(&self) -> Self` | Reprojette sur SO(3) par itération polaire (convergence quadratique). |

#### Conversions

| Fonction | Description |
|---|---|
| `to_quaternion(&self) -> [f64; 4]` | Quaternion unitaire `[x, y, z, w]`, `w ≥ 0` (méthode de Shepperd). |

---

### `Pose3` (SE(3))

```rust
pub struct Pose3 {
    pub rotation: Rotation3,
    pub translation: Vec3,
}
```

Implémente `Debug`, `Clone`, `Copy`, `PartialEq`, `Default` (identité).

| Fonction | Description |
|---|---|
| `identity() -> Self` | Pose identité (`const fn`). |
| `new(rotation: Rotation3, translation: Vec3) -> Self` | Construit une pose (`const fn`). |
| `exp(xi: &[f64; 6]) -> Self` | Exponentielle se(3) → SE(3) : `R = exp(ω)`, `t = J(ω)·v`. |
| `log(&self) -> [f64; 6]` | Logarithme SE(3) → se(3) : `ω = log R`, `v = J(ω)⁻¹·t`. |
| `compose(&self, other: &Self) -> Self` | `self ∘ other`. |
| `inverse(&self) -> Self` | Inverse `(Rᵀ, −Rᵀt)`. |
| `transform_point(&self, p: &Vec3) -> Vec3` | `R p + t`. |
| `integrate(&self, twist_body: &[f64; 6], dt: f64) -> Self` | `T ← T · exp(ξ dt)`, torseur dans le repère corps. Exacte pour ξ constant. |
| `integrate_batch(poses: &[Self], twists: &[[f64; 6]], dt: f64) -> Vec<Self>` | Intègre un lot (parallèle avec `rayon`, voir le seuil). **Panique** si les longueurs diffèrent. |
| `integrate_batch_in_place(poses: &mut [Self], twists: &[[f64; 6]], dt: f64)` | Variante en place. **Panique** si les longueurs diffèrent. |

---

### `linalg`

Algèbre linéaire 3×3 minimale, sans allocation.

```rust
pub type Vec3 = [f64; 3];
pub type Mat3 = [[f64; 3]; 3];
pub const MAT3_IDENTITY: Mat3;
```

| Fonction | Description |
|---|---|
| `dot(a, b) -> f64` | Produit scalaire. |
| `cross(a, b) -> Vec3` | Produit vectoriel. |
| `norm(a) -> f64` | Norme euclidienne. |
| `scale(a, s) -> Vec3` | Multiplication par un scalaire. |
| `mat_mul(a, b) -> Mat3` | Produit matrice × matrice. |
| `mat_vec(a, v) -> Vec3` | Produit matrice × vecteur. |
| `transpose(a) -> Mat3` | Transposée. |
| `det(a) -> f64` | Déterminant. |

---

## Feature `rayon`

```bash
cargo build --features rayon
```

Avec cette feature, `integrate_batch` et `integrate_batch_in_place` (SO(3) et SE(3)) s'exécutent en parallèle. Sans elle, le code est séquentiel et la dépendance n'est pas compilée.

**Seuil séquentiel.** Les lots de moins de **2048 éléments** restent séquentiels : à cette taille, réveiller le pool de threads coûte autant ou plus que le calcul (une intégration SO(3) dure environ 20 ns). Au-delà, le travail est découpé en tâches d'au moins 256 éléments.

Mesuré, avec un seuil de granularité seul (sans seuil séquentiel), 1 024 rotations s'intégraient **2,2 fois plus lentement** avec Rayon qu'en séquentiel. Le seuil de 2048 est un compromis : il correspond au point d'équilibre pour SO(3) et à un gain pour SE(3) (voir [Performances](#performances)). Il se règle dans `src/parallel.rs` (`PAR_THRESHOLD`), à ajuster avec `cargo bench --bench batch --features rayon`.

**Gros lots : préférez `integrate_batch_in_place`.** Le résultat d'un lot de 262 144 rotations pèse environ 19 Mo ; l'allouer devient le goulot, et la variante en place est alors nettement plus rapide.

---

## Performances

Mesures `criterion` en `--release` sur **une seule machine** (portable Linux). Les valeurs absolues dépendent du processeur ; les rapports se transfèrent mieux. Reproduisez-les chez vous (voir plus bas).

### Opérations unitaires

| Opération | Temps |
|---|---|
| `Rotation3::exp` (petit angle / moyen / proche de π) | 9,6 ns / 51 ns / 51 ns |
| `Rotation3::log` (petit / moyen / proche de π) | 27 ns / 38 ns / 65 ns |
| `compose` | 6,8 ns |
| `rotate` | 3,1 ns |
| `integrate` / `integrate_world` | 21,6 ns / 22,3 ns |
| `interpolate` | 158 ns |
| `to_quaternion` / `from_quaternion` | 10 ns / 10 ns |
| `renormalized` | 82 ns |
| `left_jacobian_inv` | 23 ns |
| `Pose3::exp` / `log` | 107 ns / 85 ns |
| `Pose3::compose` / `inverse` | 9,5 ns / 4,9 ns |
| `Pose3::integrate` | 59 ns |

### Lots : séquentiel contre `integrate_batch` (feature `rayon`)

Rapport de temps séquentiel / parallèle (supérieur à 1 = Rayon plus rapide) :

| Taille du lot | SO(3) | SE(3) |
|---|---|---|
| ≤ 1 536 | 1× (séquentiel, sous le seuil) | 1× (séquentiel) |
| 2 048 | ≈ 1× (équilibre) | ≈ 1,3× |
| 4 096 | ≈ 2,1× | ≈ 2,8× |
| 16 384 | ≈ 2,6× | — |
| 65 536 | ≈ 3,4× | ≈ 4× |
| 262 144 | ≈ 1,5× avec `integrate_batch`, ≈ 2,5× avec `integrate_batch_in_place` | — |

Le séquentiel tient à environ 50 millions de rotations par seconde (≈ 20 ns chacune).

### Reproduire

```bash
cargo bench --bench geometry                   # opérations unitaires
cargo bench --bench batch --features rayon     # lots, séquentiel contre parallèle
```

---

## Stabilité numérique

- **exp / Jacobiennes / inverses** : développements de Taylor quand θ² < 10⁻⁴, formules fermées sinon.
- **log** : `atan2` pour l'angle (précis partout), série près de 0, extraction de l'axe via la partie symétrique près de θ = π.
- **Dérive** : l'intégration exponentielle préserve l'orthogonalité à la précision machine pendant très longtemps, mais les erreurs d'arrondi s'accumulent. Appelez `renormalized()` périodiquement (par exemple toutes les quelques centaines de pas) sur les intégrations longues, et surveillez `orthogonality_error()`.

---

## Tests et validation

```bash
cargo test                         # bibliothèque seule
cargo test --features rayon        # lots parallèles
cargo test --release --features rayon
cargo test -p bevy-drone           # validation croisée contre glam (moteur de Bevy)
```

Les tests sont de trois familles.

**Propriétés aléatoires (`proptest`)**, 2 000 cas par propriété, avec les angles proches de 0 et de π sur-représentés :

- `tests/proptest_so3.rs` (18 tests) : `exp` toujours valide, aller-retour `exp` ↔ `log` sur [0, π], norme de `log` égale à l'angle, composition associative, conservation de la norme, quaternions (aller-retour, forme canonique, invariance au signe et à l'échelle), distance géodésique (symétrie, inégalité triangulaire), interpolation, équivalence entre `integrate` (corps) et `integrate_world` (monde) par conjugaison, `J(ω)·J(ω)⁻¹ = I`, `renormalized` sur des matrices perturbées. Cas déterministes à θ = π sur 9 axes, entrées `NaN`/infini sans panique.
- `tests/proptest_se3.rs` (13 tests) : aller-retour `exp` ↔ `log`, inverse, associativité, `transform_point` (cohérence avec la composition, isométrie), intégration à torseur constant, mouvement dans le repère corps, et **égalité bit à bit** entre `integrate_batch`, `integrate_batch_in_place` et la boucle séquentielle, y compris autour du seuil Rayon. Panics sur longueurs différentes et lots vides.

**Tests déterministes** (`tests/properties.rs`) : `exp`/`log`, `hat`/`vee`, quaternions, Jacobiennes, interpolation, exactitude de l'intégration à vitesse constante (SO(3) et SE(3)), dérive bornée sur 100 000 pas corrigée par `renormalized()`.

**Validation croisée** (crate `bevy-drone`) : comparaison avec `glam` pour la rotation d'un vecteur, la matrice, le quaternion et l'intégration sur 1 000 pas.

Sous Linux, Bevy requiert : `sudo apt install libasound2-dev libudev-dev`.

---

## Exemple Bevy

Un quadricoptère (corps, 4 bras, 4 moteurs, 4 hélices) vole sur une trajectoire intégrée avec `Pose3::integrate`, chaque hélice tourne avec `Rotation3::integrate`, et la caméra suit le drone.

```bash
cargo run -p bevy-drone --release
```

Les hélices avant sont rouges et celles de l'arrière blanches (l'avant est −Z). Le HUD affiche la dérive d'orthogonalité, la position et le lacet.

Bevy évolue vite et casse régulièrement son API. Pour la reproductibilité, fixez la version dans `examples/bevy-drone/Cargo.toml` :

```bash
cargo tree -p bevy-drone -i bevy --depth 0   # affiche la version résolue
```

---

## Structure du projet

```
tiny-rodriguez-std/
├── Cargo.toml
├── CHANGELOG.md
├── LICENSE
├── README.md
├── .github/workflows/ci.yml
├── src/
│   ├── lib.rs        # point d'entrée et ré-exports
│   ├── linalg.rs     # algèbre 3×3
│   ├── parallel.rs   # lots (séquentiel ou Rayon, seuil)
│   ├── so3.rs        # Rotation3
│   └── se3.rs        # Pose3
├── tests/
│   ├── common/mod.rs      # utilitaires partagés
│   ├── properties.rs      # tests déterministes
│   ├── proptest_so3.rs    # propriétés aléatoires SO(3)
│   └── proptest_se3.rs    # propriétés aléatoires SE(3) et lots
├── benches/
│   ├── geometry.rs   # opérations unitaires
│   └── batch.rs      # séquentiel contre parallèle
└── examples/
    └── bevy-drone/   # démo et tests de validation contre glam
```

---

## Feuille de route

Prévu pour les prochaines versions :

- Adjointe `Ad(T)` et `ad(ξ)`, Jacobienne droite de SO(3), Jacobiennes de SE(3).
- Opérateurs `⊞` / `⊟` (`plus` / `minus`) sur SO(3) et SE(3), pour les filtres de Kalman sur variété.
- Intégration d'ordre supérieur (Munthe-Kaas) pour une vitesse angulaire variable dans le pas.
- Seuils Rayon distincts pour SO(3) et SE(3) (le coût par élément diffère).
- Constructeurs faillibles (`try_from_quaternion`) pour ne pas masquer les entrées invalides.

---

## Licence

Distribué sous **GNU General Public License v2.0 ou ultérieure** (`GPL-2.0-or-later`). Voir le fichier [LICENSE](LICENSE).

Cette licence est copyleft : un programme qui incorpore cette bibliothèque doit être distribué sous une licence compatible GPL. Bevy (MIT/Apache-2.0) n'est utilisé que dans l'exemple, qui n'est pas publié.