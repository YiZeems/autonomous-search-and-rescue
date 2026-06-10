# Équipe & répartition des rôles

> Affectation initiale proposée — à challenger / valider en L13/L14.

## Composition

| Nom            | Email                              |
| -------------- | ---------------------------------- |
| Julien GIMENEZ | julien.gimenez@telecom-paris.fr    |
| Hugo FANCHINI  | hugo.fanchini@telecom-paris.fr     |
| Paul CINTRA    | paul.cintra@telecom-paris.fr       |
| Yimou ZHANG    | yimou.zhang@telecom-paris.fr       |

## 4 rôles × 18 h en classe

Découpage proposé par [pistes_projet-b.md §6](../../doc/orig/pistes_projet-b.md) :

| Rôle                          | Responsabilités principales                                                                                     | Paquets ROS 2 owned                                  |
| ----------------------------- | --------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------- |
| **R1 — Infra & launch**       | Workspace, `bringup.launch.py`, RViz config, Dockerfile, monde Gazebo + AprilTags, README installation          | `team_b_bringup`, `team_b_world`                     |
| **R2 — SLAM & exploration**   | `slam_toolbox` tuning, `frontier_greedy_node`, comparaison loop-closure on/off, support du `coverage_evaluator` | `team_b_exploration` (greedy), assistance `team_b_metrics` |
| **R3 — Perception & TF**      | Intégration `apriltag_ros`, `victim_registry_node`, projection TF, markers RViz, persistance JSON               | `team_b_perception`                                  |
| **R4 — Décision & bonus**     | Behavior Tree (XML + nodes C++), `information_gain_node`, `benchmark_runner`, plots matplotlib, rédaction rapport | `team_b_decision`, `team_b_exploration` (info_gain), `team_b_metrics` |

## Synchronisation

- Stand-up de 10 min en début de chaque séance.
- Revue commune en fin de séance sur le bringup global.
- Pair-programming systématique, R1 fait office de fallback intégrateur si membre absent.

## Affectation nominative

_À remplir en séance L13 puis validée en L14._

| Rôle | Personne |
| ---- | -------- |
| R1   | _(TBD)_  |
| R2   | _(TBD)_  |
| R3   | _(TBD)_  |
| R4   | _(TBD)_  |
