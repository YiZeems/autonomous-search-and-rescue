# Run nominal L18 — preuve pour le rapport (mission orchestrée par le BT, 4 victimes)

Run autonome continu de référence (`./scripts/run.sh demo-tb4`, monde `rescue_arena`,
headless GPU). Artefacts figés ici pour le rapport et la démo.

| Métrique | Valeur |
|---|---|
| Couverture finale | **97,0 %** (objectif ≥ 90 % ✅) |
| Victimes détectées | **4 / 4** — AprilTag **ids 0, 1, 2, 3**, projetées dans `map` via TF2 |
| `success_coverage_90` | true |
| `success_victims_all` | true |
| Durée (depuis Nav2) | ~4 min 50 s |

## Mission orchestrée par le Behavior Tree — 100 % autonome, conforme à l'énoncé
La **décision** est prise par le BT (`rescue_decision`, BehaviorTree.CPP v3, pas une FSM) :
une `Sequence` `WaitForMap → ExplorePhase → InspectPhase → VictimsFound → /mission_done`.
Le BT décide **quand** chaque phase démarre/s'arrête via les topics `/mission/*` ; les
nœuds Python exécutent :
1. **`ExplorePhase`** → active `frontier_explorer_node` : le robot cartographie un
   environnement **inconnu**, génère ses buts depuis la carte SLAM en direct (aucun
   waypoint pré-enregistré). Le BT le stoppe à ≥ 90 % de couverture.
2. **`InspectPhase`** → déclenche `inspection_node` : il dérive de la **carte que le robot
   vient de construire** (`inspection_planner.poses_from_grid`) une pose d'inspection par
   pièce extérieure (orientée **face au mur**, snappée sur cellule libre, boucle de
   périmètre — voir [`inspection_waypoints.yaml`](inspection_waypoints.yaml)) et les balaie
   → capte les 4 tags muraux. Signale `/mission/inspect_done` au BT.

> Conformité : **zéro coordonnée de victime, zéro waypoint écrit à la main** ; la décision
> est dans le BT, l'entrée de la phase 2 est la carte du robot. Détails et alternatives
> écartées : [`docs/parcours.md`](../../docs/parcours.md) §7bis–§7quater.

## Figures / vidéo (rapport)
Voir [`docs/report/figures/`](../../docs/report/figures/) : carte mission, courbe de
couverture, **parcours réellement emprunté + buts de frontières + victimes**, et la vidéo
replay. Régénérables via `scripts/make_report_figures.py` / `make_mission_video.py`.

> **Conformité** : aucune coordonnée de victime, aucun parcours écrit à la main dans le
> code. L'entrée de la phase 2 est **la carte SLAM**, pas la position des victimes. C'est
> de la recherche autonome systématique (inspection du périmètre découvert), pas « aller
> aux victimes ». Détails et alternatives écartées : [`docs/parcours.md`](../../docs/parcours.md) §7bis–§7quater.

## Fichiers
- `victims.json` — 4 victimes (id + position `map`), produit par `victim_registry_node`.
- `run_summary.json` — couverture, `time_to_*`, longueur de chemin, ids victimes.
- `coverage_over_time.csv` — couverture(t).
- `inspection_waypoints.yaml` — les 4 poses d'inspection **dérivées de la carte** (regénérées à chaque run).
- `final_map.yaml` / `final_map.pgm` — carte SLAM finale (`map_saver`).
- `final_map_annotated.png` — **carte finale annotée avec les 4 victimes** (livrable énoncé),
  générée par `scripts/annotate_map.py`.

## Notes de robustesse
- Run **stable, zéro OOM** : lidar `gpu_lidar` ramené à **10 Hz** (le capteur qui fuit le
  plus sous Ogre2/D3D12+WSL) → la fuite mémoire du rendu GPU est contenue à la source.
- **Carte propre (zéro dérive)** : poses d'inspection **face au mur** → un demi-tour
  suffit (vs un tour complet) → moitié moins de rotation sur place → la dérive SLAM
  rotationnelle qui dégradait la carte (cf. parcours §7ter) est évitée. Un **environnement
  WSL fraîchement redémarré** aide aussi (après ~10 runs, l'état GPU/WSL se dégrade).
