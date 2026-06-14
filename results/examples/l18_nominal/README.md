# Run nominal L18 — preuve pour le rapport

Run autonome continu de référence (`./scripts/run.sh demo-tb4` en mode hybride),
sur `rescue_arena`, headless GPU. Artefacts figés ici pour le rapport et la démo.

| Métrique | Valeur |
|---|---|
| Couverture finale | **90,6 %** (objectif ≥ 90 % ✅) |
| Victimes détectées | **3** — AprilTag **ids 0, 2, 3**, projetées dans `map` via TF2 |
| `success_coverage_90` | true |
| Durée (depuis Nav2) | 5 min 51 s |

## Fichiers
- `victims.json` — victimes (id + position `map`), produit par `victim_registry_node`.
- `run_summary.json` — couverture, `time_to_*`, longueur de chemin, ids victimes.
- `coverage_over_time.csv` — couverture(t).
- `final_map.yaml` / `final_map.pgm` — carte SLAM finale (`map_saver`).
- `final_map_annotated.png` — **carte finale annotée avec les victimes** (livrable
  énoncé), générée par `scripts/annotate_map.py`.

## Notes
- Les **4 victimes (ids 0,1,2,3) sont toutes détectables** (union des runs) ; ce run
  en capte 3 en continu. La patrouille atteint 2-3/4 pièces de façon fiable.
- Run stable (zéro OOM) grâce au **lidar `gpu_lidar` ramené à 10 Hz** + caméra coupée
  pendant l'exploration : la fuite mémoire du rendu GPU Ignition (Ogre2/D3D12+WSL) est
  ainsi contenue. Détails : [`docs/parcours.md`](../../docs/parcours.md) (bug #10).
