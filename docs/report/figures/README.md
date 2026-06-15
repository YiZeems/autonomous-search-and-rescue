# Figures & vidéo pour le rapport / la présentation L18

Générées depuis le run nominal `results/examples/l18_nominal/` (mission autonome
2 phases : **4 victimes + 97,5 % de couverture + carte propre**).

| Fichier | Usage rapport |
|---|---|
| `mission_map.png` | **Figure principale** : carte SLAM finale + les 4 victimes (étoiles) + la tournée d'inspection autonome (flèches dérivées de la carte). Illustre le résultat ET la conformité (poses ≠ positions victimes). |
| `coverage_curve.png` | Couverture(t) + distance parcourue, ligne objectif 90 %. Montre la mission en 2 phases (montée explo → plateau → inspection). |
| `annotated_map_hd.png` | Le **livrable énoncé** (« carte finale marquée avec les positions des victimes »), version agrandie de `final_map_annotated.png`. |
| `mission_replay.mp4` | **Vidéo replay** de la mission (couverture mesurée, victimes révélées au fil du temps). Pour la présentation / le rendu. |

## Régénérer (réutilisable pour tout run)
```bash
python3 scripts/make_report_figures.py <results_dir> docs/report/figures
python3 scripts/make_mission_video.py  <results_dir> docs/report/figures/mission_replay.mp4
```

## Note capture live (RViz / Gazebo)
La capture d'écran **programmatique** de RViz/Gazebo échoue sous **WSLg** : les
fenêtres sont rendues via Wayland (weston), pas sur la racine X11 que `ffmpeg
x11grab` capture (→ écran noir). RViz s'affiche normalement en **interactif** ; pour
une vidéo *live* de la démo, enregistrer côté **Windows** (OBS / Xbox Game Bar) pendant
un run avec `IA712_RVIZ=1`. Les figures ci-dessus sont produites depuis les **données**
(indépendantes de l'affichage) — plus propres et reproductibles pour un rapport.
