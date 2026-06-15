# Figures & vidéo pour le rapport / la présentation L18

Générées depuis le run nominal `results/examples/l18_nominal/` (mission **orchestrée
par le Behavior Tree** : **4 victimes + 97 % de couverture + carte propre**).

| Fichier | Usage rapport |
|---|---|
| `mission_map.png` | **Figure principale** : carte SLAM finale + les 4 victimes (étoiles) + la tournée d'inspection autonome (flèches dérivées de la carte). Résultat ET conformité (poses ≠ positions victimes). |
| `trajectory_map.png` | **Parcours réellement emprunté** (coloré par le temps) + **buts de frontières** `info_gain` (éléments d'algo) + poses d'inspection + victimes. Montre l'évolution du parcours. |
| `coverage_curve.png` | Couverture(t) + distance parcourue, ligne objectif 90 %. Montre la mission en 2 phases (explo → inspection). |
| `annotated_map_hd.png` | Le **livrable énoncé** (« carte finale marquée avec les positions des victimes »), version agrandie. |
| `mission_replay.mp4` | **Vidéo replay** (data-driven) : robot animé le long du parcours + carte + frontières + victimes révélées au fil du temps. Pour la présentation. |
| `rviz_capture.mp4` | **Capture vidéo de la vraie sortie RViz** (algos en action) sur toute la mission : carte SLAM construite en direct, frontières, poses d'inspection, plan Nav2, LaserScan, 4 marqueurs victimes. Vue large (toute l'arène). |
| `rviz_live.png` | Screenshot RViz (fin de mission) : carte complète + 4 victimes + éléments d'algo. |

> Note (rendu du parcours) : le parcours est la **vraie trajectoire** échantillonnée à 2 Hz
> en frame `map` (`trajectory.csv`), tracée en **ligne continue**. Les **murs sont dessinés
> PAR-DESSUS** le tracé (occlusion) : comme le parcours est à 100 % sur cellules libres
> (vérifié : **0/568 segment ne traverse un mur**), le masquer par les murs ne retire rien,
> mais garantit qu'il **ne peut jamais apparaître sur un mur** — il n'est visible que dans
> les trouées, donc il **contourne visiblement les murs (passe par les portes)**.

## Régénérer (réutilisable pour tout run)
```bash
# le 3e argument (exploration.log) ajoute les buts de frontières aux figures/vidéo
python3 scripts/make_report_figures.py <results_dir> docs/report/figures [exploration.log]
python3 scripts/make_mission_video.py  <results_dir> docs/report/figures/mission_replay.mp4 [exploration.log]
```

## Capturer la vraie sortie RViz en vidéo (sous WSLg)
La capture directe de la racine X11 `:0` échoue sous **WSLg** (rendu Wayland → écran
noir ; `grim`/`wf-recorder` non supportés). La solution qui marche : un **serveur X
virtuel** (`Xvfb`) + RViz en **GL logiciel** + `ffmpeg x11grab`. Deux pièges résolus :
- **RMW** : la sim utilise `rmw_cyclonedds_cpp` (+ `CYCLONEDDS_URI` sur `lo`). RViz doit
  être lancé avec le **même** RMW, sinon il ne reçoit aucun topic (carte vide).
- **Vue** : sous Xvfb il n'y a pas de gestionnaire de fenêtres ; régler `Scale` du
  `TopDownOrtho` dans le `.rviz` (~20) pour que **toute l'arène** tienne dans la vue.

```bash
Xvfb :99 -screen 0 1280x800x24 &
DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>\
<NetworkInterface name="lo" multicast="true"/></Interfaces></General></Domain></CycloneDDS>' \
  rviz2 -d <project_view.rviz> --ros-args -p use_sim_time:=true &
ffmpeg -f x11grab -framerate 12 -video_size 720x800 -i :99+0,0 -pix_fmt yuv420p rviz_capture.mkv
```
`rviz_capture.mp4` / `rviz_live.png` ci-dessus ont été produits ainsi. Alternative
*live* interactive : enregistrer côté **Windows** (OBS) pendant un run `IA712_RVIZ=1`.
Les figures `.png`, elles, viennent des **données** (indépendantes de l'affichage) —
plus propres et reproductibles.
