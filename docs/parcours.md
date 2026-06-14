# Parcours de décision — `bl` (Search & Rescue autonome)

> Record **pédagogique** des décisions clés derrière `bl`, au format
> **Problème → Investigation → Décision → Pourquoi**. C'est l'épine dorsale
> « lessons learned » du rapport. Chaque piège technique est aussi détaillé dans
> [`ERRORS_AND_FIXES.md`](ERRORS_AND_FIXES.md). La version condensée (anglais) est
> dans le [README](../README.md#decision-journey--how-bl-got-here).

---

## 1. Fondations & architecture (L13–L14)

- **4 paquets ROS 2** (`rescue_bringup` / `rescue_robot` / `rescue_world` / `rescue_decision`).
  *Pourquoi :* séparation nette (lancement / autonomie / mondes / décision) → l'équipe travaille en parallèle.
- **Système de mocks** (`mock_map/coverage/victim`). *Pourquoi :* développer et tester le BT,
  les résultats et la visu **sans Gazebo** (rapide, CI) avant que la vraie stack existe.

## 2. SLAM + exploration autonome (L15)

- **Exploration par frontières** (Yamauchi) + `slam_toolbox` async (loop closure), cible ≥ 90 %.
  *Pourquoi :* baseline de l'énoncé (CM8).
- **Blacklist de frontières inatteignables.** *Problème :* le robot boucle indéfiniment sur une
  frontière que Nav2 n'atteint pas (bloqué à 53.8 %). *Décision :* blacklister une frontière quand
  Nav2 l'`ABORTED` **ou** qu'elle est re-sélectionnée sans gain de couverture ; **clé = coordonnée
  monde quantifiée**, pas la cellule de grille. *Pourquoi monde :* l'origine de la grille se décale
  quand SLAM grandit → une clé-cellule dériverait et une frontière blacklistée reviendrait en douce.

## 3. Décision (BT) + perception (L16)

- **BehaviorTree.CPP v3, pas une FSM** (interdite par l'énoncé). **ReactiveSequence**, pas
  `RetryUntilSuccessful` (busy-loop avec `num_attempts=-1`).
- **AprilTag `tag36h11`, pas YOLO** (l'énoncé exclut la détection humaine sophistiquée).
- **AprilTags voxel, pas textures PBR.** *Problème :* un `albedo_map` PBR rend **blanc** sous
  Ogre2+Mesa (WSL *et* cluster) → `apriltag_ros` ne voit rien. *Décision :* chaque tag = **100 boîtes
  colorées** + **anneau quiet-zone blanc** autour du marqueur natif **8×8**. *Pourquoi l'anneau/8×8 :*
  un redimensionnement 10×10 naïf distord le code et supprime la quiet-zone → le tag *s'affiche* mais
  est **indétectable** (#29).
- **Source unique de placement victimes** : `generate_rescue_arena.py` possède le monde (émet les
  `<include>` AprilTag) ; `generate_apriltag_models.py` ne construit que les **assets de modèle**.
- **Registre victimes via TF2** : chaque détection projetée `caméra → victim_<id> → map`, dédupliquée
  par ID, persistée dans `results/victims.json` (exigence « projeter dans le repère global »).

## 4. Transverse — environnement & infra

- **CycloneDDS sur WSL (obligatoire).** *Problème :* la découverte Fast-RTPS est instable sur WSL →
  les contrôleurs in-Ignition ne chargent jamais, `/turtlebot4/odom` reste muet. *Décision :* le profil
  `win` sélectionne CycloneDDS en loopback, **gardé** (repli Fast-RTPS si absent, #32).
- **Rendu GPU (D3D12) — ne JAMAIS forcer software GL.** *Problème :* `llvmpipe` est ~**23× plus lent**
  et sature le CPU → l'hôte reboote. *Décision :* profil `win` rend Ogre2 sur le **GPU D3D12 WSLg**
  (override GL 4.5 + adaptateur NVIDIA). *Note :* WSL n'a **pas** de GL/Vulkan NVIDIA natif (CUDA +
  D3D12 seulement ; CUDA ≠ rendu), #33.
- **Build/run sans conda.** *Problème :* le Python 3.13 de conda casse le build (`catkin_pkg`, lien
  ncurses) **et** le runtime (`rclpy`/numpy). *Décision :* `env -i …` ou `auto_activate_base false` (#31).
- **Source dans `bl/`, exécution dans `run/bl/`** (source publiée propre ; build/run depuis une copie
  synchronisée via `rsyncDown_bl_run.sh`).

## 5. Bonus exploration (L17)

- **Greedy vs information-gain.** IG = argmax `gain(f) − λ·cost(f)` (Stachniss, ICRA 2005) ; conforme
  à CM8 (H1 Proximity = greedy, H3 Combined = info_gain, coût = **chemin** A\*/Nav2 et non euclidien).
  Voir [`exploration_benchmark.md`](exploration_benchmark.md).
- **Benchmark resumable** *(leçon de la perte de données)*. *Problème :* un long benchmark (3×2 runs)
  fait rebooter l'hôte en plein milieu → tout perdu. *Décision :* chaque run persiste son
  `run_summary.json` + `run_status.json` ; au relancement, **les runs déjà valides sont sautés** (seuil
  « ≥90 % ou ≥250 s » pour distinguer un run complet d'un tronqué par reboot) ; le script ne purge
  jamais `experiments/`. *Pourquoi :* la campagne atteint mean±σ + `time_to_90` **en autant de reboots
  qu'il faut**. Résultat n=3 : info_gain 0.85±0.08 couverture (seule à franchir 90 %) vs greedy 0.72.

---

## 6. L18 — La saga des reboots WSL (stabilité de l'hôte)

> Cas d'école d'un diagnostic qu'on corrige au fur et à mesure des **mesures**, pas des intuitions.

| Étape | Problème / hypothèse | Investigation | Verdict |
|---|---|---|---|
| Fausse piste 1 | « C'est la mémoire qui sature » | `free` (WSL **5/19 GiB**), hôte **12 GB libres**, **VRAM 0.2/8 GB** | **Écarté** : aucune saturation |
| Fausse piste 2 | « C'est Windows Update » | Event log : `RebootRequired`=False, dernières KB = 25-31/05, pause Update sans effet | **Écarté** : pas de MAJ en attente |
| Vraie cause A | Crashes matériels sous charge | Bugchecks : **`0x116` VIDEO_TDR_FAILURE**, **`0x10e` VIDEO_MEMORY_MANAGEMENT**, **`0x101` CLOCK_WATCHDOG ×2** | Instabilité **GPU + CPU** sous charge soutenue |
| Vraie cause B | Redémarrages « tout seuls » | Event **1074** « Autre (non planifié) » via `StartMenuExperienceHost`, ~toutes les 47 min, aucune tâche planifiée de reboot active | Redémarrage **gracieux programmatique** (OEM MSI ?) — non résolu à distance (compte non-admin) |

**Décisions / fixes** (ce qu'on contrôle) :
- **Knob RTF** `IA712_GZ_RT_RATE` (défaut 70 → RTF 0.7) dans `generate_rescue_arena.py` : moins de
  rendus GPU **et** d'itérations physiques **par seconde réelle** → charge soutenue plus basse, cible
  directe des `0x116/0x10e/0x101`. *Pourquoi RTF et pas software GL :* on **reste sur le GPU** (×23),
  on ralentit juste la cadence ; le **temps simulé** (donc couverture, `time_to_90`) est inchangé.
- **RTF à chaud** via le service Gazebo `set_physics` (`real_time_factor`) → ralentir un run **sans le
  tuer** ni perdre sa progression.
- **`.wslconfig` allégé** : `processors 16→10`, `memory 20→14 GB` → marge thermique/RAM pour Windows+GPU
  (mesuré : WSL n'utilise que ~5 GB, baisser ne coûte rien). À appliquer par `wsl --shutdown` entre runs.
- **Côté Windows (admin requis, hors de notre portée)** : pilote **NVIDIA Studio** (corrige les TDR),
  registre **`TdrDelay`**, plan d'alim **Équilibré** + refroidissement (les `0x101` sont thermiques),
  et chercher l'auto-reboot dans **MSI Center**.

**Leçon :** mesurer (event log, `free`, `nvidia-smi`) avant de prescrire ; un bug « évident » (Update,
mémoire) peut être faux deux fois de suite.

---

## 7. L18 — Détecter les 4 victimes en UN run continu

> Chronologie des bugs successifs. Chaque fix a éliminé une cause… et révélé la suivante.

| # | Symptôme | Investigation | Décision (fix) | Pourquoi |
|---|---|---|---|---|
| 1 | Patrouille seule : robot **immobile** (path 0.16 m), 0 victime | Nav2 `Failed to make progress` ; waypoints en zone **inconnue** | **Mode hybride** : explorer d'abord (construit la carte), **puis** patrouiller | Sans carte, les goals des pièces sont en inconnu → Nav2 ne peut pas planifier |
| 2 | Waypoints `(0, ±3.2)` rejetés/bloquants | Géométrie : ils tombent **pile sur le mur vertical x=0** (cloison) | Waypoints en **espace libre** validés contre murs/cloisons/gravats | Un goal dans un mur est inatteignable par construction |
| 3 | `Failed to make progress` persistant | Hypothèse reflex Create3 (`REFLEX_STUCK`) | **`safety_override=full`** sur `/turtlebot4/motion_control` | Le reflex de sécurité de la base peut figer le robot ; `full` laisse Nav2 piloter |
| 4 | Coins profonds `(±4.6, ±4.7)` **rejetés instantanément** (0.1 s) même à 93 % | Les 7 % non explorés = les coins → cellule **inconnue** → goal rejeté avant planif | **Goal-snapping** (`nearest_free_cell`, le même que l'explorateur) + **waypoints centraux** `(±4.3, ±3.3)` + explore jusqu'à couverture cible | Snapper un goal sur la cellule **libre** la plus proche le rend acceptable ; des waypoints moins profonds sont cartographiés tôt |
| 5 | Snap OK mais **NW timeout / SW rejeté** ; 1 seul waypoint atteint | `safety_override` appliqué ✓, snap minuscule ✓, **1 seule** instance Nav2 ✓ → mais `bt_navigator: Goal failed` ×4 | **Pivot** : abandonner la patrouille rigide, **SpinAndScan 360°** pendant l'exploration | Le **routage inter-pièces via doorways** est structurellement fragile (pas de chemin global au moment du goal) ; or l'explorateur **visite déjà toutes les pièces** pour mapper 93 % |
| 6 | SpinAndScan ne captait qu'1 victime | `0.6 rad/s × 7 s = 4.2 rad ≈ **240°** seulement` | **`spin_scan_duration 7→12 s`** (> 360°, tour complet) | À chaque goal dans une pièce, balayer **tous les murs** quelle que soit l'orientation d'arrivée → capter le tag de façon fiable |
| 7 | Spin 360° **toujours 1 victime** ; couverture cale à **88 %** | L'explore stagne, les 12 % manquants = l'**intérieur des pièces** ; `inflation_radius 0.30` + `cost_scaling 3.0` rendent les abords des murs **coûteux** → NavFn reste dans les couloirs | **`inflation_radius 0.30→0.25`** + **`cost_scaling_factor 3.0→2.0`** | Le robot **n'entre pas** dans les pièces. Baisser l'inflation/coût → le planner plonge **dans** les pièces → **couverture 90 %+** atteinte (validé v5 : 26 m parcourus vs 14 m) |
| 8 | Couverture 93 %, robot dans les pièces, mais **0 victime** | apriltag : `camera_info` arrive mais **images = 0** ; `ign topic` montre que Gazebo **rend** (320×240) ; **RTF effondré à 0.05** → ~1,5 img/s mur → fenêtre de sync apriltag à **0 paire** | **Remonter le RTF** (`IA712_GZ_RT_RATE 70→100`, cible 1.0) — plus de crash, on peut accélérer | À RTF bas la **caméra (rendu GPU)** est affamée alors que `camera_info` (métadata, pas de rendu) continue → le spin tournait **dans le vide**. Ce n'était **ni la nav ni le spin** : la caméra manquait d'images |

| 9 | Caméra OK (RTF haut), couverture 96 %, robot **partout** (31 m), mais **toujours 1 victime** | Le tag 16 cm n'est détectable que **jusqu'à ~2 m** (`apriltag_tags.yaml`) ; les waypoints « centraux » étaient à **2,6 m** → hors portée. victim_0 capté seulement quand l'explore passe <2 m du tag NE | **Waypoints rapprochés à ~1,3 m** des tags (dans la portée) + **SmacPlanner2D (A\*)** pour les atteindre + **pause `dwell_sec`** face au mur pour qu'apriltag verrouille | Ce n'était pas « le robot ne va pas dans la pièce » (il y va) mais **« il ne s'approche pas assez du tag »** : 320×240 + tag 16 cm → portée ~2 m. Il faut se **poster à ~1,3 m** face au mur, ce que seul un bon routage (A\*) permet enfin |
| 10 | **3 victimes OK, jamais 4 + carte** en un run | **OOM à ~10 min** : `mem_profile.sh` prouve que **Gazebo fuit ~35 MB/s** (par temps mural, dans le rendu Ogre2/D3D12+WSL — **irréductible** : 8 Hz, RGB-seul sans depth/pointcloud, RTF +50 % testés, aucun effet). L'OOM **coupe la patrouille à 2/4** → 3 victimes seulement. (A\* SmacPlanner aussi **écarté** entre-temps : « Starting point in lethal space » → figé à 60 %, repli **NavFn**.) | **(a) Caméra OFF pendant l'exploration** : `always_on=0` + bridge caméra coupé en phase 1, relancé en phase 2 → **rendu nul pendant la phase longue** → la fuite n'accumule que pendant la patrouille courte → plus d'OOM. **(b) Retry des waypoints échoués** (2ᵉ passe) → atteint les 4. **(c) Sortie propre** du waypoint follower (carte sauvée avant tout OOM résiduel) | L'exploration navigue au **LIDAR** (pas besoin de la caméra) ; ne la rendre que pour la **détection** (patrouille) supprime la cause de TOUS les OOM. Le retry transforme « 2-3 victimes » en **4** (chaque waypoint manqué = une victime ratée). RTF n'aide pas (fuite par temps mural mais le run doit quand même finir avant le seuil) |

**Deux causes racines distinctes, démasquées en fin de parcours** : (7) la **fonction de coût Nav2**
décourageait le robot d'entrer dans les pièces cloisonnées (corrige la **couverture**) ; (8) un **RTF
effondré affamait le rendu caméra** (corrige la **détection**) — `camera_info` trompeur car il flue sans
rendu. Les bugs 1-6 traitaient des symptômes au-dessus de ces deux causes.

**État :** run nominal en validation, combinant tous les fix — **inflation 0.25** (entre dans les pièces),
**RTF suffisant** (caméra nourrie), **waypoints à ~1,3 m** des tags (dans la portée 2 m, bug #9),
**SmacPlanner2D (A\*)** pour les atteindre, **dwell 4 s** face au mur, et **mémoire 20 GB** (l'OOM à
14 GB effondrait le RTF). Meilleur run mesuré à ce jour : **93,5 % couverture + 2 victimes** (id 0, 3,
vrais IDs) ; objectif L18 = **≥90 % + ≥3 victimes en un run continu** + carte finale annotée (auto).

---

## 8. Bugs d'outillage (transverses, instructifs)

- **`pkill -f` qui s'auto-tue.** *Problème :* un nettoyage inline `pkill -9 -f "ign gazebo|…"` matchait
  la **ligne de commande du shell wrapper** (qui contient ce motif en texte) → il se tuait lui-même
  (log vide, exit 1), faussement attribué à un crash sim. *Décision :* nettoyer via le script dédié
  `kill_sim.sh` (dont le wrapper ne contient pas le motif), **séparé** de la commande de lancement.
- **`rsyncDown_*_run` : perte de données.** *Problème :* `rsync --delete` n'excluait pas `experiments/`
  → un down-sync source→run écrasait les résultats de benchmark **générés** (coûteux). *Décision :*
  `--exclude='experiments/'` sur les 4 scripts ; flux artefacts = **run → branche**, jamais l'inverse.
- **`.wslconfig memory` trop bas → OOM → RTF effondré.** *Problème :* en croyant gagner de la marge
  thermique, `memory` a été baissé `20→14 GB`. Mais le sim TB4 complet (gazebo + ~28 bridges + Nav2 +
  SLAM + caméra) **+ des bridges orphelins** des runs précédents (kill_sim incomplet) ont saturé la RAM
  → **swap plein → RTF effondré à 0.014** (sim figée, caméra ré-affamée, patrouille en timeout). Faux
  diagnostic possible : « ça re-reboote ». *Décision :* **`memory=20 GB`** (le TB4 a besoin de headroom),
  ne réduire que `processors` (thermique) ; **toujours partir d'un état propre** (orphelins tués) avant
  un run. *Leçon :* un changement de conf « sans risque » (mesuré ~5 GB au repos) coûte cher sous **charge
  réelle** + accumulation.

---

## 9. Pistes algorithmiques pour le « chemin emprunté » (CM9 / CM10)

Le **parcours réel du robot dans la map** = `global planner` (chemin géométrique) + `controller`
(suivi local). Pistes ancrées dans le cours, à **comparer/pratiquer** pour enrichir le rapport et
améliorer le chemin emprunté.

### 9.1 Global planner — le CHEMIN (CM9, « Path Planning »)
| Algo | Plugin Nav2 | Propriété (CM9) | Statut `bl` |
|---|---|---|---|
| **Dijkstra** | `nav2_navfn_planner/NavfnPlanner` | optimal mais explore **tout** → coûteux, chemins collés aux obstacles ; **tolérant** (planifie même depuis l'inflation) | ✅ **ACTIF (choisi)** — robuste dans cette arène |
| **A\*** | `nav2_smac_planner/SmacPlanner2D` | heuristique `f=g+h` → optimal **et plus rapide** (CM9), chemins plus directs/lisses, mais **strict** | ⚠️ **TESTÉ → ÉCARTÉ (L18)** : refuse de planifier dès que le robot est en zone inflatée (« **Starting point in lethal space!** ») → **robot figé à 60 %**, plus aucune frontière atteignable. Repli NavFn commenté pour rejouer |

**Résultat de comparaison NavFn vs SmacPlanner2D (mesuré, L18)** : dans une arène **cloisonnée** où le
robot longe souvent les murs, la position courante tombe régulièrement en **zone inflatée** (« lethal »
pour le planner, car `inflation_radius 0.25 ≈ robot_radius 0.22`). **SmacPlanner2D refuse** alors tout
plan (start lethal) → robot bloqué à 60 %. **NavFn tolère** ce cas et planifie quand même → 88-96 %.
**Conclusion** : pour ce monde, **NavFn est plus robuste** ; A\* (plus direct/lisse en théorie) n'est
exploitable qu'avec un costmap moins agressif (inflation plus large) ou un controller qui évite de
coller aux murs. C'est un vrai résultat pour le rapport (pas juste « A\* > Dijkstra » sur le papier).
| **RRT** | custom / `nav2_smac_planner` Hybrid | très rapide, **non-optimal** (CM9 Algo 3) ; le « deuxième algorithme » cité par le prof | outlook rapport |

→ **Fait (L18)** : comparaison NavFn vs SmacPlanner2D **réalisée** (voir le résultat ci-dessus) →
**NavFn conservé** (SmacPlanner trop strict ici). Extension rapport : quantifier sur un costmap moins
agressif (inflation 0.35-0.40) où A\* deviendrait exploitable, et mesurer longueur de chemin / `time_to_90`.

### 9.2 Controller — le SUIVI (CM10, « local planner »)
| Algo | Plugin Nav2 | Propriété | Statut `bl` |
|---|---|---|---|
| **DWA/DWB** | `dwb_core::DWBLocalPlanner` | échantillonne des trajectoires, scoring par *critics* | **actuel** |
| **Regulated Pure Pursuit** | `nav2_regulated_pure_pursuit_controller` | suit le chemin à vitesse régulée → **plus lisse** pour diff-drive, moins d'oscillation | à essayer |
| **MPPI / TEB** | `nav2_mppi_controller` | optimisation de trajectoire, plus lourd | outlook |

→ **Recommandation pratique #2** : tester **Regulated Pure Pursuit** (adapté au diff-drive TB4) →
suivi plus lisse, pourrait réduire les `Failed to make progress` aux doorways (bug #5).

### 9.3 Exploration — la STRATÉGIE (CM8, bonus L17)
Déjà comparé en n=3 : **greedy** vs **information-gain** (cf. [`exploration_benchmark.md`](exploration_benchmark.md)).
Extension possible : **RRT-exploration** (`rrt_exploration_ros2`) en *outlook* du rapport — montre la
diversité algorithmique sans voler du temps au baseline (décision projet d'origine).

**Synthèse :** pour améliorer le **parcours réel**, le levier #1 le plus rentable est le **global
planner A\* (SmacPlanner2D)** ; pour la **robustesse du suivi**, le **Regulated Pure Pursuit**. Les
deux sont des swaps de plugin Nav2 (peu de code), idéaux pour une comparaison quantitative au rapport.

## 10. Leçons transverses

1. **Mesurer avant de prescrire** : event log Windows, `free`, `nvidia-smi`, `run_status.json` ont
   chacun retourné un diagnostic « évident » faux.
2. **Un fix révèle le bug suivant** : la détection des victimes a traversé 6 causes empilées (carte,
   géométrie, reflex, validité de goal, routage, balayage caméra).
3. **Robuste > rigide** : l'exploration adaptative + spin 360° bat la patrouille de waypoints figés dans
   une arène cloisonnée.
4. **Idempotence/résilience** : runs resumables + artefacts persistés survivent aux reboots de l'hôte.
