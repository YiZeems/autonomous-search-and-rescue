# Erreurs rencontrées et solutions

Répertoire des problèmes rencontrés pendant le développement et les tests de simulation TurtleBot4 sur ARM64 (Parallels macOS), avec les solutions appliquées.

---

## 1. Ignition Gazebo GUI crash — Ogre2 / vGPU ARM64

**Symptôme**  
`ign gazebo` crash immédiat avec `qTerminate()` ou segfault Qt5. Le plugin `Turtlebot4Hmi` appelle Ogre2 qui requiert Vulkan/OpenGL 3.3+.

**Cause**  
Le GPU virtuel Parallels (`1ab8:0010`) ne supporte pas OpenGL 3.3. Ogre2 (renderer par défaut d'Ignition) est incompatible.

**Solution**  
Lancer Ignition en mode serveur headless uniquement :
```bash
ign gazebo -s -r -v 2 world.sdf
```
RViz2 assure toute la visualisation côté ROS 2.

---

## 2. Race condition `ign_ros2_control::getURDF()` — `std::logic_error`

**Symptôme**  
`gz_ros2_control::GazeboSimROS2ControlPluginPrivate::getURDF()` lève `std::logic_error` au démarrage. Les contrôleurs (`diffdrive_controller`, `joint_state_broadcaster`) ne se chargent pas.

**Cause**  
Le plugin `ign_ros2_control` s'exécute à l'intérieur d'Ignition et tente de lire `/robot_description` dès que le robot est injecté dans la simulation. Si `robot_state_publisher` n'est pas encore prêt, le topic est vide → exception.

**Solution**  
Séquencer le démarrage :
1. `robot_state_publisher` en premier → attendre 6 s
2. Vérifier que `/turtlebot4/robot_description` est publié
3. Seulement ensuite : démarrer Ignition + spawner le robot

---

## 3. `libign_ros2_control-system.so: cannot open shared object file`

**Symptôme**  
Ignition démarre mais le plugin `ign_ros2_control` ne se charge pas.

**Cause**  
Hors d'un contexte `ros2 launch`, le chemin des plugins ROS 2 n'est pas dans `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`.

**Solution**  
```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
```

---

## 4. `ros2 run rescue_robot <node>` — "No executable found"

**Symptôme**  
`ros2 run rescue_robot mock_map_publisher` échoue avec "No executable found".

**Cause**  
`setup.cfg` contenait le nom de l'ancien paquet (`ia712_search_rescue`) au lieu de `rescue_robot` :
```ini
# AVANT (bugué)
script_dir=$base/lib/ia712_search_rescue
install_scripts=$base/lib/ia712_search_rescue
```

**Solution**  
```ini
# APRÈS
script_dir=$base/lib/rescue_robot
install_scripts=$base/lib/rescue_robot
```
Puis rebuild :
```bash
rm -rf build/rescue_robot install/rescue_robot
colcon build --symlink-install --packages-select rescue_robot
```

---

## 5. TF namespace split — `/turtlebot4/tf` vs `/tf` global

**Symptôme**  
`diffdrive_controller` tourne dans le namespace `/turtlebot4` et publie ses transforms sur `/turtlebot4/tf`. SLAM Toolbox et RViz2 écoutent `/tf` → le LaserScan MessageFilter drop toutes les scans ("queue is full").

**Cause**  
En ROS 2, les topics relatifs sont préfixés par le namespace du nœud. `/turtlebot4/tf` ≠ `/tf` global.

**Solution**  
Nœud relay Python `tf_relay_node.py` :
```python
# QoS critique : TRANSIENT_LOCAL subscriber pour matcher le publisher diffdrive
_SUB_QOS = QoSProfile(reliability=RELIABLE, durability=TRANSIENT_LOCAL)
_PUB_QOS = QoSProfile(reliability=RELIABLE, durability=VOLATILE)
```
> **Piège Fast-DDS** : un subscriber `VOLATILE` ne reçoit pas les messages d'un publisher `TRANSIENT_LOCAL` dans Fast-DDS (contrairement au standard DDS). Le subscriber doit être `TRANSIENT_LOCAL`.

---

## 6. QoS incompatibilité Fast-DDS — TRANSIENT_LOCAL publisher + VOLATILE subscriber

**Symptôme**  
Le relay TF ne reçoit aucun message malgré un publisher actif. Pas d'avertissement explicite dans les logs.

**Cause**  
Fast-DDS (le middleware RMW par défaut de ROS 2 Humble) n'est pas conforme au standard DDS sur ce point : un subscriber `BEST_EFFORT` ou `VOLATILE` n'obtient pas les messages d'un publisher `RELIABLE + TRANSIENT_LOCAL`, même si le standard DDS dit que c'est compatible (le subscriber demande moins que ce que le publisher offre).

**Solution**  
Le subscriber du relay doit déclarer exactement `RELIABLE + TRANSIENT_LOCAL` pour matcher le publisher `diffdrive_controller`.

---

## 7. Frames TF non-namespaced dans robot_state_publisher

**Symptôme**  
`robot_state_publisher` dans le namespace `/turtlebot4` publie sur `/tf_static` global, mais avec des frames sans préfixe (`base_link → rplidar_link`). Le frame du scan Ignition est `turtlebot4/rplidar_link/rplidar`. SLAM ne peut pas relier les deux arbres TF.

**Cause**  
RSP publie le nom des frames tel que défini dans l'URDF (`base_link`, `rplidar_link`). L'Ignition SDF nomme les frames avec le préfixe du modèle (`turtlebot4::rplidar_link::rplidar` → bridgé en `turtlebot4/rplidar_link/rplidar`).

**Solution**  
Deux transforms statiques identity (bridges entre les deux arbres) :
```bash
# turtlebot4/base_link ≡ base_link
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id turtlebot4/base_link --child-frame-id base_link

# rplidar_link ≡ turtlebot4/rplidar_link/rplidar
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id rplidar_link --child-frame-id turtlebot4/rplidar_link/rplidar
```

---

## 8. `local_setup.bash` échoue sur NFS Parallels

**Symptôme**  
```
no such file or directory: /media/psf/.../setup.sh
```
`source install/local_setup.bash` échoue dans les sous-shells (`bash -c`, heredocs). Les paquets `rescue_robot` ne sont pas trouvés via `ros2 pkg list`.

**Cause**  
Le script `local_setup.bash` utilise `$(builtin cd "$(dirname "${BASH_SOURCE[0]}")")` pour résoudre son propre chemin. Sur un dossier NFS Parallels (`/media/psf/`), la résolution de symlinks échoue dans les sous-processus.

**Solution**  
Contourner `local_setup.bash` en définissant les variables directement :
```bash
export AMENT_PREFIX_PATH="${WS}/install/rescue_robot:${WS}/install/rescue_bringup:..."
export PYTHONPATH="${WS}/build/rescue_robot:${PYTHONPATH}"  # pour rescue_robot.egg-info
export IA712_SLAM_PARAMS="/abs/path/to/slam_params_tb4.yaml"
export IA712_NAV2_PARAMS="/abs/path/to/nav2_params_tb4.yaml"
```
Le launch file `navigation_tb4.launch.py` lit `IA712_SLAM_PARAMS` en priorité pour éviter `FindPackageShare("rescue_robot")`.

---

## 9. `importlib.metadata.PackageNotFoundError: rescue-robot`

**Symptôme**  
`ros2 run rescue_robot waypoint_follower_node` échoue avec :
```
PackageNotFoundError: No package metadata was found for rescue-robot
```

**Cause**  
Le script d'exécution généré par `setuptools` (`install/rescue_robot/lib/rescue_robot/waypoint_follower_node`) appelle `pkg_resources.load_entry_point('rescue-robot', ...)` qui cherche le `rescue_robot.egg-info` dans `sys.path`. Ce répertoire se trouve dans `build/rescue_robot/` mais n'est pas dans `PYTHONPATH`.

**Solution**  
```bash
export PYTHONPATH="${WS}/build/rescue_robot:${PYTHONPATH}"
```

---

## 10. SLAM map figée à 7×7 cellules (modèle `lite`)

**Symptôme**  
La carte SLAM reste à 7×7 cellules (0.35 m × 0.35 m) quelle que soit la durée d'attente. Le robot ne voit aucun obstacle.

**Cause**  
Le modèle `turtlebot4_lite` a un problème de self-hit RPLIDAR dans Ignition Gazebo : tous les rayons retournent `range_min` (0.164 m) car les rayons frappent le corps du robot.

**Solution**  
Utiliser le modèle `standard` :
```bash
MODEL=standard ./scripts/run.sh demo-tb4
```
Vérification : `ign topic -e -t /world/maze/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan -n 1` doit montrer des `ranges` > 0.164.

---

## 11. Nav2 — `worldToMap failed: goal off the global costmap`

**Symptôme**  
```
worldToMap failed: mx,my: 33,33, size_x,size_y: 7,7
The goal sent to the planner is off the global costmap.
```

**Cause 1** — Costmap avec `width: 20, height: 20` explicite : les waypoints à ±1.5 m dépassent le costmap de 1 m × 1 m.

**Solution 1** : Supprimer `width`/`height` du global costmap pour qu'il utilise la taille de la carte SLAM.

**Cause 2** — Costmap non encore initialisé quand les goals sont envoyés (SLAM map = 7×7 au démarrage).

**Solution 2** : Attendre que la map SLAM soit suffisamment grande avant d'envoyer le premier waypoint. Vérifier `ros2 topic echo /map | grep width`.

---

## 12. Nav2 BT — `Node not recognized: RemovePassedGoals`

**Symptôme**  
```
[bt_navigator]: Exception when loading BT: Error at line 12: Node not recognized: RemovePassedGoals
[lifecycle_manager]: Failed to bring up all requested nodes. Aborting bringup.
```

**Cause**  
Le fichier BT `navigate_through_poses_w_replanning_and_recovery.xml` utilise des nœuds `RemovePassedGoals` et `ComputePathThroughPoses` qui ne sont pas dans la liste `plugin_lib_names` du bt_navigator.

**Solution**  
Ajouter dans `nav2_params_tb4.yaml` :
```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_remove_passed_goals_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
```

---

## 13. `behavior_server` — `No Transform available: "odom" does not exist`

**Symptôme**  
```
[behavior_server]: No Transform available Error looking up target frame: "odom"
```
Malgré `global_frame: turtlebot4/odom` dans `recoveries_server`.

**Cause**  
Dans Nav2 Humble, le nœud de recovery est nommé `behavior_server` (pas `recoveries_server`). La configuration YAML doit avoir une section `behavior_server:` séparée.

**Solution**  
Ajouter une section `behavior_server:` dans `nav2_params_tb4.yaml` avec `global_frame: turtlebot4/odom`.

---

## 14. Waypoint follower — timer ne se déclenche jamais (`use_sim_time=true`)

**Symptôme**  
`WaypointFollower ready — N waypoints` s'affiche mais rien ne se passe ensuite. Le nœud attend indéfiniment.

**Cause**  
`create_timer(1.0, callback)` avec `use_sim_time=true` s'appuie sur l'horloge de simulation. Si l'horloge est pausée ou trop lente au démarrage, le timer ne se déclenche jamais.

**Solution**  
Forcer le timer à utiliser l'horloge murale (wall clock) :
```python
self._start_timer = self.create_timer(
    1.0, self._start_once,
    clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME),
)
```

---

## 15. Waypoint follower — action `send_goal_async` ne reçoit jamais l'accusé de réception

**Symptôme**  
`spin_until_future_complete(self, future, timeout_sec=10.0)` timeout systématiquement. Nav2 accepte le goal (log : `Received a goal, begin computing control effort`) mais le client ne reçoit pas la confirmation.

**Cause**  
Le nœud utilise un `SingleThreadedExecutor` (via `rclpy.spin(node)`). `_run()` est appelé depuis un callback de timer qui bloque l'executor. `spin_until_future_complete()` crée un second executor sur le même nœud — les callbacks de réponse action ne peuvent pas être traités.

**Solution**  
Utiliser un `MultiThreadedExecutor` + thread dédié pour la navigation :
```python
# main()
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()

# __init__()
self._nav_thread = threading.Thread(target=self._run, daemon=True)
self._nav_thread.start()

# _send_goal() : poll time.time() au lieu de spin_until_future_complete
deadline = time.time() + 30.0
while not future.done() and time.time() < deadline:
    time.sleep(0.05)
```

---

## 16. Ignition simulation gèle sous charge prolongée

**Symptôme**  
Après plusieurs minutes de navigation avec SLAM + Nav2 + RViz2 simultanément, `/clock` cesse de publier. Tous les nœuds `use_sim_time=true` se figent.

**Cause**  
Les behaviors de recovery Nav2 (`spin`, `backup`) envoient des commandes `cmd_vel` prolongées. Sur ARM64 Parallels sous forte charge CPU, le plugin `ign_ros2_control` (exécuté dans la boucle physics d'Ignition) se bloque en attendant une réponse ROS 2 qui n'arrive pas → deadlock → la simulation gèle.

**Solution**  
Désactiver les behaviors `spin` et `backup`, garder uniquement `wait` :
```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["wait"]
    wait:
      plugin: "nav2_behaviors/Wait"
```

---

## 17. `MockMapPublisher` — QoS incompatible avec SLAM Toolbox

**Symptôme**  
RViz2 affiche "No map received" ou l'erreur QoS incompatible dans les logs. Le mock `/map` est publié mais RViz2/Nav2 ne le reçoit pas.

**Cause**  
SLAM Toolbox publie `/map` avec `RELIABLE + TRANSIENT_LOCAL` (pour que les late subscribers reçoivent la dernière carte). Le mock utilisait le QoS par défaut (`RELIABLE + VOLATILE`).

**Solution**  
```python
_MAP_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)
publisher = node.create_publisher(OccupancyGrid, '/map', _MAP_QOS)
```

---

## 18. Bridge Ignition — topic RPLIDAR chemin complet requis

**Symptôme**  
`ros2 run ros_gz_bridge parameter_bridge "/scan@..."` ne reçoit aucune donnée.

**Cause**  
Dans Ignition Gazebo, le topic du RPLIDAR utilise le chemin complet du modèle :  
`/world/<world>/model/<robot>/link/rplidar_link/sensor/rplidar/scan`  
Le bridge ne peut pas se connecter à un topic nommé simplement `/scan`.

**Solution**  
```bash
ros2 run ros_gz_bridge parameter_bridge \
    "/world/maze/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan" \
    --ros-args -r "/world/maze/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan:=/scan"
```

---

## 19. `slam_params.yaml` — `max_laser_range` trop grand (LDS-01)

**Symptôme**  
SLAM crée une carte incohérente ou inclut des artefacts de scan à grande distance.

**Cause**  
La valeur par défaut `max_laser_range: 20.0` dépasse les capacités physiques du LDS-01 (max 3.5 m). Les scans au-delà de 3.5 m retournent des valeurs invalides.

**Solution**  
```yaml
slam_toolbox:
  ros__parameters:
    min_laser_range: 0.12   # minimum LDS-01
    max_laser_range: 3.5    # maximum LDS-01
```

---

## 20. Processus Nav2 dupliqués — conflit nœud `controller_server`

**Symptôme**  
Au redémarrage de Nav2, `controller_server` échoue à se configurer immédiatement (error en <10 ms) sans message d'erreur apparent.

**Cause**  
Un `controller_server` d'une session précédente est toujours en cours d'exécution. Les deux instances tentent d'enregistrer le même nœud ROS 2 → conflit DDS.

**Solution**  
Tuer explicitement toutes les instances Nav2 avant de redémarrer :
```bash
pkill -9 -f "controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager|smoother_server"
sleep 2
```

---

## 21. Transport Fast-DDS (SHM) corrompu — `controller_manager` injoignable

**Symptôme**
Le service `/turtlebot4/controller_manager/list_controllers` apparaît dans
`ros2 service list` mais ne répond jamais. Les `spawner` restent bloqués sur
`waiting for service ... to become available`. Logs parsemés de :
```
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portXXXX: open_and_lock_file failed
```

**Cause**
Des fichiers de mémoire partagée Fast-DDS périmés (`/dev/shm/fastrtps_port*`)
laissés par des sessions précédentes (kill -9, crashs) bloquent le transport
SHM. Le `controller_manager` tourne dans la boucle physique d'Ignition et ne
peut plus ouvrir ses ports SHM → ses services sont annoncés sur le graphe DDS
mais aucune requête n'aboutit.

**Solution**
Désactiver le transport SHM (forcer UDP) **et** nettoyer le SHM avant tout
démarrage ROS 2 :
```bash
# config/fastdds_udp_only.xml — profil UDPv4 only
export FASTRTPS_DEFAULT_PROFILES_FILE="config/fastdds_udp_only.xml"
find /dev/shm -name "fastrtps_port*" -delete   # avant de lancer quoi que ce soit
```
`run_demo_tb4.sh` applique les deux automatiquement au démarrage.

---

## 22. Nav2 BT — `Node not recognized: GlobalUpdatedGoal`

**Symptôme**
```
[bt_navigator]: Exception when loading BT: ... Node not recognized: GlobalUpdatedGoal
[bt_navigator]: Error loading XML file: navigate_to_pose_w_replanning_and_recovery.xml
[lifecycle_manager]: Failed to bring up all requested nodes. Aborting bringup.
```

**Cause**
Le BT XML Nav2 par défaut (`navigate_to_pose_w_replanning_and_recovery.xml`)
référence des nœuds (`GlobalUpdatedGoal`, `spin`, `backup`, `RemovePassedGoals`)
dont les plugins ne sont pas chargés une fois la recovery désactivée (#16).
Charger le BT complet alors que les actions `spin`/`backup` sont absentes
échoue à l'activation de `bt_navigator`.

**Solution**
Pointer `bt_navigator` vers un BT minimal (replanification périodique, sans
recovery) qui n'utilise que `ComputePathToPose` + `FollowPath` :
```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: ".../behavior_trees/navigate_w_replanning_time.xml"
    default_nav_through_poses_bt_xml: ".../behavior_trees/navigate_w_replanning_time.xml"
```

---

## 23. Nav2 lifecycle — `smoother_server` / `velocity_smoother` non configurés

**Symptôme**
```
[lifecycle_manager]: Failed to change state for node: smoother_server
[lifecycle_manager]: Failed to bring up all requested nodes. Aborting bringup.
```

**Cause**
`nav2_bringup/navigation_launch.py` (Humble) lance **en dur** `smoother_server`
et `velocity_smoother` et les inclut dans la liste des nœuds gérés par le
lifecycle. Si `nav2_params_tb4.yaml` ne définit pas ces deux sections, leur
`Configuring` échoue et tout le bringup avorte.

**Solution**
Ajouter les sections `smoother_server` (plugin `SimpleSmoother`) et
`velocity_smoother` (limites de vitesse TB4, `odom_topic: /turtlebot4/odom`),
et lister les deux dans `lifecycle_manager_navigation.node_names` :
```yaml
lifecycle_manager_navigation:
  ros__parameters:
    node_names:
      - controller_server
      - smoother_server
      - planner_server
      - behavior_server
      - bt_navigator
      - velocity_smoother
```

---

## 24. RViz2 — `indexed_8bit_image` GLSL link error (carte non texturée)

**Symptôme**
```
[rviz2] Vertex Program:rviz/glsl120/indexed_8bit_image.vert
        Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result :
        active samplers with a different type refer to the same texture image unit
```
Le display **Map** ne se texture pas (LaserScan, caméra, RobotModel s'affichent
normalement).

**Cause**
Le shader `indexed_8bit_image` de RViz2 est incompatible avec le renderer
logiciel Mesa/llvmpipe utilisé sur le GPU virtuel Parallels (`1ab8:0010`) en
`LIBGL_ALWAYS_SOFTWARE=1`. Limitation du renderer, pas du code.

**Solution**
Cosmétique uniquement — n'empêche ni la navigation ni le SLAM. Contournements :
- utiliser `Costmap` (RGBA) au lieu de `Map` pour visualiser l'occupation, ou
- exécuter RViz2 sur l'hôte macOS / une machine avec GPU réel.

---

## 25. SLAM — carte figée à 7×7, `Message Filter dropping ... queue is full`

**Symptôme**
SLAM enregistre le capteur mais la carte reste 7×7, `map→odom` n'est pas publié
et le log répète :
```
Message Filter dropping message: frame 'turtlebot4/rplidar_link/rplidar'
  ... reason 'discarding message because the queue is full'
```
La navigation autonome sur un long parcours ne peut pas aboutir (costmap global
trop petit).

**Causes (deux, cumulatives)**
1. **TF statiques en horloge murale** : les `static_transform_publisher` lancés
   sans `use_sim_time:=true` estampillent leurs TF en temps mur, alors que le
   buffer tf2 de SLAM est en temps sim → `scan → odom` ne se résout pas →
   aucun scan intégré, `map→odom` jamais publié.
2. **Débit LiDAR trop élevé** : `/scan` sort à ~62 Hz dans Ignition. Le
   `MessageFilter` de slam_toolbox (file courte) déborde plus vite que les
   lookups TF n'aboutissent → tous les scans sont jetés.

**Solutions**
1. Lancer les `static_transform_publisher` **avec** `--ros-args -p use_sim_time:=true`
   (corrigé dans `run_demo_tb4.sh`). À lui seul, ceci rétablit `map→odom`.
2. Réduire le débit des scans vus par SLAM :
   - **option A (recommandée)** : abaisser `update_rate` du LiDAR dans le SDF, ou
   - **option B** : intercaler un throttle 10 Hz (`/scan` → `/scan_throttled`,
     republié au plus toutes les 0,1 s) et pointer SLAM dessus via
     `scan_topic: /scan_throttled` **dans `slam_params_tb4.yaml`** (c'est un
     paramètre du noeud, pas un argument de launch).

**Statut**
Cause #1 corrigée et vérifiée (`map→odom` rétabli). Cause #2 : le throttle 10 Hz
est validé côté débit (8,6 Hz mesuré) ; le réglage `scan_topic` doit être posé
dans le YAML pour être pris en compte. Limitation pré-existante de la simulation
sur ce VM ARM64, indépendante du refactoring.

---

## Résumé des fichiers modifiés

| Fichier | Problème résolu |
|---|---|
| `setup.cfg` | #4 — nom du paquet |
| `config/slam_params.yaml` | #19 — max_laser_range |
| `config/slam_params_tb4.yaml` | #5 + #7 — frames namespaced, min_travel=0 |
| `config/nav2_params_tb4.yaml` | #11 + #12 + #13 + #16 + #22 + #23 — frames TB4, BT minimal, recovery, smoother lifecycle |
| `config/waypoints_tb4_maze.yaml` | #11 — waypoints dans les bornes de la map |
| `config/fastdds_udp_only.xml` | #21 — transport UDP-only (désactive SHM) |
| `utils/tf_relay_node.py` | #5 + #6 — relay QoS TRANSIENT_LOCAL (base `TopicRelay`) |
| `utils/cmd_vel_relay_node.py` | cmd_vel Nav2 → diffdrive namespace (base `TopicRelay`) |
| `utils/node_runner.py` | refactor — cycle de vie rclpy partagé (init/spin/shutdown) |
| `utils/topic_relay.py` | refactor — base commune des deux relais |
| `mocks/mock_map_publisher.py` | #17 — QoS TRANSIENT_LOCAL |
| `navigation/waypoint_follower_node.py` | #14 + #15 — wall timer + MultiThreadedExecutor |
| `launch/navigation_tb4.launch.py` | #8 — env var override pour NFS |
| `scripts/sh/run_demo_tb4.sh` | #2 + #3 + #8 + #9 + #16 + #21 — séquence robuste + nettoyage SHM |
