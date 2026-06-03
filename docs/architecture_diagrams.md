# IA712 Search and Rescue — Architecture Diagrams

Ce document regroupe les principaux diagrammes d'architecture du projet **IA712 Mobile Robotics — Search and Rescue**.

Objectif du projet : faire évoluer un robot mobile autonome dans un environnement simulé Gazebo, construire une carte, explorer une zone inconnue, détecter des victimes avec caméra, projeter leurs positions dans le repère `map`, puis produire une carte annotée et des résultats exploitables.

---

## 1. Arborescence globale du projet

```mermaid
flowchart TD
    ROOT["autonomous-robotz-search-and-rescue/"]

    ROOT --> README["README.md"]
    ROOT --> AGENTS["AGENTS.md"]
    ROOT --> CONTRIB["CONTRIBUTING.md"]
    ROOT --> PYPROJECT["pyproject.toml"]
    ROOT --> DOCS["docs/"]
    ROOT --> SCRIPTS["scripts/"]
    ROOT --> TESTS["tests/"]
    ROOT --> RESULTS["results/"]
    ROOT --> ROSWS["ros2_ws/"]

    ROSWS --> SRC["src/"]
    SRC --> PKG["rescue_robot/"]

    PKG --> PKGXML["package.xml"]
    PKG --> SETUPPY["setup.py"]
    PKG --> SETUPCFG["setup.cfg"]
    PKG --> LAUNCH["launch/"]
    PKG --> CONFIG["config/"]
    PKG --> WORLDS["worlds/"]
    PKG --> MODELS["models/"]
    PKG --> MAPS["maps/"]
    PKG --> RVIZ["rviz/"]
    PKG --> BTXML["behavior_trees/"]
    PKG --> PYMOD["rescue_robot/"]

    PYMOD --> MOCKS["mocks/"]
    PYMOD --> EXPLORATION["exploration/"]
    PYMOD --> DETECTION["detection/"]
    PYMOD --> RESNODES["results/"]
    PYMOD --> BTNODE["bt/"]
```

---

## 2. Convention ROS 2 / Python du double dossier

Le chemin suivant est normal :

```text
ros2_ws/src/rescue_robot/rescue_robot/
```

Le premier `rescue_robot` est le **package ROS 2**.  
Le second `rescue_robot` est le **module Python importable**.

```mermaid
flowchart TD
    A["ros2_ws/src/rescue_robot/"] --> B["Package ROS 2"]
    B --> C["package.xml"]
    B --> D["setup.py"]
    B --> E["launch/"]
    B --> F["config/"]
    B --> G["worlds/"]
    B --> H["rescue_robot/"]

    H --> I["Module Python"]
    I --> J["mocks/"]
    I --> K["exploration/"]
    I --> L["detection/"]
    I --> M["results/"]
    I --> N["bt/"]

    D --> O["Entry points console_scripts"]
    O --> J
    O --> K
    O --> L
    O --> M
    O --> N
```

Ne pas renommer ni supprimer le deuxième dossier `rescue_robot/`, sinon les entry points définis dans `setup.py` ne fonctionneront plus.

---

## 3. Architecture fonctionnelle globale

```mermaid
flowchart LR
    GZ["Gazebo Classic 11"] --> TB3["TurtleBot3 Waffle Pi"]
    TB3 --> LIDAR["LiDAR /scan"]
    TB3 --> CAM["Camera /camera/image_raw"]
    TB3 --> ODOM["Odometry /odom"]
    TB3 --> TF["TF /tf /tf_static"]

    LIDAR --> SLAM["slam_toolbox"]
    ODOM --> SLAM
    TF --> SLAM
    SLAM --> MAP["/map"]

    MAP --> EXPLORER["frontier_explorer_node"]
    EXPLORER --> NAV2["Nav2 NavigateToPose"]
    NAV2 --> CMD["/cmd_vel"]
    CMD --> TB3

    CAM --> DETECT["victim_detector_node"]
    TF --> DETECT
    DETECT --> VICTIMS["/victims_map"]

    MAP --> COVERAGE["coverage_evaluator_node"]
    COVERAGE --> COVERAGE_TOPIC["/coverage"]

    VICTIMS --> RVIZMARKERS["rviz_marker_node"]
    VICTIMS --> EXPORTER["result_exporter_node"]
    COVERAGE_TOPIC --> EXPORTER
    COVERAGE_TOPIC --> BT["bt_supervisor_node"]

    EXPORTER --> FILES["results/*.csv / run_summary.json"]
    RVIZMARKERS --> RVIZ_OUT["RViz markers"]
    BT --> MISSION["Mission supervision"]
```

---

## 4. Architecture ROS 2 — simulation réelle Waffle Pi

Ce diagramme représente l'état actuel validé avec Gazebo, TurtleBot3 Waffle Pi, caméra, LiDAR, odométrie et téléopération.

```mermaid
flowchart LR
    TELEOP["/teleop_keyboard"] --> CMDVEL["/cmd_vel"]
    CMDVEL --> DIFF["/turtlebot3_diff_drive"]

    DIFF --> ODOM["/odom"]
    DIFF --> TF_DYN["/tf"]

    JOINT_PLUGIN["/turtlebot3_joint_state"] --> JOINTS["/joint_states"]
    JOINTS --> RSP["/robot_state_publisher"]
    RSP --> TF["/tf"]

    CAMERA_DRIVER["/camera_driver"] --> IMAGE["/camera/image_raw"]
    CAMERA_DRIVER --> CAMINFO["/camera/camera_info"]

    LASER["/turtlebot3_laserscan"] --> SCAN["/scan"]
    IMU["/turtlebot3_imu"] --> IMU_TOPIC["/imu"]

    GAZEBO["/gazebo"] --> CLOCK["/clock"]

    TF --> CONSUMERS["TF consumers: RViz, SLAM, detection"]
    TF_DYN --> CONSUMERS
    SCAN --> SLAM["slam_toolbox"]
    ODOM --> SLAM
    IMAGE --> DETECTION["victim_detector_node"]
    CAMINFO --> DETECTION
```

---

## 5. Topics principaux validés

```mermaid
flowchart TD
    A["/cmd_vel<br/>geometry_msgs/Twist"] --> B["Contrôle vitesse robot"]
    C["/odom<br/>nav_msgs/Odometry"] --> D["Odométrie robot"]
    E["/scan<br/>sensor_msgs/LaserScan"] --> F["LiDAR pour SLAM/Nav2"]
    G["/tf<br/>tf2_msgs/TFMessage"] --> H["Frames dynamiques"]
    I["/tf_static<br/>tf2_msgs/TFMessage"] --> J["Frames statiques"]
    K["/camera/image_raw<br/>sensor_msgs/Image"] --> L["Image caméra pour détection"]
    M["/camera/camera_info<br/>sensor_msgs/CameraInfo"] --> N["Intrinsèques caméra"]
    O["/joint_states<br/>sensor_msgs/JointState"] --> P["État des roues/joints"]
    Q["/clock<br/>rosgraph_msgs/Clock"] --> R["Temps simulation"]
```

---

## 6. Pipeline SLAM

Objectif : transformer les mesures LiDAR + odométrie + TF en carte `/map`.

```mermaid
flowchart LR
    SCAN["/scan"] --> SLAM["slam_toolbox"]
    ODOM["/odom"] --> SLAM
    TF["/tf"] --> SLAM
    CLOCK["/clock<br/>use_sim_time=true"] --> SLAM

    SLAM --> MAP["/map"]
    SLAM --> MAPMETA["/map_metadata"]
    SLAM --> MAPTF["TF: map -> odom"]

    MAP --> RVIZ["RViz Map display"]
    MAPTF --> RVIZ
```

Pour visualiser la carte dans RViz :

```text
Fixed Frame = map
Add -> Map -> /map
Add -> LaserScan -> /scan
Add -> TF
Add -> RobotModel
```

---

## 7. Pipeline Nav2

Objectif : recevoir une carte, localiser le robot et envoyer des commandes vers `/cmd_vel`.

```mermaid
flowchart LR
    MAP["/map"] --> NAV2["Nav2 stack"]
    TF["/tf"] --> NAV2
    ODOM["/odom"] --> NAV2
    SCAN["/scan"] --> NAV2

    GOAL["Navigation goal"] --> NAV2
    NAV2 --> CMDVEL["/cmd_vel"]
    CMDVEL --> ROBOT["TurtleBot3 Waffle Pi"]

    ROBOT --> ODOM
    ROBOT --> SCAN
    ROBOT --> TF
```

Nav2 deviendra nécessaire quand l'exploration autonome enverra des objectifs vers des frontières.

---

## 8. Pipeline exploration autonome

Objectif : choisir automatiquement les prochaines zones inconnues à explorer.

```mermaid
flowchart TD
    MAP["/map"] --> FRONTIER["frontier_explorer_node"]
    FRONTIER --> FRONTIERS["Détection des frontières<br/>entre cellules libres et inconnues"]
    FRONTIERS --> SCORE["Score / sélection de la meilleure frontière"]
    SCORE --> GOAL["Goal Pose"]
    GOAL --> NAV2["Nav2 NavigateToPose"]
    NAV2 --> CMDVEL["/cmd_vel"]
    CMDVEL --> ROBOT["TurtleBot3 Waffle Pi"]
    ROBOT --> MAPUPDATE["Nouvelle perception /scan"]
    MAPUPDATE --> MAP
```

Critère de fin possible :

```text
coverage >= 0.90
```

---

## 9. Pipeline détection des victimes

Objectif : détecter des victimes dans l'image caméra, puis projeter leur position dans le repère `map`.

```mermaid
flowchart LR
    IMAGE["/camera/image_raw"] --> DETECTOR["victim_detector_node"]
    CAMINFO["/camera/camera_info"] --> DETECTOR
    TF["/tf"] --> DETECTOR

    DETECTOR --> CV["OpenCV / détection couleur / ArUco / AprilTag"]
    CV --> CAMPOSE["Position relative dans camera_rgb_optical_frame"]
    CAMPOSE --> TF2["tf2 transform camera -> map"]
    TF2 --> VICTIMS["/victims_map<br/>geometry_msgs/PoseArray"]

    VICTIMS --> MARKERS["rviz_marker_node"]
    VICTIMS --> EXPORTER["result_exporter_node"]
    MARKERS --> RVIZ["RViz victim markers"]
    EXPORTER --> CSV["victims_detected.csv"]
```

---

## 10. Arbre TF attendu

```mermaid
flowchart TD
    MAP["map"] --> ODOM["odom"]
    ODOM --> BASEFOOT["base_footprint"]
    BASEFOOT --> BASE["base_link"]

    BASE --> SCAN["base_scan"]
    BASE --> IMU["imu_link"]
    BASE --> CAMLINK["camera_link"]

    CAMLINK --> CAMRGB["camera_rgb_frame"]
    CAMRGB --> CAMOPT["camera_rgb_optical_frame"]

    BASE --> LEFT["wheel_left_link"]
    BASE --> RIGHT["wheel_right_link"]
```

Pour vérifier :

```bash
ros2 run tf2_tools view_frames
xdg-open frames.pdf
```

Pendant la simulation sans SLAM, `map` peut être absent. Avec `slam_toolbox`, on attend :

```text
map -> odom
```

---

## 11. Mock system

Le mock system permet de tester les modules sans Gazebo/Nav2/SLAM.

```mermaid
flowchart LR
    MOCKMAP["mock_map_publisher"] --> MAP["/map"]
    MOCKVICTIMS["mock_victim_publisher"] --> VICTIMS["/victims_map"]
    MOCKCOV["mock_coverage_publisher"] --> COVERAGE["/coverage"]

    MAP --> COVERAGE_NODE["coverage_evaluator_node"]
    COVERAGE_NODE --> COVERAGE

    VICTIMS --> RVIZMARKERS["rviz_marker_node"]
    VICTIMS --> EXPORTER["result_exporter_node"]
    COVERAGE --> EXPORTER
    COVERAGE --> BT["bt_supervisor_node"]

    EXPORTER --> RESULTS["results/*.csv<br/>run_summary.json"]
    BT --> END["Mission ready to finalize"]
```

Commande :

```bash
./scripts/run.sh mock
```

---

## 12. Différence entre mock et simulation réelle

```mermaid
flowchart TD
    subgraph MOCK["Mock system"]
        A1["mock_map_publisher"] --> A2["/map"]
        B1["mock_victim_publisher"] --> B2["/victims_map"]
        C1["mock_coverage_publisher"] --> C2["/coverage"]
    end

    subgraph REAL["Real simulation"]
        R1["Gazebo"] --> R2["TurtleBot3 Waffle Pi"]
        R2 --> R3["/scan"]
        R2 --> R4["/odom"]
        R2 --> R5["/camera/image_raw"]
        R2 --> R6["/tf"]
        R3 --> R7["slam_toolbox"]
        R7 --> R8["/map"]
    end

    A2 --> COMMON["Common downstream modules"]
    B2 --> COMMON
    C2 --> COMMON
    R8 --> COMMON
    R5 --> DETECTION["victim_detector_node"]

    COMMON --> EXPORT["results / RViz / BT"]
```

---

## 13. Launch files

```mermaid
flowchart TD
    RUN["./scripts/run.sh"] --> BUILD["build"]
    RUN --> MOCK["mock"]
    RUN --> SIM["simulation"]
    RUN --> HOUSE["simulation-house"]
    RUN --> HOUSESAFE["simulation-house-safe"]
    RUN --> TELEOP["teleop"]
    RUN --> CAMCHECK["camera-check"]
    RUN --> TB3CHECK["check-tb3"]
    RUN --> TB4CHECK["check-tb4"]

    MOCK --> MOCKLAUNCH["mock_system.launch.py"]
    SIM --> SIMLAUNCH["simulation.launch.py"]
    HOUSE --> SIMLAUNCH
    HOUSESAFE --> SIMLAUNCH

    SIMLAUNCH --> TB3GAZEBO["turtlebot3_gazebo launch files"]
    TB3GAZEBO --> EMPTY["empty_world.launch.py"]
    TB3GAZEBO --> BASE["turtlebot3_world.launch.py"]
    TB3GAZEBO --> HOUSEWORLD["turtlebot3_house.launch.py"]

    TELEOP --> TB3TELEOP["turtlebot3_teleop teleop_keyboard"]
```

---

## 14. Scripts et rôle des wrappers shell

Les scripts `.sh` simplifient l'usage, mais les vrais points d'entrée ROS restent les launch files.

```mermaid
flowchart LR
    USER["User"] --> RUNSH["scripts/run.sh"]

    RUNSH --> COMMON["_common.sh<br/>source ROS + workspace"]
    RUNSH --> TB3HELPER["_turtlebot3_overlay.sh<br/>find TurtleBot3 overlay"]
    RUNSH --> GFX["_gazebo_graphics.sh<br/>safe graphics mode"]

    COMMON --> BUILD["colcon build"]
    TB3HELPER --> SIM["run_simulation.sh"]
    GFX --> HSAFE["run_simulation_house_safe.sh"]

    SIM --> ROSLAUNCH["ros2 launch rescue_robot simulation.launch.py"]
    HSAFE --> ROSLAUNCH
```

---

## 15. Architecture de développement par rôles

```mermaid
flowchart TD
    A["Person A<br/>Integration / Git / Launch / BT"] --> CORE["Core architecture"]
    B["Person B<br/>SLAM / Nav2 / Exploration"] --> EXPLORATION["frontier_explorer_node"]
    C["Person C<br/>Gazebo / Worlds / Robot"] --> SIM["simulation.launch.py / worlds / models"]
    D["Person D<br/>Results / RViz / Metrics"] --> RESULTS["result_exporter_node / rviz_marker_node"]

    CORE --> INTERFACES["docs/interfaces.md"]
    EXPLORATION --> INTERFACES
    SIM --> INTERFACES
    RESULTS --> INTERFACES

    INTERFACES --> DEMO["Final integrated demo"]
```

---

## 16. Architecture cible finale

```mermaid
flowchart TD
    START["Start mission"] --> SIM["Launch Gazebo + Waffle Pi"]
    SIM --> SLAM["Start SLAM"]
    SLAM --> NAV2["Start Nav2"]
    NAV2 --> EXPLORE["Start autonomous exploration"]

    EXPLORE --> MOVE["Navigate to frontier"]
    MOVE --> UPDATE["Update /map and /coverage"]
    UPDATE --> DETECT["Detect victims from camera"]
    DETECT --> PROJECT["Project victims to map with tf2"]
    PROJECT --> ANNOTATE["Publish /victims_map and RViz markers"]

    UPDATE --> CHECK{"coverage >= 90% ?"}
    CHECK -- No --> EXPLORE
    CHECK -- Yes --> SAVE["Save map and export results"]

    SAVE --> REPORT["coverage_over_time.csv<br/>victims_detected.csv<br/>run_summary.json<br/>annotated map"]
```

---

## 17. Commandes utiles pour générer les graphes réels

### Graphe nodes/topics

```bash
rqt_graph
```

Mode safe graphics si nécessaire :

```bash
QT_QPA_PLATFORM=xcb \
GDK_BACKEND=x11 \
LIBGL_ALWAYS_SOFTWARE=1 \
MESA_GL_VERSION_OVERRIDE=3.3 \
MESA_GLSL_VERSION_OVERRIDE=330 \
rqt_graph
```

### Graphe TF

```bash
ros2 run tf2_tools view_frames
xdg-open frames.pdf
```

### Liste nodes

```bash
ros2 node list
```

### Liste topics avec types

```bash
ros2 topic list -t
```

### Informations sur topics importants

```bash
ros2 topic info /cmd_vel
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /camera/image_raw
ros2 topic info /camera/camera_info
ros2 topic info /tf
```
