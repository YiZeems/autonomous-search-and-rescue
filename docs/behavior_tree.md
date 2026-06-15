# Behavior Tree plan

The mission is orchestrated by a Behavior Tree rather than a finite-state machine, as
required by the assignment ("decision-making with Behavior Trees, not FSMs"). The BT is
implemented in C++ with **BehaviorTree.CPP v3** in the `rescue_decision` package. The
tree decides *when* each mission phase starts and stops; the actual robot work runs in
the Python nodes, which the BT gates through the `/mission/*` topics.

## Engine and tooling

- Runner: `rescue_decision/src/bt_runner.cpp` (executable `bt_runner`), built on
  BehaviorTree.CPP v3.
- Tree: `rescue_decision/bt_xml/mission.xml`, loaded at runtime via the `bt_xml`
  parameter. The XML is also directly loadable in Groot / Groot2 for editing.
- Launch: `ros2 launch rescue_decision bt_mission.launch.py`. Arguments: `bt_xml`,
  `tick_rate_hz` (default `1.0`), `groot_zmq` (default `true`), `use_sim_time`.
- Visualization: a ZMQ publisher (`BT::PublisherZMQ`, port **1666**) exposes the live
  tree to **Groot** in *Monitor* mode.

The runner ticks the root at `tick_rate_hz` from a single long-lived executor until the
tree returns `SUCCESS`, then idles so the latched topics stay published and Groot stays
connected.

## Mission tree

The root is a plain `Sequence` *with memory*: it resumes at the `RUNNING` child each tick
and only advances on `SUCCESS`. This makes the tree a genuine ordered sequence of phases.

```xml
<root main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Sequence name="search_and_rescue_mission">
      <WaitForMap name="wait_for_slam_map"/>
      <ExplorePhase name="explore_until_coverage" threshold="0.90"/>
      <InspectPhase name="inspect_discovered_rooms"/>
      <VictimsFound name="report_victims" min_count="0"/>
      <PublishMissionDone name="finalize"/>
    </Sequence>
  </BehaviorTree>
</root>
```

## Nodes

- **WaitForMap** (Condition) — `SUCCESS` once a `/map` (`nav_msgs/OccupancyGrid`)
  message has arrived from SLAM, `FAILURE` otherwise (the Sequence retries on the next
  tick). No ports.
- **ExplorePhase** (`StatefulActionNode`) — on start, publishes
  `/mission/explore_enable = true`, which activates `frontier_explorer_node` (Phase 1:
  frontier exploration). Stays `RUNNING` until `/coverage >= threshold`, then clears
  `/mission/explore_enable` (stopping the explorer and freeing Nav2) and returns
  `SUCCESS`. Port: `threshold` (coverage ratio in `[0,1]`, default `0.90`, set to
  `0.90` in `mission.xml`). `onHalted` also clears the enable flag.
- **InspectPhase** (`StatefulActionNode`) — on start, ensures exploration is stopped and
  publishes `/mission/inspect_enable = true`, which activates `inspection_node`
  (Phase 2: inspection). The inspection node derives one inspection pose per room from
  the live `/map` via
  `rescue_robot/.../navigation/inspection_planner.py::poses_from_grid` — purely from the
  occupancy grid, with **no** victim coordinates as input. Stays `RUNNING` until
  `/mission/inspect_done` is received. No ports.
- **VictimsFound** (Condition) — `SUCCESS` when `/victims_map`
  (`geometry_msgs/PoseArray`) holds at least `min_count` poses. Port: `min_count`
  (default `0`, set to `0` in `mission.xml`, so it never blocks the sequence; it surfaces
  the live victim count for monitoring).
- **PublishMissionDone** (Action) — latches `std_msgs/Bool true` on `/mission_done`,
  signaling the result exporter / demo scripts that the mission is complete.

The runner also registers `CoverageReached` (Condition) and `MissionLog` (Action) leaves
that are available to the factory but not used by the current `mission.xml`.

## Topic interface (BT to Python nodes)

| Topic | Type | Direction | Role |
|-------|------|-----------|------|
| `/map` | `nav_msgs/OccupancyGrid` | in | SLAM map; gates `WaitForMap` and feeds inspection planning |
| `/coverage` | `std_msgs/Float32` | in | exploration coverage ratio; ends Phase 1 |
| `/victims_map` | `geometry_msgs/PoseArray` | in | victim registry; read by `VictimsFound` |
| `/mission/inspect_done` | `std_msgs/Bool` | in | inspection node signals Phase 2 complete |
| `/mission/explore_enable` | `std_msgs/Bool` (latched) | out | start/stop `frontier_explorer_node` |
| `/mission/inspect_enable` | `std_msgs/Bool` (latched) | out | trigger `inspection_node` |
| `/mission_done` | `std_msgs/Bool` (latched) | out | mission completion latch |

The two output command topics and `/mission_done` use `TRANSIENT_LOCAL` (latched) QoS so
late-joining subscribers still receive the last command.

## Legacy scaffold (abandoned)

An earlier Python scaffold remains in the tree but is **not** the current BT and is not
launched by the mission:

- `ros2_ws/src/rescue_robot/rescue_robot/bt/bt_supervisor_node.py`
- `ros2_ws/src/rescue_robot/behavior_trees/search_and_rescue_bt.xml`
- `ros2_ws/src/rescue_robot/launch/bt.launch.py`

The authoritative implementation is the BehaviorTree.CPP v3 runner described above.

## Files

- `ros2_ws/src/rescue_decision/src/bt_runner.cpp` — BT.CPP v3 runner and leaf nodes
- `ros2_ws/src/rescue_decision/bt_xml/mission.xml` — mission tree (Groot-loadable)
- `ros2_ws/src/rescue_decision/launch/bt_mission.launch.py` — launch file
- `ros2_ws/src/rescue_robot/rescue_robot/navigation/inspection_planner.py` —
  `poses_from_grid` (per-room inspection poses)
