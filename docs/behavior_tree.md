# Behavior Tree plan

The project must use Behavior Trees rather than a pure finite-state machine. The first version keeps a lightweight BT supervisor and an XML file documenting the intended high-level behavior.

## High-level BT

```text
Sequence
  CheckSystemReady
  StartMapping
  ExploreArea
  CheckCoverage(threshold=0.90)
  ExportVictims
  SaveResults
```

## Implementation options

1. Use Nav2's existing BT navigator for navigation behaviors.
2. Add a project-level `bt_supervisor_node` that monitors high-level conditions such as coverage and result export.
3. If time allows, replace the placeholder with a BehaviorTree.CPP-based implementation.

## Files

- `ros2_ws/src/ia712_search_rescue/behavior_trees/search_and_rescue_bt.xml`
- `ros2_ws/src/ia712_search_rescue/ia712_search_rescue/bt/bt_supervisor_node.py`
- `ros2_ws/src/ia712_search_rescue/launch/bt.launch.py`
- `ros2_ws/src/ia712_search_rescue/config/bt_params.yaml`
