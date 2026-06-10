# Architecture Questions and Decisions

This document is the starting point for the professor validation discussion. It lists the questions we asked before implementing the system, our answer, the justification, and the likely criticism.

## 1. What is the minimum viable project?

**Decision:** TurtleBot3 + Gazebo + SLAM Toolbox + Nav2 + frontier exploration + simple victim detection + tf2 localization + final results export.

**Justification:** The project evaluates autonomous robotic integration. A simple victim representation is acceptable; the professor explicitly allows QR codes, ArUco/AprilTag markers, or colored objects instead of complex human detection.

**Possible criticism:** The detection may be too simple.

**Correction:** Detection is intentionally simple in the MVP. The hard part is the integrated loop: map, explore, detect, transform to map, and export. If stable, ArUco/AprilTag will replace color detection.

## 2. Why ROS 2 Humble and Gazebo Classic?

**Decision:** Use Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11.

**Justification:** This is the most compatible stack for the course context, TurtleBot3, Nav2, and SLAM Toolbox. Stability matters more than using the newest ROS release.

**Possible criticism:** Newer ROS versions exist.

**Correction:** The short project timeline makes reproducibility and package compatibility the priority.

## 3. Why one package instead of many packages?

**Decision:** Use one package named `rescue_robot`, structured internally by modules.

**Justification:** Multiple packages are cleaner at large scale but add overhead. One package with separate submodules, launch files, configs, and docs is simpler for a four-person student project.

**Possible criticism:** Less modular than a professional multi-package stack.

**Correction:** Modularity is still enforced through ROS topics, launch files, configs, and file ownership. The architecture can be split later if needed.

## 4. Why mocks?

**Decision:** Provide mock publishers for `/map`, `/victims_map`, and `/coverage`.

**Justification:** Mocks allow parallel development and early interface validation. They prevent one person from blocking everyone else.

**Possible criticism:** Mocks do not prove the final robot works.

**Correction:** Mocks are only for development. Final validation will use Gazebo, SLAM, Nav2, camera, and tf2.

## 5. How will coverage be measured?

**Decision:** Start with occupancy-grid known-cell ratio, then refine by excluding obstacles or using an explorable mask.

**Justification:** The project requires >90% mapping, so a measurable metric is needed from the start.

**Possible criticism:** Counting occupied cells or map boundaries can bias the result.

**Correction:** The metric document and node explicitly state the MVP limitation and plan to refine the denominator once the final world is fixed.

## 6. How will Behavior Tree be real, not decorative?

**Decision:** Add a project-level BT scaffold connected to `/coverage` and a BT XML file from day one.

**Justification:** The project requires Behavior Trees instead of FSMs. The scaffold monitors a real mission condition and will later trigger export/finalization.

**Possible criticism:** The BT is too simple.

**Correction:** The first version is intentionally minimal. The final version must orchestrate real actions such as checking system readiness, monitoring coverage, and triggering result export.

## 7. Why shell scripts if launch files exist?

**Decision:** Keep ROS 2 launch files as the true entrypoints; use shell scripts as wrappers for easy execution.

**Justification:** The professor asked for easily executable scripts. The wrappers reduce terminal mistakes for the team.

**Possible criticism:** Scripts may hide the ROS architecture.

**Correction:** Each script maps directly to one documented ROS 2 launch file. The launch files remain the canonical ROS entrypoints.
