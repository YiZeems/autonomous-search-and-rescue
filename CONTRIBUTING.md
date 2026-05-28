# Contribution rules

## Simple rule
Work only in your own files. Do not modify integration files unless A approves it.

## Branches
- `main`: stable demo only.
- `dev`: integration branch.
- `a-integration`: integration lead.
- `b-exploration`: exploration work.
- `c-simulation`: simulation work.
- `d-results`: metrics/visualization/results work.

## Daily workflow for B/C/D
```bash
git pull
git add .
git commit -m "update my module"
git push
```

## Protected files owned by A
- `README.md`
- `CONTRIBUTING.md`
- `.github/`
- `ros2_ws/src/ia712_search_rescue/package.xml`
- `ros2_ws/src/ia712_search_rescue/setup.py`
- `ros2_ws/src/ia712_search_rescue/launch/bringup.launch.py`
- `ros2_ws/src/ia712_search_rescue/launch/mock_system.launch.py`
- `docs/interfaces.md`

## Before asking for integration
- Your code should not include `build/`, `install/`, or `log/`.
- Your Python files should have no syntax errors.
- Your module launch file should start if applicable.
- Do not touch another member's module.
