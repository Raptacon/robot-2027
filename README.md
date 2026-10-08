# robot-2027

FRC Team 3200 (Raptacon) robot code for the 2027 season, in Python with
RobotPy and the Commands2 framework.

The repo was seeded from [robot-2026](https://github.com/Raptacon/robot-2026)
with its tooling, CI, controller input system and game-independent utilities,
but none of the 2026 robot code. The new swerve drivetrain is being built on
the `swerve-dev` branch; see the [code plan](doc/plans/robot-2027-code-plan.md)
for milestones.

## Getting started

```bash
make                        # create a venv and install requirements
python -m robotpy test      # run the tests
python -m robotpy sim       # run the simulator
make lint                   # flake8, same rules as CI
```

Deploy with `make deploy` (or `python -m robotpy deploy`) while connected to
the robot.

## Layout

| Path | What's there |
| --- | --- |
| `robot.py` | Robot entry point: logging, loop timing, health telemetry |
| `subsystem/` | Subsystems (`health_and_status.py` today; drivetrain on `swerve-dev`) |
| `utils/input/`, `utils/controller/`, `utils/math/` | Config-driven controller input (InputFactory) |
| `utils/` | Logging bridge, loop timing, geometry helpers, macOS sim controller bridge |
| `data/inputs/` | Controller mappings (`swerve_test_bot.yaml`) |
| `examples/` | Stand-alone SysId example robots for reference |
| `doc/` | Team guides: swerve theory, tuning and calibration, plans |
| `tests/` | pytest tests, run by `python -m robotpy test` and CI |

## Branches

- `main`: reviewed, green merges only.
- `swerve-dev`: integration branch for the new swerve; merges to `main` at each milestone.
- `swerve/<topic>`: one feature branch per PR into `swerve-dev`.
