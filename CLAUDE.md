# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 3200 (Raptacon) robot code for the 2027 season. Written in Python using RobotPy (WPILib's Python bindings) with the Commands2 framework. The robot will run a 4-module swerve drivetrain (SDS MK4i L2 by default, gearing selectable by preset) with SparkMax motor controllers (REV NEO motors), CTRE CANcoders and a NavX. The controller moves from the roboRIO to SystemCore this season.
Note that wpilib is avaiable in C++, Java and python. Most resources are listed as C++ or java but almost all code transfers to python with minor updates to match python nuances and style.

This repo was seeded from robot-2026 at commit 7b5e8e0 with tooling and game-independent utilities only. The plan, milestones and design decisions are in `doc/plans/robot-2027-code-plan.md`; swerve background is in `doc/swerve/`.

## Build & Development Commands

**Setup (first time):**
```bash
make                    # Creates venv and installs dependencies (uses Makefile)
# OR manually:
python -m venv venv
pip install -r requirements.txt
python -m robotpy sync  # Installs pyproject.toml dependencies to robotpy cache
```

**Run simulator:**
```bash
make sim                # Runs coverage tests then launches simulator
python -m robotpy sim   # Direct simulator launch
```

**Run tests:**
```bash
python -m robotpy test              # Run all tests
python -m robotpy coverage test     # Run tests with coverage (used by CI)
```

**Lint:**
```bash
make lint               # ruff check + ruff format --check (same as CI)
make format             # auto-fix lint and format
make typecheck          # pyright (informational for now)
pre-commit install      # once per clone: runs ruff on every commit
```

**Deploy to robot:**
```bash
make deploy             # Deploys libraries and code to the robot. (requires robot connection)
python -m robotpy deploy
```

**Style:**
Code is formatted with `ruff format` and linted with `ruff check`; settings live in `pyproject.toml`. Don't hand-format or argue style in reviews: run `make format`. Add type hints to new code; pyright runs in CI (non-blocking until the existing findings are fixed).

## Architecture

### Entry Point & Robot Lifecycle

`robot.py` defines `MyRobot(commands2.TimedCommandRobot)` running at 50 Hz (20 ms period). On `main` it only sets up logging (wpilog via `DataLogManager`, Python logging via `utils/datalog_bridge.py`), loop timing (`utils/loop_timing.py`, published under `/FrameTiming/` at 10 Hz) and `HealthAndStatus` telemetry. The swerve drivetrain is added on the `swerve-dev` branch.

`MyRobot.callAndCatch` wraps calls to catch and log exceptions without crashing the robot on hardware (exceptions are re-raised in simulation so tests fail).

### Swerve design rules (see `doc/plans/robot-2027-code-plan.md`)

- Vendor code (REV, CTRE, NavX) only appears in IO implementations named `*_spark.py`, `*_cancoder.py`, `*_navx.py`; everything else uses the `ModuleIO`, `AbsoluteEncoderIO` and `GyroIO` interfaces. Sim IO implementations run in CI.
- Module gearing comes from presets (MK4i L1 to L3, MK4 L1 to L4) that also carry steer inversion and encoder direction; per-robot config holds CAN IDs, offsets and drive inversion, in git.
- One steering angle source per module, SI units everywhere, `ChassisSpeeds.discretize` before kinematics.
- Read each hardware signal once per loop into an inputs dataclass and log that object; publish dashboard-only values at 10 Hz.
- Configure every SparkMax from defaults on each boot; don't rely on controller flash.

### Robot config (`config/`)

`config/module_presets.py` holds the SDS gearing presets (MK4i L1 to L3, MK4 L1 to L4; `MK4I_L2` is the default) with steer and encoder direction. `config/robot_config.py` defines the frozen `CornerConfig` and `RobotConfig` types; each robot is one file in `config/robots/` exporting `CONFIG`, registered in `config/loader.py`. `load_robot_config()` picks the robot from the persistent NT value `/robot/name` (default `swerve_test_bot`). Offsets are in rotations, wrapped to [-0.5, 0.5); log every offset or wheel radius change in `doc/swerve/calibration-log.md`. `tests/drivetrain/test_swerve_config.py` checks every preset and robot config.

### Controller Config (`utils/controller/`) and Input Factory (`utils/input/`)

Shared data model (`utils/controller/model.py`) defines `ActionDefinition`, `ControllerConfig`, and `FullConfig`. Actions use qualified names: `group.name` (e.g. `drivetrain.rotate`). Input types: BUTTON, ANALOG, OUTPUT, BOOLEAN_TRIGGER, VIRTUAL_ANALOG. D-pad directions are treated as buttons. YAML I/O in `config_io.py`. Portable curve math in `utils/math/curves.py`.

`InputFactory` loads a YAML config (`data/inputs/swerve_test_bot.yaml`), creates `wpilib.XboxController` instances, and provides `getButton()`, `getAnalog()`, `getRumbleControl()` and raw variants. Analog shaping pipeline: inversion -> deadband -> curve -> scale -> slew rate limit, applied per axis. Parameters are published to NT under `/inputs/actions/<group>/<action>/` for runtime tuning.

Swerve translation axes are `raw` in the YAML on purpose: per-axis shaping makes a square deadband and bends diagonals, so the teleop drive command shapes the stick vector's length (radial deadband, curve, slew) instead. Rotation uses the normal pipeline.

### Examples (`examples/`)
Stand-alone example robots, each with its own `robot.py`. The `*-sysid` examples are references for the shared SysId tool planned in `utils/sysid/`.

### CAN ID Convention

Drivetrain modules start at CAN ID 50 with 3 consecutive IDs per module (drive, steer, encoder). Additional mechanisms count backwards from CAN ID 40. CAN IDs 15-19 are reserved for unit tests and must not be used for hardware.

## Key Libraries

- `robotpy` 2026.x - WPILib Python bindings (2027 / SystemCore move tracked by the preview CI job)
- `robotpy-rev` - REV SparkMax motor controllers
- `phoenix6` - CTRE CANcoder absolute encoders
- `robotpy-navx` - NavX gyroscope
- `robotpy-pathplannerlib` - Autonomous path planning
- `photonlibpy` - PhotonVision camera integration
- `commands2` - WPILib command-based framework
- `robotpy-urcl` - Unoffical library to enable capture of Rev Robotics CAN control frames.

## CI Pipeline

GitHub Actions (`.github/workflows/robot_ci.yml`):
- Unit tests on Linux (with coverage report artifact and sim smoke test), Windows and macOS: `python -m robotpy coverage test`
- RobotPy 2027 preview job (informational: stays green, reports breakage as a warning and in the job summary)
- Lint and format: `ruff check .` and `ruff format --check .`, rules from `pyproject.toml` (same as `make lint` and pre-commit; blocking)
- Type check: pyright, informational (stays green, reports the error count as a warning)
- pip caching on every job, and a new push cancels the superseded run
- pdoc docs build, deployed to GitHub Pages from `main`
- Dependabot keeps GitHub Actions versions current

## Unit Tests
 - Unit tests should be encouraged and written
 - Unit tests generated by Claude should be commented as such
 - Unit tests should use sim feedback when working with hardware based devices
 - Prefer pytest style (plain classes + `assert`) over `unittest.TestCase` — pyfrc/robotpy uses pytest as its test runner

## NetworkTables

**Prefer `ntproperty` over `SmartDashboard`** for publishing subsystem state. `ntproperty` creates proper NT entries under the subsystem's own path (e.g. `/SubsystemName/value`) and is the standard pattern for this codebase. Avoid `wpilib.SmartDashboard.putString/putNumber` in subsystems — it puts values under `/SmartDashboard/` which doesn't organize well and doesn't create a proper subsystem NT entry. During code reviews, flag any `SmartDashboard.put*` in subsystem code as something to convert to `ntproperty`.

**Persistence:** Use `ntproperty` with `persistent=True` and `writeDefault=False` for values that need to persist across reboots (calibration data, saved positions, etc.) so existing persisted values are not overwritten on startup:

```python
from ntcore.util import ntproperty


class MySubsystem:
    saved_limit = ntproperty("/MySubsystem/saved_limit", 0.0, writeDefault=False, persistent=True)
```

**Non-persistent state:** Use `ntproperty` with `writeDefault=True` (default) for runtime telemetry that doesn't need to persist:

```python
class MySubsystem:
    status = ntproperty("/MySubsystem/status", "unknown", writeDefault=True)
```


## WPILib Upstream Development

### Repository & Fork

- **Upstream:** `wpilibsuite/allwpilib` — the main WPILib C++/Java/Python library
- **Our fork:** `tuffinmuffin/allwpilib` — cloned to `/Users/nbeasley/FRC/allwpilib`
- **Docs:** WPILib docs are at https://docs.wpilib.org — source code docs are sparse, mostly inline comments
- **Contribution policy:** BSD-3 license, no CLA required. Bug fixes generally accepted. Must pass `wpiformat` and `./gradlew check`.

### Key Directories in allwpilib

- `simulation/halsim_gui/` — Simulation GUI (driver station, joysticks, hardware viz)
- `hal/` — Hardware Abstraction Layer
- `wpigui/` — GLFW-based GUI framework used by halsim_gui
- `wpilib/` — Main WPILib library (Java)
- `wpimath/` — Math utilities
- `cscore/` — Camera support (has Objective-C++ examples for macOS patterns)

### Building & Installing (macOS)

Requires Java 17 (Gradle 8.x does not support Java 25):

```bash
brew install openjdk@17
export JAVA_HOME=/opt/homebrew/opt/openjdk@17
export PATH="/opt/homebrew/opt/openjdk@17/bin:$PATH"

cd /Users/nbeasley/FRC/allwpilib
./gradlew :simulation:halsim_gui:build
```

Built binary: `simulation/halsim_gui/build/libs/halsim_gui/shared/osxuniversal/release/libhalsim_gui.dylib`

**Installing into robotpy venv:**

**The allwpilib branch must match your robotpy version to avoid ABI mismatches.**
Check `pyproject.toml` for the robotpy version (e.g. `2026.2.1`) and build from the matching tag (e.g. `v2026.2.1`).

```bash
# Back up original
cp venv/lib/python3.*/site-packages/halsim_gui/lib/libhalsim_gui.dylib \
   venv/lib/python3.*/site-packages/halsim_gui/lib/libhalsim_gui.dylib.bak

# Install built binary
cp /Users/nbeasley/FRC/allwpilib/simulation/halsim_gui/build/libs/halsim_gui/shared/osxuniversal/release/libhalsim_gui.dylib \
   venv/lib/python3.*/site-packages/halsim_gui/lib/libhalsim_gui.dylib

# Restore original
cp venv/lib/python3.*/site-packages/halsim_gui/lib/libhalsim_gui.dylib.bak \
   venv/lib/python3.*/site-packages/halsim_gui/lib/libhalsim_gui.dylib
```

### Code Style

- C++ formatting: run `wpiformat` (install via `pip install wpiformat`)
- `wpiformat` needs `git remote set-head origin main` if origin/HEAD is not set
- Gradle spotless checks run automatically during build

## Commit messages
 - Leave off Claude coauthor for main files

## Claude.md updates
If Claude sees an area that would benifit for remembering or having instructions in the future Claude should suggest adding it to CLAUDE.md
