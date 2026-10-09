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

`robot.py` defines `MyRobot(commands2.TimedCommandRobot)` running at 50 Hz (20 ms period). It sets up logging (wpilog via `DataLogManager`, Python logging via `utils/datalog_bridge.py`), loop timing (`utils/loop_timing.py`, published under `/FrameTiming/` at 10 Hz), `HealthAndStatus` telemetry, and the swerve `Drivetrain`. In simulation the drivetrain is built by `DrivetrainSim`; on the robot `build_drivetrain()` (`subsystem/drivetrain/drivetrain_hardware.py`) builds it from the real IO and URCL logs every SparkMax. In test mode, holding A on the driver controller runs whatever is picked on the dashboard's `Characterization` chooser (letting go stops it; B and the left bumper step through the options): `ModuleCheck` (`commands/drive/module_check.py`, every wheel to 0, 90 and 180 degrees on blocks) by default, or a SysId or calibration test.

`MyRobot.callAndCatch` wraps calls to catch and log exceptions without crashing the robot on hardware (exceptions are re-raised in simulation so tests fail).

### Swerve design rules (see `doc/plans/robot-2027-code-plan.md`)

- Vendor code (REV, CTRE, NavX) only appears in IO implementations named `*_spark.py`, `*_cancoder.py`, `*_navx.py`; everything else uses the `ModuleIO`, `AbsoluteEncoderIO` and `GyroIO` interfaces. Sim IO implementations run in CI.
- Module gearing comes from presets (MK4i L1 to L3, MK4 L1 to L4) that also carry steer inversion and encoder direction; per-robot config holds CAN IDs, offsets and drive inversion, in git.
- One steering angle source per module, SI units everywhere, `ChassisSpeeds.discretize` before kinematics.
- Read each hardware signal once per loop into an inputs dataclass and log that object; publish dashboard-only values at 10 Hz.
- Configure every SparkMax from defaults on each boot; don't rely on controller flash.

### Robot config (`config/`)

`config/module_presets.py` holds the SDS gearing presets (MK4i L1 to L3, MK4 L1 to L4; `MK4I_L2` is the default) with steer and encoder direction. `config/robot_config.py` defines the frozen `CornerConfig` and `RobotConfig` types; each robot is one file in `config/robots/` exporting `CONFIG`, registered in `config/loader.py`. `load_robot_config()` picks the robot from the persistent NT value `/robot/name` (default `swerve_test_bot`). Offsets are in rotations, wrapped to [-0.5, 0.5); log every offset or wheel radius change in `doc/swerve/calibration-log.md`. `tests/drivetrain/test_swerve_config.py` checks every preset and robot config.

### Drivetrain IO (`subsystem/drivetrain/io/`)

`ModuleIO`, `AbsoluteEncoderIO` and `GyroIO` are abstract interfaces, each with an inputs dataclass (`ModuleInputs`, `AbsoluteEncoderInputs`, `GyroInputs`) filled once per loop by `update_inputs()`. Sim versions (`module_io_sim.py`, `abs_encoder_sim.py`, `gyro_io_sim.py`) use the pure-Python `SimMotor` (kS/kV/kA model, no WPILib) and can add latency, boot delay, a dead sensor, offset error, gyro drift and gear or wheel mismatch for tests. The sim steer encoder reads 0 at boot like a real SparkMax, so code must seed it from the absolute encoder. `utils/inputs_publisher.py` publishes an inputs dataclass to NT (and so to the wpilog) every loop. Tests: `tests/drivetrain/test_io_sim.py`.

Hardware versions: `module_io_spark.py` (two SparkMax + NEO, configured from defaults each boot with SI conversion factors and steer position wrapping; `SparkSettings` in the robot config holds current limits and gains), `abs_encoder_cancoder.py` (CANcoder magnet offset left at 0, the config offset is added in code), `gyro_io_onboard.py` (SystemCore built-in IMU, yaw unwrapped to a continuous angle) and `gyro_io_navx.py` (NavX on a roboRIO; NavX is clockwise-positive, so the sign is flipped). `build_gyro()` picks the onboard IMU when the controller has one. REV calls go through `_read()` so they work on 2026 (plain numbers) and 2027 (`Signal` objects). Tests: `tests/drivetrain/test_hardware_io.py` (vendor sims; skipped where a vendor library is missing, CAN IDs 15-17). Only one NavX may be created per pytest process, so tests pass `gyro=GyroIOSim()` to `build_drivetrain`.

### Drivetrain (`subsystem/drivetrain/`)

- `swerve_math.py`: pure-Python kinematics (inverse, forward least-squares), `discretize`, `desaturate`, `optimize`, `cosine_scale`, `field_to_robot`, `pose_exp`. Uses its own `ChassisSpeeds`/`ModuleTarget` dataclasses so it is the same on RobotPy 2026 and 2027.
- `module.py`: `SwerveModule` seeds the steer encoder from the absolute encoder on the first good reading, re-seeds while disabled if they disagree by more than 2 degrees, and flags `encoder_failed` (dashboard alert, wheel stopped) after `SEED_TIMEOUT_LOOPS`.
- `drivetrain.py`: `Drivetrain(commands2.Subsystem)` reads modules then gyro each loop, keeps a continuous heading (falls back to wheel spin rate if the gyro drops out), runs WPILib's pose estimator and plain odometry, and logs struct-typed poses, module states and chassis speeds under `/Drive/`.
- `drivetrain_sim.py`: `DrivetrainSim` builds a simulated drivetrain and tracks `true_pose` from each wheel's real motion; use it in tests.

WPILib math classes that moved or were renamed in 2027 (geometry, kinematics, estimator) are imported from `utils/wpimath_compat.py`; alerts from `utils/alerts.py`. Use `wpilib.RobotState.isDisabled()` (not `DriverStation`), which exists in both versions.

### Teleop driving (`commands/drive/`)

`bindings.py` makes `TeleopDrive` the drivetrain's default command and binds the driver actions from the YAML (robot-relative hold, slow toggle, heading re-zero, X-lock hold, D-pad heading snaps, cancel all). `TeleopDrive` takes plain callables for the sticks so tests drive it without a controller; it shapes the translate stick as a vector (`stick_shaping.py`: round deadband, curve, `VectorSlewLimiter`), flips driver directions for the red alliance (`utils/alliance.py`, works on 2026 and 2027), and holds heading with `HeadingLock` when the rotate stick is released. Driver feel lives in `TeleopSettings`. `robot.py` creates the `InputFactory` before any subsystem. Tests: `tests/drivetrain/test_teleop.py` (both alliances, on the drivetrain sim) and `tests/test_teleop_controller.py` (whole robot with a simulated Xbox controller).

### SysId and calibration (`utils/sysid/`, `commands/drive/characterization.py`, `commands/drive/calibrate_offsets.py`)

`utils/sysid/` is mechanism-agnostic: a `Characterizable` (name, subsystem, `set_voltage`, `read` returning a `Reading` of applied volts, position and velocity, `angular` flag, `SysIdSettings`, `Gravity` (NONE, ELEVATOR or ARM, which adds kG to the fit; an arm's position is radians with 0 = level) and optional `min_position`/`max_position` that end a test before a hard stop) is wrapped in `SysIdTests`, which builds the four `commands2.sysid.SysIdRoutine` tests (each settles at 0 V first; dynamic tests have their own shorter timeout), writes the wpilog only on the real robot (`log_to_wpilog`), and publishes a quick least-squares kS/kV/kA estimate (`fit.py`) under `/Characterization/<name>/estimate/`. Each mechanism's settings are editable on the dashboard under `/Characterization/<name>/settings/` with a `SysId <name> preset` chooser (Config default, Gentle, Slow ramp) (`tunable.py`); tests are `DeferredCommand`s that read them when they start, values reset to the config on boot, and `max_volts` can't be raised past the config. `CharacterizationChooser` puts every option on one dashboard chooser and `bind()` ties it to controller buttons that only work while enabled in test mode: hold `characterization.run_test` (A) to run the picked test, release to stop (WPILib's SysId safety advice; nothing starts on its own when test mode is enabled). Angular mechanisms log rotations to the wpilog (WPILib's `angularPosition` takes turns), so the SysId app's angular kV/kA are per rotation. Swerve registers drive (wheels locked at 0°, averaged over the four modules, 4 V step), a slow-ramp drive kS/kV test (`Drive feedforward (slow)`, 0.1 V/s, `fit_ka=False`), steer (on blocks, unwrapped angles), `WheelRadiusCharacterization` (spin in place at 0.25 rad/s, gyro vs wheel rotation) and `SteerStepTest` (90° steps, settle time and overshoot per corner). `CalibrateOffsets` runs while disabled from the `Characterization/Calibrate offsets` dashboard button and publishes `offsets_rot` text to paste into the robot config. Measured drive and steer kS/kV/kA in the robot config (`drive_feedforward`, `steer_feedforward`) also set the sim motor plants. Tests run commands through the real `CommandScheduler` with paused sim timing (`tests/test_sysid.py`, `tests/drivetrain/test_characterization.py`, `tests/test_test_mode.py`).

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

## Documentation for students
The code's main readers are students who are fairly new to programming. Code they will read or edit (config, subsystems, commands, utilities) needs:
 - A module docstring that says what the file is for and how to use it, in plain language
 - Docstrings on every public class, function and property with `Args:`, `Returns:` and units (meters, radians, rotations)
 - A short `>>>` example on anything a student calls or edits directly
 - Comments on config values saying where the number came from and how to re-measure it
Docstring examples in `config/` are run by `tests/drivetrain/test_swerve_config.py` (doctest), so keep them correct; add new modules with examples to that test.

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
