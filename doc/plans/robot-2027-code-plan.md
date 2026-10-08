# robot-2027 Code Plan

Oct 8, 2026 · @Tuffin

We start robot-2027 as a new repo seeded with robot-2026's tooling (CI, requirements, Makefile, templates, input system) but none of its robot code, then build the new swerve on a long-lived `swerve-dev` branch in small, tested steps. Design decisions come from the [research doc](https://claude.ai/code/artifact/346eeaaa-d25a-42b2-893d-d802298dc40d).

## 1. Repo setup

A raptacon org admin creates an empty `raptacon/robot-2027` (no README, no license, so the seed commit goes in cleanly) and gives Claude push access. Claude then pushes the seed commit and opens the swerve branch.

Settings to turn on (branch protection is planned as part of M9; any of these can be done sooner):

- [ ] Branch protection on `main`: require a PR, one CODEOWNERS review, and passing CI checks; no force pushes
- [ ] Same protection on `swerve-dev` once it exists, minus the CODEOWNERS review so students can iterate faster
- [ ] GitHub Pages from Actions (for the pdoc docs)
- [ ] `CODE_SIGN_CERT` environment and its two secrets, only if host tools move to this repo (section 2)
- [ ] Issue labels: `swerve`, `sim`, `ci`, `hardware`, `good first issue`
- [ ] Add the repo to this project in Project settings so every thread can work in it

## 2. What carries over from robot-2026

The rule: tooling, team infrastructure and game-independent utilities come over with their tests; anything specific to the 2026 robot or game stays behind. The seed is copied from robot-2026 at commit [7b5e8e0](https://github.com/raptacon/robot-2026/tree/7b5e8e0d87e1e0eaeee1dd6f158ede12b141aa50) as one commit, with a note in the message saying where it came from.

| Item | Decision | Notes |
| --- | --- | --- |
| `.github/` workflows, PR and issue templates | Carry | CI changes in section 3 |
| `CODEOWNERS`, `.gitignore`, `LICENSE` | Carry (`.flake8` replaced by ruff config in `pyproject.toml`) | Review CODEOWNERS list for 2027 mentors |
| `Makefile`, `Dockerfile`, `deploy.bat`, `deploy_utils/` | Carry | Rename the docker image tag from `raptacon2022_build` |
| `pyproject.toml`, `requirements.txt`, `robot_requirements.txt` | Carry, trimmed | Keep robotpy 2026.\* for now (rev, phoenix6, navx, pathplannerlib, photonlibpy, urcl); drop `cryptography`, `py2app`, `macholib`, `modulegraph` until host tools return |
| `CLAUDE.md`, `README.md` | Carry, rewritten | Architecture section rewritten for the new layout; keep build, test, NT and unit-test rules |
| `robot.py` | Carry, simplified | Keep `TimedCommandRobot`, the exception-catching wrapper and logging setup; fix the 20 Hz comment |
| `utils/input/`, `utils/controller/`, `utils/math/` (InputFactory) | Carry with tests | Needed for teleop driving; game-independent and already tested |
| `utils/datalog_bridge.py`, `utils/loop_timing.py`, `utils/geometry.py`, `utils/sim/` | Carry with tests | Logging, loop timing, geometry helpers, macOS controller bridge |
| `doc/build_docs.py` | Carry | Docs build step stays in CI; new guides go in `doc/` (section 9) |
| `examples/` SysId examples (flywheel, turret, hood) | Carry | References for the shared SysId tool (section 8); drop the `examples/robotpy` submodule |
| `host/` tools and `gui_release.yml`, `utils/match_monitor/`, `utils/nfc/`, NFC battery tracker | **Phase 2** | Controller automations and match tooling come back after the swerve is solid (section 10) |
| `subsystem/drivetrain/`, `physics.py`, swerve constants | **Leave behind** | Replaced by the new design |
| `subsystem/mechanisms/`, `commands/`, `autonomous/`, `constants/field_target_constants_2026.py`, `utils/odometry_logic_2026.py` | Leave behind | 2026 game and robot specific |
| `deploy/pathplanner/`, `data/inputs/2026bot.yaml` | Leave behind; start fresh | New controller YAML with only drivetrain actions |
| `subsystem/localization/` + `VISION.md` | Leave for now | Port in M7 once the drivetrain has a stable pose estimator API |

**Utility and telemetry assessment.** Only standalone utilities with no tie to the old robot structure come over; robot logic and structure stay behind.

| File | Verdict | Why |
| --- | --- | --- |
| `data/telemetry.py` | Leave behind, keep the ideas | Hard-wired to the old drivetrain class, re-reads the CANcoder and module state a second time every loop, and repeats DS state that `DriverStation.startDataLog` already records. Its good idea (struct topics for module states, chassis speeds and pose that AdvantageScope draws) is built into the new drivetrain's logging (section 7) |
| `utils/datalog_bridge.py` | Carry | Sends Python `logging` into the wpilog and lets the dashboard change log level at runtime; no robot coupling |
| `utils/loop_timing.py` | Carry, small fix | Loop and scheduler timing stats are exactly what we need on a Python robot; switch its SmartDashboard output to `ntproperty` and publish at 10 Hz |
| `subsystem/health_and_status.py` | Carry, small fix | Battery, brownout, CAN utilization and errors, CPU and memory, which section 7 needs. Default update rate is every loop (dozens of NT writes plus /proc reads); change the default to 0.5 s |
| `subsystem/robot_state.py` | Phase 2 | Mirrors DS state to NT for the match monitor; comes back with it |
| `utils/spark_utils.py` | Fold into `module_io_spark.py` | Its signal-frame setup (bus voltage, applied output, current, temperature, position, velocity) is the right set for SysId logging |
| `utils/geometry.py` | Carry with tests | Inch/transform helpers; robot dimensions move into the per-robot config instead of `constants/robot_geometry.py` |
| `utils/position_calibration.py`, `utils/spark_max_callbacks.py` | Phase 2 | Homing for turrets and arms, not needed for swerve |
| `utils/passive_range_finder.py` | Leave behind | 2026 shooter specific |

Tests that come with the carried utilities: `conftest.py`, `pyfrc_test.py`, `test_input_shaping.py`, `test_input_validation.py`, `test_nt_mapping.py`, `test_virtual_analog.py`, `test_managed_str.py`, `test_deadband_graph.py`, `test_geometry.py`. `test_fuzz_teleop.py` is rewritten against the new `robot.py`.

**Controller mapping baseline.** Keep the InputFactory YAML format and start a new `data/inputs/swerve_test_bot.yaml` with drivetrain actions only. Translation axes get deadband 0 in the YAML because InputFactory shapes each axis separately; `TeleopDrive` applies the radial deadband and squared magnitude to the stick vector instead.

| Driver input | Action | Change from 2026 |
| --- | --- | --- |
| Left stick | Translate (field-relative) | Same; radial shaping moves into `TeleopDrive` |
| Right stick X | Rotate | Same |
| Right bumper (hold) | Robot-relative | Same |
| Y | Slow mode toggle | Same |
| Start | Re-zero heading for the current alliance | New |
| X (hold) | X-lock | New |
| D-pad | Snap heading to 0°, 90°, 180°, 270° | New (D-pad was intake) |
| Back | Cancel all commands | Same |
| Right trigger (hold) | Auto-align to target | Kept as a slot; returns with vision in M7 |

The operator controller starts empty and is filled in with the 2027 mechanisms.

## 3. Automations

Keep robot-2026's pipeline as the base and add a few checks that would have caught this year's problems.

| Job | Status | Change |
| --- | --- | --- |
| Unit and integration tests on Windows and macOS | Keep | Add Linux, the cheapest runner, and make it the required check |
| Sim smoke test (5 s headless) | Keep | Run on Linux too; later extend to a scripted drive test |
| flake8 critical (blocking) and extra (non-blocking) | Replaced | Ruff lint and `ruff format --check` (blocking), rules in `pyproject.toml`, the same ones pre-commit runs on every commit |
| Pre-commit hooks | Add | Ruff fix and format, YAML check, merge-conflict and large-file checks before each commit |
| Type check (pyright) | Add, informational | Reports the error count as a warning until existing findings are fixed, then becomes blocking |
| CI speed | Add | pip caching on every job; a new push cancels the superseded run |
| pdoc docs build and GitHub Pages deploy | Keep |  |
| Nightly scheduled run | Keep | Catches upstream package breaks |
| Host tools release build | Phase 2 | Returns with the host tools (section 10) |
| Coverage report | Add | Upload the report; set a floor (for example 70%) only for `subsystem/drivetrain/` once it exists |
| RobotPy 2027 preview job | Add, non-blocking | Installs the latest `robotpy==2027.*` pre-release and runs the tests, so SystemCore breakage shows up early |
| Dependabot for GitHub Actions | Add | Keeps `actions/checkout` and friends current |
| Config sanity test | Add | Every module preset and per-robot config loads; offsets in range; CAN IDs unique |

CI tests run against the sim IO layer, so they need no hardware.

## 4. Branches and workflow

`main` holds the seed and only ever gets reviewed, green merges. Swerve work happens on a long-lived `swerve-dev` branch, built up through small PRs, and merges into `main` at each milestone in section 6.

- `main`: the seed (tooling, utilities, a `robot.py` that boots with no subsystems). Protected, CODEOWNERS review.
- `swerve-dev`: branched from `main` right after the seed. Each milestone is one or more PRs into it, each with tests.
- Feature branches `swerve/<topic>` (for example `swerve/config-presets`) for each PR, so students can work in parallel. (The integration branch isn't named plain `swerve`, because git can't have both `swerve` and `swerve/<topic>`.)
- At each milestone, `swerve-dev` merges into `main` with a short release note and a tag (`swerve-m1`, `swerve-m2`, and so on).
- Use the repo's PR template; link each PR to an issue on the board.

## 5. Swerve code layout

Vendor code (REV, CTRE, NavX) appears only in files ending `_spark.py`, `_cancoder.py`, `_navx.py`; everything else imports the interfaces. That keeps us vendor-neutral and limits the SystemCore port to those files.

```
config/
  module_presets.py      # MK4I / MK4 gearing tables (frozen dataclasses, incl. steer inversion, encoder direction)
  robot_config.py        # CornerConfig / RobotConfig types and the mirrored_corners helper (any rectangle)
  robots/
    swerve_test_bot.py   # per-robot: preset choice, CAN IDs + bus, offsets, drive inversions, gains, geometry
  loader.py              # picks the robot config from persistent NT /robot/name
subsystem/drivetrain/
  io/
    module_io.py         # ModuleIO interface + ModuleInputs dataclass (SI units)
    module_io_spark.py   # SparkMax + NEO drive and steer
    module_io_sim.py     # DCMotorSim plants fitted from SysId values
    abs_encoder_io.py    # AbsoluteEncoderIO interface
    abs_encoder_cancoder.py
    gyro_io.py           # GyroIO interface
    gyro_io_navx.py
    gyro_io_sim.py
  swerve_math.py         # pure-Python kinematics, discretize, desaturate, optimize (same on 2026 and 2027)
  module.py              # optimize, cosine scale, feedforward, seeding + health checks
  drivetrain.py          # kinematics, discretize, desaturate, odometry, pose estimator, PathPlanner, SysId
  drivetrain_sim.py      # builds a sim drivetrain and tracks the true pose for tests
  odometry_sampler.py    # swappable: once per loop now, thread later
commands/drive/
  teleop_drive.py        # radial deadband, shaping, slew, field-relative, heading lock
  heading_lock.py
  x_lock.py
  characterization.py    # drive SysId, wheel radius, steer step test
  calibrate_offsets.py   # test-mode: prints CANcoder offsets as Python
physics.py               # thin: steps the sim IO and BatterySim
tests/drivetrain/        # config sanity, kinematics, module, sim scenarios
```

## 6. Milestones

Each milestone ends with green CI and a merge of `swerve-dev` into `main`. M0 to M4 need no robot; M5 onward needs the test bot on blocks or carpet.

1. **M0 Seed:** new repo with the carried-over files from section 2, the CI changes from section 3, wpilog logging on, and the four guides copied into doc/ (section 9). Done when CI is green and `robotpy sim` boots with no subsystems.
2. **M1 Config:** module presets (MK4i L1 to L3 with L2 default, MK4 L1 to L4) and the first robot config. Done when config sanity tests pass and wheel travel per motor rotation matches each preset.
3. **M2 IO interfaces and sim IO:** `ModuleIO`, `AbsoluteEncoderIO`, `GyroIO`, and their sim versions with latency and boot-failure options, plus per-module input logging (section 7). Done when a single simulated module tracks angle and speed commands in a unit test.
4. **M3 Module and drivetrain:** module logic, kinematics, discretize, desaturate, odometry, pose estimator. Done when sim tests show no drift driving straight while rotating, and a CANcoder boot failure flags the module.
5. **M4 Teleop:** `TeleopDrive` with the InputFactory, heading re-zero per alliance, heading lock, X-lock, robot-relative and slow modes. Done when sim tests cover both alliances and a driver can drive the sim with a controller.
6. **M5 Real hardware IO:** SparkMax, CANcoder and NavX implementations with the configuration baseline from the tuning guide. Done when all four modules point at 0°, 90° and 180° correctly on blocks.
7. **M6 Calibration and characterization:** offset calibration command, steer step test, drive SysId (on carpet, wheels locked at 0°), steer SysId (on blocks), wheel radius. SysId runs through the shared SysId tool (section 8) on a "Characterization" chooser in the main robot code, using the same IO classes and config, not a separate robot program, so the constants measured always match the code that uses them; logs go to wpilog with URCL for the SysId tool. The fitted drive and steer values also feed the sim plants. Done when measured constants are committed and the robot drives 5 m straight within 2 cm.
8. **M7 Autonomous and vision:** PathPlanner config from measured values, port localization from robot-2026. Done when a 3 m path ends within 3 cm and 2°, with and without vision.
9. **M8 SystemCore readiness:** track the RobotPy 2027 preview job, move CAN bus assignment to `CANPort`, decide on an odometry thread. Done when tests pass on the 2027 packages.
10. **M9 Repo hardening and deploy review:** turn on branch protection for `main` and `swerve-dev` (section 1) with the Linux tests and ruff lint as required checks; make pyright blocking once its findings are cleared; review deploy provenance and event deploys (below) and adopt what's useful. Done when protections are on and the deploy decision is written down here.

**Deploy provenance and event deploys (to review at M9).** 6328 and other teams make sure the code on the robot is always in git: their event deploy commits any uncommitted changes to an event branch before deploying. A Python version for us: a `make deploy` that refuses a dirty tree (or commits to `event/<event-name>`), tags each event deploy, and publishes the git hash, branch and dirty flag on the dashboard and in the wpilog, so the pit checklist's "deployed code matches the latest commit" check is one glance.

## 7. Data logging

Logging is part of the swerve work from M2, not an add-on: every tuning step and the sim validation depend on it. Everything goes to one wpilog per match that AdvantageScope opens directly.

| What | How | When |
| --- | --- | --- |
| All NetworkTables values, console output, DS state and joysticks | `DataLogManager.start()`, `logNetworkTables(True)`, `DriverStation.startDataLog` (already in robot-2026's `robot.py`) | M0 |
| SparkMax CAN data (voltage, current, position, velocity, faults) | URCL (`robotpy-urcl`), needed by the SysId tool | M2 |
| Per-module inputs each loop: measured angle and speed, setpoints, applied volts, current, temperature | One `ModuleInputs` dataclass per module, published as struct-typed NT values (`SwerveModuleState[]` arrays AdvantageScope draws as swerve vectors) | M2 |
| Drivetrain: pose, odometry-only pose, vision poses accepted and rejected, chassis speeds commanded vs measured, gyro | Struct-typed NT values under `/Drive/` | M3 |
| Health: loop time, CAN utilization, battery voltage, brownouts, seed failures, module angle disagreement | `LoopTimer` plus a health publisher; alerts via WPILib `Alert` | M3 |
| Logs to USB stick when present, roboRIO otherwise; old logs pruned | `DataLogManager` default behavior; a cleanup step in the deploy script | M0 |

Two rules keep it cheap in Python: read each signal once per loop into the inputs dataclass and log that same object, and publish dashboard-only values at 10 Hz instead of every loop.

## 8. Reusable SysId tool

One SysId helper serves every mechanism, so the swerve is just its first user. A mechanism registers how to apply voltage and read position and velocity, and gets the four SysId tests as commands on a "Characterization" chooser.

- `utils/sysid/characterizable.py`: a small interface (`set_voltage`, `get_position`, `get_velocity`, units, safe limits).
- `utils/sysid/routines.py`: builds `commands2.sysid.SysIdRoutine` quasistatic and dynamic tests from that interface, with ramp rate, step voltage and timeout from config; logs to wpilog in the format the SysId tool reads.
- `utils/sysid/chooser.py`: collects every registered mechanism into one dashboard chooser, enabled in test mode only.
- Swerve registers three: drive (all wheels locked at 0°, on carpet), steer (on blocks), and a wheel-radius routine that isn't SysId but lives on the same chooser.
- Later mechanisms (shooter flywheel, turret, arm) register the same way, replacing the separate `examples/*-sysid` robots.

## 9. Docs in the repo

The research doc, theory manual, tuning guide and this plan are copied into the repo as Markdown at M0, under `doc/` (robot-2026's existing docs source folder; `docs/` is where CI writes the generated pdoc site).

```
doc/
  swerve/
    research-and-review.md
    theory-manual.md
    tuning-and-calibration.md
    calibration-log.md     # offsets and wheel radius, dated, per module
  plans/
    robot-2027-code-plan.md
```

The repo copy becomes the version of record once it lands; changes after that go through PRs so they're reviewed with the code they describe.

## 10. After the swerve: phase 2

Once M6 is done (calibrated, characterized swerve), bring back the rest of robot-2026's tooling, each as its own PR with its tests:

1. Controller automations: the host controller config editor and web editor, `gui_release.yml`, and the full controller YAML.
2. Match tooling: match monitor and log uploader.
3. NFC battery tracking.
4. Vision and localization (M7 above).
5. Mechanisms for the 2027 game, each using the shared SysId tool.
