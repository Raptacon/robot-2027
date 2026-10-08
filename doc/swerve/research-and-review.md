# Swerve software research and robot-2026 review

Oct 7, 2026 · @Tuffin

robot-2026's swerve problems come mostly from sensor timing, two competing steering angle sources, and gear constants that don't match the hardware, not from the overall class layout. A fresh bot should keep the module/drivetrain split but rebuild the module around one angle source, fast odometry, characterized feedforward, and a gear ratio chosen in config.

## 1. How strong teams structure swerve code

The teams whose swerve "just works" (6328, 254, 1690, and anyone using the CTRE or AdvantageKit templates) all split the code the same way: hardware access at the bottom, one module class that knows nothing about the robot, one drivetrain that owns kinematics and odometry, and commands that only send `ChassisSpeeds`.

| Layer | Owns | Must not do |
| --- | --- | --- |
| Config (dataclasses / constants) | Module type, gear ratio preset, wheel radius, CAN IDs, offsets, inversions, gains, max speed | Contain logic or be mutated at runtime |
| Module IO (one per hardware combo) | Talking to motors, encoders: configure once, read signals, send setpoints in SI units | Know about the field, the driver, or other modules |
| Module | Optimize + cosine-scale a `SwerveModuleState`, drive feedforward, report position/state | Read joysticks or the gyro |
| Gyro IO | Yaw (and rate) in CCW-positive radians | Apply offsets that odometry doesn't know about |
| Drivetrain subsystem | Kinematics, discretize, desaturate, pose estimator, high-rate odometry, field-relative math, PathPlanner hookup, SysId routines | Shape joystick input |
| Commands | Teleop drive, heading lock, auto-align, characterization | Touch motor objects |

**Why the IO split matters for us.** An IO interface (`ModuleIO` with `ModuleIOSpark`, `ModuleIOTalonFX`, `ModuleIOSim`) lets the same module code run on the real robot, in simulation with a real motor model, and with a different motor vendor. It's how AdvantageKit's templates work, and it ports to Python cleanly as an abstract base class.

**Gear ratio as a selectable preset.** Good code never hard-codes the ratio inside the module class. It has a table of SDS presets and a per-robot choice, and every conversion factor is derived from that choice.

| SDS preset | Drive ratio | Steer ratio | NEO free speed, 4 in wheel (m/s, computed) |
| --- | --- | --- | --- |
| MK4i L1 | 8.14 : 1 | 150/7 (21.43 : 1) | 3.71 |
| MK4i L2 (our default, matches current code) | 6.75 : 1 | 150/7 (21.43 : 1) | 4.47 |
| **MK4i L3** | 6.12 : 1 | 150/7 (21.43 : 1) | 4.93 |
| MK4 L1 to L4 (alternative) | 8.14 / 6.75 / 6.12 / 5.14 : 1 | 12.8 : 1 | 3.71 / 4.47 / 4.93 / 5.87 |

Our default is the MK4i with L2 gearing, the values already in the current code; L3 and the others are one-line preset changes. The MK4 stays in the table so a different module is a config change, not a rewrite: it has the same drive ratios but a 12.8 : 1 steer ratio and steers in the opposite direction, so steer inversion changes too. Free speeds are theoretical (motor free RPM ÷ ratio × wheel circumference); real top speed is usually 80 to 90% of that.

A Python sketch of the shape we want:

```python
@dataclass(frozen=True)
class ModuleGearing:
    drive_ratio: float
    steer_ratio: float
    steer_inverted: bool


MK4I = {
    "L1": ModuleGearing(8.14, 150 / 7, True),
    "L2": ModuleGearing(6.75, 150 / 7, True),
    "L3": ModuleGearing(6.12, 150 / 7, True),
}
MK4 = {
    "L1": ModuleGearing(8.14, 12.8, False),
    "L2": ModuleGearing(6.75, 12.8, False),
    "L3": ModuleGearing(6.12, 12.8, False),
    "L4": ModuleGearing(5.14, 12.8, False),
}


@dataclass(frozen=True)
class DriveConfig:
    gearing: ModuleGearing = MK4I["L2"]
    wheel_radius_m: float = 0.0508  # replace with measured value
    drive_motor: DCMotor = DCMotor.NEO(1)
    # everything else (m/rot, max speed, kV) is derived from these
```

## 2. Control: what good teams get right

Reliable swerve is mostly about timing and units, not clever math. These are the practices that show up in every well-regarded codebase.

**Steering**

- One angle source of truth. Either close the loop on the absolute encoder directly (CANcoder fused/remote into a TalonFX, or a through-bore/duty-cycle encoder on the Spark), or seed the motor's relative encoder from the absolute encoder and then read *only* the relative encoder for both control and odometry. Mixing the two is a classic source of jitter.
- Seeding must be verified: wait for a valid, fresh absolute reading, retry, and re-seed while disabled if the module is still and the two disagree by more than a degree or two.
- Steer loop runs on the motor controller (1 kHz) with position wrapping, high kP, a little kD, no ramp rate, and brake mode.
- Call `SwerveModuleState.optimize()` and `cosineScale()` against the *same* angle the controller is using.

**Drive**

- Velocity control = feedforward does the work, PID trims. Get kS, kV, kA from SysId or a simple FF characterization run, not from 12 V ÷ max speed.
- Measure wheel radius on carpet with a wheel-radius characterization (spin in place, compare gyro rotation to wheel travel). Don't fold the error into friction coefficient or a fudge factor.
- For NEOs, cut the built-in encoder's velocity filtering (`uvwMeasurementPeriod`, `uvwAverageDepth`); the default 32 ms × 8-sample average adds roughly 100 ms of lag, which is why Spark drive PID often feels "dead".
- Current limits sized to avoid wheel slip (often 40 to 60 A drive on NEO, lower on light robots) and brownout.

**Odometry and the main loop**

- Discretize every commanded `ChassisSpeeds` (`ChassisSpeeds.discretize(speeds, dt)`) before kinematics, or the robot arcs when it drives and spins at the same time.
- Sample drive position, steer angle and gyro yaw together and fast: CTRE's swerve API and AdvantageKit run a separate odometry thread (100 Hz on Sparks, 250 Hz on CAN FD with TalonFX). At minimum, status frames for drive position and gyro must be at or faster than the robot loop.
- Use one consistent unit system (meters, radians, seconds) everywhere; convert degrees only for display.
- Log everything (module setpoint vs measured, loop time, CAN utilization) so failures can be diagnosed from a match log.

**Characterize, then tune, in this order:** steer PID, drive kS/kV (SysId), wheel radius, max speed and acceleration, PathPlanner translation and rotation PID, vision standard deviations.

## 3. Driver input schemes

Most top teams converge on the same base scheme: field-relative, left stick translates, right stick X rotates, with a few assists layered on through buttons.

| Feature | How good teams do it | Notes for us |
| --- | --- | --- |
| Field-relative drive | Use the pose estimator's rotation, with an "operator forward" that is 0° on blue and 180° on red (always-blue coordinates) | Rotate the field frame by 180° on red rather than negating stick axes, so pose and driving can't disagree |
| Re-zero heading | A dedicated button sets heading to "facing away from my driver station" for the current alliance | Essential when auto didn't run or the gyro drifted |
| Translation shaping | Apply deadband to the stick *magnitude* (radial), then square or cube the magnitude, keep the direction | Per-axis deadband makes diagonals sticky; our InputFactory curves can do the radial version |
| Rotation shaping | Separate deadband and curve, usually squared, scaled to a lower max angular speed than the theoretical one |  |
| Slew-rate limiting | Limit chassis acceleration (or translational vector acceleration) to prevent tipping and wheel slip | Better on the chassis vector than per stick axis |
| Heading lock / snap | A ProfiledPIDController on heading with continuous input; holds heading when the rotation stick is idle, snaps to 0/90/180/270° or a target on a button | Our auto-align command is a version of this |
| Robot-relative toggle | Held button for fine intake alignment | We already have this on right bumper |
| Slow mode | Held or toggled scale factor | Already present |
| X-lock | Button points all wheels inward to resist pushing | Cheap to add |

The driver should never feel loop-rate artifacts. Square the magnitude, not the axes, and keep everything in one place (a `TeleopDrive` command) so it can be unit-tested.

## 4. Reference code worth starting from

We're staying on NEO + SparkMax and want to avoid vendor lock-in, so the best fit is our own Python port of AdvantageKit's IO-layer structure: a SparkMax implementation first, with room for TalonFX later. CTRE's Phoenix 6 swerve API is excellent but only works with TalonFX, so it's a reference, not our base.

| Reference | Language | Hardware | Why it's useful |
| --- | --- | --- | --- |
| [Phoenix 6 swerve API](https://v6.docs.ctr-electronics.com/en/2025/docs/api-reference/mechanisms/swerve/swerve-overview.html) | Java, C++, **Python** | TalonFX/FXS + CANcoder + Pigeon 2 | Vendor-maintained module, odometry thread, SysId routines; Tuner X generates the constants. Doesn't support Spark/NEO |
| [AdvantageKit Spark swerve template](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template) | Java | Spark Max/Flex, any abs encoder, Pigeon/NavX | Best-documented IO-layer design, 100 Hz odometry thread, built-in FF and wheel-radius characterization |
| [AdvantageKit TalonFX swerve template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/) | Java | TalonFX + CANcoder | Same structure for CTRE hardware |
| [YAGSL](https://yagsl.yassrobotics.com) | Java | Most combos, including MK4 + NEO + CANcoder | Huge config coverage and a list of known Spark pitfalls; good for cross-checking our constants |
| [WPILib kinematics and odometry docs](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html) | All | Any | The math we build on, including discretize and pose estimation |
| [RobotPy examples (SwerveBot)](https://github.com/robotpy/examples) | Python | Generic | Minimal working Python swerve, good for idioms |
| [swervepy](https://github.com/EWall25/swervepy) | Python | Configurable | Python swerve library with a component (IO) design worth reading |
| [PyKit (1757)](https://www.chiefdelphi.com/t/introducing-pykit-log-replay-for-python/508084) | Python | Any | AdvantageKit-style logging/replay for Python; early (v0.2.x), no swerve template yet |
| [Python Swerve Template (CD)](https://www.chiefdelphi.com/t/python-swerve-template/467987) | Python | Generic | Small and readable, author says it is incomplete |

Useful Chief Delphi threads: [Spark Max speed feedback lag](https://www.chiefdelphi.com/t/split-thread-spark-max-speed-feedback-control/418107?page=2), [snap to heading](https://www.chiefdelphi.com/t/snap-to-heading/454211), [one slow module causing drift](https://www.chiefdelphi.com/t/1-swerve-module-is-slow-causing-drift/512889), [REV swerve in RobotPy](https://www.chiefdelphi.com/t/coding-for-rev-robotics-swerve-drive-using-robotpy/482757).

## 5. What went wrong in robot-2026

Reviewed at commit [7b5e8e0](https://github.com/raptacon/robot-2026/tree/7b5e8e0d87e1e0eaeee1dd6f158ede12b141aa50) (May 16, 2026). The layout (module class, drivetrain subsystem, pose estimator, PathPlanner) is reasonable; the unreliability comes from the items below, ordered by how likely each is to cause the symptoms. Paths are relative to the repo; line numbers are from that commit.

| # | Problem | Where | Effect on the robot | Fix in the new design |
| --- | --- | --- | --- | --- |
| 1 | Two steering angle sources. The steer PID runs on the NEO's relative encoder, seeded once at boot, while optimize, cosine scaling and odometry read the CANcoder. If the CANcoder isn't ready at boot, the seed silently falls back to 0° | `swerve_module.py` L236, L313-329, L345-360, L402 | A bad or stale seed leaves one module permanently off; the controller and the optimizer disagree near setpoints, causing jitter and "crooked wheel after boot" | One angle source, seeded with retries and verified, re-seeded while disabled |
| 2 | Gear constants don't match the hardware. Code is MK4i L2 (drive 6.75, steer 150/7); that matches the chosen default, but the measured speeds below don't fit L2. Measured max speed of 4.6 m/s, and 4.75 m/s on front-right, is above the NEO L2 free speed of 4.47 m/s, which suggests the real gearing is faster than the code assumes (inferred) | `constants/swerve_constants.py` L57, L94-118; `swerve_drivetrain.py` L38-39 | If the modules are L3, every distance and speed is about 10% off (6.75 ÷ 6.12), so odometry, feedforward and PathPlanner are all wrong | Gearing chosen from a preset table, verified on the robot (mark a wheel, rotate the motor N turns) |
| 3 | Wheel friction coefficient (1.013) multiplied into the distance conversion | `swerve_constants.py` L99-101 | Hides #2 behind a fudge factor and gives PathPlanner a meaningless COF | Measure wheel radius with a characterization routine; keep COF for PathPlanner only |
| 4 | Position status frames slowed to 50 ms on all 8 Sparks while the loop runs at 20 ms | `swerve_module.py` L40-41, L85-86 | Odometry uses wheel positions up to 50 ms old that aren't time-aligned with the gyro; pose drifts and vision fusion fights it | Drive position and gyro at loop rate or faster, ideally a 100 Hz odometry thread |
| 5 | NEO velocity filtering left at default. The `quadrature*` settings apply to an external quadrature encoder, not the NEO hall sensor; `uvwMeasurementPeriod` / `uvwAverageDepth` are never set | `swerve_module.py` L242-247 | Drive velocity feedback lags about 100 ms, so drive kP (0.0021) does essentially nothing and drive is open-loop | Set the uvw window (for example 8 to 16 ms, depth 2 to 4), then tune kP |
| 6 | Drive feedforward not characterized: kS = 0, kV = 12 V ÷ max speed, kA = 0. Front-right got its own max speed to make it match | `swerve_module.py` L159-164; `swerve_drivetrain.py` L38-39, L58 | Modules run at different real speeds, so the robot curves and auto paths miss; the front-right override masks a mechanical or config problem | SysId or a simple FF run; identical constants on all modules, and a hardware check of any outlier |
| 7 | 0.25 s ramp rate on the steer motor's closed loop (and the drive's) | `swerve_module.py` L230-231, L268-269 | Wheels lag behind direction changes, so the robot skids and slides when the driver changes direction | No ramp on steer; limit acceleration at the chassis level instead |
| 8 | No `ChassisSpeeds.discretize` before kinematics | `swerve_drivetrain.py` L266 | Robot arcs when translating and rotating at once, in teleop and auto | Discretize with the loop period |
| 9 | No driver heading reset. Heading is set at boot, and teleopInit resets the pose to a default start pose when no auto ran. `reset_heading()` sets rotation to 0°, which is wrong for a red driver | `robotswerve.py` L159-166; `swerve_drivetrain.py` L149-159 | If the robot wasn't placed exactly at that pose, field-relative is rotated for the whole match with no way to fix it | Re-zero button that uses the alliance's "forward" (0° blue, 180° red) |
| 10 | Red alliance handled by negating stick X/Y, separately from the pose's rotation | `swerve_drivetrain.py` L193-201 | Works only while pose rotation is exactly right; any reset error doubles up | Rotate the field frame by the operator-forward angle |
| 11 | Mutable default argument: one `SwerveModuleMk4iL2Consts()` instance is shared by FL, BL and BR, and `setattr` writes each module's offset onto it | `swerve_module.py` L108, L140 | All three modules report BR's offset; only `physics.py` L81 reads it today, so simulation steering offsets are wrong | Frozen dataclass per module; never mutate shared constants |
| 12 | Flash is written on every boot for all 8 Sparks, without a reset to defaults; disabledInit re-applies full configs | `swerve_module.py` L195, L199, L304-311; `robotswerve.py` L126 | Slower boot, flash wear, and old settings can survive since nothing resets them | Reset safe parameters + configure every boot; persist only when changed; idle mode via a light call |
| 13 | CANcoder update rate is set on `position`, but the code reads `absolute_position` | `swerve_module.py` L187 vs L321, L340 | The intended rate isn't applied to the signal used (inferred; impact depends on firmware defaults) | Set rates on the signals actually read |
| 14 | Simulation is ideal: it writes the commanded state straight into the encoders | `physics.py` L136-150 | Sim can't reveal any of the control or timing problems above | Simulate each module with `DCMotorSim` and real gains |
| 15 | If module creation throws, the drivetrain runs with zero modules and only logs an error | `swerve_drivetrain.py` L82-87 | Robot enables but won't drive, with no obvious cause on the field | Fail loudly on the drivetrain; show module health on the dashboard |
| 16 | Many blocking-style reads per loop: each module reads the CANcoder several times per cycle, plus SmartDashboard reads in every module's periodic | `swerve_module.py` L340, L400-402, L477 | Python loop overruns make the 20 ms loop jittery (not measured here) | Read each signal once per loop into an inputs struct; check `LoopTimer` output |

Smaller items: the steer motor idles in coast (`swerve_module.py` L227), docstrings still mention Falcon 500s, and `robot.py` comments say 20 Hz while the code runs at 50 Hz.

## 6. Recommendations for the fresh MK4i swerve bot

Build it in this order, and don't move to the next step until the current one is verified on the robot.

1. **Decide the motor stack first.** We stay on NEO + SparkMax + CANcoder and avoid tying the code to one vendor. Write our own module using the layer design in section 1, with abstract interfaces for the motor (`ModuleIO`), the absolute encoder (`AbsoluteEncoderIO`: CANcoder now, Thrifty or Redux possible) and the gyro (`GyroIO`: NavX now, Pigeon 2 possible). Vendor code lives only inside those implementations, so a later move to Krakens means writing one new class, not a rewrite.
2. **Config layer:** frozen dataclasses with MK4i and MK4 presets, MK4i L2 selected; per-module CAN IDs, offsets and inversions; everything else derived.
3. **Module IO:** reset and configure each controller every boot; one angle source; drive and steer in meters, radians, seconds; status frames at or faster than odometry; NEO uvw velocity window shortened.
4. **Module tests on blocks:** each module points forward at 0°, turns CCW positive, and drives forward with positive speed; verify gear ratio by counting wheel turns.
5. **Drivetrain:** kinematics, discretize, desaturate, `SwerveDrive4PoseEstimator`, odometry at 100 Hz if Python loop time allows (measure first), heading re-zero, X-lock.
6. **Characterize:** steer PID, drive SysId (kS, kV, kA), wheel radius on carpet, real max speed and acceleration. Update PathPlanner's robot config with the measured values.
7. **Teleop command:** radial deadband, squared magnitude, chassis slew limit, heading lock and snap, robot-relative and slow modes through InputFactory.
8. **Simulation and logging:** per-module `DCMotorSim` with the same gains; log setpoint vs measured per module, loop time, and CAN utilization to wpilog for AdvantageScope.

Before building, confirm which gear set is physically in the modules (L2 or L3) with the wheel-turn count in the tuning guide. If it isn't L2, change one config line.

## 7. Simulation plan

I didn't find any team that publishes measured sim-vs-real accuracy for a swerve sim; even the best ones say they verify code logic, not hardware fidelity. The reliable way to get a matching sim is to fit it to our own robot's SysId data and check it against our match logs, so that's the plan. Priority is a sim that is good enough for code checks: the IO-layer sim, the findings it must model, and the automated tests come first; the log-replay validation below is a later, optional step.

**What to crib from**

| Source | Language | What it models | Fit for us |
| --- | --- | --- | --- |
| [AdvantageKit Spark swerve template](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template) (6328) | Java | `ModuleIOSim` per module with `DCMotorSim` for drive and steer, swapped in for real IO | Best structure to port: the sim is just another IO implementation. It uses separate sim kS/kV, so it doesn't claim to match hardware |
| [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton) (5516 Iron Maple) | Java | 2D rigid-body physics (dyn4j): wheel friction, collisions with field elements and robots | Most physical sim in FRC; Java only, and users report it misbehaving with non-default motor configs ([CD thread](https://www.chiefdelphi.com/t/maplesim-strange-behavior-need-help/502245)) |
| [CTRE swerve simulation](https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/swerve-simulation.html) | Java, C++, Python | Steer inertia and control latency; no wheel slip or scrub | Good ideas (latency, faster sim loop), but TalonFX only |
| WPILib + robotpy-rev in Python | Python | `DCMotorSim`, `LinearSystemId`, `BatterySim`, `rev.SparkSim` (runs the Spark's onboard PID) | What we build on |

**What our sim must model, mapped to the findings**

| Finding (section 5) | Sim feature that would have caught it |
| --- | --- |
| #1 two angle sources, bad boot seed | CANcoder sim that can fail or report late at boot, and a steer encoder that can be seeded wrong |
| #2, #3 wrong gear ratio and wheel radius | Plant built from *measured* ratio and radius, code built from config; a mismatch shows up as odometry error vs sim ground truth |
| #4, #5 slow status frames, NEO velocity lag | Sensor values delayed by the configured status frame period and filtered like the uvw window |
| #6 uncharacterized feedforward | Drive plant from `LinearSystemId.identifyVelocitySystem(kV, kA)` using SysId values, plus kS friction; one module can be made "tight" |
| #7 steer ramp rate | Steer plant with real inertia, run through `SparkSim` so ramp and onboard PID behave like hardware |
| #8 no discretize | Ground-truth pose integrated at 1 ms sub-steps, so arcing shows up |
| #9, #10 heading and red alliance | Scripted tests on both alliances and after heading resets |
| #16 loop overruns | Optional injected loop delay to see control degrade |

Also model battery sag with `BatterySim` and a simple wheel-slip limit (cap each wheel's force at μ × its share of robot weight), so acceleration limits can be tested.

**How we'll know it matches the robot**

1. Fit drive and steer plants from SysId logs of the real robot.
2. Replay recorded voltages from a real wpilog into the sim and compare speeds and angles. Target: module speed within about 5% and steer angle within about 2° over the run.
3. Drive the same path on the robot and in sim, then compare end pose error.
4. Re-check after any mechanical or gearing change.

**Automated tests** (pytest, run in CI): drive 3 m straight while rotating and check drift; boot with one CANcoder missing and check the module is flagged; flip alliance and check field-relative direction; verify each MK4i/MK4 preset produces correct wheel travel per motor rotation.

## Sources

- [SDS MK4 swerve module](https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module) (steer ratio 12.8 : 1, ratio options L1 to L4; drive ratio numbers are in SDS's ratio image)
- [AdvantageKit Spark swerve template](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template)
- [AdvantageKit TalonFX swerve template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/)
- [CTRE Phoenix 6 swerve overview](https://v6.docs.ctr-electronics.com/en/2025/docs/api-reference/mechanisms/swerve/swerve-overview.html)
- [REV EncoderConfig API](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/encoderconfig); uvw defaults (32 ms, depth 8) checked in the robotpy-rev 2026.0.4 stubs
- [Chief Delphi: Spark Max speed feedback/control](https://www.chiefdelphi.com/t/split-thread-spark-max-speed-feedback-control/418107?page=2)
- [Chief Delphi: Snap to heading](https://www.chiefdelphi.com/t/snap-to-heading/454211)
- [Chief Delphi: PyKit log replay for Python](https://www.chiefdelphi.com/t/introducing-pykit-log-replay-for-python/508084)
- [Chief Delphi: Python Swerve Template](https://www.chiefdelphi.com/t/python-swerve-template/467987)
- [YAGSL chassis speed discretization](https://docs.yagsl.com/overview/our-features/chassis-speed-discretization)
- [WPILib swerve kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)
- [raptacon/robot-2026 at 7b5e8e0](https://github.com/raptacon/robot-2026/tree/7b5e8e0d87e1e0eaeee1dd6f158ede12b141aa50)
