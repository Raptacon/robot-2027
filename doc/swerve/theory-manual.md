# Swerve Theory Manual

Oct 7, 2026 · @Tuffin

This manual teaches the ideas behind Raptacon's swerve drive: how a robot with four steerable wheels moves, the math that turns a driver's joystick into wheel commands, and the WPILib tools that do that math for us. Read it in order; each section builds on the last.

## 1. What swerve is

A swerve drive can move in any direction while facing any direction, because every wheel can both spin and point. Engineers call this **holonomic** motion: the robot controls all three of its planar degrees of freedom (forward/back, left/right, rotation) independently.

A tank drive can only push along its wheels, so to move sideways it must first turn. A swerve robot just points all four wheels sideways and goes.

**Parts of one SDS MK4i module**

| Part | What it does | On our robot |
| --- | --- | --- |
| Drive motor | Spins the wheel through the drive gear reduction | NEO on a SparkMax |
| Drive reduction | Trades motor speed for wheel torque; L1 is slowest and strongest, L4 fastest | L2, 6.75 : 1 (L3, 6.12 : 1, is a config option) |
| Steer (azimuth) motor | Rotates the whole wheel fork to point the wheel | NEO on a SparkMax |
| Steer reduction | Motor turns per one full wheel rotation | 150/7 (21.43 : 1) on MK4i; 12.8 : 1 on MK4 |
| Absolute encoder | Tells us the wheel's angle the instant the robot turns on, before anything has moved | CTRE CANcoder on top of the module |
| Motor encoder | Built into each NEO; counts motor rotations, but forgets where it was at power-off | NEO hall sensor |
| Wheel | 4 in (0.1016 m) nominal diameter; tread wear shrinks it | Measure, don't assume |

Two words you'll hear all the time: **azimuth** (the wheel's steering angle) and **module state** (a pair: wheel speed and wheel angle).

## 2. Coordinate frames and conventions

Almost every swerve bug is a sign or frame mistake, so learn these rules before any math.

- **Robot frame (NWU):** +X points out the front of the robot, +Y points out the left side, +Z points up.
- **Rotation is counterclockwise-positive** when viewed from above. A positive angular velocity spins the robot to the left.
- **Units are SI:** meters, seconds, radians. Convert to degrees or inches only for display.
- **Field frame ("always blue origin"):** the origin is the corner at the blue alliance's right. +X points from the blue wall toward the red wall. A robot facing the red wall has heading 0.
- **Field-relative vs robot-relative:** robot-relative "forward" is wherever the bumper points; field-relative "forward" is away from the driver, no matter which way the robot faces. On the red alliance, the driver's forward is the field's −X, a 180° offset.
- **Module angle 0** means the wheel points straight forward (+X) and a positive drive speed pushes the robot forward.

Rule of thumb: if the robot does the right thing on blue and the mirror-image wrong thing on red, the alliance offset is being applied twice or not at all.

## 3. Kinematics: chassis speeds to wheel commands

Kinematics converts what we want the whole robot to do (chassis speeds vx, vy, ω) into what each wheel must do (speed and angle), and back again.

**Inverse kinematics (what WPILib's `toSwerveModuleStates` does).** A point on a rotating body moves with the body's translation plus ω × r. For module i at position (xᵢ, yᵢ) from the robot's center:

```latex
v_{ix} = v_x - \omega \, y_i \qquad v_{iy} = v_y + \omega \, x_i
```

```latex
\text{speed}_i = \sqrt{v_{ix}^2 + v_{iy}^2} \qquad \theta_i = \operatorname{atan2}(v_{iy},\, v_{ix})
```

Stacked for all four modules, this is one matrix multiply, which is how WPILib computes it:

```latex
\begin{bmatrix} v_{1x} \\ v_{1y} \\ \vdots \\ v_{4x} \\ v_{4y} \end{bmatrix}
=
\begin{bmatrix} 1 & 0 & -y_1 \\ 0 & 1 & x_1 \\ \vdots & \vdots & \vdots \\ 1 & 0 & -y_4 \\ 0 & 1 & x_4 \end{bmatrix}
\begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}
```

**Forward kinematics (`toChassisSpeeds`)** goes the other way: from four measured module states back to vx, vy, ω. There are 8 measurements and 3 unknowns, so WPILib solves it by least squares (the matrix pseudo-inverse). Odometry uses this every loop.

**Desaturation.** If any wheel would need to exceed the motor's top speed, `desaturateWheelSpeeds` scales *every* wheel down by the same factor, so the robot keeps the requested direction and just goes slower. Clipping one wheel alone would make the robot curve.

**Worked example** with our module positions (x = ±0.264 m, y = ±0.287 m), driving forward at 2 m/s while turning left at 1 rad/s:

| Module | x, y (m) | vₓ (m/s) | vᵧ (m/s) | Speed (m/s) | Angle |
| --- | --- | --- | --- | --- | --- |
| Front left | 0.264, 0.287 | 1.713 | 0.264 | 1.733 | 8.8° |
| Front right | 0.264, −0.287 | 2.287 | 0.264 | 2.302 | 6.6° |
| Back left | −0.264, 0.287 | 1.713 | −0.264 | 1.733 | −8.8° |
| Back right | −0.264, −0.287 | 2.287 | −0.264 | 2.302 | −6.6° |

The right side runs faster than the left (that's what turns the robot left), and both front wheels angle left while both back wheels angle right, which is what makes the robot pivot around its center.

## 4. Optimization, cosine scaling and discretization

Three small corrections sit between kinematics and the motors. Skipping any of them causes a visible problem.

**Optimization: never turn a wheel more than 90°.** Pointing a wheel at 180° and driving forward is the same as leaving it at 0° and driving backward. So if the target angle is more than 90° from the current angle, flip the target by 180° and negate the speed. `SwerveModuleState.optimize(currentAngle)` does this. Without it, wheels spin halfway around every time the driver reverses.

**Cosine scaling: don't push while the wheel is still turning.** If a wheel is 30° away from its target, only cos(30°) ≈ 0.87 of its push goes the right way; the rest shoves the robot sideways. Multiply the commanded speed by the cosine of the angle error:

```latex
v_{\text{cmd}} = v_{\text{target}} \cdot \cos(\theta_{\text{target}} - \theta_{\text{measured}})
```

`SwerveModuleState.cosineScale(currentAngle)` does this. Optimization guarantees the error is at most 90°, so the cosine is never negative.

**Discretization: fixing the arc.** Our code runs every 20 ms and holds each command constant for that whole period. When the robot drives and rotates at the same time, its heading changes during those 20 ms, so a command that was "straight forward" at the start of the period is "a bit sideways" by the end. The robot drifts in an arc. `ChassisSpeeds.discretize(speeds, dt)` computes the constant speeds that land exactly where a smooth twist would, using the exponential map from Lie group math. You don't need the derivation; you need to call it every loop, before kinematics.

## 5. Odometry and pose estimation

Odometry answers "where am I on the field?" by adding up small wheel movements, and vision corrects the drift that odometry always builds up.

**Odometry.** Every loop we read each module's *position* (total meters driven and current angle) and the gyro heading. WPILib finds how far each wheel moved since the last loop, runs forward kinematics to get a chassis twist (Δx, Δy, Δθ), and integrates it onto the previous pose. The gyro, not the wheels, supplies heading, because wheels slip when turning.

Odometry is only as good as its inputs:

- **Wheel radius** error scales every distance. A 2% error is 0.33 m over a 16.5 m field.
- **Gear ratio** error does the same, but bigger (L2 vs L3 is 10%).
- **Stale or mismatched samples** (a wheel position from 50 ms ago combined with a gyro reading from now) add error whenever the robot accelerates or turns.
- **Wheel slip** during hard acceleration or pushing fights is invisible to the encoders.

**Pose estimation.** `SwerveDrive4PoseEstimator` is a simplified Kalman filter. It trusts odometry over short times and blends in AprilTag poses from PhotonVision when they arrive. Each vision measurement carries *standard deviations*, which say how much to trust it: small numbers pull the pose hard toward the vision estimate, large numbers barely nudge it.

```latex
\hat{x}_{\text{new}} = \hat{x}_{\text{odom}} + K\,(x_{\text{vision}} - \hat{x}_{\text{odom}}), \qquad K = \frac{\sigma_{\text{odom}}^2}{\sigma_{\text{odom}}^2 + \sigma_{\text{vision}}^2}
```

That's the one-dimensional intuition: K is between 0 and 1, and it gets close to 1 when vision is much more certain than odometry. Vision measurements also carry a *timestamp*; the estimator rewinds to that moment, applies the correction, and replays odometry forward, which is why accurate timestamps matter.

## 6. Control: PID, feedforward and motion profiles

Each module runs two controllers: a **position** loop that points the wheel and a **velocity** loop that spins it. Good control is mostly feedforward (predicting the voltage we need) with PID cleaning up the small error that's left.

**PID feedback.** The error e is setpoint minus measurement. The output reacts to the error now (P), its build-up over time (I), and how fast it's changing (D):

```latex
u(t) = K_p\, e(t) + K_i \int_0^t e(\tau)\, d\tau + K_d\, \frac{de(t)}{dt}
```

For the steer loop we use mostly P with a little D, and I = 0. The error must be *wrapped*: going from 350° to 10° is a 20° move, not −340°. The SparkMax does this with position wrapping enabled; WPILib's `PIDController.enableContinuousInput(-pi, pi)` does it in code.

**Feedforward.** A DC motor's voltage splits into overcoming friction, making speed and making acceleration. WPILib's `SimpleMotorFeedforward` models that:

```latex
V = K_s \operatorname{sgn}(v) + K_v\, v + K_a\, a
```

| Gain | Units | Meaning | How to find it |
| --- | --- | --- | --- |
| Kₛ | V | Voltage just to start moving (static friction) | SysId, or raise voltage until the wheel creeps |
| Kᵥ | V per (m/s) | Voltage per unit of steady speed | SysId; roughly 12 V ÷ free speed as a first guess |
| Kₐ | V per (m/s²) | Voltage per unit of acceleration | SysId |

The drive command to the SparkMax is then "hold velocity v" with feedforward V passed as the arbitrary feedforward, and a small kP on the velocity error.

**Motion profiles.** Asking a mechanism to jump instantly to a new setpoint makes it slam and overshoot. A trapezoidal profile (`TrapezoidProfile`) limits velocity and acceleration so the setpoint moves smoothly: speed up, cruise, slow down. `ProfiledPIDController` combines a profile with PID, which is what we use to rotate the robot to a heading or drive to a pose.

**Following a path.** PathPlanner's holonomic controller sends the path's planned chassis speeds (feedforward) plus a PID correction on position and heading error. If the robot is on the path, PID adds almost nothing; if it's been bumped, PID pulls it back.

**Why the order of tuning matters.** The steer loop must be good first: a wheel pointing the wrong way makes every drive measurement wrong. Then drive feedforward, then drive P, then path PID.

## 7. Driver input math

Joysticks are noisy and drivers need fine control at low speed, so stick values are cleaned up before they become chassis speeds.

**Radial deadband.** Treat the left stick as a vector (x, y). Ignore it if its length is below the deadband d, and rescale the rest so output still starts at 0:

```latex
m = \sqrt{x^2 + y^2}, \qquad m' = \begin{cases} 0 & m < d \\ \dfrac{m - d}{1 - d} & m \ge d \end{cases}
```

Applying the deadband to x and y separately makes the robot "snap" to the axes near center; doing it on the length doesn't.

**Shaping.** Square (or cube) the magnitude, keep the direction: output = m'² · (x, y)/m. Half stick then gives a quarter of top speed, which gives drivers precision near center.

**Scale to real units.** Multiply by max speed (m/s) for translation and max angular speed (rad/s) for rotation. The theoretical max angular speed is v\_max ÷ r, where r is the distance from center to a module.

**Field-relative rotation.** Driver input is in the field frame; the modules need robot-frame speeds. Rotate the vector by minus the robot's heading θ (WPILib's `ChassisSpeeds.fromFieldRelativeSpeeds` does this):

```latex
\begin{bmatrix} v_x^{\text{robot}} \\ v_y^{\text{robot}} \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} v_x^{\text{field}} \\ v_y^{\text{field}} \end{bmatrix}
```

On red, add 180° to θ for the driver's perspective, once, in one place.

**Heading lock.** When the rotation stick is idle, a `ProfiledPIDController` on heading holds the last heading so the robot doesn't slowly twist. A button can snap the target heading to 0°, 90°, 180° or a scoring target.

## 8. WPILib and RobotPy classes you will use

In Python these live in `wpimath` (math), `wpilib` (hardware and dashboard), `commands2` (command framework), `rev` (SparkMax), `phoenix6` (CANcoder) and `pathplannerlib`.

| Class | Module | What it does |
| --- | --- | --- |
| `Translation2d`, `Rotation2d`, `Pose2d`, `Twist2d` | `wpimath.geometry` | Points, angles, robot poses, small motions |
| `ChassisSpeeds` | `wpimath.kinematics` | vx, vy, ω for the robot; `fromFieldRelativeSpeeds`, `discretize` |
| `SwerveModuleState` | `wpimath.kinematics` | Wheel speed + angle; `optimize`, `cosineScale` |
| `SwerveModulePosition` | `wpimath.kinematics` | Wheel distance + angle, for odometry |
| `SwerveDrive4Kinematics` | `wpimath.kinematics` | `toSwerveModuleStates`, `toChassisSpeeds`, `desaturateWheelSpeeds` |
| `SwerveDrive4Odometry` | `wpimath.kinematics` | Pose from wheels + gyro only |
| `SwerveDrive4PoseEstimator` | `wpimath.estimator` | Odometry fused with vision (`addVisionMeasurement`) |
| `PIDController`, `ProfiledPIDController` | `wpimath.controller` | Feedback loops in code |
| `SimpleMotorFeedforwardMeters` | `wpimath.controller` | kS/kV/kA model |
| `TrapezoidProfile`, `SlewRateLimiter` | `wpimath.trajectory`, `wpimath.filter` | Smooth setpoints, limit acceleration |
| `DCMotor`, `DCMotorSim` | `wpimath.system.plant`, `wpilib.simulation` | Motor models for simulation |
| `SysIdRoutine` | `commands2.sysid` | Runs characterization tests |
| `Field2d`, `Mechanism2d` | `wpilib` | Dashboard views of pose and modules |
| `SparkMax`, `SparkMaxConfig` | `rev` | Motor controller and its configuration |
| `CANcoder` | `phoenix6.hardware` | Absolute steering encoder |
| `AutoBuilder`, `PPHolonomicDriveController` | `pathplannerlib` | Autonomous path following |

The [WPILib kinematics and odometry docs](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/index.html) and the [RobotPy API reference](https://robotpy.readthedocs.io/projects/robotpy/en/stable/) show each class's methods; examples are mostly in Java, but names match in Python.

## 9. Check your understanding

Try each one before reading the answer.

1. The robot is commanded to strafe left at 1 m/s with no rotation. What angle and speed should every module have?
   - Answer: all four at +90°, 1 m/s (vₓ = 0, vᵧ = 1).
2. A wheel is at 10° and the target is 200° at 2 m/s. What does `optimize` command?
   - Answer: 20° at −2 m/s. The difference is 190°, over 90°, so flip the angle by 180° and negate speed.
3. Our L2 NEO module has a free speed of about 4.47 m/s. Kinematics asks one wheel for 6 m/s and another for 3 m/s. What does desaturation produce?
   - Answer: both scaled by 4.47 ÷ 6 ≈ 0.745, giving 4.47 and 2.24 m/s.
4. Why does the gyro supply heading for odometry instead of the wheels?
   - Answer: wheels slip, especially when turning, and the gyro measures rotation directly.
5. The robot drifts right whenever it drives forward and spins left at the same time. What is the most likely missing step?
   - Answer: `ChassisSpeeds.discretize` before kinematics.
6. Our module's kV is 2.4 V per m/s and kS is 0.15 V. Roughly what voltage holds 3 m/s?
   - Answer: 0.15 + 2.4 × 3 = 7.35 V.
7. A field-relative robot drives correctly on blue but goes toward the driver on red. What's wrong?
   - Answer: the red 180° offset is missing, or applied twice.
8. A wheel shows 4% less distance than a tape measure over 5 m. Which constant is off, and by how much?
   - Answer: wheel radius (or gear ratio) is off by about 4%; re-run the wheel radius characterization.

## Sources and further reading

- [WPILib: Kinematics and odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/index.html)
- [WPILib: Swerve drive kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)
- [WPILib: Coordinate system](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
- [WPILib: Introduction to PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
- [WPILib: Feedforward control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html)
- [WPILib: System identification (SysId)](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html)
- [WPILib: Pose estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [RobotPy API reference](https://robotpy.readthedocs.io/projects/robotpy/en/stable/)
- [SDS MK4 swerve module](https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module)
- [Controls Engineering in FRC (Tyler Veness)](https://file.tavsys.net/control/controls-engineering-in-frc.pdf), for students who want the full theory, including the discretization math
- [Chief Delphi: Snap to heading](https://www.chiefdelphi.com/t/snap-to-heading/454211)
