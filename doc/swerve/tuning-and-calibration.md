# Swerve Tuning and Calibration Guide

Oct 7, 2026 · @Tuffin

For our SDS MK4i L2 modules (L3 is a config option) with NEO drive and steer motors on SparkMax controllers and CTRE CANcoders. Calibrate the hardware first, then tune in a fixed order (steer, drive feedforward, drive P, wheel radius, limits, path PID), and keep every constant in git with a self-check at boot so drift is caught before a match.

## 1. Hardware checks before any tuning

No amount of tuning fixes a mechanical problem, and many "software" swerve bugs are loose parts. Do these with the robot on blocks.

- **Free spin:** each wheel spins freely by hand on both axes with no grinding or tight spots. A tight module needs more voltage than the others and will look like a tuning problem (our old front-right module is a candidate).
- **Backlash:** wiggle each wheel's steering by hand. More than a degree or two of slop means a loose gear, set screw or bearing.
- **CANcoder magnet:** it must be seated flush and not rotate in its pocket. Mark the magnet and housing with a paint pen line so movement is visible. Loose magnets are a known cause of offsets that "change by themselves."
- **CANcoder mount and cable:** screws tight, connector latched, cable strain-relieved so steering doesn't pull it.
- **NEO sensor cables:** the small 6-pin encoder cable on each NEO must be fully seated and not tensioned. A loose one causes status frame errors and jumpy readings.
- **Gear set matches the code:** verify the drive ratio. Mark the wheel, turn the drive motor exactly 6.75 turns (6.12 for L3) (read it in REV Hardware Client), and the wheel should make one turn. Do the same for steering: 21.43 (150/7) motor turns per full wheel rotation on the MK4i.
- **Wheels:** same tread type and similar wear on all four. Measure diameter at the center of the tread.
- **CAN bus:** in Phoenix Tuner X and REV Hardware Client, every device shows up with the expected ID, current firmware, and no faults. Bus utilization stays under about 70%.
- **Electrical:** battery at full charge (above 12.5 V resting) for any tuning run, so results are repeatable.

## 2. SparkMax + NEO configuration baseline

Configure every SparkMax from code on every boot, starting from defaults, so a swapped controller behaves exactly like the old one. These are starting values for MK4i L2; tune from here.

| Setting | Drive SparkMax | Steer SparkMax | Why |
| --- | --- | --- | --- |
| `configure()` reset mode | `kResetSafeParameters` | `kResetSafeParameters` | Clears anything left over from earlier code or the Hardware Client |
| Persist mode | `kPersistParameters` only when the config changed; otherwise `kNoPersistParameters` | same | Avoids writing flash every boot and slowing startup |
| Idle mode | Brake | Brake | Steer in coast lets wheels get knocked off angle while disabled |
| Smart current limit | 40 to 50 A | 20 A | Limits wheel slip and brownouts; raise drive only if acceleration is too low |
| Voltage compensation | 12 V | 12 V | Same output on a fresh or tired battery |
| Ramp rates | none | none | Ramps delay the closed loop; limit acceleration at the chassis instead |
| Position conversion | 2πr ÷ 6.75 (m per motor rotation; 6.12 for L3) | 2π ÷ (150/7) (rad per motor rotation) | Everything downstream in meters and radians |
| Velocity conversion | position factor ÷ 60 (m/s per RPM) | position factor ÷ 60 (rad/s per RPM) | NEO velocity is natively RPM |
| NEO velocity filter | `uvwMeasurementPeriod` 8 to 16 ms, `uvwAverageDepth` 2 to 4 | default is fine | Default 32 ms × 8 samples lags about 100 ms |
| Closed loop | velocity, kP small, feedforward from wpimath as `arbFeedforward` in volts | position, `positionWrappingEnabled`, input range −π to π |  |
| Status frame: primary encoder position | 10 to 20 ms | 10 to 20 ms | Odometry needs fresh wheel position every loop |
| Status frame: primary encoder velocity | 20 ms | 20 ms or slower |  |
| Unused frames (analog, alt encoder, absolute encoder) | 500 ms or off | 500 ms or off | Frees CAN bandwidth |

CANcoder: absolute sensor range −0.5 to 0.5 rotations (signed), counterclockwise positive viewed from above, `absolute_position` update frequency 50 to 100 Hz, and apply the config with a timeout and check the returned status.

## 3. Calibrating the CANcoder offsets

The offset tells the code what CANcoder reading means "wheel pointing straight forward." Get it to within about 1°; a 3° error on one module makes the robot drift noticeably over a field length.

1. Put the robot on blocks, disabled, steer motors in coast (or unpowered) so the wheels turn by hand.
2. Point every wheel straight forward with **all bevel gears facing the same side** (pick left, write it down). A wheel pointing forward with its bevel on the wrong side is 180° off, and the robot will drive that module backward.
3. Square them with a straightedge: clamp a long aluminum bar or a 1×1 tube against both wheels on each side. A printed or 3D-printed alignment jig per module is even better.
4. Read each CANcoder's raw absolute position with offset set to 0 (Phoenix Tuner X self-test, or a dashboard value in test mode).
5. The offset is the negative of that reading. Record all four, in rotations, with the date and who measured them.
6. Put them in the robot's config file in git (see section 5), deploy, and reboot.
7. Verify: on blocks, enabled, command 0° and check all four wheels are straight with the bar. Then command 90° and 180°. Then drive forward on carpet a few meters; the robot should go straight with no rotation.

A good habit is a **test-mode calibration command**: it reads the four raw values and prints them in Python syntax, ready to paste into the config. It removes copy errors and makes recalibration a two-minute job.

## 4. Tuning order, step by step

Tune in this order, because each step depends on the one before. Log setpoint and measured value for every module to wpilog and plot them in AdvantageScope; tuning by eye from the dashboard misses most problems.

**Step 1: steer position loop (on blocks).**

1. Set kI = 0, kD = 0, and kP to a low start (about 0.3 duty per radian, which is our old 0.011 per degree halved).
2. Command steps between 0° and 90°, then 0° and 179°, and plot angle vs setpoint.
3. Raise kP until the wheel overshoots or buzzes, then back off about 30%.
4. Add a little kD only if it still overshoots.
5. Target: a 90° step settles in under about 0.15 s with no visible overshoot, and all four modules look the same. One module that needs very different gains has a mechanical problem.

**Step 2: drive feedforward (on carpet, open space).** Use WPILib SysId. In Python, `commands2.sysid.SysIdRoutine` runs the four tests (quasistatic and dynamic, forward and backward) with all wheels locked at 0°; log with `wpilib.DataLogManager`, and our repo already has `robotpy-urcl` for SparkMax data. Load the log in the SysId tool and read kS, kV, kA. Sanity check: kV should be near 12 V ÷ 4.47 m/s ≈ 2.7 V per m/s for L2 NEO (about 2.4 for L3), and kS usually 0.1 to 0.3 V. All four modules share one set of gains.

**Step 3: drive velocity P.** With feedforward in, add kP until measured speed tracks the setpoint during acceleration without chatter. Start around 0.1 V per m/s of error if using wpimath on the roboRIO, or the equivalent duty-cycle value on the SparkMax. Keep kI = 0.

**Step 4: wheel radius (on carpet).** Rotate the robot slowly in place for several full turns. Every wheel travels on a circle of radius R around the center (0.390 m for our module layout). Then:

```latex
r_{\text{wheel}} = \frac{\Delta\theta_{\text{gyro}} \cdot R}{\overline{\Delta\varphi}_{\text{wheel}}}
```

where Δθ is the gyro's total rotation in radians and Δφ is the average wheel rotation in radians (motor rotations ÷ drive ratio × 2π). Expect slightly under 0.0508 m on worn tread. Repeat three times and average. Then check with a tape measure over 5 m: odometry should agree within 1 to 2 cm.

**Step 5: real limits.** Drive full speed in a straight line and log speed; set max translation speed to about 95% of what you measured. Find the acceleration where wheels start to slip (odometry and vision disagree after hard launches) and set the chassis slew limit and PathPlanner max acceleration below it.

**Step 6: PathPlanner.** Enter measured mass, moment of inertia, wheel radius, max speed, drive current limit and wheel COF in the PathPlanner GUI robot config. Tune translation kP on a straight 3 m path and rotation kP on a path that turns 180°; start both near 5 and adjust until end-of-path error is under about 3 cm and 2°.

**Step 7: heading controller and vision.** Tune the snap/heading ProfiledPIDController, then vision standard deviations, last. Vision tuning on top of bad odometry hides the odometry problem.

## 5. Keeping calibration good

Store every calibration value in git, have the robot check itself at boot, and re-verify the hardware on a schedule. Most "it worked yesterday" swerve failures are a magnet that moved, a swapped module, or a value someone changed on a laptop and never committed.

**Where to store offsets and gains**

| Option | Good | Bad | Verdict |
| --- | --- | --- | --- |
| Constants in a per-robot config file in git | Reviewed, versioned, survives any device swap, one place to look | Needs a deploy to change | **Use this as the source of truth** |
| CANcoder magnet offset in device flash (set by Tuner X) | Works even with no code | Invisible in git; lost or wrong when a CANcoder is swapped | Avoid as the only copy. If used, have code write it from the config every boot |
| NetworkTables persistent values on the roboRIO | Change in the pits without a deploy | Not in git, differs between roboRIOs, easy to forget | Only for a temporary field fix, then copy into git |
| SparkMax flash (gains, conversions) | Survives power-off | Silently differs between controllers | Never rely on it; reset and configure from code |

Keep a short calibration log next to the config (date, module, old and new offset, who, why). If one module's offset keeps changing, it's a hardware problem, not a calibration one.

**Hardware upkeep**

- Check magnet marks and module screws at every event and after any hard hit.
- Re-measure wheel radius when tread is replaced and about every two events; tread wear changes it.
- Swapping a module: give each physical module a label and keep its offset with the module label, not the corner. The offset only carries over if the module is mounted in the same orientation at its new corner; otherwise recalibrate. Either way, re-verify with the straightedge.
- Re-check backlash and free spin whenever a module is opened.

**Software self-checks**

- At boot, wait for a valid CANcoder reading (retry with a timeout) before seeding the steer encoder, and show a red dashboard alert if a module fails.
- While disabled, compare each module's steer encoder with its CANcoder every second; re-seed if they differ by more than about 2° and log that it happened.
- Log, per module, setpoint vs measured angle and speed. A module whose error is consistently larger than the others is the first thing to inspect.
- Put a "module health" panel on the driver dashboard: CANcoder connected, seed OK, angle error, motor temperature, CAN faults.
- Add a unit test that loads the config and checks every offset is between −0.5 and 0.5 rotations and every gear ratio matches a known MK4 preset.

## 6. Pit checklist

Copy this list for each event day.

**Every match**

- [ ] Battery above 12.5 V resting
- [ ] Robot on blocks, enable briefly: all four wheels snap straight at 0°
- [ ] Dashboard module health panel all green

**Start of each event day**

- [ ] Magnet marks lined up on all four CANcoders
- [ ] Free spin and backlash check on each module
- [ ] Straightedge check of wheel alignment at 0°
- [ ] Drive 5 m straight on carpet: no drift, odometry within 2 cm of tape
- [ ] Config in the deployed code matches the latest commit (git hash on the dashboard)

**After any module repair, swap or hard hit**

- [ ] Re-run offset calibration (section 3) and commit the new values
- [ ] Re-run the straight-line drive check
- [ ] Note it in the calibration log

## Sources

- [REV: Closed loop control getting started](https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started)
- [REV: EncoderConfig API](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/encoderconfig); uvw filter defaults checked in the robotpy-rev 2026.0.4 stubs
- [YAGSL: Fix common SparkMAX/SparkFlex problems](https://yagsl.yassrobotics.com/how-to-guides/fix-sparkmax-common-problems.md)
- [AdvantageKit Spark swerve template](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template) (feedforward and wheel radius characterization routines)
- [URCL (REV CAN logger)](https://github.com/Mechanical-Advantage/URCL) and [AdvantageKit SysId compatibility](https://docs.advantagekit.org/data-flow/sysid-compatibility)
- [WPILib: System identification (SysId)](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html)
- [Chief Delphi: Angle offset on swerve modules changing (loose magnets)](https://www.chiefdelphi.com/t/angle-offset-on-swerve-modules-changing-solved/495495)
- [Chief Delphi: Zeroing CANcoders](https://www.chiefdelphi.com/t/zeroing-cancoders/390744)
- [Chief Delphi: Spark Max speed feedback/control](https://www.chiefdelphi.com/t/split-thread-spark-max-speed-feedback-control/418107?page=2)
- [SDS MK4 swerve module](https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module)
