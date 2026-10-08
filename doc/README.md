# Team docs

Hand-written guides live here. `docs/` (with an s) is the generated pdoc API
site that CI builds from `doc/build_docs.py`; don't edit it by hand.

## Swerve

| Doc | What it's for |
| --- | --- |
| [Theory manual](swerve/theory-manual.md) | Student guide: kinematics, optimize and cosine scaling, discretize, odometry, PID and feedforward, driver input math, the WPILib classes |
| [Tuning and calibration](swerve/tuning-and-calibration.md) | SparkMax + NEO + CANcoder baseline, offset calibration, tuning order, pit checklist |
| [Calibration log](swerve/calibration-log.md) | Dated record of every offset and wheel radius change |
| [Research and robot-2026 review](swerve/research-and-review.md) | How strong teams structure swerve, what went wrong in robot-2026, the sim plan |

## Plans

| Doc | What it's for |
| --- | --- |
| [robot-2027 code plan](plans/robot-2027-code-plan.md) | What carried over from robot-2026, CI, branches, layout, milestones M0 to M8, logging, SysId tool, phase 2 |

These started as shared docs during the 2026 off-season. The copies here are
now the version of record: change them through PRs so they're reviewed with
the code they describe.
