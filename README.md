# 5472 Robot Vision Template

A complete FTC localization, path-following and shooting stack: a **goBILDA
Pinpoint** odometry computer and a **Limelight 3A** AprilTag camera fused into
one field pose by a Kalman estimator, driving a Bézier path follower, a
**shoot-on-the-move** aiming solution for a turretless robot, and an
**FRC-style command framework** to tie it together — plus a visual path planner
you can open in a browser.

This README explains **how it works**. Two companion documents cover the rest:

| Document | What it is for |
|----------|----------------|
| **[MANUAL.md](MANUAL.md)** | Step-by-step setup, tuning and troubleshooting. Start here if you want to *run* this. |
| **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** | One-page cheat sheet: API calls, constants, telemetry, controls. |
| [docs/FTC_SDK_README.md](docs/FTC_SDK_README.md) | The upstream FTC SDK readme and release notes. |

Deeper dives live next to the code they describe:

- [`LOCALIZATION.md`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/LOCALIZATION.md) — pose fusion
- [`PATHING.md`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pathing/PATHING.md) — path following
- [`SHOOTING.md`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooting/SHOOTING.md) — shoot on the move
- [`COMMANDS.md`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/command/COMMANDS.md) — command framework

---

## Contents

- [The idea in one page](#the-idea-in-one-page)
- [The coordinate frame](#the-coordinate-frame)
- [Part 1 — Localization](#part-1--localization)
- [Part 2 — Path following](#part-2--path-following)
- [Part 3 — Shooting on the move](#part-3--shooting-on-the-move)
- [Part 4 — The command framework](#part-4--the-command-framework)
- [Part 5 — Alliances](#part-5--alliances)
- [Part 6 — The Path Planner](#part-6--the-path-planner)
- [What runs each loop](#what-runs-each-loop)
- [File map](#file-map)
- [Verifying the math](#verifying-the-math)
- [Hardware assumed](#hardware-assumed)
- [Credits and licensing](#credits-and-licensing)

---

## The idea in one page

Two sensors know where the robot is, and neither is good enough alone.

**Odometry** (the Pinpoint) is smooth, fast and never confused, but it *drifts*.
Every inch of wheel slip is an inch of error that never comes back. After thirty
seconds of a match it can be several inches off, and it has no way to know.

**Vision** (the Limelight reading AprilTags) is *absolute* — it tells you where
you are on the field with no memory of how you got there — but it is noisy,
arrives late, and is unavailable whenever no tag is in frame.

The fix is the standard one from FRC: run odometry continuously as the backbone,
and treat each camera frame as a *correction* whose strength depends on how much
you trust it. A tag two feet away seen by three tags moves the estimate a lot. A
single tag across the field barely nudges it. Nothing snaps; the pose stays
smooth enough to drive on.

```
  Pinpoint ──── absolute pose, every loop ────┐
  (2 dead wheels + IMU)                       │
                                              ▼
                                      ┌───────────────┐
                                      │ PoseEstimator │──► fused field pose
                                      │  (Kalman)     │
                                      └───────────────┘
                                              ▲
  Limelight ─── AprilTag pose, when visible ──┘
  (MegaTag1 / MegaTag2)   weighted by distance² / tagCount
```

That fused pose is what everything else consumes: the path follower steers by
it, field-centric TeleOp rotates the driver's stick by it, and telemetry
displays it.

---

## The coordinate frame

**There is one field frame and it never changes.**

| | |
|---|---|
| Origin | centre of the field |
| +X | to the right (as seen from the audience) |
| +Y | away from the audience |
| Heading | counter-clockwise positive, `0` = facing +X |
| Units | inches, radians internally (degrees at the edges) |
| Extent | 144″ × 144″, so coordinates run about −72 … +72 |

This is the same frame the AprilTag field map uses, which is what makes vision
and odometry directly comparable. It is also **alliance-independent**: a robot
parked on a given tile reports the same pose whether it is red or blue. That
property is deliberate and is what makes the alliance handling in
[Part 5](#part-5--alliances) simple.

The robot's own frame, used for motor mixing, is **+X forward, +Y left, turn
CCW-positive**.

---

## Part 1 — Localization

### The estimator

`lib/estimator/PoseEstimator` is a port of WPILib's `PoseEstimator` — the same
class FRC teams use and the one the AdvantageKit vision template feeds. It keeps:

- a **rolling history** of odometry poses (1.5 s, in `TimeInterpolatableBuffer`),
- the most recent **vision correction**, stored as an offset anchored to the
  odometry pose at the moment the frame was captured.

Every loop the current odometry pose comes in and the stored correction is
re-applied on top of it. That is the whole trick: the correction is a *rigid
offset*, so odometry keeps providing smooth motion while vision provides the
absolute anchor.

### How much a frame moves the pose

Per axis, the Kalman gain is

```
    gain = q / (q + √(q · r))
```

where `q` is the odometry variance and `r` the vision variance for that frame.
Vision std devs are computed **per frame**:

```
    xyStdDev = VISION_XY_STD_DEV_COEFFICIENT · (avgTagDistance² / tagCount)
```

Distance hurts quadratically, tag count helps linearly. This is the AdvantageKit
scaling, and it is why a far-off single-tag glimpse cannot yank the robot
sideways mid-path. With the shipped coefficient that works out to:

| tag distance | tags | std dev | gain |
|---|---|---|---|
| 18″ | 2 | 0.08″ | **0.86** — close multi-tag snaps |
| 40″ | 1 | 0.80″ | 0.38 |
| 60″ | 2 | 0.90″ | 0.36 |
| 100″ | 1 | 5.00″ | **0.09** — far single-tag barely nudges |

These are measured, not estimated; the suite prints this table (see
[Verifying the math](#verifying-the-math)).

> **If you retune that coefficient, mind the units.** AdvantageKit publishes
> `0.02`, but its distances are in **metres**; this template works in inches and
> the distance term is *squared*, so the value converts to about `0.0005` rather
> than carrying across unchanged. Get this wrong and the std devs come out in
> the hundreds or thousands of inches, the gain collapses to roughly zero, and
> vision fusion silently stops doing anything — with no outward sign, because
> odometry keeps the pose looking perfectly plausible.

### Heading is deliberately gyro-dominated

Heading gets its own, much larger std dev (`VISION_HEADING_STD_DEV`, 45°),
giving a gain of about **0.04**. The Pinpoint's IMU is excellent over a match and
a single-tag heading is not, so the gyro owns heading short-term and vision only
slowly bleeds off drift. Raise this constant further if you would rather vision
never touched heading at all.

### MegaTag1 and MegaTag2: which solver, and when

The Limelight offers two AprilTag solvers, and they fail in opposite ways.

**MegaTag1** (`getBotpose`) solves the pose from tag geometry alone. Because it
never looks at the gyro, it is the only thing in the system that can *check* the
gyro. Its weakness is **pose ambiguity**: a single tag viewed near head-on has
two mathematically valid solutions that are mirror images, and the solver can
pick the wrong one.

**MegaTag2** (`getBotpose_MT2`) takes the yaw you push down via
`updateRobotOrientation()` and uses it to constrain the solve, which removes the
ambiguous solution entirely. It is far steadier, especially on one tag and at
distance. Its weakness is the mirror image of MegaTag1's: the heading it reports
is just your own yaw handed back, so it can never correct heading drift — and if
your heading is wrong, it returns a *confidently* wrong position.

So the template uses each for what it is good at:

| | source | why |
|---|---|---|
| **Heading** | always MegaTag1 | gyro-independent, so it can audit the gyro |
| **Position** | MegaTag2, once heading is trusted | immune to ambiguity, given a good heading |

**The heading-trust gate** connects them. MegaTag1's independent heading is
compared against the current estimate every frame. Agreement for
`HEADING_TRUST_FRAMES` consecutive frames earns trust and hands position over to
MegaTag2; a gross disagreement revokes it and drops straight back to MegaTag1,
which a bad heading cannot poison. The current state shows on telemetry as
`heading TRUSTED` or `unverified`.

### The 180° problem, and where it actually comes from

The most damaging localization failure in FTC is not drift — it is being
**seeded backwards**. The causes are mundane: the wrong alliance button at init,
or the robot physically placed facing the other way.

Note what it is *not*: alliance flipping. The field frame is absolute and
AprilTags are surveyed into it, so `AllianceFlip` transforms **plans** (paths,
start poses) and never **measurements**. Vision reads identically whichever
alliance you are playing.

A backwards seed is nasty because it is quiet. Odometry is self-consistent, the
pose looks reasonable, and the robot simply drives the wrong way. Heading fusion
alone will not save you in time: at a 0.04 gain MegaTag1 needs seconds to drag
180° back, and auto has already run by then.

The template attacks it where it is cheap — **before the match**:

- Alliance-selecting OpModes run the estimator during **init**, while the robot
  sits still with a clear view of a tag (the best look MegaTag1 will ever get).
- `getStartPoseCheck()` compares MegaTag1's heading against the seeded one and
  reports a verdict; `isStartPoseSuspect()` drives a loud telemetry warning.
- In `FieldCentricDrive`, **Y** calls `seedFromVision()`, snapping the whole pose
  to what the camera sees. It uses MegaTag1 deliberately, so it can recover a
  heading the gyro has wrong, and it ignores fixes older than half a second so a
  stray press cannot corrupt a good pose.

That is deliberately better than a driver-operated "am I facing toward you?"
button: it needs no judgement under pressure, and it fires before auto rather
than after it.

### Outlier rejection

A frame landing more than `MAX_POSE_JUMP_IN` from a *settled* estimate is
dropped — the last defence against an ambiguous solve teleporting the robot
across the field. Two details keep this from backfiring:

- It only applies once the estimate has settled. At startup a badly seeded
  estimate is the wrong one and vision is right.
- If the camera insists on the same disagreement for `JUMP_REJECT_LIMIT` frames,
  the *estimate* is what is wrong, so rejection latches off until a frame agrees
  again. Otherwise a wrong pose could reject every correction forever.

### Latency compensation

A camera frame describes the past. By the time the pose reaches your code it is
old by:

```
    captureLatency + targetingLatency   (the Limelight's own pipeline)
  + staleness                            (how long the finished result sat on
                                          the Robot Controller before we read it)
```

The frame is timestamped `now − all of that`, the estimator looks up where
odometry thought the robot was *at that instant*, applies the correction there,
and replays the newer odometry on top. Skip this and every correction drags the
robot backwards along its own path.

### Rejection filters

A frame is discarded if it is invalid, has too few tags, is staler than
`MAX_STALENESS_MS`, has a null botpose, reports exactly the field origin (the
Limelight's "no fix" value), or lands off the field plus a margin. Whatever
happens is reported on telemetry as a reason string, so a robot that is not
being corrected tells you *why*.

### The camera is not at the robot's centre

An off-centre camera sees the field from a shifted position, and when the robot
rotates that offset sweeps an arc. There are two ways to handle it and you must
pick exactly one, or the offset is applied twice:

- **(A), recommended** — enter the mount in the Limelight web UI. The botpose
  you read is then already robot-centre. Keep `APPLY_CAMERA_OFFSET_IN_CODE = false`.
- **(B)** — zero the UI offset and describe the mount in `VisionConstants`
  instead. The code then treats botpose as the *camera's* field pose and
  recovers the robot pose with a full 3D transform
  (`cameraPose.transformBy(ROBOT_TO_CAMERA.inverse())`), handling a raised,
  pitched or rolled camera, then projects to 2D.

> **Mind the pitch sign.** Pitch rotates about +Y (left), so by the right-hand
> rule a **positive pitch tilts the camera down**. A camera angled 15° *upward*
> to see tags is `-15.0`. Getting this backwards doubles the mount error instead
> of removing it.

The template ships with the offsets zeroed and option (A) selected, because a
wrong mount model is worse than no mount model.

### 3D diagnostics

The fused estimate is 2D — the robot drives on the floor and the Pinpoint only
measures x/y/heading, so there is nothing to fuse z/pitch/roll *against*. The
full 3D vision pose is still exposed (`getVisionPose3d()`, `getVisionPitch()`,
…) for detecting tipping or sanity-checking the mount: a constant non-zero
pitch on a flat field almost always means the camera offset is misconfigured.

### Running without a Limelight

Set `VISION_ENABLED = false`, or construct `new Localization(hardwareMap, false)`.
The camera is then never initialised, so an unplugged Limelight will not crash
the OpMode, and the fused pose is simply the Pinpoint pose. Everything
downstream — including path following — works unchanged.

---

## Part 2 — Path following

### Paths

A `Path` is a Bézier curve plus a rule for where to point:

| Heading mode | Behaviour |
|---|---|
| `setTangentHeading()` | face along the path |
| `setReverseTangentHeading()` | face backwards along it (drive in reverse) |
| `setLinearHeading(a, b)` | sweep from `a` to `b` by `t`, shortest way round |
| `setConstantHeading(h)` | hold one heading |

A `PathChain` is several of them run back to back.

### The follower

Each cycle, `Follower` projects the robot onto the current path — searching
*forward only*, so the projection cannot slip backwards if the robot overshoots
— and sums three field-frame vectors:

- **Translational** — a PIDF pull toward the closest point on the path. This is
  what fixes cross-track error.
- **Centripetal** — `CENTRIPETAL_SCALE · speed² · curvature`, pointed to the
  inside of the curve. Without it the robot drifts wide on every arc, because
  the translational term only reacts to error that has *already happened*.
- **Drive** — a push along the tangent, sized by remaining path length.

The corrective vector (translational + centripetal) gets **priority** in the
unit power budget; the drive vector only gets the headroom that is left. Staying
*on* the path beats making progress along it.

Heading runs as a completely independent PIDF, so the robot can translate and
rotate on separate schedules.

### Stopping cleanly

Rather than relying on an aggressive `kD`, the drive term includes a
physics-shaped feedforward:

```
    targetSpeed = √(2 · ZERO_POWER_DECEL_RATE · remaining)
    driveMag    = max(targetSpeed / MAX_ROBOT_SPEED, drivePIDOutput)
```

That is just the constant-deceleration equation solved for speed, so the robot
arrives at the endpoint with near-zero velocity. Measure `ZERO_POWER_DECEL_RATE`
once: drive at full speed, cut power, and compute `v² / (2·d)` from the entry
speed and stopping distance.

### The convention that must hold

The follower assumes:

| command | robot must |
|---|---|
| forward power | drive forward |
| strafe power | slide **left** |
| positive turn power | rotate **counter-clockwise** |

If any of these is inverted the follower does not merely track badly — it steers
itself *away* from the target and the error grows. Run the **Drivetrain
Direction Check** OpMode before trusting an auto; see the manual's
[first-run section](MANUAL.md#4-check-the-drivetrain-before-anything-else).

---

## Part 3 — Shooting on the move

A game piece leaving a moving robot **keeps the robot's velocity**, so aiming
straight at the goal while driving misses by roughly `speed × flight time` — about
**48 inches** at the velocities the test suite sweeps. The fix is to aim at a
*virtual goal*, offset from the real one by exactly what the robot's motion will
add:

```
    virtualGoal = goal − shooterVelocity × timeOfFlight
```

That is circular (flight time comes from distance, distance from the virtual
goal), so it is iterated to a fixed point.

This was ported from team 5472's FRC turret code and adapted for a robot with **no
turret**, which changes three things:

- The output is a **field-relative robot heading**, not a turret angle clamped to
  a mechanical sweep. The robot is the turret.
- The driver keeps **translation** while the aiming solution owns **heading**, so
  the robot never has to stop.
- The solution includes an **angular feedforward**. While translating past the
  goal the aim direction keeps sweeping, and a position-only heading controller
  permanently trails it. A turret is quick enough to mostly get away without
  this; a whole robot is not.

The **shot table** (`ShooterMap`) maps distance → flywheel rpm, hood angle and
flight time, interpolating between rows you measured and clamping outside them.
Flight time is the column teams skip and the one this depends on — leave it at
zero and the correction silently does nothing.

Firing is gated on range, aim, flywheel speed **and pose trust**. That last one
reuses the heading-trust check from Part 1: a pose seeded 180° out looks perfectly
healthy from the inside, and without the gate a mis-seeded robot confidently
shoots at empty field.

Full detail, setup order and the tuning workflow:
[`SHOOTING.md`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooting/SHOOTING.md).

---

## Part 4 — The command framework

`lib/command` is a WPILib-shaped command system: **subsystems** that only one
command may use at a time, **commands** with a lifecycle that compose into
sequences and parallels, and a **scheduler** that runs it all. If you have written
FRC code it reads exactly as you expect.

```java
whileHeld(() -> gamepad1.a, Commands.sequence(
        Commands.runOnce(() -> shooter.setFlywheelRpm(2500), shooter),
        Commands.waitUntil(shooter::atSpeed),
        Commands.run(shooter::runFeeder, shooter).withTimeout(1.5)));
```

Nothing external is needed — no FTCLib, no SolversLib, no extra Gradle
dependency — which also means it is covered by the offline suite.

**One deliberate difference from WPILib:** the scheduler is an ordinary object
owned by the OpMode, not a static singleton. An FTC app runs many OpModes in one
process, and a static scheduler carries commands, bindings and stale hardware
handles between runs — the failure where an OpMode works the first time after a
Robot Controller restart and misbehaves every time after.

Full detail:
[`COMMANDS.md`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/command/COMMANDS.md).

---

## Part 5 — Alliances

The field frame is absolute, so the alliance changes exactly two things and
neither of them touches the pose estimate.

**1. The driver's point of view.** The two drive teams stand at opposite ends,
so "push the stick away from me" is +X for one and −X for the other.
`FieldCentricDrive` rotates the stick vector by `Alliance.driverForward()`
before handing a field-frame command to the drivetrain:

```
    driver frame ──(driverForward)──► field frame ──(−heading)──► robot frame
```

**2. Which side an auto runs on.** Author the plan once, flip it at run time:

```java
Pose2d    start = AllianceFlip.forAlliance(START,       AUTHORED_FOR, alliance);
PathChain toRun = AllianceFlip.forAlliance(buildPlan(), AUTHORED_FOR, alliance);
```

`forAlliance` returns the original object untouched when the alliances already
match, so there is no cost on your home side.

The flip is a **180° rotation about the field centre**, not a mirror, because
the BIOBUZZ field's alliance halves are rotations of each other. This matters
more than it sounds: on a square field a wrong symmetry looks almost right and
puts the robot in the wrong place. `PATHING.md` shows the field-coordinate check
that settles it. A mirror also reverses handedness, turning a left-curving path
into a right-curving one.

---

## Part 6 — The Path Planner

`TeamCode/src/main/assets/pathplanner.html` is a self-contained visual editor —
no build step, no server, no dependencies. Drag control points over a picture of
the field, pick a heading mode per segment, and it generates Java you paste
straight into an OpMode:

```java
localization.setStartingPose(new Pose2d(-58.2, -35.7, Rotation2d.fromDegrees(90.0)));

Path path0 = new Path(new BezierCurve(
        new Translation2d(-58.2, -35.7),
        new Translation2d(-20.5, -10.1),
        new Translation2d(12.3, 24.8)))
        .setTangentHeading();

PathChain chain = new PathChain(path0);
follower.followPath(chain);
```

Those variable names match the ones `PathAuto` already declares, so the block
pastes in verbatim between its two marker comments.

Open it two ways: directly as a file in any browser, or — because
`PathPlannerServer` registers a route on the Robot Controller's embedded web
server — from the robot itself at `http://192.168.43.1:8080/pathplanner`.

It ships with the 2026-2027 BIOBUZZ field as the backdrop, uses the same
coordinate convention as the code, and has a **⟳ Swap Alliance** button that
applies the same 180° rotation `AllianceFlip` does at run time.

---

## What runs each loop

```
 ┌── Localization.update() ──────────────────────────────────┐
 │  pinpoint.update()            read encoders + IMU         │
 │  poseEstimator.updateWithTime(now, odometryPose)          │
 │  limelight.updateRobotOrientation(heading)   for MegaTag2 │
 │  ── if a frame is available and passes the filters ──     │
 │      heading from MegaTag1; position from MegaTag2 once   │
 │        MegaTag1 has vouched for the heading               │
 │      compute per-frame std devs from distance & tag count │
 │      reject an implausible jump from a settled estimate   │
 │      timestamp it at capture time                         │
 │      poseEstimator.addVisionMeasurement(...)              │
 └───────────────────────────────────────────────────────────┘
                               │  fused pose
                               ▼
 ┌── Follower.update() ──────────────────────────────────────┐
 │  project robot onto path (forward-only)                   │
 │  translational + centripetal + drive  ──► field vector    │
 │  heading PIDF                         ──► turn power      │
 └───────────────────────────────────────────────────────────┘
                               │
                               ▼
 ┌── MecanumDrivetrain.driveFieldCentric() ──────────────────┐
 │  rotate field vector by −heading  ──► robot frame         │
 │  mecanum mixing, normalise, set four motor powers         │
 └───────────────────────────────────────────────────────────┘
```

While aiming, the middle stage is replaced by the shooting path:

```
 ┌── AimLogic.calculate() ───────────────────────────────────┐
 │  look ahead by the phase delay                            │
 │  find the shooter's own position and velocity (+ omega x r)│
 │  iterate: virtualGoal = goal − shooterVel × timeOfFlight   │
 │  ──► heading target, angular feedforward, distance         │
 └───────────────────────────────────────────────────────────┘
                               │
                    ┌──────────┴──────────┐
                    ▼                     ▼
   driveWithHeadingLock(          setShotForDistance(
       driver translation,            effective distance)
       heading target,            └─► flywheel rpm + hood angle
       feedforward)
                    │
                    ▼
   feed only if: in range, aimed, at speed, pose trusted
```

---

## File map

All paths relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

### Localization

| File | Role |
|---|---|
| `subsystems/VisionConstants.java` | **Every localization tuning knob.** Device names, Pinpoint geometry, camera mount, std devs, filters. |
| `subsystems/Localization.java` | The subsystem. `update()` each loop, `getPose()` to read. Implements `Localizer`. |
| `subsystems/PinpointOdometry.java` | Configures and reads the Pinpoint. |
| `subsystems/LimelightVision.java` | Configures and reads the Limelight. |
| `lib/estimator/PoseEstimator.java` | The Kalman fusion (WPILib port). |
| `lib/util/TimeInterpolatableBuffer.java` | Odometry history for latency compensation. |
| `lib/geometry/*.java` | `Pose2d`/`Rotation2d`/`Twist2d` (WPILib port) and a small SE(3) layer for the camera mount. |

### Pathing

| File | Role |
|---|---|
| `pathing/PathConstants.java` | **Every follower tuning knob.** Motor names and directions, PIDF gains, tolerances. |
| `pathing/Follower.java` | The control loop. |
| `pathing/Path.java`, `BezierCurve.java`, `PathChain.java` | Path geometry and heading rules. |
| `pathing/MecanumDrivetrain.java` | Field-centric command → four motor powers. |
| `pathing/PIDFController.java` | PIDF with explicit timestep. |
| `pathing/Alliance.java`, `AllianceFlip.java`, `FieldSymmetry.java` | Alliance handling. |
| `pathing/Drivetrain.java`, `Localizer.java` | The two interfaces the follower depends on. |
| `pathing/PathPlannerServer.java` | Serves the planner from the robot. |

### Shooting

| File | Role |
|---|---|
| `shooting/ShootingConstants.java` | **Every shooting knob.** Goal position, shooter geometry, the shot table, hardware. Fill it in top to bottom. |
| `shooting/AimLogic.java` | The shoot-on-the-move maths. Pure static, no hardware. |
| `shooting/AimSolution.java` | Its output: heading, feedforward, distance, readiness. |
| `shooting/ShooterMap.java`, `ShooterSetpoint.java` | The shot table and one row of it. |
| `subsystems/ShooterSubsystem.java` | Flywheel, hood, feeder, driven from the table. |
| `subsystems/DriveSubsystem.java` | The drivetrain as a subsystem, with heading-lock drive. |
| `commands/AimAndShootCommand.java` | Ties it together and gates the feeder. |

### Command framework

| File | Role |
|---|---|
| `lib/command/CommandOpMode.java` | Base OpMode. Subclass it and override `configure()`. |
| `lib/command/CommandScheduler.java` | The loop and subsystem arbitration. |
| `lib/command/Command.java`, `Commands.java` | Lifecycle, decorators, static factories. |
| `lib/command/Trigger.java` | Condition → command bindings. |
| `lib/command/*CommandGroup.java` | Sequential, parallel, race, deadline. |
| `lib/util/InterpolatingDoubleTreeMap.java` | Linear lookup table, clamped at the ends. |
| `lib/util/LoopTimer.java` | Measures the loop rate the gains were tuned at. Mean, worst, tail, overruns. |
| `lib/util/BatteryMonitor.java` | Polls voltage on an interval (a bus read is not free), warns on low and sag. |
| `logging/MatchLog.java` | Bounded lock-free ring + background writer. The loop never waits for storage. |
| `logging/MatchRecorder.java` | The schema: pose, drift, velocity, vision, aim, flywheel, one row per loop. |

### OpModes

| OpMode | Group | Purpose |
|---|---|---|
| **Drivetrain Direction Check** | `Setup` | **Run first.** Confirms forward / strafe / turn conventions and each wheel. |
| **Drivetrain Characterization** | `Setup` | Measures `MAX_ROBOT_SPEED`, `ZERO_POWER_DECEL_RATE`, `TURN_POWER_PER_RAD_PER_SEC`, the stiction floor and flywheel spin-up. Prints them pasteable. |
| **Shooter Map Tuning** | `Setup` | Build the shot table: auto-aims so the distance is honest, bypasses the map, logs pasteable rows. |
| **Shoot On The Move** | `Drive` | Command-based TeleOp. Drive and shoot without stopping. |
| **Shoot On The Move Auto** | `Auto` | Drives a route while tracking the goal and shooting, either alliance. |
| Localization Test | `Vision` | Streams fused vs. raw odometry vs. raw vision. The tuning and diagnosis tool. |
| Robot-Centric Mecanum Drive | `Drive` | Plain TeleOp; drives relative to the robot's own front. |
| Field-Centric Mecanum Drive | `Drive` | TeleOp relative to the driver's point of view, with alliance select. |
| Path Auto | `Auto` | Paste Path Planner output between the markers. |
| Follow Path Example | `Pathing` | A worked two-segment path. |
| Alliance Auto Example | `Auto` | One plan, flipped to whichever alliance is selected at init. |

---

## Verifying the math

```bash
./tools/verify/run.sh
```

**JDK only — no Android SDK, no Gradle, no robot.** First it typechecks the
whole TeamCode tree against a hand-written FTC SDK stub, so a compile error
surfaces without opening Android Studio. Then it runs numeric checks covering:

- the SE(3) camera-offset transform (round trip, orthonormality, pitch-sign
  convention, gimbal-lock guard);
- the mecanum sign conventions, by running the drivetrain's own mixing through
  forward kinematics and asking what the chassis actually does;
- the estimator's Kalman gains, convergence, latency compensation and buffer
  expiry — **including the std devs the real pipeline produces**, not just
  hand-picked ones;
- the **heading-trust gate**, by driving the real `Localization` subsystem
  against synthetic Limelight frames: earning and revoking trust, the
  MegaTag1 → MegaTag2 switchover, catching a 180° seed, re-seeding from vision,
  and outlier rejection with its escape hatch;
- the **shoot-on-the-move aiming**, by simulating the game piece: it launches
  from the shooter at the speed the shot table implies, adds the robot's velocity,
  flies for the table's flight time, and checks where it lands. Across 151 swept
  cases the worst miss is 0.0018 in, against 47.99 in for naive aiming;
- the **command framework's** semantics — subsystem exclusivity, interruption,
  default commands, group behaviour, decorators, trigger edges, and that
  scheduling from inside a running command does not corrupt the loop;
- the **shot table's** interpolation and clamping, plus internal consistency of
  the shipped constants;
- alliance flipping, including that it is an exact involution and that flipped
  tangent headings still match the flipped geometry;
- a **closed-loop follower simulation** that drives the example path to
  completion and reports time-to-finish and worst heading error;
- that a Path Planner output block still **compiles against the live API**, so
  the generator and the Java cannot silently drift apart.

Run it after changing any constant, sign convention or gain. It will not tell
you the right gains for your hardware, and the stubs are a hand-written subset
of the SDK rather than the real thing — so it is a fast safety net, not a
substitute for building the app before a competition. What it does catch is a
change that makes the follower diverge, never terminate, or quietly stop
fusing vision.

---

## Hardware assumed

- **Control Hub** (or REV Expansion Hub + phone) with the FTC SDK **11.1**.
- **goBILDA Pinpoint** odometry computer, I²C, configured as `pinpoint`, with
  two dead-wheel pods.
- **Limelight 3A**, configured as `limelight`, with an AprilTag pipeline and the
  season's field map uploaded.
- **Mecanum drivetrain**, four motors configured as `fl`, `fr`, `bl`, `br`.

Every one of those names is a constant you can change — see
[QUICK_REFERENCE.md](QUICK_REFERENCE.md). The Limelight is optional (see
[running without one](#running-without-a-limelight)); the Pinpoint is not.

---

## Credits and licensing

- `lib/geometry/` (2D) and `lib/estimator/PoseEstimator` are ported from
  **WPILib** (BSD-3-Clause, FIRST). The fusion approach and the
  distance²/tag-count std-dev scaling follow the **AdvantageKit** vision
  template.
- The SE(3) layer (`Rotation3d`, `Translation3d`, `Pose3d`, `Transform3d`), the
  path follower, the alliance handling and the Path Planner are original to this
  template.
- The path follower's structure — a vector follower with translational,
  centripetal, drive and heading contributions — follows the approach popularised
  by **Pedro Pathing**, written here against this template's own estimator.
- Everything else is the standard FTC SDK; see
  [docs/FTC_SDK_README.md](docs/FTC_SDK_README.md) and `LICENSE`.
