# Quick Reference

One-page lookup. Full explanations in [README.md](README.md); step-by-step setup
and troubleshooting in [MANUAL.md](MANUAL.md).

---

## Conventions

| | |
|---|---|
| **Field frame** | origin at centre, +X right, +Y away from audience, heading CCW-positive |
| **Robot frame** | +X forward, +Y left, turn CCW-positive |
| **Units** | inches and radians (degrees only at the edges) |
| **Field size** | 144″ × 144″, coordinates ≈ −72 … +72 |
| **Alliance** | does **not** change the frame — only the driver's viewpoint and which side an auto runs on |

Heading `0` faces +X. `90°` faces +Y.

---

## OpModes

| OpMode | Group | Use it to |
|---|---|---|
| **Drivetrain Direction Check** | `Setup` | **Run first.** Verify forward / strafe / turn and each wheel |
| **Shooter Map Tuning** | `Setup` | Build the shot table; auto-aims, bypasses the map, logs rows |
| **Shoot On The Move** | `Drive` | Command-based TeleOp; drive and shoot without stopping |
| Localization Test | `Vision` | Watch fused vs. odometry vs. vision; diagnose everything |
| Robot-Centric Mecanum Drive | `Drive` | Drive relative to the robot's own front |
| Field-Centric Mecanum Drive | `Drive` | Drive relative to the driver, with alliance select |
| Path Auto | `Auto` | Paste Path Planner output between the markers |
| Follow Path Example | `Pathing` | A worked two-segment path |
| Alliance Auto Example | `Auto` | One plan, flipped to the selected alliance |

---

## Controls

**Field-Centric Drive**

| Control | Action |
|---|---|
| left stick | translate (driver's point of view) |
| right stick X | turn |
| right bumper | hold for slow mode (0.4×) |
| `X` / `B` *(init)* | select blue / red alliance |
| `back` | re-zero field heading to current facing |
| `Y` | re-seed whole pose from vision (MegaTag1; needs a fresh tag) |

**Drivetrain Direction Check** — hold each:

| Control | Robot should |
|---|---|
| `dpad up` / `down` | drive forward / backward |
| `dpad left` / `right` | slide left / right |
| `X` / `B` | rotate CCW (left) / CW (right) |
| `A` / `Y` | front-left / front-right wheel alone rolls forward |
| `left` / `right bumper` | back-left / back-right wheel alone rolls forward |

---

## Hardware names

| Device | Name | Constant |
|---|---|---|
| Motors | `fl` `fr` `bl` `br` | `PathConstants.*_MOTOR` |
| Pinpoint (I2C) | `pinpoint` | `VisionConstants.PINPOINT_NAME` |
| Limelight (Ethernet) | `limelight` | `VisionConstants.LIMELIGHT_NAME` |

---

## API

```java
// --- set up ---
Localization localization = new Localization(hardwareMap);        // vision per VISION_ENABLED
Localization localization = new Localization(hardwareMap, false); // odometry only
MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
Follower follower = new Follower(localization, drivetrain);

localization.setStartingPose(new Pose2d(x, y, Rotation2d.fromDegrees(deg)));

// --- each loop ---
localization.update();                    // Follower.update() does this for you
Pose2d pose = localization.getPose();
localization.addTelemetry(telemetry);

// --- follow a path ---
follower.followPath(chain);
while (opModeIsActive() && follower.isBusy()) follower.update();

// --- shut down ---
drivetrain.stop();
localization.stop();
```

### Building paths

```java
Path p = new Path(new BezierCurve(
        new Translation2d(0, 0),          // 2 points = line, 3 = quadratic,
        new Translation2d(20, 0),         // 4 = cubic, more = higher order
        new Translation2d(30, 30)))
    .setTangentHeading();                 // face along the path
//  .setReverseTangentHeading()           // face backwards (drive in reverse)
//  .setLinearHeading(startRad, endRad)   // sweep, shortest way round
//  .setConstantHeading(rad)              // hold one heading

PathChain chain = new PathChain(p0, p1, p2);
```

### Localization readouts

| Call | Returns |
|---|---|
| `getPose()` | fused field pose — **the one to use** |
| `getOdometryPose()` | Pinpoint only, for comparison |
| `getVisionPose2d()` | last raw vision pose (robot-centre) |
| `wasLastVisionAccepted()` / `getLastVisionReject()` | last frame's fate and why |
| `getLastTagCount()` / `getLastAvgTagDistance()` | how good that frame was |
| `getLastVisionSource()` | `MegaTag1` or `MegaTag2` |
| `isHeadingTrusted()` | has MegaTag1 vouched for the heading? |
| `getHeadingDisagreementDegrees()` | MegaTag1 heading − estimate; ≈180 means seeded backwards |
| `getStartPoseCheck()` / `isStartPoseSuspect()` | pre-match seed verdict |
| `seedFromVision()` | snap pose to what the camera sees; `false` if no fresh fix |
| `setVisionEnabled(b)` / `isVisionEnabled()` | toggle fusion at run time |
| `getVisionZ()` / `getVisionPitch()` / `getVisionRoll()` | 3D diagnostics |

### Follower readouts

`isBusy()` · `getCrossTrackError()` · `getHeadingError()` · `getRemainingLength()`
· `getSpeed()` · `getPathIndex()` · `getClosestT()` · `stop()`

### Alliance flipping

```java
Pose2d       s = AllianceFlip.forAlliance(START,  AUTHORED_FOR, running);
PathChain    c = AllianceFlip.forAlliance(plan,   AUTHORED_FOR, running);
Translation2d p = AllianceFlip.forAlliance(point, AUTHORED_FOR, running);
```

Returns the original untouched when the alliances match. Default symmetry is
`ROTATIONAL` (180° about field centre) — correct for BIOBUZZ.

**Shoot On The Move** (command-based TeleOp)

| Control | Action |
|---|---|
| left stick | translate (driver's point of view) |
| **left bumper** | hold to **aim** at the goal (shoot on the move) |
| **right trigger** | fire, once the shot is ready |
| right stick X | turn (only when not aiming) |
| right bumper | slow mode |
| `Y` | re-seed pose from vision |
| `back` | re-zero driver-forward |
| dpad up/down | rpm trim ±50 |
| dpad left/right | hood trim ∓0.5° |
| `X` | clear trim |

**Shooter Map Tuning**

| Control | Action |
|---|---|
| **left bumper** | hold to aim (keeps the distance readout honest) |
| **right trigger** | fire |
| dpad up/down | rpm ±25 |
| dpad left/right | hood ∓0.5° |
| `A` | log the current row |
| `B` | clear the log |
| right bumper | cycle which goal you are tuning |

---

## Shooting API

```java
// aiming
AimSolution s = AimLogic.calculate(
        pose, fieldVelocity, omegaRadPerSec, goal, shooterMap, AIM_CONFIG);
s.targetHeadingRadians           // where to point the ROBOT
s.headingFeedforwardRadPerSec    // sweep rate; add to the heading controller
s.effectiveDistanceInches        // look the shot up with THIS, not the actual distance
s.canShootFrom(currentHeading)   // in range AND pointed correctly

// goal selection (handles alliance flipping for you)
GoalSelector sel = ShootingConstants.newGoalSelector();
Goal g = sel.update(pose, localization.getVisibleTagIds(), alliance);
Translation2d goal = sel.getTargetPosition(alliance);
sel.lockTo("high") / cycle() / auto(strategy) / freeze(true)
sel.describe()                   // one telemetry line

// derive goals from the surveyed tag poses (preferred)
GOALS = TagGoals.from(TAG_FIELD_POSES)
        .goal("hive").fromTag(11).outward(6).radius(7).worth(5).map(HIGH_MAP)
        .goal("flower").fromTags(21, 22).outward(6).radius(9).worth(2)
        .build();
//   outward(n)    n in front of the tag's face (toward the robot)
//   alongFace(n)  n sideways along the face, + to the tag's left

// or enter coordinates directly
Goal.named("high", 0, 60).tags(21, 22).radius(8).worth(5).map(HIGH_MAP).build();

// shot table
ShooterMap map = ShooterMap.builder()
        //     distance(in)   rpm   hood(deg)  flight(s)
        .add(           36,  2350,     27.0,      0.40)
        .build();
map.setpointAt(d) / rpmAt(d) / hoodAt(d) / timeOfFlightAt(d) / covers(d)

// shooter
shooter.setShotForDistance(s.effectiveDistanceInches);
shooter.atSpeed() / idle() / stow() / runFeeder() / stopFeeder()
shooter.addRpmTrim(50) / addHoodTrim(0.5) / clearTrim()

// drive
drive.driveWithHeadingLock(fieldX, fieldY, targetHeading, feedforward);
drive.driveDriverRelative(fwd, left, turn, alliance);
drive.getFieldVelocity() / getAngularVelocity()
```

---

## Command system

```java
public class MyOpMode extends CommandOpMode {
    @Override public void configure() {
        register(drive, shooter);
        setDefaultCommand(drive, new RunCommand(() -> drive.drive(...), drive));
        whileHeld(() -> gamepad1.a, myCommand);
        whenPressed(() -> gamepad1.b, Commands.runOnce(shooter::stow, shooter));
    }
}
```

| Factory | Does |
|---|---|
| `Commands.runOnce(r, subs)` | once, then ends |
| `Commands.run(r, subs)` | every loop, never ends |
| `Commands.startEnd(a, b, subs)` | hold-to-run |
| `Commands.waitSeconds(t)` / `waitUntil(cond)` | timing / gating |
| `Commands.sequence(...)` | one after another |
| `Commands.parallel(...)` | together, ends when all end |
| `Commands.race(...)` | together, ends when the first ends |
| `Commands.deadline(d, ...)` | others run while `d` does |

| Decorator | Does |
|---|---|
| `.withTimeout(s)` | give up after `s` |
| `.until(cond)` / `.onlyWhile(cond)` | end early |
| `.andThen(c)` / `.alongWith(c)` / `.raceWith(c)` | compose |
| `.repeatedly()` | restart forever |
| `.finallyDo(r)` | cleanup however it ends |
| `.onlyIf(cond)` / `.unless(cond)` | conditional schedule |
| `.withInterruptBehavior(b)` | `CANCEL_SELF` (default) or `CANCEL_INCOMING` |

| Binding | Fires |
|---|---|
| `whenPressed` | rising edge |
| `whileHeld` | while true, cancels on release |
| `toggleWhenPressed` | press on, press off |

Rules: one command per subsystem at a time; a new command cancels the holder;
default commands fill idle subsystems; groups claim all their children's
subsystems; parallel children may not share one; a command instance belongs to
one group. The scheduler is per-OpMode, **not** a static singleton.

---

## Constants

### `PathConstants` — drivetrain and follower

| Constant | Default | Meaning |
|---|---|---|
| `FRONT/BACK_LEFT_DIRECTION` | `REVERSE` | **Sides must be opposite** |
| `FRONT/BACK_RIGHT_DIRECTION` | `FORWARD` | |
| `TRANSLATIONAL_kP / kD` | `0.04 / 0.010` | pull onto the path |
| `DRIVE_kP / kD` | `0.040 / 0.010` | push along the path |
| `HEADING_kP / kD` | `0.30 / 0.08` | hold target heading |
| `HEADING_CORRECTION_SIGN` | `1.0` | leave at `+1.0`; check wiring instead |
| `CENTRIPETAL_SCALE` | `0.0006` | raise if it swings wide on curves |
| `ZERO_POWER_DECEL_RATE` | `30.0` in/s² | **measure**: `v²/(2·d)` |
| `MAX_ROBOT_SPEED` | `50.0` in/s | top speed at full output |
| `END_TRANSLATION_TOLERANCE` | `1.0` in | finished-position window |
| `END_HEADING_TOLERANCE` | `2°` | finished-heading window |
| `END_VELOCITY_TOLERANCE` | `2.0` in/s | must be settled |
| `ADVANCE_LENGTH_TOLERANCE` | `2.0` in | when to start the next segment |

### `ShootingConstants` — shooting

| Constant | Default | Meaning |
|---|---|---|
| `TAG_FIELD_POSES` | 2 placeholders | **PLACEHOLDER — set these.** Tag ID → field pose; goals are derived from them |
| `GOALS` | derived | Built by `TagGoals.from(TAG_FIELD_POSES)`; add `outward`/`alongFace` offsets per goal |
| `AUTHORED_FOR` | `RED` | Alliance the goal positions are written for |
| `GOAL_STRATEGY` | `TAG_VISIBLE` | How the robot picks a goal |
| `GOAL_TAG_MEANING` | `VISIBLE_MEANS_AVAILABLE` | **Check the manual** — invert if a covered tag marks the open goal |
| `GOAL_SWITCH_FRAMES` | `12` | Frames a challenger must win before the chassis re-aims |
| `GOAL_RADIUS_IN` | `6.0` | Default opening half-width; sets the heading tolerance |
| `SHOOTER_FORWARD/LEFT_OFFSET_IN` | `0.0` | Shooter position from the turn centre |
| `SHOOTER_YAW_OFFSET_DEG` | `0.0` | 0 = fires forward, 180 = out the back |
| `PHASE_DELAY_SECONDS` | `0.02` | Control-latency lookahead |
| `MAX_AIM_ITERATIONS` | `10` | Cap on virtual-goal passes |
| `AIM_CONVERGENCE_TOLERANCE_IN` | `0.01` | Stop iterating below this |
| `MAX_HEADING_TOLERANCE_DEG` | `20.0` | Ceiling on the scaled tolerance |
| `MIN/MAX_SHOT_DISTANCE_IN` | `18 / 96` | Must sit **inside** `SHOT_MAP`'s range |
| `SHOT_MAP` | 5 rows | **PLACEHOLDER — measure this.** distance → rpm, hood, flight time |
| `FLYWHEEL_TICKS_PER_REV` | `28.0` | Motor encoder ticks (bare goBILDA/REV = 28) |
| `FLYWHEEL_GEAR_RATIO` | `1.0` | Flywheel revs per motor rev |
| `FLYWHEEL_P/I/D/F` | `12/0/0/14` | Velocity PIDF; tune **F** first (≈32767/maxTicksPerSec) |
| `FLYWHEEL_TOLERANCE_RPM` | `75.0` | "At speed" window |
| `FLYWHEEL_IDLE_RPM` | `1200.0` | Held while waiting |
| `HOOD_DEG_AT_SERVO_0/1` | `15 / 45` | Maps hood angle onto servo travel |
| `FEED_TIME_SECONDS` | `0.35` | One shot's feed duration |

### `VisionConstants` — localization

| Constant | Default | Meaning |
|---|---|---|
| `VISION_ENABLED` | `true` | `false` = odometry only, camera never initialised |
| `LIMELIGHT_PIPELINE` | `0` | AprilTag pipeline index |
| `PINPOINT_X_OFFSET` | `-3.125` | X pod sideways offset (left +) |
| `PINPOINT_Y_OFFSET` | `3.375` | Y pod forward offset (forward +) |
| `PINPOINT_OFFSET_UNIT` | `INCH` | **must match the numbers above** |
| `APPLY_CAMERA_OFFSET_IN_CODE` | `false` | `false` = mount configured in the Limelight UI |
| `CAMERA_PITCH_OFFSET_DEG` | `0.0` | **negative = tilted up** |
| `ODOMETRY_STD_DEVS` | `{0.5, 0.5, 2°}` | lower = trust odometry more |
| `VISION_XY_STD_DEV_COEFFICIENT` | `0.0005` | **inches, not metres** — see below |
| `VISION_HEADING_STD_DEV` | `45°` | large on purpose; gyro owns heading |
| `PREFER_MEGATAG2` | `true` | use MegaTag2 for position once heading is trusted |
| `HEADING_TRUST_FRAMES` | `10` | agreeing frames needed to trust heading |
| `HEADING_TRUST_TOLERANCE` | `20°` | counts as agreement |
| `HEADING_DISTRUST_THRESHOLD` | `60°` | revokes trust |
| `HEADING_SEED_WARN_THRESHOLD` | `45°` | fires the pre-match warning |
| `MAX_POSE_JUMP_IN` | `36.0` | reject a frame this far from a settled estimate |
| `MIN_TAG_COUNT` | `1` | raise to `2` if single-tag fixes are noisy |
| `MAX_STALENESS_MS` | `200` | drop frames older than this |

---

## The three sign conventions that break things

| Convention | Correct | Symptom when wrong |
|---|---|---|
| Motor directions | left and right **opposite** | "forward" spins the robot in place |
| Gamepad turn | `turn = -right_stick_x` | robot turns opposite the stick |
| `HEADING_CORRECTION_SIGN` | `+1.0` | robot spins away from target, accelerating |

---

## Vision std devs (the units trap)

```
xyStdDev = VISION_XY_STD_DEV_COEFFICIENT · avgTagDistance² / tagCount
gain     = q / (q + √(q · r))          q = odometry var, r = vision var
```

| distance | tags | std dev | gain |
|---|---|---|---|
| 18″ | 2 | 0.08″ | 0.86 |
| 40″ | 1 | 0.80″ | 0.38 |
| 60″ | 2 | 0.90″ | 0.36 |
| 100″ | 1 | 5.00″ | 0.09 |

AdvantageKit publishes `0.02` **in metres**. This template is in **inches** and
the distance term is squared, so the equivalent is `0.02 / 39.37 ≈ 0.0005`.
Using `0.02` or `2.0` here produces std devs of hundreds of inches, drives the
gain to ~0, and **silently disables vision fusion** — the pose still looks fine
because odometry carries it.

---

## MegaTag1 vs MegaTag2

| | MegaTag1 (`getBotpose`) | MegaTag2 (`getBotpose_MT2`) |
|---|---|---|
| Uses the gyro | no | yes |
| Can audit the gyro | **yes** | no (echoes your yaw back) |
| Single-tag ambiguity | vulnerable | resolved |
| Fails when | one tag, viewed head-on | your heading is wrong |
| Used here for | **heading, always** | **position, once heading is trusted** |

Telemetry shows `Vision source: MegaTag2 | heading TRUSTED`.

---

## Telemetry cheat sheet

| Reject reason | Means |
|---|---|
| `no valid result` | no tag in frame, or wrong pipeline |
| `too few tags` | under `MIN_TAG_COUNT` |
| `stale` | older than `MAX_STALENESS_MS`; loop running slow |
| `no MegaTag1 fix` | botpose missing or exactly the origin |
| `off field` | solved outside the field + margin |
| `jump NN in` | too far from a settled estimate |
| `vision disabled` | `VISION_ENABLED = false` |

---

## Shoot-on-the-move gotchas

| Symptom | Cause |
|---|---|
| Good standing, misses while **moving** | flight time left at 0 in `SHOT_MAP` — the correction computes a zero offset and does nothing |
| Good standing, misses while **turning** | `SHOOTER_FORWARD/LEFT_OFFSET_IN` wrong (the ω × r term) |
| Points 180° wrong | `SHOOTER_YAW_OFFSET_DEG` |
| Always trails a moving aim | raise `PHASE_DELAY_SECONDS`; check `TURN_POWER_PER_RAD_PER_SEC` in `DriveSubsystem` |
| Never fires | read `AimAndShootCommand.getStatus()` — it names the blocker |
| Distance disagrees with tape | the goal's position, or the seeded start pose |
| Robot swings between two goals | `GOAL_SWITCH_FRAMES` too low |
| Always picks the wrong goal | `GOAL_TAG_MEANING` inverted, or wrong tag IDs |

Distances are **inches**. The FRC original was metres.

---

## Commands

```bash
./tools/verify/run.sh     # typecheck + math suite; no Android SDK or robot needed
```

Path Planner: open `TeamCode/src/main/assets/pathplanner.html`, or browse to
`http://192.168.43.1:8080/pathplanner` on the robot's Wi-Fi.
