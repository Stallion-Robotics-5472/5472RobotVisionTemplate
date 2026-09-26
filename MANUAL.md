# Manual

How to get this template running on your robot, tune it, and diagnose it when it
misbehaves. For *how it works*, read [README.md](README.md). For a one-page
lookup table, see [QUICK_REFERENCE.md](QUICK_REFERENCE.md).

Work through Part A once. After that, Part B (tuning) and Part C
(troubleshooting) are reference material.

---

## Contents

**Part A — Setup**
1. [Before you start](#1-before-you-start)
2. [Configure the robot](#2-configure-the-robot)
3. [Set up the Limelight](#3-set-up-the-limelight)
4. [Check the drivetrain before anything else](#4-check-the-drivetrain-before-anything-else)
5. [Enter your robot's measurements](#5-enter-your-robots-measurements)
6. [Verify localization](#6-verify-localization)
7. [Drive it](#7-drive-it)

**Part B — Using it**
8. [Tune the path follower](#8-tune-the-path-follower)
9. [Draw a path](#9-draw-a-path)
10. [Write an autonomous](#10-write-an-autonomous)
11. [One auto, both alliances](#11-one-auto-both-alliances)
12. [Set up shooting](#12-set-up-shooting)
13. [Write command-based code](#13-write-command-based-code)
14. [Competition day checklist](#14-competition-day-checklist)

**Part C — When it goes wrong**
15. [Troubleshooting](#15-troubleshooting)
16. [Reading the telemetry](#16-reading-the-telemetry)

---

# Part A — Setup

## 1. Before you start

You need:

- **Android Studio** (Ladybug 2024.2 or later) and this project cloned.
- A **Control Hub** with the FTC SDK **11.1**.
- A **mecanum drivetrain**, four motors.
- A **goBILDA Pinpoint** with two dead-wheel pods. *Required.*
- A **Limelight 3A**. *Optional* — see [step 3](#3-set-up-the-limelight) to run
  without one.
- A tape measure. You will need it more than you expect.

Sanity-check the codebase before you touch the robot:

```bash
./tools/verify/run.sh
```

That typechecks everything and runs the math suite, with no Android SDK or robot
required. It should end with `All checks passed.` If it does not, fix that first
— you would otherwise be debugging a known-broken build on a real robot.

---

## 2. Configure the robot

In the Robot Controller configuration, create these devices. **The names must
match exactly**, or change the constants to match your names.

| Device | Type | Name | Constant |
|---|---|---|---|
| Front left motor | DcMotor | `fl` | `PathConstants.FRONT_LEFT_MOTOR` |
| Front right motor | DcMotor | `fr` | `PathConstants.FRONT_RIGHT_MOTOR` |
| Back left motor | DcMotor | `bl` | `PathConstants.BACK_LEFT_MOTOR` |
| Back right motor | DcMotor | `br` | `PathConstants.BACK_RIGHT_MOTOR` |
| Pinpoint | I2C Device | `pinpoint` | `VisionConstants.PINPOINT_NAME` |
| Limelight 3A | Ethernet Device | `limelight` | `VisionConstants.LIMELIGHT_NAME` |

Deploy the app. All seven OpModes should appear on the Driver Station.

---

## 3. Set up the Limelight

1. Connect to the Limelight's web UI.
2. Create an **AprilTag** pipeline and note its index — that goes in
   `VisionConstants.LIMELIGHT_PIPELINE` (default `0`).
3. **Upload the current season's field map.** Without it the Limelight has no
   idea where the tags are and botpose is meaningless.
4. Enter the **camera→robot offset** in the UI (forward / left / up and the
   angles). This is option (A) and it is what the template expects — leave
   `APPLY_CAMERA_OFFSET_IN_CODE = false`.

> Do **not** enter the offset in both the UI and the code. It gets applied twice
> and the pose will be wrong by exactly one camera offset, which is just small
> enough to be maddening.

If you would rather configure the mount in code, zero the UI offset, set
`APPLY_CAMERA_OFFSET_IN_CODE = true`, and fill in the `CAMERA_*` constants.

> **The pitch sign catches people out.** Pitch rotates about +Y (left), so a
> **positive** pitch tilts the camera **down**. A camera angled 15° *upward* to
> see tags is `-15.0`. The wrong sign doubles the mount error instead of
> removing it.

### Running without a Limelight

Set `VisionConstants.VISION_ENABLED = false`. The camera is never initialised,
so an unplugged Limelight will not crash the OpMode, and the fused pose is
simply the Pinpoint pose. Path following works unchanged — it just drifts.

---

## 4. Check the drivetrain before anything else

**Run the `Drivetrain Direction Check` OpMode (group `Setup`) before any other
driving OpMode.** Put the robot on blocks if you are unsure.

Everything downstream assumes three conventions:

| command | robot must |
|---|---|
| forward power | drive forward |
| strafe power | slide **left** |
| positive turn power | rotate **counter-clockwise** |

If one is inverted the path follower does not just track badly — it steers
*away* from the target and the error grows until the match ends.

Hold each button and confirm what the robot does:

| Button | Expect |
|---|---|
| `dpad up` / `dpad down` | drives forward / backward |
| `dpad left` / `dpad right` | slides left / right |
| `X` / `B` | rotates CCW (left) / CW (right) |
| `A` | front-left wheel alone rolls forward |
| `Y` | front-right wheel alone rolls forward |
| `left bumper` | back-left wheel alone rolls forward |
| `right bumper` | back-right wheel alone rolls forward |

**Fixing what you find.** The per-wheel tests tell you which motor `Direction`
in `PathConstants` is wrong. Fix those first, then re-check the whole-robot
motions.

> **The two sides must be opposite.** On a mecanum the left and right motors
> face opposite ways. Setting all four the same does *not* reverse the robot —
> it swaps translation and rotation, so "drive forward" spins the robot in
> place. The OpMode warns at init if it spots this. If forward/back is inverted
> but each wheel is individually correct, flip **both** sides together, never
> one alone.

---

## 5. Enter your robot's measurements

Now the tape measure. These are in `VisionConstants`.

### Odometry pod offsets

Measured from the robot's tracking point (usually its centre):

- `PINPOINT_X_OFFSET` — how far **sideways** the X (forward) pod is.
  Left is positive.
- `PINPOINT_Y_OFFSET` — how far **forward** the Y (strafe) pod is.
  Forward is positive.

Both are read in `PINPOINT_OFFSET_UNIT`, which defaults to `INCH`. **The numbers
and that unit must agree** — measuring in millimetres and leaving the unit at
`INCH` puts you off by a factor of 25.

### Pod type and direction

```java
PINPOINT_POD_TYPE    = goBILDA_4_BAR_POD;   // or goBILDA_SWINGARM_POD
PINPOINT_X_DIRECTION = REVERSED;
PINPOINT_Y_DIRECTION = FORWARD;
```

To check the directions, run `Localization Test`, push the robot **forward by
hand**, and watch `Raw Odometry`. `x` must increase. Then push it **left**; `y`
must increase. Flip the matching direction constant if either reads backwards.

---

## 6. Verify localization

Run `Localization Test` (group `Vision`) and work through these in order. Do not
skip ahead — each one depends on the last.

**a. Odometry tracks.** Push the robot 24″ forward by hand. `Raw Odometry` x
should read about 24. If it reads 10 or 60, the pod type or offsets are wrong.

**b. Vision sees tags.** Point the camera at an AprilTag. `Vision` should read
`accepted true`. If not, the reject reason tells you why — see
[§16](#16-reading-the-telemetry).

**c. Vision agrees with reality.** Put the robot at a known spot on the field
and compare `Raw Vision 2D` against where it actually is. They should agree
within an inch or two. If there is a constant offset of a few inches, the camera
mount offset is wrong or double-applied.

**d. Fusion corrects drift.** Drive around for thirty seconds without looking at
a tag, then point the camera at one. `Fused` should walk toward `Raw Vision 2D`
over roughly a second while `Raw Odometry` stays wrong. **If `Fused` tracks
`Raw Odometry` exactly and ignores vision entirely, vision fusion is not
happening** — check `VISION_XY_STD_DEV_COEFFICIENT`, which must be about
`0.0005` for inches.

**e. Heading earns trust.** With a tag in view, `Vision source` should move from
`MegaTag1 | heading unverified` to `MegaTag2 | heading TRUSTED` within a second.
If it never does, the seeded heading disagrees with what the camera sees —
which is exactly what step (f) is about.

**f. The start-pose check works.** Seed a deliberately wrong heading (set
`setStartingPose` to 180° off), run again, and confirm you get
`*** START POSE LOOKS WRONG ***`. Better to see that warning now, on purpose,
than for the first time at a competition.

---

## 7. Drive it

Start with `Robot-Centric Mecanum Drive` — it has no dependence on the pose
estimate, so it isolates the drivetrain.

Then `Field-Centric Mecanum Drive`:

| Control | Does |
|---|---|
| left stick | translate, from the **driver's** point of view |
| right stick X | turn |
| right bumper | hold for slow mode |
| `X` / `B` *(init)* | select blue / red alliance |
| `back` | re-zero field heading to the robot's current facing |
| `Y` | re-seed the whole pose from what the camera sees |

Set `START_POSE` to where the robot actually starts. `(0, 0, 0)` is the middle
of the field facing +X, which is almost certainly not where it is.

**Check the field-centric convention:** stand where your drive team stands,
select your alliance, and push the left stick away from you. The robot must
drive away from you *regardless of which way it is facing*. If it drives toward
you, the heading is 180° off — press `Y` with a tag in view, or fix `START_POSE`.

---

# Part B — Using it

## 8. Tune the path follower

Everything here is in `PathConstants`. Tune in this order; each step assumes the
ones before it are done.

### a. Measure the deceleration rate

Not a guess — measure it. Drive at full speed, cut power, and record the entry
speed `v` and stopping distance `d`:

```
ZERO_POWER_DECEL_RATE = v² / (2·d)
```

Set `MAX_ROBOT_SPEED` to the top speed you actually reach, in inches/second.
Both feed the deceleration feedforward that brings the robot to a stop at the
path end.

### b. Heading first

Give the robot a short path with `setConstantHeading`, start it facing 90° off,
and watch it turn.

| Symptom | Fix |
|---|---|
| turns the wrong way and accelerates | **stop** — go back to [§4](#4-check-the-drivetrain-before-anything-else); the drivetrain is mis-wired |
| oscillates around the target | lower `HEADING_kP`, raise `HEADING_kD` |
| stops short and never arrives | raise `HEADING_kP` |

`HEADING_CORRECTION_SIGN` should stay at `+1.0`. It exists only for a drivetrain
that inverts rotation, and reaching for it is almost always treating the symptom
of a wiring problem.

### c. Then translation

Run a straight path and watch the `cross` (cross-track) telemetry.

| Symptom | Fix |
|---|---|
| drifts off the line | raise `TRANSLATIONAL_kP` |
| wobbles along the line | lower `TRANSLATIONAL_kP`, raise `TRANSLATIONAL_kD` |
| overshoots the endpoint | raise `ZERO_POWER_DECEL_RATE` |
| brakes far too early | lower `ZERO_POWER_DECEL_RATE` |

### d. Finally curves

Run an arc.

| Symptom | Fix |
|---|---|
| swings wide on curves | raise `CENTRIPETAL_SCALE` |
| cuts corners tight | lower `CENTRIPETAL_SCALE` |

Keep it small — it scales with *speed squared*, so a value that feels right at
low speed can dominate at full speed.

---

## 9. Draw a path

Open the Path Planner, either way:

- **Locally** — open `TeamCode/src/main/assets/pathplanner.html` in a browser.
- **From the robot** — connect to the robot's Wi-Fi and browse to
  `http://192.168.43.1:8080/pathplanner`.

Then:

1. Set the start pose (x, y, heading).
2. Click to add control points. Drag to shape the curve.
3. Pick a heading mode per segment:
   - **Tangent** — face along the path.
   - **Reverse tangent** — face backwards along it (drive in reverse).
   - **Linear** — sweep from one heading to another.
   - **Constant** — hold one heading.
4. Add more segments as needed; each starts where the last ended.
5. Copy the generated Java.

The field is 144″ × 144″, origin at centre, +X right, +Y up, heading degrees
CCW — the same convention the code uses, so what you draw is what you get.

**⟳ Swap Alliance** rotates the whole plan 180° about the field centre, the same
transform `AllianceFlip` applies at run time. The two Mirror buttons are for
genuinely mirror-symmetric fields and are *not* an alliance swap on BIOBUZZ.

---

## 10. Write an autonomous

Open `PathAuto`, paste between the markers, done:

```java
// ===================== PASTE PATH PLANNER OUTPUT BELOW =====================
localization.setStartingPose(new Pose2d(-58.2, -35.7, Rotation2d.fromDegrees(90.0)));

Path path0 = new Path(new BezierCurve(
        new Translation2d(-58.2, -35.7),
        new Translation2d(-20.5, -10.1),
        new Translation2d(12.3, 24.8)))
        .setTangentHeading();

PathChain chain = new PathChain(path0);
follower.followPath(chain);
// ===================== PASTE PATH PLANNER OUTPUT ABOVE =====================
```

The planner emits exactly the variable names `PathAuto` already declares, so it
pastes in verbatim.

**Seed the real starting pose.** `setStartingPose` must match where the robot
physically sits. Odometry dead-reckons from whatever you give it, so a wrong
seed means a wrong pose until vision pulls it in — and in a 30-second auto you
may not get that long.

To run your own actions between segments, split the chain and wait on each:

```java
follower.followPath(new PathChain(path0));
while (opModeIsActive() && follower.isBusy()) follower.update();

scoreTheThing();

follower.followPath(new PathChain(path1));
while (opModeIsActive() && follower.isBusy()) follower.update();
```

---

## 11. One auto, both alliances

Author the plan once for one alliance and flip it at run time. See
`AllianceAutoExample`:

```java
private static final Alliance AUTHORED_FOR = Alliance.RED;
private static final Pose2d START = new Pose2d(-58, -58, Rotation2d.fromDegrees(45));

// during init, once the driver has picked `alliance`:
localization.setStartingPose(AllianceFlip.forAlliance(START, AUTHORED_FOR, alliance));

// after start:
PathChain plan = AllianceFlip.forAlliance(buildPlan(), AUTHORED_FOR, alliance);
follower.followPath(plan);
```

`forAlliance` returns the original object untouched when the alliances already
match, so there is no cost on your home side.

**Seed during init, not after start.** That gives the camera time to check the
seed while the robot sits still, so a wrong alliance button shows up before the
match rather than during it. Re-seed only when the selection actually *changes*
— `setStartingPose` resets the heading-trust counters, so calling it every loop
stops vision ever vouching for the seed.

**Does the flip affect vision?** No. AprilTags define one absolute field frame,
so `AllianceFlip` transforms plans, never measurements. Vision reads identically
for both alliances. The thing that *does* go wrong is the heading **seed** —
which is what the init-time check exists to catch.

---

## 12. Set up shooting

Shoot-on-the-move aims at a *virtual goal* offset by whatever the robot's motion
will add to the shot, so the robot never has to stop. Read
[`SHOOTING.md`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooting/SHOOTING.md)
for how it works; this is the order to set it up in. Everything is in
`ShootingConstants.java`.

### a. The goals — nothing works until these are right

BIOBUZZ scores on hives and flowers, so every goal you shoot at is listed in
`ShootingConstants.GOALS`. **Derive them from the AprilTag poses** rather than typing
coordinates twice — the tags are already surveyed into the field frame, so a tag
beside a goal already says where that goal is:

```java
public static final Map<Integer, Pose2d> TAG_FIELD_POSES = TagGoals.tagTable(
        //        id,            x,     y,  facing (deg)
        11, TagGoals.tagAt(  -36.0,  66.0,  -90.0),
        21, TagGoals.tagAt(   36.0,  66.0,  -90.0));

public static final Goal[] GOALS = TagGoals.from(TAG_FIELD_POSES)
        .goal("hive").fromTag(11).outward(6.0).radius(7.0).worth(5)
        .goal("flower").fromTag(21).outward(6.0).radius(9.0).worth(2)
        .build();
```

A typo in a hand-entered goal coordinate is silent — aiming is confidently wrong and
nothing flags it. A wrong tag pose instead makes localization and aiming visibly
disagree.

You still supply the small offset from the tag to the point the piece must pass
through, since only the drawings know it: `outward(n)` is *n* inches in front of the
tag's face, `alongFace(n)` slides sideways along it. `fromTags(2, 3)` averages two
tags for a goal flanked by one either side.

**The shipped entry is a placeholder and will aim at empty field.** Read the real
values off the season's Competition Manual and field drawings, converting positions
into this template's frame (origin at field centre, +X right, +Y away from the
audience, inches). Aim at the point the piece must pass *through* — the middle of the
opening, not of the structure.

Give a goal **its own `.map(...)`** whenever its height differs — one flywheel and
hood curve cannot serve two heights. Give it **tag IDs** so the robot can tell which
goal it is looking at.

To check a position:

1. Run **Shooter Map Tuning** (group `Setup`).
2. Park at a known spot and compare the `DISTANCE` readout to a tape measure.
3. Adjust the position until they agree.

`radius` is a bit less than the true half-width — two thirds is a fair start, since
the piece has size, the pose has error and the shot has spread.

Then choose how the robot picks between them. `GOAL_STRATEGY` defaults to
`TAG_VISIBLE` (prefer goals whose tags the camera can see, falling back to nearest).
**Check `GOAL_TAG_MEANING` against the manual:** if a tag gets *covered* as its goal
fills, the hidden tag marks the open one and you want `HIDDEN_MEANS_AVAILABLE`.

Leave `GOAL_SWITCH_FRAMES` non-zero. On a turretless robot the goal sets the whole
chassis heading, and tag visibility flickers while driving — switching instantly
makes the robot swing between two headings and never settle.

### b. Where the shooter sits

Measure `SHOOTER_FORWARD_OFFSET_IN` and `SHOOTER_LEFT_OFFSET_IN` from the robot's
**centre of rotation**, and set `SHOOTER_YAW_OFFSET_DEG` (0 = fires forward, 180 =
out the back).

These matter more than they look. An off-centre shooter is swung sideways when the
robot rotates and the piece inherits that motion, which is the classic cause of a
robot that shoots well standing still and misses while turning.

### c. Tune the flywheel

Set `FLYWHEEL_TICKS_PER_REV` (28 for a bare goBILDA 5202/5203 or REV HD Hex;
multiply by the gearbox if geared) and `FLYWHEEL_GEAR_RATIO`.

Then the velocity PIDF. **Tune `FLYWHEEL_F` first** — it does most of the work and
should be roughly `32767 / maxTicksPerSecond`. Add `FLYWHEEL_P` until recovery
after a shot is quick without oscillating. Leave I and D at zero unless you have a
reason.

Check it: command a speed and watch the `Flywheel` telemetry reach it and hold.
`FLYWHEEL_TOLERANCE_RPM` is the "at speed" window that gates firing — tight enough
that a shot is repeatable, loose enough that it is reachable.

### d. Build the shot table

Run **Shooter Map Tuning**. The robot auto-aims the whole time so the distance
readout stays honest, but the map is bypassed and rpm/hood come from what you dial
in.

1. Confirm `DISTANCE` against a tape measure. If it disagrees, stop and fix
   the goal's position — nothing downstream will work.
2. Park at a distance. Hold **LB** to aim and let it settle.
3. `dpad up/down` for rpm, `dpad left/right` for hood. **RT** to fire.
4. Once shots go in, press **A** to log the row.
5. Move and repeat. Five or six rows across your usable range is plenty.
6. Paste the rows into `SHOT_MAP`.

### e. Fill in the flight times

This is the step teams skip, and it is the one shoot-on-the-move depends on.

**With flight time left at zero, the moving correction computes a zero offset and
does nothing at all** — the robot shoots well standing still and misses while
moving, with no error anywhere.

You cannot read it off the robot. Film a shot in slow motion on a phone and time
from the piece leaving the shooter to reaching the goal. Even a rough value is far
better than zero.

### f. Check the range matches the table

`MIN_SHOT_DISTANCE_IN` / `MAX_SHOT_DISTANCE_IN` must sit **inside** `SHOT_MAP`'s
measured range. A shot is in range only when both allow it, so widening these past
the table does not extend the robot's reach. `./tools/verify/run.sh` asserts this.

### g. Shoot while moving

Run **Shoot On The Move** (group `Drive`). Hold **LB** to aim; the robot keeps
driving wherever you put it while rotating to lead the goal. Pull **RT** to fire.

If it will not fire, the telemetry `Shot` line names the blocker — range, aim,
flywheel speed, or pose trust.

---

## 13. Write command-based code

`lib/command` is a WPILib-shaped command system. Full detail in
[`COMMANDS.md`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/command/COMMANDS.md);
the shape is:

```java
@TeleOp(name = "My Robot")
public class MyOpMode extends CommandOpMode {
    @Override
    public void configure() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        register(drive, shooter);

        // What each subsystem does when nothing else asks for it.
        setDefaultCommand(drive, new RunCommand(
                () -> drive.driveDriverRelative(
                        -gamepad1.left_stick_y, -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x, alliance),
                drive));
        setDefaultCommand(shooter, new RunCommand(shooter::idle, shooter));

        // Hold a button to run something else; release restores the defaults.
        whileHeld(() -> gamepad1.a, Commands.sequence(
                Commands.runOnce(() -> shooter.setFlywheelRpm(2500), shooter),
                Commands.waitUntil(shooter::atSpeed),
                Commands.run(shooter::runFeeder, shooter).withTimeout(1.5)));
    }
}
```

Three rules worth knowing up front:

- **One command per subsystem.** Scheduling a command whose subsystem is taken
  cancels the holder. This is the feature — it is what stops two bits of code
  fighting over a motor.
- **Always stop your mechanism in `end(boolean interrupted)`.** A cancelled
  command that skips this leaves a motor running.
- **Don't reuse a command instance** in two groups. Build a second one; the
  framework rejects it rather than misbehaving.

`ShootOnTheMoveTeleOp` is the worked reference.

---

## 14. Competition day checklist

Before the first match:

- [ ] `./tools/verify/run.sh` passes.
- [ ] `Drivetrain Direction Check` — all motions correct.
- [ ] `Localization Test` — vision `accepted true`, fused pose tracks reality.
- [ ] `Vision source` reaches `MegaTag2 | heading TRUSTED` with a tag in view.
- [ ] Start poses in your autos match where the robot is actually placed.
- [ ] Limelight field map is **this season's**.
- [ ] Every goal's position verified against a tape measure.
- [ ] Robot's start heading puts a goal tag in the camera's view, so the
      pre-match start-pose check can actually run.
- [ ] `GOAL_TAG_MEANING` matches how the manual uses tags.
- [ ] `SHOT_MAP` built from real shots, with **non-zero flight times**.
- [ ] Shoot On The Move fires while driving, not just standing still.

Before every match:

- [ ] Correct alliance selected during init.
- [ ] Robot placed facing the way the start pose claims.
- [ ] Start-pose check reads OK, not `*** START POSE LOOKS WRONG ***`.
- [ ] Robot held still through init — the Pinpoint calibrates its IMU then.

---

# Part C — When it goes wrong

## 15. Troubleshooting

### The robot spins in place when told to drive forward

All four motor `Direction`s are set the same. On a mecanum the two sides must be
opposite. See [§4](#4-check-the-drivetrain-before-anything-else).

### The robot turns the wrong way from the stick

`turn` should be `-gamepad1.right_stick_x`. Turn power is CCW-positive, but
pushing the stick right must turn the robot right, which is clockwise.

### Auto never finishes; the robot spins faster and faster

The heading loop is inverted — positive feedback. Check the motor directions
first ([§4](#4-check-the-drivetrain-before-anything-else)), then
`HEADING_CORRECTION_SIGN` (should be `+1.0`).

### The pose snaps or jitters on every vision frame

`VisionConstants.BOTPOSE_AVG_DIST_UNIT` is probably wrong. `getBotposeAvgDist()`
is a bare double with no unit attached; the Limelight reports metres natively.
Treating metres as inches shrinks the std dev ~1550x (it goes as distance squared)
and pins the Kalman gain near 1.0, so every frame snaps the pose and distance
weighting is gone.

Check it: stand a measured distance from a tag and compare the `Vision tags/dist`
telemetry, which is in inches. ~39x too small means set it to `INCH`; ~39x too
large means `METER`. The subsystem prints `*** TAG DISTANCE LOOKS WRONG ***` when
the value is not a believable distance on an FTC field.

### The fused pose ignores vision completely

`Fused` tracks `Raw Odometry` exactly even with `accepted true`. The Kalman gain
has collapsed to near zero: check `VISION_XY_STD_DEV_COEFFICIENT`. It must be
about **`0.0005`** because this template works in **inches**. AdvantageKit's
published `0.02` assumes metres, and the distance term is squared — copying it
across gives std devs in the hundreds of inches and silently disables fusion.

### Vision never gets accepted

Read the reject reason ([§16](#16-reading-the-telemetry)). Most common: no field
map uploaded, wrong pipeline index, or the tag is too far away.

### The pose is off by a constant few inches

The camera mount offset is wrong, or applied in *both* the Limelight UI and the
code. Pick one ([§3](#3-set-up-the-limelight)).

### The pose jumps around when only one tag is visible

Single-tag pose ambiguity. The heading-trust gate should move you to MegaTag2,
which resolves it — check that `Vision source` reads `MegaTag2`. If it is stuck
on `MegaTag1`, your heading is not being trusted, so fix that first. Raising
`MIN_TAG_COUNT` to `2` is the blunt fallback.

### Everything drives exactly backwards in field-centric mode

The heading is 180° off. Press `Y` with a tag in view, or fix `START_POSE`. If
this happens every match, you are selecting the wrong alliance at init.

### `Vision source` never reaches `TRUSTED`

MegaTag1 keeps disagreeing with the seeded heading. Either the seed is wrong
(most likely), or the camera mount yaw offset is wrong, or the field map does
not match the field you are on.

### Shoots well standing still, misses while moving

The flight times in `SHOT_MAP` are zero (or far too small). With zero flight time
the moving-shot correction computes a zero offset and does nothing. Time them from
slow-motion video — see [§12e](#12-set-up-shooting).

### Shoots well standing still, misses while turning

`SHOOTER_FORWARD_OFFSET_IN` / `SHOOTER_LEFT_OFFSET_IN` are wrong. An off-centre
shooter is swung sideways by rotation and the piece inherits that motion.

### The robot points 180° away from the goal when aiming

`SHOOTER_YAW_OFFSET_DEG`. It is 180 for a shooter firing out the back, 0 for one
firing forward.

### Aiming always trails a moving target

Raise `PHASE_DELAY_SECONDS`, then check `TURN_POWER_PER_RAD_PER_SEC` in
`DriveSubsystem` — it converts the angular feedforward into turn power and should
be 1 / (your robot's turn rate at full power, rad/s).

### It aims but never fires

Read the `Shot` telemetry line; `AimAndShootCommand.getStatus()` names exactly
which gate is open: out of range, still turning, still spinning up, or pose not
trusted. "Pose not trusted" means vision has not vouched for the heading yet —
point the camera at a tag, and see [§6e](#6-verify-localization).

### A command keeps a motor running after I let go of the button

That command is not stopping its mechanism in `end(boolean interrupted)`.
`whileHeld` cancels on release, which calls `end(true)`.

### An OpMode works once after a restart, then misbehaves

This is the failure a static scheduler causes, and it is why the scheduler here is
per-OpMode. If you see it, check you are not holding subsystems or commands in
`static` fields of your own.

### The Path Planner will not load from the robot

It depends on SDK web-server internals. Confirm the app is deployed and you are
on the robot's Wi-Fi. Failing that, open the HTML file directly in a browser —
it has no server dependency at all.

---

## 16. Reading the telemetry

Every OpMode calls `localization.addTelemetry(telemetry)`, which prints:

```
Fused          x  12.4  y  -33.1  h  88.2 deg
Raw Odometry   x  11.9  y  -32.8  h  88.5 deg
Vision         enabled true | accepted true | none
Vision source  MegaTag2 | heading TRUSTED | MT1 gap 2 deg
Raw Vision 2D  x  12.5  y  -33.2  h  88.0 deg  (age 0.03s)
Vision tags/dist  tags 2  dist 41.2
Vision 3D      z 4.6  pitch 0.4  roll -0.2 deg
```

| Line | Read it for |
|---|---|
| `Fused` | the pose everything else uses |
| `Raw Odometry` | dead reckoning alone. Compare against `Fused` to see how hard vision is working |
| `Vision` | whether the last frame was used, and if not, why |
| `Vision source` | which solver is supplying position, and whether heading is trusted |
| `Raw Vision 2D` | what the camera thinks, uncorrected. `age` says how stale |
| `Vision tags/dist` | more tags and less distance mean a more trusted frame |
| `Vision 3D` | `pitch`/`roll` near zero on a flat field. A constant offset means the mount config is wrong |

### Reject reasons

| Reason | Meaning |
|---|---|
| `no valid result` | no tag in frame, or the pipeline is not an AprilTag pipeline |
| `too few tags` | fewer than `MIN_TAG_COUNT` |
| `stale` | older than `MAX_STALENESS_MS` — usually a loop running slowly |
| `no MegaTag1 fix` | botpose missing, or exactly the origin (the "no fix" value) |
| `off field` | solved outside the field plus margin — garbage frame |
| `jump NN in` | too far from a settled estimate; likely an ambiguous single-tag solve |
| `vision disabled` | `VISION_ENABLED = false`, or constructed with vision off |

### Follower telemetry

```
Path    seg 1  t=0.62  remaining=18.4 in
Errors  cross 0.42 in  heading 1.8 deg
```

| Field | Read it for |
|---|---|
| `seg` | which segment of the chain is running |
| `t` | progress along it, 0 → 1 |
| `remaining` | inches of arc left; drives the deceleration feedforward |
| `cross` | distance off the path. Should stay under an inch or two |
| `heading` | heading error. Should trend to zero, never grow |
