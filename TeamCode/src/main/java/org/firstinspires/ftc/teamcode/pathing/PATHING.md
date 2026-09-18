# Path Following

A Bézier-curve path follower that drives off the fused Kalman pose estimate
(`Localization`). It's a clean-room implementation of the same approach Pedro
Pathing uses — a vector follower with translational, centripetal, drive, and
heading contributions — written specifically to consume this template's
`PoseEstimator`.

## How it works

Every cycle the follower:
1. Pulls the fused pose from the `Localizer` (implemented by `Localization`).
2. Projects the robot onto the current `Path` (closest point, searched
   forward-only so it can't slip backward).
3. Builds a field-frame command from:
   - **translational** — PIDF pull toward the closest point (cross-track error),
   - **centripetal** — `scale · speed² · curvature` nudge toward the inside of a
     curve so the robot doesn't drift wide,
   - **drive** — PIDF push along the tangent, sized by remaining path length, so
     the robot decelerates into the endpoint.
   The corrective (translational + centripetal) vector gets priority in the unit
   power budget; the drive vector fills what's left.
4. Controls heading independently with a PIDF toward the path's target heading.
5. Hands the field-centric command to the `Drivetrain`, which rotates it into
   the robot frame and mixes mecanum powers.

## Usage

```java
Localization localization = new Localization(hardwareMap);
MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
Follower follower = new Follower(localization, drivetrain);

localization.setStartingPose(new Pose2d(0, 0, new Rotation2d(0)));

Path s = new Path(new BezierCurve(
        new Translation2d(0, 0), new Translation2d(20, 0),
        new Translation2d(10, 30), new Translation2d(30, 30)))
        .setLinearHeading(0, Math.toRadians(90));

follower.followPath(new PathChain(s));
while (opModeIsActive() && follower.isBusy()) {
    follower.update();
}
```

See `opmodes/FollowPathExample` for a complete OpMode.

## Heading modes

- `setTangentHeading()` — face along the path.
- `setReverseTangentHeading()` — face backward along the path (drive in reverse).
- `setLinearHeading(start, end)` — interpolate by t, shortest angular route.
- `setConstantHeading(h)` — hold a fixed heading.

## Tuning

All gains and tolerances are in `PathConstants`: the translational, drive, and
heading PIDF gains, the centripetal scale, and the completion/advancement
tolerances. The defaults are validated in a kinematic simulation but must be
tuned on the real robot. Increase `TRANSLATIONAL_kP` if the robot tracks loosely;
raise `CENTRIPETAL_SCALE` if it cuts curves wide; tune `HEADING_kP` for crisp
turns without oscillation.

### Deceleration feedforward

The drive command is the maximum of a PID output and a physics-based feedforward:

```
targetSpeed = √(2 · ZERO_POWER_DECEL_RATE · remaining)
driveMag    = max(targetSpeed / MAX_ROBOT_SPEED, drivePIDOut)
```

This naturally decelerates the robot to zero at the path end without requiring an
aggressive `DRIVE_kD`. To measure `ZERO_POWER_DECEL_RATE`: drive the robot at
full speed, cut motor power, measure stopping distance `d` and entry speed `v`,
then `decelRate = v² / (2·d)`. Increase it if the robot overshoots endpoints;
decrease it if braking starts too far out. `MAX_ROBOT_SPEED` is the top speed at
full drive PID output (inches/sec); it normalizes the feedforward to `[0, 1]`.

Both constants can also be edited in the **Gains** tab of the Path Planner and
pasted back into `PathConstants.java` via the generated snippet.

## Path Planner (visual tool)

`assets/pathplanner.html` is a self-contained visual editor. Drag control points
on the field, choose a heading mode per segment, and copy the generated Java
(`BezierCurve` / `Path` / `PathChain`) straight into an OpMode. The field
convention matches the pose estimator: 144"×144", origin at center, +X right,
+Y up, heading degrees CCW.

To use it:
- **From the robot's IP** — `PathPlannerServer` registers a route on the Robot
  Controller's embedded web server, so after deploying the app you can connect
  to the robot's Wi-Fi and browse to:

  ```
  http://192.168.43.1:8080/pathplanner
  ```

  (Use the robot's actual IP; `192.168.43.1` is the typical Control Hub address.)
  This relies on SDK web-server internals, so confirm it once on hardware after
  the first deploy.
- **Locally** — you can also just open `assets/pathplanner.html` in any browser;
  it has no server dependency.

### Swap to the other alliance

**⟳ Swap Alliance** rotates the whole plan 180° about the field center — start
pose, every control point, and the stored heading angles. Click it again to go
back; it's an exact round trip. The generated Java includes a comment showing
the `AllianceFlip` call to do the same thing at run time.

Two true mirrors (**Mirror ⇆ L/R**, **Mirror ⇅ T/B**) live in the Robot tab for
fields that really are mirror-symmetric. They reverse handedness, so they are
*not* an alliance swap on BIOBUZZ — see below.

## Coordinate system and alliances

**There is one field frame and it never changes.** Origin at the field center,
+X right, +Y away from the audience, heading CCW, inches. This is the same frame
the AprilTag map uses, so `Localization` returns absolute poses: a robot parked
on a given tile reports the same pose whether it's red or blue, and whichever
side it started on. Nothing in the follower, the paths, or the gains is
alliance-dependent.

Two things *do* depend on the alliance, and they're kept separate from the frame:

### 1. The driver's point of view (TeleOp)

The two drive teams stand at opposite ends of the field, so "push the stick away
from me" is +X for red and −X for blue. `Alliance.driverForward()` supplies that
offset, and `FieldCentricDrive` rotates the stick vector by it before handing a
field-frame command to the drivetrain:

```
driver frame --(alliance.driverForward())--> field frame --(−heading)--> robot
```

The pose estimate is untouched. If your drive team stands somewhere else,
`driverForward()` is the single value to change.

`FieldCentricDrive` also has a re-zero button (**back**) that redefines the
driver's forward as the robot's current facing, to recover from a bad heading
without restarting.

### 2. Which side an auto runs on

Author the auto once, then flip it at run time with `AllianceFlip`:

```java
private static final Alliance AUTHORED_FOR = Alliance.RED;
private static final Pose2d START = new Pose2d(-58, -58, Rotation2d.fromDegrees(45));

// after the driver picks `alliance` during init:
Pose2d    start = AllianceFlip.forAlliance(START,       AUTHORED_FOR, alliance);
PathChain toRun = AllianceFlip.forAlliance(buildPlan(), AUTHORED_FOR, alliance);
localization.setStartingPose(start);
follower.followPath(toRun);
```

`forAlliance` returns the original object untouched when the alliances match, so
there's no cost on your home side. See `opmodes/AllianceAutoExample`.

TANGENT paths need no heading data adjustment — the tangent is recomputed from
the transformed control points. CONSTANT and LINEAR headings are transformed
explicitly. Curvature (and so the centripetal correction) is also derived from
the transformed points, so it stays correct even under a handedness-reversing
mirror.

### Why 180° rotation and not a mirror

`FieldSymmetry.ROTATIONAL` is the default because BIOBUZZ's alliance halves are
rotations of each other, not reflections. Checked against the Competition Manual
field coordinates — rotating red's elements 180° lands exactly on blue's, while
neither mirror does:

| red element  | red coords                    | rot 180° →            | blue actual           |
|--------------|-------------------------------|-----------------------|-----------------------|
| GARDEN       | x −70.5..−47.4, y −70.0..−68.0 | x 47.4..70.5, y 68..70 | x 47.4..70.5, y 68..70 |
| LOADING ZONE | x −70.5..−59.5, y 23.9..46.6  | x 59.5..70.5, y −46.6..−23.9 | x 59.5..70.5, y −46.6..−23.9 |

A left/right mirror would put red's garden at the *bottom*-right instead of the
top-right. Pass an explicit `FieldSymmetry` to any `AllianceFlip` method if a
future field is genuinely mirror-symmetric.

### Setting the starting pose

Seed `setStartingPose` with the robot's real absolute field pose, not `(0,0,0)`
— that's the middle of the field. Odometry is dead-reckoned from whatever you
seed, so a wrong seed means a wrong pose until vision sees a tag and pulls the
estimate in. The template's example OpModes use `(0,0,0)` as a placeholder;
replace it with your actual start.

### Field map

The planner ships with the **2026-2027 BIOBUZZ** field as its backdrop
(`assets/field-biobuzz-2027.png`), so it matches the current season out of the
box. `PathPlannerServer` registers a route for the image alongside the page, so
it also renders when served from the robot.

To plan against a different image (a new season, or the official FIRST field
PNG when you have it), drop the image onto the canvas or use the **Field image**
file picker. It's stretched to the full 144"×144" field, so use a top-down image
whose edges are the field perimeter. The **opacity** slider fades it, and
**Reset** returns to the bundled BIOBUZZ field. A dropped image is saved in the
browser (localStorage), so it persists across reloads and when the page is
served from the robot — no extra asset or server change needed.

If the image is ever missing, the planner falls back to drawing a procedural
field (24" foam tiles, perimeter wall, alliance walls with red at −X and blue at
+X, audience at −Y), so the tool never comes up blank.
