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

## Path Planner (visual tool)

`assets/pathplanner.html` is a self-contained visual editor. Drag control points
on the field, choose a heading mode per segment, and copy the generated Java
(`BezierCurve` / `Path` / `PathChain`) straight into an OpMode. The field
convention matches the pose estimator: 144"×144", origin at center, +X right,
+Y up, heading degrees CCW.

To use it:
- **Locally** — open `pathplanner.html` in any browser.
- **From the robot's IP** — the file lives under `TeamCode/src/main/assets/`, so
  it ships inside the app and can be served by the Robot Controller's embedded
  web server (reachable at the robot's IP). Wiring up that web handler is
  optional and not enabled by default.
