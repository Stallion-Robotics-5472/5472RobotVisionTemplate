# Vision + Odometry Localization

This template fuses a **goBILDA Pinpoint** odometry computer with a **Limelight 3A**
AprilTag camera into a single field pose, using the same Kalman-style pose
estimator that FRC teams use (a direct port of WPILib's `PoseEstimator`, the one
the AdvantageKit vision template feeds). It is modeled on how FRC teams such as
the Robonauts structure vision: odometry is the backbone, and each vision frame
is blended in as an absolute measurement weighted by standard deviations.

## How it works

1. **Odometry backbone** — The Pinpoint fuses two dead-wheel encoders with its
   IMU and reports an absolute pose. We push that pose into the estimator every
   loop. Between vision updates, the fused pose tracks odometry exactly.
2. **Kalman blending** — Each accepted vision frame nudges the estimate toward
   the camera's reported pose. The size of the nudge is the Kalman gain
   `q / (q + sqrt(q * r))`, where `q` is the odometry variance and `r` is the
   per-frame vision variance. Trust vision less by raising its std dev.
3. **Dynamic std devs (AdvantageKit style)** — Vision XY std dev scales with
   `avgTagDistance² / tagCount`. Close, multi-tag fixes snap the pose; far-away
   or single-tag fixes barely move it.
4. **MegaTag1 for heading, MegaTag2 for position** — The two solvers fail in
   opposite ways, so each is used for what it is good at.

   **MegaTag1** (`getBotpose()`) is solved from tag geometry alone. Because it
   never consults the gyro, it is the only thing that can *check* the gyro — so
   heading always comes from it. Its weakness is pose ambiguity: a single tag
   viewed near head-on has two mathematically valid solutions that are mirror
   images, and the solver can pick the wrong one.

   **MegaTag2** (`getBotpose_MT2()`) uses the yaw we push down via
   `updateRobotOrientation()` to constrain the solve, eliminating that ambiguous
   solution. It is much steadier, so position comes from it — but only once the
   heading is trusted, because MegaTag2 is computed *from* our heading and a
   wrong heading makes it confidently wrong.

   **The heading-trust gate** decides. MegaTag1's independent heading is compared
   with the current estimate each frame; `HEADING_TRUST_FRAMES` consecutive
   agreements earn trust and hand position to MegaTag2, while a gap beyond
   `HEADING_DISTRUST_THRESHOLD` revokes it and falls back to MegaTag1.

   Heading is *fused* rather than snapped: the gyro/odometry is the backbone and
   the large `VISION_HEADING_STD_DEV` keeps the blend heavily weighted toward it
   (~4% vision per frame), so vision only gently corrects drift.
5. **Latency compensation** — Each frame is timestamped at *capture* time
   (`now − captureLatency − targetingLatency − staleness`). The staleness term
   covers how long the finished result sat on the Robot Controller before we
   read it; without it the frame is timestamped later than it really was. The
   estimator looks up where
   odometry was at that instant, applies the correction there, then replays
   newer odometry on top.
6. **Rejection filters** — Frames are dropped if invalid, too few tags, stale,
   reporting the field origin (no fix), off-field, or landing more than
   `MAX_POSE_JUMP_IN` from a settled estimate. That last one is the final guard
   against an ambiguous solve teleporting the robot; it only applies once the
   estimate has settled (at startup a bad seed is the wrong one, not vision),
   and it latches off after `JUMP_REJECT_LIMIT` consecutive rejections so a
   genuinely wrong estimate can still be corrected.

## Catching a flipped start pose

The most damaging failure here is not drift — it is being seeded backwards
(wrong alliance button, or the robot placed facing the other way). It is quiet:
odometry stays self-consistent and the robot simply drives the wrong way.

Note this is *not* caused by alliance flipping. AprilTags define one absolute
field frame, so `AllianceFlip` transforms plans, never measurements; vision
reads identically for both alliances. What goes wrong is the **seed**.

Heading fusion alone will not rescue you in time — at a ~4% gain, dragging 180°
back takes seconds, and auto has already run. So the check happens at init:

| Call | Use |
|------|-----|
| `getStartPoseCheck()` | human-readable verdict for telemetry |
| `isStartPoseSuspect()` | true when vision and the seed disagree past `HEADING_SEED_WARN_THRESHOLD` |
| `isHeadingTrusted()` | has MegaTag1 vouched for the heading yet? |
| `getHeadingDisagreementDegrees()` | signed gap; near ±180 means seeded backwards |
| `seedFromVision()` | snap the whole pose to the latest MegaTag1 fix |

`AllianceAutoExample` and `FieldCentricDrive` run the estimator during init and
display the verdict, so a seeding mistake surfaces while the robot sits still
with a clear view of a tag — the best look MegaTag1 will ever get. In
`FieldCentricDrive`, **Y** calls `seedFromVision()` as a last resort; it ignores
fixes older than `SEED_FROM_VISION_MAX_AGE_S` so a stray press cannot corrupt a
good pose.

## File map

| Path | Purpose |
|------|---------|
| `lib/geometry/*` | `Rotation2d`, `Translation2d`, `Transform2d`, `Twist2d`, `Pose2d` (SE(2) exp/log) — ported from WPILib. |
| `lib/util/TimeInterpolatableBuffer` | Rolling odometry history for latency compensation. |
| `lib/estimator/PoseEstimator` | The Kalman fusion with state + vision std devs. |
| `subsystems/VisionConstants` | **All tuning lives here** — names, offsets, std devs, filters. |
| `subsystems/PinpointOdometry` | Configures + reads the Pinpoint. |
| `subsystems/LimelightVision` | Configures + reads the Limelight 3A. |
| `subsystems/Localization` | The combined "vision subsystem". Call `update()`, read `getPose()`. |
| `opmodes/LocalizationTest` | Example TeleOp streaming fused vs. odometry pose. |

## Setup checklist

1. **Robot configuration** — Add the Pinpoint as an I2C device named `pinpoint`
   and the Limelight as a USB device named `limelight` (see the
   `SensorGoBildaPinpoint` and `SensorLimelight3A` samples for wiring/config
   details). Names are in `VisionConstants`.
2. **Limelight pipeline** — In the Limelight web UI, set up an AprilTag pipeline,
   upload the current season's field map, and note its index
   (`LIMELIGHT_PIPELINE`, default 0).
3. **Pinpoint geometry** — Measure your odometry pod offsets and set
   `PINPOINT_X_OFFSET` / `PINPOINT_Y_OFFSET`, the pod type, and pod
   directions in `VisionConstants`. The offsets are read in
   `PINPOINT_OFFSET_UNIT` (inches by default) — the numbers and that unit
   must agree.
4. **Coordinate frame** — Vision (`getBotpose`, MegaTag1) returns field coordinates
   from the uploaded map; the convention places the origin at field center
   (±72"). Make sure your `setStartingPose` and any field bounds use the same
   frame.

## Disabling vision (odometry-only)

To run without the Limelight — no camera plugged in, or to compare fusion
against dead reckoning — disable it any of these ways:

- **Globally:** set `VisionConstants.VISION_ENABLED = false`. The Limelight is
  then never initialized (so a missing/unplugged camera won't crash the OpMode)
  and `Localization` runs on the Pinpoint alone.
- **Per OpMode:** construct with `new Localization(hardwareMap, false)`.
- **At runtime:** call `localization.setVisionEnabled(false)` (only effective if
  the Limelight was initialized; check `isVisionEnabled()`).

In odometry-only mode everything else (path following, etc.) works unchanged —
the fused pose is simply the Pinpoint pose.

## Camera offset (Limelight not at robot center)

The Limelight is rarely mounted at the robot's center, and that lever arm
matters: when the robot rotates, an off-center camera sees the field from a
shifted position. Pick **one** of these (never both):

- **(A) Recommended** — Enter the camera→robot offset in the Limelight web UI.
  The botpose this code reads is then already the robot-center pose. Keep
  `APPLY_CAMERA_OFFSET_IN_CODE = false`. This is best because MegaTag2 also uses
  the offset internally when solving.
- **(B) In code (full 3D)** — Leave the Limelight UI offset at zero and set the
  complete mount in `VisionConstants`: `CAMERA_FORWARD_OFFSET_IN`,
  `CAMERA_LEFT_OFFSET_IN`, `CAMERA_UP_OFFSET_IN`, `CAMERA_ROLL_OFFSET_DEG`,
  `CAMERA_PITCH_OFFSET_DEG`, `CAMERA_YAW_OFFSET_DEG`, with
  `APPLY_CAMERA_OFFSET_IN_CODE = true`. **Mind the pitch sign:** pitch rotates
  about +Y (left), so a positive pitch tilts the camera *down*. A camera angled
  15° upward to see tags is `-15.0`. `ROBOT_TO_CAMERA` is then a full 3D
  (SE(3)) transform; the subsystem treats botpose as the camera's 3D field pose
  and recovers the robot-center pose via
  `cameraPose3d.transformBy(ROBOT_TO_CAMERA.inverse())`, then projects to 2D for
  fusion. This correctly handles a raised, tilted (pitch), or rolled camera.
  Note (A) is still the most accurate, because MegaTag2 also uses the mount
  inside its own tag solve; (B) assumes botpose carries the true 3D camera pose
  (cleanest with a level robot / MegaTag1).

## 3D pose (z, pitch, roll)

The Limelight AprilTag botpose is fully 3D, but the fused estimate used for
driving is 2D — the robot lives on the floor, and the Pinpoint only measures
x/y/heading, so there is nothing meaningful to fuse z/pitch/roll against. The
full 3D vision pose is still exposed for diagnostics via `Localization`:

- `getVisionPose3d()` — the latest valid `Pose3d` (x, y, z, roll, pitch, yaw).
- `getVisionZ()`, `getVisionPitch()`, `getVisionRoll()` — individual axes.
- `getVisionPose3dAge(nowSeconds)` — how stale that 3D pose is.

Typical uses: detect tipping (a pitch/roll spike), confirm the robot is on a
ramp, or sanity-check the camera mount (a constant nonzero pitch/roll on a flat
field usually means the camera-offset config is wrong). If you need a fused
pitch/roll for control, add the Control Hub IMU as the source — the Pinpoint
does not expose it.

## Units, and the one place they are not automatic

The Limelight works in **metres** natively, in FTC exactly as in FRC — it is the
same firmware and the same JSON. This template works in **inches**. Most of that
seam is handled for you, but not all of it:

| Value | Safe? | Why |
|---|---|---|
| `getBotpose()` position | **yes** | arrives as a `Position` carrying its own unit; the code calls `.toUnit(DistanceUnit.INCH)` |
| `getBotpose()` orientation | **yes** | read with an explicit `AngleUnit.RADIANS` |
| `getStaleness()`, `getCaptureLatency()`, `getTargetingLatency()` | yes | milliseconds, no ambiguity |
| `getBotposeTagCount()` | yes | a count |
| **`getBotposeAvgDist()`** | **needs a constant** | a bare `double` with no unit attached |

That last one is set by `VisionConstants.BOTPOSE_AVG_DIST_UNIT`, defaulting to
`METER` because that is what the Limelight documents reporting and the SDK most
likely passes straight through.

**Why it matters more than it looks.** The vision std dev goes as distance
*squared*, so reading metres as inches shrinks the distance 39x and the std dev
about 1550x. That pins the Kalman gain near 1.0 at every range:

| true distance | gain, correct | gain, unit wrong |
|---|---|---|
| 20″ | 0.72 | 1.00 |
| 39″ | 0.39 | 1.00 |
| 98″ | 0.09 | 0.99 |

Distance weighting disappears entirely, close and far frames are trusted the
same, and a single noisy tag yanks the pose across the field — the exact mirror
of the coefficient-units bug described under Tuning.

**You do not have to take the default on faith.** The converted distance is on
telemetry in inches (`Vision tags/dist`). Stand a measured distance from a tag and
compare. Roughly 39x too small means it should be `INCH`; 39x too large means
`METER`. The subsystem also range-checks it against the size of an FTC field, says
`*** TAG DISTANCE LOOKS WRONG ***` on telemetry when it is out of range, and clamps
the value used for weighting so a wrong unit degrades the trust curve instead of
destroying it.

## Tuning

All knobs are in `VisionConstants`:

- `ODOMETRY_STD_DEVS` — lower = trust odometry more (slower vision correction).
- `VISION_XY_STD_DEV_COEFFICIENT` — lower = trust vision more. **Mind the
  units:** AdvantageKit publishes `0.02` for *metres*; in inches, with the
  distance term squared, the equivalent is `0.02/39.37 ≈ 0.0005`. Too large a
  value produces std devs of hundreds of inches, collapses the Kalman gain to
  ~0, and silently disables vision fusion while the pose still looks plausible.
- `PREFER_MEGATAG2`, `HEADING_TRUST_FRAMES`, `HEADING_TRUST_TOLERANCE`,
  `HEADING_DISTRUST_THRESHOLD` — the heading-trust gate.
- `MAX_POSE_JUMP_IN`, `JUMP_REJECT_LIMIT` — outlier rejection.
- `VISION_HEADING_STD_DEV` — keep large to let the gyro own heading.
- `MIN_TAG_COUNT`, `MAX_STALENESS_MS`, `FIELD_*` — rejection thresholds.

The fusion math is covered by the offline suite in `tools/verify/` — run
`./tools/verify/run.sh` (JDK only, no robot needed). It checks that a vision
frame moves the estimate by exactly the Kalman gain, that the heading gain stays
gyro-dominant (~0.04), that a delayed but agreeing frame leaves the estimate
alone, and that frames older than the history buffer are dropped.
