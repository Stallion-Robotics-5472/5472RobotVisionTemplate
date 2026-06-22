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
4. **MegaTag2** — We feed the robot heading to the Limelight each loop
   (`updateRobotOrientation`) and read `getBotpose_MT2()`. Heading is taken from
   the gyro, not vision, so vision is given a huge heading std dev (gain ~0).
5. **Latency compensation** — Each frame is timestamped at *capture* time
   (`now − captureLatency − targetingLatency`). The estimator looks up where
   odometry was at that instant, applies the correction there, then replays
   newer odometry on top.
6. **Rejection filters** — Frames are dropped if invalid, too few tags, stale,
   reporting the field origin (no fix), or off-field.

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
   `PINPOINT_X_OFFSET_MM` / `PINPOINT_Y_OFFSET_MM`, the pod type, and pod
   directions in `VisionConstants`.
4. **Coordinate frame** — Vision (`getBotpose_MT2`) returns field coordinates
   from the uploaded map; the convention places the origin at field center
   (±72"). Make sure your `setStartingPose` and any field bounds use the same
   frame.

## Camera offset (Limelight not at robot center)

The Limelight is rarely mounted at the robot's center, and that lever arm
matters: when the robot rotates, an off-center camera sees the field from a
shifted position. Pick **one** of these (never both):

- **(A) Recommended** — Enter the camera→robot offset in the Limelight web UI.
  `getBotpose_MT2()` is then already the robot-center pose. Keep
  `APPLY_CAMERA_OFFSET_IN_CODE = false`. This is best because MegaTag2 also uses
  the offset internally when solving.
- **(B) In code** — Leave the Limelight UI offset at zero and set
  `CAMERA_FORWARD_OFFSET_IN`, `CAMERA_LEFT_OFFSET_IN`, `CAMERA_YAW_OFFSET_DEG`
  in `VisionConstants`, with `APPLY_CAMERA_OFFSET_IN_CODE = true`. The subsystem
  then treats botpose as the camera's field pose and converts it to the
  robot-center pose via `cameraPose.transformBy(ROBOT_TO_CAMERA.inverse())`.

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

## Tuning

All knobs are in `VisionConstants`:

- `ODOMETRY_STD_DEVS` — lower = trust odometry more (slower vision correction).
- `VISION_XY_STD_DEV_COEFFICIENT` — lower = trust vision more.
- `VISION_HEADING_STD_DEV` — keep large to let the gyro own heading (MegaTag2).
- `MIN_TAG_COUNT`, `MAX_STALENESS_MS`, `FIELD_*` — rejection thresholds.

The fusion math is verified: a vision frame moves the estimate by exactly the
Kalman gain toward the measurement, and a large heading std dev leaves heading
untouched.
