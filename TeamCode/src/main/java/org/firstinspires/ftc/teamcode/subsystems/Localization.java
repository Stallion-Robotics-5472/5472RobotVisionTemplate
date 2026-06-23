/*
 * Localization subsystem: fuses goBILDA Pinpoint odometry with Limelight 3A
 * AprilTag vision into a single field pose, using the FRC-style Kalman pose
 * estimator in lib/estimator.
 *
 * Design follows the AdvantageKit vision template:
 *   - Odometry runs continuously and is the backbone of the estimate.
 *   - Each accepted vision frame is added as an absolute measurement, weighted
 *     by standard deviations that grow with tag distance and shrink with tag
 *     count, so far-away or single-tag fixes barely move the pose while close,
 *     multi-tag fixes snap it.
 *   - We use MegaTag2 (getBotpose_MT2), feeding the robot heading down to the
 *     camera each loop, and we deliberately do NOT let vision correct heading
 *     (the gyro is better) by using a huge heading std dev.
 *   - Bad frames (no fix, too few tags, stale, off-field) are rejected.
 *   - Vision measurements are timestamped at capture time (now minus pipeline
 *     latency) so the estimator can latency-compensate them.
 *
 * Usage each loop: call {@link #update()}, then read {@link #getPose()}.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.lib.estimator.PoseEstimator;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.pathing.Localizer;

public class Localization implements Localizer {
    private final PinpointOdometry odometry;
    /** Null when vision is disabled at construction (odometry-only mode). */
    private final LimelightVision vision;
    private final PoseEstimator poseEstimator;

    // Runtime gate for vision fusion. Starts from VisionConstants.VISION_ENABLED
    // but can be toggled live (only effective if the Limelight was initialized).
    private boolean visionEnabled;

    // Telemetry / debugging state from the most recent update.
    private Pose2d lastOdometryPose = new Pose2d();
    private boolean lastVisionAccepted = false;
    private String lastVisionReject = "none";
    private int lastTagCount = 0;
    private double lastAvgTagDist = 0.0;

    // Full 3D pose from the most recent valid vision frame. Only x/y/yaw are
    // fused into the 2D estimate (the robot drives on the floor); z/pitch/roll
    // are kept for diagnostics (tipping, ramps, mount sanity checks).
    private Pose3d lastVisionPose3d = new Pose3d();
    private double lastVisionPose3dTimestamp = Double.NEGATIVE_INFINITY;

    public Localization(HardwareMap hardwareMap) {
        this(hardwareMap, VisionConstants.VISION_ENABLED);
    }

    /**
     * @param enableVision if false, the Limelight is never initialized and the
     *     estimate runs on the Pinpoint alone. Use this for odometry-only runs
     *     or when no Limelight is plugged in (avoids a missing-hardware crash).
     */
    public Localization(HardwareMap hardwareMap, boolean enableVision) {
        odometry = new PinpointOdometry(hardwareMap);
        vision = enableVision ? new LimelightVision(hardwareMap) : null;
        visionEnabled = enableVision;
        poseEstimator = new PoseEstimator(
                VisionConstants.ODOMETRY_STD_DEVS,
                VisionConstants.DEFAULT_VISION_STD_DEVS);
    }

    /**
     * Seeds the estimate with a known field pose (e.g. the autonomous start).
     * Resets both the Pinpoint's internal pose and the estimator history.
     */
    public void setStartingPose(Pose2d pose) {
        odometry.update();
        odometry.setPose(pose);
        poseEstimator.resetPose(pose);
        lastOdometryPose = pose;
    }

    /** Runs one fusion cycle. Call once per loop. */
    public void update() {
        double now = currentTimeSeconds();

        // 1) Odometry: read the Pinpoint and feed it as the backbone estimate.
        odometry.update();
        lastOdometryPose = odometry.getPose();
        poseEstimator.updateWithTime(now, lastOdometryPose);

        // Vision disabled (or no Limelight): odometry-only, skip the rest.
        if (!visionEnabled || vision == null) {
            lastVisionAccepted = false;
            lastVisionReject = "vision disabled";
            return;
        }

        // 2) Hand the camera the GYRO heading so MegaTag2 can localize. Heading
        // always comes from the Pinpoint IMU, never from the camera.
        double headingDegrees = lastOdometryPose.getRotation().getDegrees();
        vision.updateRobotOrientation(headingDegrees);

        // 3) Vision: validate and, if good, fuse it.
        processVision(vision.getLatestResult(), now);
    }

    /**
     * Enables or disables vision fusion at runtime. Has no effect if the
     * Limelight was not initialized (constructed with vision disabled).
     */
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled && vision != null;
    }

    public boolean isVisionEnabled() {
        return visionEnabled && vision != null;
    }

    private void processVision(LLResult result, double now) {
        lastVisionAccepted = false;

        if (result == null || !result.isValid()) {
            lastVisionReject = "no valid result";
            return;
        }

        lastTagCount = result.getBotposeTagCount();
        if (lastTagCount < VisionConstants.MIN_TAG_COUNT) {
            lastVisionReject = "too few tags";
            return;
        }

        if (result.getStaleness() > VisionConstants.MAX_STALENESS_MS) {
            lastVisionReject = "stale";
            return;
        }

        // MegaTag2 pose: relies on the heading we pushed in updateRobotOrientation.
        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            lastVisionReject = "null botpose";
            return;
        }

        Position position = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double xIn = position.x;
        double yIn = position.y;

        // MegaTag2 reports the origin when it has no real fix. Check the raw
        // botpose here, before any camera-offset transform shifts it away from 0.
        if (xIn == 0.0 && yIn == 0.0) {
            lastVisionReject = "origin (no fix)";
            return;
        }

        double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        // The Limelight botpose is a full 3D pose. Build it as reported (this is
        // the camera's field pose when the offset is configured in code, or the
        // robot's field pose when configured in the Limelight UI).
        Pose3d reportedPose3d = new Pose3d(
                xIn, yIn, position.z,
                botpose.getOrientation().getRoll(AngleUnit.RADIANS),
                botpose.getOrientation().getPitch(AngleUnit.RADIANS),
                headingRad);

        // If the camera offset is handled in code, convert the camera's 3D field
        // pose to the robot-center 3D pose with the inverse SE(3) transform
        // (forward/left/up + roll/pitch/yaw). Otherwise botpose is already the
        // robot pose. See VisionConstants for the two configuration options.
        Pose3d robotPose3d = VisionConstants.APPLY_CAMERA_OFFSET_IN_CODE
                ? reportedPose3d.transformBy(VisionConstants.ROBOT_TO_CAMERA.inverse())
                : reportedPose3d;

        // Keep the robot-center 3D pose for diagnostics (z, pitch, roll). Note:
        // under MegaTag2 the botpose yaw is just the gyro heading we fed in.
        lastVisionPose3d = robotPose3d;
        lastVisionPose3dTimestamp = now;

        // Heading: fuse an INDEPENDENT vision heading from MegaTag1 (getBotpose),
        // which is derived from tag geometry rather than the gyro we feed MegaTag2.
        // This lets vision slowly correct gyro drift. The heavy heading std dev
        // (VISION_HEADING_STD_DEV) keeps the fusion strongly biased toward the
        // gyro/odometry. Falls back to the gyro heading if MegaTag1 has no fix.
        double visionHeadingRad = lastOdometryPose.getRotation().getRadians();
        Pose3D botposeMT1 = result.getBotpose();
        if (botposeMT1 != null) {
            Position mt1 = botposeMT1.getPosition().toUnit(DistanceUnit.INCH);
            if (!(mt1.x == 0.0 && mt1.y == 0.0)) {
                Pose3d mt1Pose3d = new Pose3d(
                        mt1.x, mt1.y, mt1.z,
                        botposeMT1.getOrientation().getRoll(AngleUnit.RADIANS),
                        botposeMT1.getOrientation().getPitch(AngleUnit.RADIANS),
                        botposeMT1.getOrientation().getYaw(AngleUnit.RADIANS));
                if (VisionConstants.APPLY_CAMERA_OFFSET_IN_CODE) {
                    mt1Pose3d = mt1Pose3d.transformBy(VisionConstants.ROBOT_TO_CAMERA.inverse());
                }
                visionHeadingRad = mt1Pose3d.getYaw();
            }
        }

        // x/y from MegaTag2 (gyro-assisted), heading from MegaTag1 (independent).
        Pose2d visionPose = new Pose2d(
                robotPose3d.getX(), robotPose3d.getY(), new Rotation2d(visionHeadingRad));

        // Off-field results are garbage (checked on the robot-center pose).
        double limit = VisionConstants.FIELD_HALF_SIZE_IN + VisionConstants.FIELD_MARGIN_IN;
        if (Math.abs(visionPose.getX()) > limit || Math.abs(visionPose.getY()) > limit) {
            lastVisionReject = "off field";
            return;
        }

        // Dynamic std devs (AdvantageKit style): trust scales with distance^2 / tagCount.
        lastAvgTagDist = result.getBotposeAvgDist();
        double stdDevFactor = (lastAvgTagDist * lastAvgTagDist) / lastTagCount;
        double xyStdDev = VisionConstants.VISION_XY_STD_DEV_COEFFICIENT * stdDevFactor;
        double[] visionStdDevs = {xyStdDev, xyStdDev, VisionConstants.VISION_HEADING_STD_DEV};

        // Latency compensation: timestamp the frame at capture time.
        double latencySeconds =
                (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
        double captureTimestamp = now - latencySeconds;

        poseEstimator.addVisionMeasurement(visionPose, captureTimestamp, visionStdDevs);
        lastVisionAccepted = true;
        lastVisionReject = "none";
    }

    /**
     * The fully fused field pose (inches, radians). X/Y and heading all come from
     * the Kalman estimate. Heading is dominated by the Pinpoint gyro/odometry
     * (it tracks gyro deltas every loop) and only slowly corrected by MegaTag1
     * vision, per VISION_HEADING_STD_DEV.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Raw Pinpoint-only pose, for comparison/telemetry. */
    public Pose2d getOdometryPose() {
        return lastOdometryPose;
    }

    public boolean wasLastVisionAccepted() {
        return lastVisionAccepted;
    }

    public String getLastVisionReject() {
        return lastVisionReject;
    }

    public int getLastTagCount() {
        return lastTagCount;
    }

    public double getLastAvgTagDistance() {
        return lastAvgTagDist;
    }

    /**
     * Full 3D pose from the most recent valid vision frame (inches, radians).
     * Only x/y/yaw drive the robot; z/pitch/roll are diagnostic. Check
     * {@link #getVisionPose3dAge(double)} for freshness before trusting it.
     */
    public Pose3d getVisionPose3d() {
        return lastVisionPose3d;
    }

    /**
     * Robot-center 2D pose (x, y, yaw) from the most recent valid vision frame.
     * Check {@link #getVisionPose3dAge(double)} for freshness before trusting it.
     */
    public Pose2d getVisionPose2d() {
        return lastVisionPose3d.toPose2d();
    }

    public double getVisionZ() {
        return lastVisionPose3d.getZ();
    }

    /** Pitch in radians (nose up/down). ~0 on a flat field. */
    public double getVisionPitch() {
        return lastVisionPose3d.getPitch();
    }

    /** Roll in radians (lean left/right). ~0 on a flat field. */
    public double getVisionRoll() {
        return lastVisionPose3d.getRoll();
    }

    /** Seconds since the 3D vision pose was last updated (large if never/stale). */
    public double getVisionPose3dAge(double nowSeconds) {
        return nowSeconds - lastVisionPose3dTimestamp;
    }

    public PinpointOdometry getOdometry() {
        return odometry;
    }

    /** The Limelight wrapper, or null if vision was disabled at construction. */
    public LimelightVision getVision() {
        return vision;
    }

    /** Stops the Limelight polling thread (if any). Call when the OpMode ends. */
    public void stop() {
        if (vision != null) {
            vision.stop();
        }
    }

    /** Monotonic clock shared by odometry samples and vision timestamps. */
    private static double currentTimeSeconds() {
        return System.nanoTime() / 1.0e9;
    }
}
