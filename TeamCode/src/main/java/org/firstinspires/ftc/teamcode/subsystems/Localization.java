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
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

public class Localization {
    private final PinpointOdometry odometry;
    private final LimelightVision vision;
    private final PoseEstimator poseEstimator;

    // Telemetry / debugging state from the most recent update.
    private Pose2d lastOdometryPose = new Pose2d();
    private boolean lastVisionAccepted = false;
    private String lastVisionReject = "none";
    private int lastTagCount = 0;
    private double lastAvgTagDist = 0.0;

    public Localization(HardwareMap hardwareMap) {
        odometry = new PinpointOdometry(hardwareMap);
        vision = new LimelightVision(hardwareMap);
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
    }

    /** Runs one fusion cycle. Call once per loop. */
    public void update() {
        double now = currentTimeSeconds();

        // 1) Odometry: read the Pinpoint and feed it as the backbone estimate.
        odometry.update();
        lastOdometryPose = odometry.getPose();
        poseEstimator.updateWithTime(now, lastOdometryPose);

        // 2) Hand the camera our best heading so MegaTag2 can localize.
        double headingDegrees = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        vision.updateRobotOrientation(headingDegrees);

        // 3) Vision: validate and, if good, fuse it.
        processVision(vision.getLatestResult(), now);
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
        Pose3D botpose = result.getBotpose_MT2();
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
        Pose2d reportedPose = new Pose2d(xIn, yIn, new Rotation2d(headingRad));

        // The Limelight is usually not at the robot's center. If the offset is
        // configured in code (rather than the Limelight UI), botpose is the
        // camera's field pose; convert it to the robot-center pose. See
        // VisionConstants for the two configuration options.
        Pose2d visionPose = VisionConstants.APPLY_CAMERA_OFFSET_IN_CODE
                ? reportedPose.transformBy(VisionConstants.ROBOT_TO_CAMERA.inverse())
                : reportedPose;

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

    /** The fused field pose (inches, radians). This is the localization output. */
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

    public PinpointOdometry getOdometry() {
        return odometry;
    }

    public LimelightVision getVision() {
        return vision;
    }

    /** Stops the Limelight polling thread. Call when the OpMode ends. */
    public void stop() {
        vision.stop();
    }

    /** Monotonic clock shared by odometry samples and vision timestamps. */
    private static double currentTimeSeconds() {
        return System.nanoTime() / 1.0e9;
    }
}
