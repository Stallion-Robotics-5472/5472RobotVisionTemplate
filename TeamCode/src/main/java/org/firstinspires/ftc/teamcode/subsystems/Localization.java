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
 *   - Heading always comes from MegaTag1 (getBotpose), which is solved without
 *     the gyro and is therefore the only thing that can check the gyro. It is
 *     fused in but heavily biased toward the gyro (large VISION_HEADING_STD_DEV),
 *     so the gyro dominates short-term and vision only slowly corrects drift.
 *   - Position comes from MegaTag2 (getBotpose_MT2) once the heading has been
 *     vouched for by MegaTag1. MegaTag2 uses our yaw to discard the mirror-image
 *     solution that makes a single-tag fix ambiguous, so it is much steadier --
 *     but it is computed FROM our heading, so a wrong heading makes it
 *     confidently wrong. Until heading is trusted, position stays on MegaTag1.
 *   - Bad frames (no fix, too few tags, stale, off-field, or an implausible
 *     jump from the settled estimate) are rejected.
 *   - Vision measurements are timestamped at capture time (now minus the
 *     Limelight pipeline latency and minus how long the result has been
 *     waiting on the Robot Controller) so the estimator can
 *     latency-compensate them.
 *
 * Usage each loop: call {@link #update()}, then read {@link #getPose()}.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.lib.estimator.PoseEstimator;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.RobotClock;
import org.firstinspires.ftc.teamcode.pathing.Follower;
import org.firstinspires.ftc.teamcode.pathing.Localizer;

import java.util.Collections;
import java.util.List;

public class Localization implements Localizer, Follower.VelocityAware {
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
    private String lastVisionSource = "none";
    private List<Integer> lastVisibleTagIds = Collections.emptyList();
    private boolean avgTagDistanceImplausible = false;

    // Heading-trust gate. MegaTag2 is only used for position once MegaTag1 --
    // which is computed without the gyro -- has agreed with our heading for
    // HEADING_TRUST_FRAMES consecutive frames. See VisionConstants.
    private boolean headingTrusted = false;
    private int headingAgreementFrames = 0;
    private double lastHeadingDisagreement = 0.0;

    // Outlier-rejection bookkeeping.
    private int acceptedFrames = 0;
    private int consecutiveJumpRejects = 0;

    // Velocity, differentiated from the ODOMETRY pose (never the fused pose --
    // a vision correction is a step, and differentiating a step gives a huge
    // false spike). Used by shoot-on-the-move.
    private Pose2d prevOdometryPose = null;
    private double prevOdometryTime = Double.NaN;
    private Translation2d fieldVelocity = new Translation2d();
    private double omegaRadPerSec = 0.0;

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

        // The new seed is unverified until MegaTag1 vouches for it, and the
        // settled-estimate outlier gate must not reject the frames that would
        // correct a bad seed.
        headingTrusted = false;
        headingAgreementFrames = 0;
        acceptedFrames = 0;
        consecutiveJumpRejects = 0;

        // The pose just teleported, so any velocity differentiated across that
        // jump would be nonsense.
        prevOdometryPose = null;
        prevOdometryTime = Double.NaN;
        fieldVelocity = new Translation2d();
        omegaRadPerSec = 0.0;
    }

    /** Runs one fusion cycle. Call once per loop. */
    public void update() {
        double now = currentTimeSeconds();

        // 1) Odometry: read the Pinpoint and feed it as the backbone estimate.
        odometry.update();
        lastOdometryPose = odometry.getPose();
        poseEstimator.updateWithTime(now, lastOdometryPose);
        updateVelocity(lastOdometryPose, now);

        // Vision disabled (or no Limelight): odometry-only, skip the rest.
        if (!visionEnabled || vision == null) {
            lastVisionAccepted = false;
            lastVisionReject = "vision disabled";
            lastVisibleTagIds = Collections.emptyList();
            return;
        }

        // 2) Feed the gyro heading to the camera. This is only used by MegaTag2
        // (getBotpose_MT2); it's harmless with the MegaTag1 path below, and keeps
        // things ready if you switch the vision pose to MegaTag2.
        double headingDegrees = lastOdometryPose.getRotation().getDegrees();
        vision.updateRobotOrientation(headingDegrees);

        // 3) Vision: validate and, if good, fuse it.
        LLResult result = vision.getLatestResult();

        // Which tags are in frame is recorded regardless of whether the pose
        // solve is usable -- goal selection wants to know what the camera can
        // see even on a frame we reject for fusion.
        lastVisibleTagIds = LimelightVision.visibleTagIds(result);

        processVision(result, now);
    }

    /** Differentiates the odometry pose into a filtered field velocity. */
    private void updateVelocity(Pose2d odometryPose, double now) {
        if (prevOdometryPose != null && !Double.isNaN(prevOdometryTime)) {
            double dt = now - prevOdometryTime;
            if (dt > 1e-6 && dt < VisionConstants.MAX_VELOCITY_DT_SECONDS) {
                double vx = (odometryPose.getX() - prevOdometryPose.getX()) / dt;
                double vy = (odometryPose.getY() - prevOdometryPose.getY()) / dt;
                double omega = shortestAngle(
                        odometryPose.getHeading() - prevOdometryPose.getHeading()) / dt;

                double a = VisionConstants.VELOCITY_FILTER_ALPHA;
                fieldVelocity = new Translation2d(
                        fieldVelocity.getX() + a * (vx - fieldVelocity.getX()),
                        fieldVelocity.getY() + a * (vy - fieldVelocity.getY()));
                omegaRadPerSec += a * (omega - omegaRadPerSec);
            }
        }
        prevOdometryPose = odometryPose;
        prevOdometryTime = now;
    }

    /**
     * Filtered translational velocity in FIELD coordinates, inches/sec. This is
     * what the shoot-on-the-move aiming maths runs on.
     */
    public Translation2d getFieldVelocity() {
        return fieldVelocity;
    }

    /** Filtered angular velocity, radians/sec CCW. */
    public double getAngularVelocity() {
        return omegaRadPerSec;
    }

    /** Speed regardless of direction, inches/sec. */
    public double getSpeed() {
        return fieldVelocity.getNorm();
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

        int tagCount = result.getBotposeTagCount();
        if (tagCount < VisionConstants.MIN_TAG_COUNT) {
            lastVisionReject = "too few tags";
            return;
        }

        long stalenessMs = result.getStaleness();
        if (stalenessMs > VisionConstants.MAX_STALENESS_MS) {
            lastVisionReject = "stale";
            return;
        }

        // ---- MegaTag1: the gyro-INDEPENDENT solve. Always computed, because it
        // is the only thing that can tell us whether our heading is right. ----
        Pose3d mt1Pose3d = toRobotPose3d(result.getBotpose());
        if (mt1Pose3d == null) {
            lastVisionReject = "no MegaTag1 fix";
            return;
        }
        Pose2d mt1Pose = mt1Pose3d.toPose2d();

        // Keep the robot-center 3D pose for diagnostics (z, pitch, roll).
        lastVisionPose3d = mt1Pose3d;
        lastVisionPose3dTimestamp = now;

        if (isOffField(mt1Pose)) {
            lastVisionReject = "off field";
            return;
        }

        // ---- Heading trust gate ----
        // Compare MegaTag1's independent heading against what we currently
        // believe. Sustained agreement earns trust (and unlocks MegaTag2);
        // gross disagreement -- a flipped seed, a 180-degree placement error --
        // revokes it immediately.
        double estimateHeading = poseEstimator.getEstimatedPosition().getHeading();
        lastHeadingDisagreement =
                shortestAngle(mt1Pose.getHeading() - estimateHeading);
        double absDisagreement = Math.abs(lastHeadingDisagreement);

        if (absDisagreement > VisionConstants.HEADING_DISTRUST_THRESHOLD) {
            headingTrusted = false;
            headingAgreementFrames = 0;
        } else if (absDisagreement < VisionConstants.HEADING_TRUST_TOLERANCE) {
            headingAgreementFrames++;
            if (headingAgreementFrames >= VisionConstants.HEADING_TRUST_FRAMES) {
                headingTrusted = true;
            }
        } else {
            headingAgreementFrames = 0;
        }

        // ---- Pick the position source ----
        // MegaTag2 only once the heading is trusted: it is steadier, but it is
        // computed FROM our heading, so a wrong heading turns it into a
        // confidently wrong position. MegaTag1 cannot be poisoned that way.
        Pose2d positionPose = mt1Pose;
        lastVisionSource = "MegaTag1";
        if (VisionConstants.PREFER_MEGATAG2 && headingTrusted) {
            Pose3d mt2Pose3d = toRobotPose3d(result.getBotpose_MT2());
            if (mt2Pose3d != null) {
                Pose2d mt2Pose = mt2Pose3d.toPose2d();
                if (!isOffField(mt2Pose)) {
                    positionPose = mt2Pose;
                    lastVisionSource = "MegaTag2";
                }
            }
        }

        // X/Y from the chosen source, heading always from MegaTag1. The
        // estimator gains each axis independently, so pairing them is sound --
        // and it keeps MegaTag2 from feeding our own heading back to us.
        Pose2d visionPose = new Pose2d(
                positionPose.getX(), positionPose.getY(), mt1Pose.getRotation());

        // ---- Outlier rejection ----
        // Last defence against an ambiguous solve teleporting the robot. Only
        // applied once the estimate has settled: at startup a badly seeded
        // estimate is the wrong one and vision is right. The consecutive-reject
        // escape hatch stops a genuinely wrong estimate from rejecting every
        // correction forever.
        if (acceptedFrames >= VisionConstants.MIN_FRAMES_BEFORE_JUMP_REJECT) {
            Pose2d current = poseEstimator.getEstimatedPosition();
            double jump = Math.hypot(visionPose.getX() - current.getX(),
                    visionPose.getY() - current.getY());
            if (jump > VisionConstants.MAX_POSE_JUMP_IN) {
                if (consecutiveJumpRejects < VisionConstants.JUMP_REJECT_LIMIT) {
                    consecutiveJumpRejects++;
                    lastVisionReject = String.format("jump %.0f in", jump);
                    return;
                }
                // Limit reached: the camera has insisted on the same
                // disagreement for too long, so the ESTIMATE is the thing
                // that is wrong. Stop rejecting and let vision pull us back.
                // The counter deliberately stays latched at the limit so
                // every following frame is accepted too -- releasing one
                // frame in every JUMP_REJECT_LIMIT would take many seconds
                // to converge. It resets below once a frame agrees again.
            } else {
                consecutiveJumpRejects = 0;
            }
        } else {
            consecutiveJumpRejects = 0;
        }

        // Frame accepted: publish its diagnostics.
        lastTagCount = tagCount;

        // getBotposeAvgDist() is a bare double with no unit attached, unlike the
        // botpose itself. Convert from whatever the SDK reports into inches.
        lastAvgTagDist = VisionConstants.BOTPOSE_AVG_DIST_UNIT.toInches(
                result.getBotposeAvgDist());
        acceptedFrames++;

        // Sanity-check the converted distance against the size of an FTC field.
        // This catches BOTPOSE_AVG_DIST_UNIT being wrong in either direction: a
        // metres-as-inches mistake reads ~39x too small, the reverse ~39x too
        // large. Clamping keeps a misconfiguration from driving the Kalman gain to
        // 1.0 (which would make vision snap the pose on every frame); the flag
        // tells the driver station why the numbers look odd.
        double trustDistance = lastAvgTagDist;
        avgTagDistanceImplausible =
                lastAvgTagDist < VisionConstants.MIN_PLAUSIBLE_TAG_DISTANCE_IN
                        || lastAvgTagDist > VisionConstants.MAX_PLAUSIBLE_TAG_DISTANCE_IN;
        if (avgTagDistanceImplausible) {
            trustDistance = Math.max(VisionConstants.MIN_PLAUSIBLE_TAG_DISTANCE_IN,
                    Math.min(VisionConstants.MAX_PLAUSIBLE_TAG_DISTANCE_IN,
                            lastAvgTagDist));
        }

        // Dynamic std devs (AdvantageKit style): trust scales with distance^2 / tagCount.
        double stdDevFactor = (trustDistance * trustDistance) / tagCount;
        double xyStdDev = VisionConstants.VISION_XY_STD_DEV_COEFFICIENT * stdDevFactor;
        double[] visionStdDevs = {xyStdDev, xyStdDev, VisionConstants.VISION_HEADING_STD_DEV};

        // Latency compensation: timestamp the frame at capture time. The frame's
        // total age is the Limelight's own pipeline latency (capture + targeting)
        // PLUS however long the finished result has been sitting on the Robot
        // Controller waiting to be read (staleness). Counting only the pipeline
        // latency would timestamp the frame later than it really was.
        double latencySeconds =
                (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
        double captureTimestamp = now - latencySeconds - (stalenessMs / 1000.0);

        poseEstimator.addVisionMeasurement(visionPose, captureTimestamp, visionStdDevs);
        lastVisionAccepted = true;
        lastVisionReject = "none";
    }

    /**
     * Converts a raw Limelight botpose into the robot-center 3D pose, applying
     * the camera mount transform when it is configured in code. Returns null if
     * the pose is missing or is the field origin, which is what the Limelight
     * reports when it has no real fix.
     */
    private Pose3d toRobotPose3d(Pose3D botpose) {
        if (botpose == null) {
            return null;
        }
        Position position = botpose.getPosition().toUnit(DistanceUnit.INCH);

        // Check the RAW botpose for the no-fix sentinel, before any camera
        // offset shifts it away from exactly zero.
        if (position.x == 0.0 && position.y == 0.0) {
            return null;
        }

        Pose3d reported = new Pose3d(
                position.x, position.y, position.z,
                botpose.getOrientation().getRoll(AngleUnit.RADIANS),
                botpose.getOrientation().getPitch(AngleUnit.RADIANS),
                botpose.getOrientation().getYaw(AngleUnit.RADIANS));

        return VisionConstants.APPLY_CAMERA_OFFSET_IN_CODE
                ? reported.transformBy(VisionConstants.ROBOT_TO_CAMERA.inverse())
                : reported;
    }

    private static boolean isOffField(Pose2d pose) {
        double limit = VisionConstants.FIELD_HALF_SIZE_IN + VisionConstants.FIELD_MARGIN_IN;
        return Math.abs(pose.getX()) > limit || Math.abs(pose.getY()) > limit;
    }

    /** Wraps an angle to [-pi, pi]. */
    private static double shortestAngle(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
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

    // ---------------------------------------------------------------------
    // Heading integrity: catching a flipped or mis-seeded start pose.
    // ---------------------------------------------------------------------

    /**
     * True once MegaTag1 -- which is solved without the gyro -- has agreed with
     * our heading for long enough to trust it. While false, position falls back
     * to MegaTag1 because MegaTag2 would inherit the bad heading.
     */
    public boolean isHeadingTrusted() {
        return headingTrusted;
    }

    /**
     * Signed gap in degrees between MegaTag1's independent heading and our
     * current estimate, from the most recent MegaTag1 fix. Near 180 means the
     * robot is seeded backwards -- almost always the wrong alliance selected,
     * or the robot placed facing the other way.
     */
    public double getHeadingDisagreementDegrees() {
        return Math.toDegrees(lastHeadingDisagreement);
    }

    /**
     * AprilTag IDs in the most recent camera frame, empty if none or if vision is
     * disabled. Used to pick which goal to aim at.
     */
    public List<Integer> getVisibleTagIds() {
        return lastVisibleTagIds;
    }

    /** True if the given tag was in the most recent frame. */
    public boolean isTagVisible(int id) {
        return lastVisibleTagIds.contains(id);
    }

    /**
     * True when the reported average tag distance does not look like a distance on
     * an FTC field, which almost always means
     * {@link VisionConstants#BOTPOSE_AVG_DIST_UNIT} is set to the wrong unit.
     */
    public boolean isAvgTagDistanceImplausible() {
        return avgTagDistanceImplausible;
    }

    /** Which solver supplied the most recent accepted position. */
    public String getLastVisionSource() {
        return lastVisionSource;
    }

    /** True if vision has ever produced a usable fix this OpMode. */
    public boolean hasVisionFix() {
        return lastVisionPose3dTimestamp > Double.NEGATIVE_INFINITY;
    }

    /**
     * Pre-match sanity check, meant to be polled during init while the robot
     * sits still looking at tags -- the best conditions MegaTag1 will ever get,
     * and the last moment a seeding mistake is cheap to fix.
     *
     * Returns a short human-readable verdict for telemetry. Pair it with
     * {@link #isStartPoseSuspect()} to decide whether to shout.
     */
    public String getStartPoseCheck() {
        if (!isVisionEnabled()) {
            return "vision disabled - cannot verify start pose";
        }
        if (!hasVisionFix()) {
            return "no tag in view yet - point the camera at a tag";
        }
        double gap = Math.abs(getHeadingDisagreementDegrees());
        if (gap > Math.toDegrees(VisionConstants.HEADING_SEED_WARN_THRESHOLD)) {
            return String.format(
                    "*** START POSE LOOKS WRONG: heading is %.0f deg off ***", gap);
        }
        return String.format("start pose OK (heading within %.0f deg)", gap);
    }

    /**
     * True when vision disagrees with the seeded heading badly enough that the
     * start pose is probably wrong. Check this during init, not mid-match.
     */
    public boolean isStartPoseSuspect() {
        return isVisionEnabled() && hasVisionFix()
                && Math.abs(lastHeadingDisagreement)
                        > VisionConstants.HEADING_SEED_WARN_THRESHOLD;
    }

    /**
     * Snaps the whole estimate to the latest MegaTag1 fix, overriding whatever
     * was seeded. MegaTag1 is used deliberately: it is solved from tag geometry
     * alone, so it can recover a heading the gyro has wrong.
     *
     * Use it from an init-time "accept what the camera sees" button, or as a
     * driver's last resort. Returns false if there is no usable fix, in which
     * case nothing is changed.
     */
    public boolean seedFromVision() {
        if (!isVisionEnabled() || !hasVisionFix()) {
            return false;
        }
        // Only act on a fix from the last moment, not one the robot has since
        // driven away from.
        if (getVisionPose3dAge(currentTimeSeconds())
                > VisionConstants.SEED_FROM_VISION_MAX_AGE_S) {
            return false;
        }
        setStartingPose(lastVisionPose3d.toPose2d());
        return true;
    }

    public PinpointOdometry getOdometry() {
        return odometry;
    }

    /** The Limelight wrapper, or null if vision was disabled at construction. */
    public LimelightVision getVision() {
        return vision;
    }

    /**
     * Dumps the full localization state to telemetry: fused pose, raw odometry,
     * and raw vision (when available) plus vision diagnostics. Shared by every
     * OpMode so logging is consistent everywhere.
     */
    public void addTelemetry(Telemetry telemetry) {
        double now = currentTimeSeconds();
        Pose2d fused = getPose();
        Pose2d odo = getOdometryPose();
        telemetry.addData("Fused", "x %.1f  y %.1f  h %.1f deg",
                fused.getX(), fused.getY(), fused.getRotation().getDegrees());
        telemetry.addData("Raw Odometry", "x %.1f  y %.1f  h %.1f deg",
                odo.getX(), odo.getY(), odo.getRotation().getDegrees());
        telemetry.addData("Vision", "enabled %s | accepted %s | %s",
                isVisionEnabled(), wasLastVisionAccepted(), getLastVisionReject());
        telemetry.addData("Vision source", "%s | heading %s | MT1 gap %.0f deg",
                getLastVisionSource(),
                isHeadingTrusted() ? "TRUSTED" : "unverified",
                getHeadingDisagreementDegrees());
        if (isStartPoseSuspect()) {
            telemetry.addLine("*** HEADING DISAGREES WITH VISION ***");
            telemetry.addLine("Wrong alliance, or robot placed backwards?");
        }
        if (lastVisionPose3dTimestamp > Double.NEGATIVE_INFINITY) {
            Pose2d vis = getVisionPose2d();
            telemetry.addData("Raw Vision 2D", "x %.1f  y %.1f  h %.1f deg  (age %.2fs)",
                    vis.getX(), vis.getY(), vis.getRotation().getDegrees(),
                    getVisionPose3dAge(now));
            telemetry.addData("Vision tags/dist", "tags %d  dist %.1f in",
                    getLastTagCount(), getLastAvgTagDistance());
            if (avgTagDistanceImplausible) {
                telemetry.addLine("*** TAG DISTANCE LOOKS WRONG ***");
                telemetry.addLine("Check VisionConstants.BOTPOSE_AVG_DIST_UNIT:");
                telemetry.addLine("~39x too small = should be INCH, too large = METER");
            }
            telemetry.addData("Vision 3D", "z %.1f  pitch %.1f  roll %.1f deg",
                    getVisionZ(), Math.toDegrees(getVisionPitch()), Math.toDegrees(getVisionRoll()));
        } else {
            telemetry.addData("Raw Vision 2D", "none yet");
        }
    }

    /** Stops the Limelight polling thread (if any). Call when the OpMode ends. */
    public void stop() {
        if (vision != null) {
            vision.stop();
        }
    }

    /** Monotonic clock shared by odometry samples and vision timestamps. */
    private static double currentTimeSeconds() {
        return RobotClock.nowSeconds();
    }
}
