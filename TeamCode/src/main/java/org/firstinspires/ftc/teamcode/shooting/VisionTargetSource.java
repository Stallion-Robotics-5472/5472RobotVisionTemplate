/*
 * Shoot at whatever the camera is looking at, rather than at a coordinate from a
 * table.
 *
 * WHY THIS MODE EXISTS
 *
 * The goal-table mode asks the robot to subtract its own estimated position from a
 * configured field coordinate. That needs two things to be right: the coordinates,
 * and the pose. When either is wrong the robot aims at empty field with complete
 * confidence, which is why the shoot commands refuse to fire until vision has
 * vouched for the heading.
 *
 * This mode needs neither. It measures where the target is RELATIVE TO THE ROBOT,
 * from the camera frame, every loop.
 *
 * And the pose error genuinely cancels -- this is worth spelling out because it
 * looks like it should not. The target's field position is computed as
 *
 *     targetField = robotPosition + rotate(relativeToRobot, heading)
 *
 * and the aiming solution then works out where the target is relative to the
 * shooter as
 *
 *     targetField - (robotPosition + rotate(robotToShooter, heading))
 *               = rotate(relativeToRobot - robotToShooter, heading)
 *
 * The robot position drops out algebraically. A heading error delta shifts the
 * computed target heading by delta, but the heading the controller measures is
 * shifted by the same delta, so the robot physically points in the right
 * direction. Velocity comes from the same (possibly rotated) odometry frame as
 * everything else, so the virtual-goal correction stays consistent too. The whole
 * calculation runs in whatever frame odometry happens to believe in, and the
 * physical result is correct. The offline suite measures this: a pose displaced 60
 * inches and rotated 35 degrees changes the physical aim by under a hundredth of a
 * degree.
 *
 * WHAT IT COSTS
 *
 * It can only shoot at something it can see. The goal-table mode can aim at a goal
 * behind the robot while crossing the field; this cannot. When the target leaves
 * frame the last measurement is held for HOLD_SECONDS -- as a FIELD point, so the
 * robot keeps aiming correctly as it drives, on odometry, which is accurate to
 * inches over a second or two. After that it reports no target and the shot is
 * refused.
 *
 * WHAT YOU MUST MEASURE
 *
 * Two heights and a camera angle, with a tape measure and a phone level:
 * CAMERA_LENS_HEIGHT_IN, TARGET_CENTER_HEIGHT_IN, and
 * VisionConstants.CAMERA_PITCH_OFFSET_DEG. That is the entire configuration -- no
 * field coordinates, which is the point. If the geometry is left unconfigured this
 * refuses to produce a target and says so, rather than reporting a plausible wrong
 * distance.
 *
 * RANGING
 *
 * Distance comes from the elevation angle, not from the tag's 3D pose solve:
 *
 *     horizontalDistance = (targetHeight - cameraHeight) / tan(cameraElevation + ty)
 *
 * A single tag's 3D pose is noisy and can flip orientation at range; the elevation
 * angle is one number off the sensor and degrades gracefully. The Limelight does
 * publish getTargetPoseRobotSpace(), and it is deliberately not used here: its axis
 * convention could not be verified offline, and an axis guess is a silent 90-degree
 * aiming error.
 */
package org.firstinspires.ftc.teamcode.shooting;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.RobotClock;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;

import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class VisionTargetSource implements TargetSource {

    /** The measured geometry this mode needs, and nothing else. */
    public static final class Config {
        /** Height of the camera lens above the floor, inches. */
        public final double cameraHeightInches;
        /** Height of the point you are shooting at above the floor, inches. */
        public final double targetHeightInches;
        /**
         * Camera elevation above horizontal, radians. Derived from
         * VisionConstants.CAMERA_PITCH_OFFSET_DEG, whose sign convention is that a
         * POSITIVE pitch tilts the camera DOWN -- so a camera angled up to see tags
         * has a negative pitch and a positive elevation here.
         */
        public final double cameraElevationRadians;
        /**
         * Multiplies the camera's tx before it is used as a bearing.
         *
         * Limelight reports tx positive when the target is to the RIGHT of the
         * crosshair, and this codebase is CCW-positive, so a right-hand target is a
         * NEGATIVE bearing: -1.0 is correct. It is a constant rather than a buried
         * minus sign because it is the one thing here that cannot be checked
         * offline, and it is a thirty-second check on the robot: point the camera
         * left of a tag and confirm the robot turns left.
         */
        public final double txSign;
        /** Shot table for whatever is being shot at. */
        public final ShooterMap map;
        /** Half-width of the opening, inches. Widens the heading tolerance. */
        public final double radiusInches;
        /** How long a measurement is still used after the target leaves frame. */
        public final double holdSeconds;
        /**
         * Tag IDs that count as the target. Empty means any tag in frame.
         *
         * Worth filling in even though the mode does not need field coordinates:
         * it stops the robot ranging off a tag that is not a goal.
         */
        public final Set<Integer> tagIds;
        /** Consecutive frames a different tag must win before the target switches. */
        public final int switchFrames;

        public Config(double cameraHeightInches, double targetHeightInches,
                      double cameraElevationRadians, double txSign, ShooterMap map,
                      double radiusInches, double holdSeconds, Set<Integer> tagIds,
                      int switchFrames) {
            this.cameraHeightInches = cameraHeightInches;
            this.targetHeightInches = targetHeightInches;
            this.cameraElevationRadians = cameraElevationRadians;
            this.txSign = txSign;
            this.map = map;
            this.radiusInches = radiusInches;
            this.holdSeconds = holdSeconds;
            this.tagIds = tagIds == null ? Collections.emptySet() : tagIds;
            this.switchFrames = switchFrames;
        }

        public Config withTagIds(Set<Integer> ids) {
            return new Config(cameraHeightInches, targetHeightInches,
                    cameraElevationRadians, txSign, map, radiusInches, holdSeconds,
                    ids, switchFrames);
        }

        public Config withMap(ShooterMap newMap) {
            return new Config(cameraHeightInches, targetHeightInches,
                    cameraElevationRadians, txSign, newMap, radiusInches, holdSeconds,
                    tagIds, switchFrames);
        }

        /**
         * Whether the geometry can produce a distance at all.
         *
         * A camera at the same height as the target gives a zero height difference
         * and an infinite distance; both defaulting to zero (as they ship) gives
         * 0/0. Neither must be allowed to look like a measurement.
         */
        public boolean isUsable() {
            return Math.abs(targetHeightInches - cameraHeightInches)
                    > MIN_HEIGHT_DIFFERENCE_IN;
        }
    }

    /**
     * Least height difference the trig will be trusted with, inches.
     *
     * Near zero the tangent divides by almost nothing: a tenth of a degree of
     * measurement noise swings the reported distance by feet. Refusing is the only
     * honest answer.
     */
    public static final double MIN_HEIGHT_DIFFERENCE_IN = 6.0;

    /** Sanity bounds on a computed range, inches. Outside these it is noise. */
    public static final double MIN_PLAUSIBLE_RANGE_IN = 6.0;
    public static final double MAX_PLAUSIBLE_RANGE_IN = 300.0;

    private final Config config;
    private final Supplier<List<LLResultTypes.FiducialResult>> fiducials;

    private int lockedTagId = -1;
    private int challengerTagId = -1;
    private int challengerStreak = 0;
    private boolean frozen = false;

    private Translation2d heldFieldPosition = null;
    private double heldAt = Double.NEGATIVE_INFINITY;
    private int heldTagId = -1;
    private double lastRangeInches = Double.NaN;
    private double lastBearingRadians = Double.NaN;
    private String rejectReason = "no frame yet";

    /**
     * @param fiducials the tags in the current camera frame. Pass
     *     {@code localization::getFiducials} -- the frame the pose estimator
     *     already fetched this loop, not a second poll of the camera, which would
     *     be a second bus transaction for the same data.
     */
    public VisionTargetSource(Config config,
                              Supplier<List<LLResultTypes.FiducialResult>> fiducials) {
        this.config = config;
        this.fiducials = fiducials;
    }

    @Override
    public Target update(Pose2d pose, Translation2d fieldVelocity, double omegaRadPerSec,
                         Alliance alliance) {
        if (!config.isUsable()) {
            rejectReason = String.format(
                    "camera/target heights not set (%.1f vs %.1f in) - see"
                            + " VISION_TARGET_CONFIG",
                    config.cameraHeightInches, config.targetHeightInches);
            return heldOrNothing(pose);
        }

        LLResultTypes.FiducialResult chosen = chooseTag();
        if (chosen == null) {
            return heldOrNothing(pose);
        }

        double elevation = config.cameraElevationRadians
                + Math.toRadians(chosen.getTargetYDegrees());
        // Above the horizon for a target below the camera, or vice versa: the
        // tangent has the wrong sign and the "distance" comes out negative or
        // enormous. This is what a mis-signed camera pitch looks like.
        double tangent = Math.tan(elevation);
        double heightDifference = config.targetHeightInches - config.cameraHeightInches;
        if (Math.abs(tangent) < 1e-6 || (heightDifference / tangent) <= 0.0) {
            rejectReason = String.format(
                    "tag %d elevation %.1f deg cannot range a target %.1f in %s the"
                            + " camera - check CAMERA_PITCH_OFFSET_DEG's sign",
                    chosen.getFiducialId(), Math.toDegrees(elevation),
                    Math.abs(heightDifference), heightDifference > 0 ? "above" : "below");
            return heldOrNothing(pose);
        }

        double range = heightDifference / tangent;
        if (range < MIN_PLAUSIBLE_RANGE_IN || range > MAX_PLAUSIBLE_RANGE_IN) {
            rejectReason = String.format("tag %d ranged at %.0f in, outside %.0f-%.0f",
                    chosen.getFiducialId(), range,
                    MIN_PLAUSIBLE_RANGE_IN, MAX_PLAUSIBLE_RANGE_IN);
            return heldOrNothing(pose);
        }

        double bearing = config.txSign * Math.toRadians(chosen.getTargetXDegrees());
        lastRangeInches = range;
        lastBearingRadians = bearing;

        // Robot frame -> field frame. The rotation uses the same heading the aiming
        // solution will use, which is why a heading error cancels rather than
        // accumulating; see the note at the top of this file.
        Translation2d relative = new Translation2d(
                range * Math.cos(bearing), range * Math.sin(bearing));
        Translation2d fieldPosition = pose.getTranslation()
                .plus(relative.rotateBy(pose.getRotation()));

        heldFieldPosition = fieldPosition;
        heldAt = RobotClock.nowSeconds();
        heldTagId = chosen.getFiducialId();
        rejectReason = null;
        return Target.fresh(name(heldTagId), fieldPosition, config.map,
                config.radiusInches);
    }

    /**
     * Which tag to range off, with enough stickiness that a flickering detection
     * cannot swing the chassis.
     *
     * Prefers the tag it is already tracking whenever that tag is in frame at all.
     * A challenger has to be the best candidate for switchFrames consecutive frames
     * before it takes over -- the same reasoning as GoalSelector's hysteresis, and
     * for the same reason: here the target sets the whole robot's heading.
     */
    private LLResultTypes.FiducialResult chooseTag() {
        List<LLResultTypes.FiducialResult> frame = fiducials.get();
        if (frame == null || frame.isEmpty()) {
            rejectReason = config.tagIds.isEmpty()
                    ? "no tag in frame"
                    : "none of tags " + config.tagIds + " in frame";
            return null;
        }

        LLResultTypes.FiducialResult incumbent = null;
        LLResultTypes.FiducialResult best = null;
        int matching = 0;
        for (LLResultTypes.FiducialResult candidate : frame) {
            if (candidate == null) {
                continue;
            }
            if (!config.tagIds.isEmpty()
                    && !config.tagIds.contains(candidate.getFiducialId())) {
                continue;
            }
            matching++;
            if (candidate.getFiducialId() == lockedTagId) {
                incumbent = candidate;
            }
            // Largest in frame: nearest, or at least the least foreshortened, which
            // is the one whose elevation angle is worth ranging off.
            if (best == null || candidate.getTargetArea() > best.getTargetArea()) {
                best = candidate;
            }
        }
        if (matching == 0) {
            rejectReason = config.tagIds.isEmpty()
                    ? "no tag in frame"
                    : "none of tags " + config.tagIds + " in frame";
            return null;
        }

        if (frozen && incumbent != null) {
            return incumbent;       // a shot is in progress; do not move the target
        }
        if (incumbent == null) {
            // Whatever we were tracking is gone. Take the best on offer now: there
            // is nothing to be sticky about.
            lockedTagId = best.getFiducialId();
            challengerTagId = -1;
            challengerStreak = 0;
            return best;
        }
        if (best.getFiducialId() == incumbent.getFiducialId()) {
            challengerTagId = -1;
            challengerStreak = 0;
            return incumbent;
        }
        if (best.getFiducialId() == challengerTagId) {
            challengerStreak++;
        } else {
            challengerTagId = best.getFiducialId();
            challengerStreak = 1;
        }
        if (challengerStreak > config.switchFrames) {
            lockedTagId = best.getFiducialId();
            challengerTagId = -1;
            challengerStreak = 0;
            return best;
        }
        return incumbent;
    }

    /**
     * The last measurement while it is still fresh enough to use, else nothing.
     *
     * Held as a FIELD point rather than a relative one, so the robot keeps aiming
     * at the right place while it drives. That leans on odometry, which is worth a
     * fraction of an inch over the hold window even when the absolute pose is
     * wrong -- the hold is a relative-motion question, not an absolute one.
     */
    private Target heldOrNothing(Pose2d pose) {
        if (heldFieldPosition == null) {
            return null;
        }
        double age = RobotClock.nowSeconds() - heldAt;
        if (age > config.holdSeconds) {
            heldFieldPosition = null;
            return null;
        }
        return new Target(name(heldTagId), heldFieldPosition, config.map,
                config.radiusInches, false, age);
    }

    private static String name(int tagId) {
        return "tag " + tagId;
    }

    @Override
    public void reset() {
        lockedTagId = -1;
        challengerTagId = -1;
        challengerStreak = 0;
        heldFieldPosition = null;
        heldAt = Double.NEGATIVE_INFINITY;
        heldTagId = -1;
        lastRangeInches = Double.NaN;
        lastBearingRadians = Double.NaN;
        rejectReason = "no frame yet";
    }

    @Override
    public void freeze(boolean frozen) {
        this.frozen = frozen;
    }

    @Override
    public String describe() {
        if (rejectReason != null) {
            return "vision target: " + rejectReason;
        }
        return String.format("vision target: tag %d at %.0f in, %+.1f deg off",
                heldTagId, lastRangeInches, Math.toDegrees(lastBearingRadians));
    }

    @Override
    public String getRejectReason() {
        return rejectReason;
    }

    /**
     * False: this mode measures the target relative to the robot, so the field pose
     * cancels out of the answer. Demanding a trusted pose would refuse a shot that
     * is correct, in exactly the situation this mode exists for.
     */
    @Override
    public boolean requiresTrustedPose() {
        return false;
    }

    /** Range from the last accepted frame, inches. NaN before there is one. */
    public double getRangeInches() {
        return lastRangeInches;
    }

    /** Bearing from the last accepted frame, radians CCW. NaN before there is one. */
    public double getBearingRadians() {
        return lastBearingRadians;
    }

    /** The tag being tracked, or -1. */
    public int getTrackedTagId() {
        return heldTagId;
    }

    public Config getConfig() {
        return config;
    }

    /**
     * The config assembled from ShootingConstants and VisionConstants.
     *
     * Note the sign: VisionConstants.CAMERA_PITCH_OFFSET_DEG is positive DOWN, and
     * elevation here is positive UP.
     */
    public static Config configFromConstants() {
        return new Config(
                ShootingConstants.CAMERA_LENS_HEIGHT_IN,
                ShootingConstants.TARGET_CENTER_HEIGHT_IN,
                Math.toRadians(-VisionConstants.CAMERA_PITCH_OFFSET_DEG),
                ShootingConstants.VISION_TARGET_TX_SIGN,
                ShootingConstants.SHOT_MAP,
                ShootingConstants.GOAL_RADIUS_IN,
                ShootingConstants.VISION_TARGET_HOLD_SECONDS,
                ShootingConstants.VISION_TARGET_TAG_IDS,
                ShootingConstants.VISION_TARGET_SWITCH_FRAMES);
    }

    /** Builds one wired to a localizer's cached camera frame. */
    public static VisionTargetSource fromConstants(
            Supplier<List<LLResultTypes.FiducialResult>> fiducials) {
        return new VisionTargetSource(configFromConstants(), fiducials);
    }

    /** Convenience for {@link Rotation2d} users: the bearing as a rotation. */
    public Rotation2d getBearing() {
        return new Rotation2d(Double.isNaN(lastBearingRadians) ? 0.0 : lastBearingRadians);
    }
}
