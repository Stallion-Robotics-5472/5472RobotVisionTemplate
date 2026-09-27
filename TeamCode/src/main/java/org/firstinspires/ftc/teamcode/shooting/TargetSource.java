/*
 * Where the thing we are shooting at is.
 *
 * There are two genuinely different ways to answer that, and they fail in
 * opposite ways, which is why this is an interface rather than one implementation.
 *
 *   GoalTargetSource   -- the goal is a FIELD COORDINATE from a table, and the
 *                         robot works out where that is relative to itself from
 *                         its own pose estimate. Needs the goal coordinates to be
 *                         right AND the pose to be right. In exchange it can aim
 *                         at a goal the camera cannot currently see, which is what
 *                         makes shooting while crossing the field possible.
 *
 *   VisionTargetSource -- the target is WHATEVER THE CAMERA IS LOOKING AT, ranged
 *                         from the frame itself. Needs no goal table and no field
 *                         position (see the note in that class for why the pose
 *                         cancels out), so it still works on a robot that was
 *                         never seeded or was seeded wrong. In exchange it can
 *                         only shoot at something it can see.
 *
 * Everything downstream -- the virtual-goal solve, the angular feedforward, the
 * readiness gates, the heading source that lets a path aim while following -- is
 * identical either way. Only the answer to "where is it" changes.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;

public interface TargetSource {

    /** A thing to shoot at, in absolute field coordinates. */
    final class Target {
        /** For telemetry: a goal name, or the tag being tracked. */
        public final String name;

        /**
         * Absolute field position, already flipped for the alliance being played.
         *
         * A vision source computes this from the robot's pose plus what the camera
         * measured, so it is a field coordinate even though nothing about the field
         * was configured.
         */
        public final Translation2d fieldPosition;

        /** The shot table for this target. Different goals can shoot differently. */
        public final ShooterMap map;

        /** Half-width of the opening, inches. Widens the heading tolerance. */
        public final double radiusInches;

        /** False when this is a held measurement rather than a fresh one. */
        public final boolean measuredThisLoop;

        /** Seconds since the target position was actually measured. */
        public final double ageSeconds;

        public Target(String name, Translation2d fieldPosition, ShooterMap map,
                      double radiusInches, boolean measuredThisLoop, double ageSeconds) {
            this.name = name;
            this.fieldPosition = fieldPosition;
            this.map = map;
            this.radiusInches = radiusInches;
            this.measuredThisLoop = measuredThisLoop;
            this.ageSeconds = ageSeconds;
        }

        /** A target measured this loop. */
        public static Target fresh(String name, Translation2d fieldPosition,
                                   ShooterMap map, double radiusInches) {
            return new Target(name, fieldPosition, map, radiusInches, true, 0.0);
        }

        @Override
        public String toString() {
            return String.format("%s at (%.1f, %.1f)%s", name,
                    fieldPosition.getX(), fieldPosition.getY(),
                    measuredThisLoop ? "" : String.format(" [held %.2fs]", ageSeconds));
        }
    }

    /**
     * The target to shoot at right now, or null when there is nothing to shoot at.
     *
     * Null is a normal answer, not an error -- a vision source with nothing in
     * frame has no target, and the command's job is then to say so rather than to
     * aim somewhere arbitrary. {@link #getRejectReason()} says why.
     *
     * @param pose current fused field pose.
     * @param fieldVelocity field-frame velocity, inches/sec.
     * @param omegaRadPerSec angular velocity, radians/sec CCW.
     * @param alliance which alliance is being played.
     */
    Target update(Pose2d pose, Translation2d fieldVelocity, double omegaRadPerSec,
                  Alliance alliance);

    /** Clears any per-run state. Called when a command using this starts. */
    default void reset() {}

    /**
     * While frozen, the target must not change. Held during a shot: on a
     * turretless robot a change of target moves the whole chassis.
     */
    default void freeze(boolean frozen) {}

    /** One line for telemetry. */
    String describe();

    /** Why there is no target, or null when there is one. */
    default String getRejectReason() {
        return null;
    }

    /**
     * Whether a shot from this source rests on the field pose being right.
     *
     * True for a goal table: the robot subtracts its own estimated position from a
     * configured coordinate, so a pose seeded 180 degrees out aims at empty field
     * with complete confidence. That is what the heading-trust gate exists to
     * catch.
     *
     * False for a source that measures the target relative to the robot. There the
     * pose error cancels (see VisionTargetSource), so demanding a trusted pose
     * would refuse a shot that is actually correct -- and refuse it in exactly the
     * situation this mode was added for.
     */
    default boolean requiresTrustedPose() {
        return true;
    }
}
