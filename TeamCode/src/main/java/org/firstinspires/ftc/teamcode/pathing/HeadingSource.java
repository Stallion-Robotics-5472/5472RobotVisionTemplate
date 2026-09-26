/*
 * Where a path's target heading comes from.
 *
 * The built-in rules (face along the path, sweep from A to B, hold a fixed angle)
 * only need the path parameter t. But "keep pointing at the goal" depends on where
 * the robot actually is and how fast it is moving, and it wants an angular
 * feedforward as well as a target -- so heading is an interface rather than an
 * enum, and {@code AimAtGoalHeading} plugs into it.
 *
 * That is what lets the robot follow a path and aim at the same time. Before this,
 * the follower took heading from the path and the aiming command took it from the
 * shot solution, both of them requiring the drivetrain -- so the scheduler let only
 * one run and "drive this route while tracking the goal" was impossible.
 */
package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public interface HeadingSource {

    /** A heading to hold, and the rate that heading is itself changing. */
    final class Target {
        /** Field-relative heading to hold, radians. */
        public final double headingRadians;

        /**
         * How fast the target heading is sweeping, radians/sec CCW.
         *
         * Zero for a heading fixed in the field frame. Non-zero when the target
         * moves on its own -- aiming at a goal while translating past it is the
         * case that matters, because a position-only controller permanently
         * trails a sweeping target and a chassis is slow to rotate.
         */
        public final double omegaFeedforwardRadPerSec;

        public Target(double headingRadians, double omegaFeedforwardRadPerSec) {
            this.headingRadians = headingRadians;
            this.omegaFeedforwardRadPerSec = omegaFeedforwardRadPerSec;
        }

        /** A target with no feedforward, for headings fixed in the field frame. */
        public static Target of(double headingRadians) {
            return new Target(headingRadians, 0.0);
        }
    }

    /**
     * The heading to hold right now.
     *
     * @param t how far along the path we are, 0..1.
     * @param pose current fused field pose.
     * @param fieldVelocity field-frame translational velocity, inches/sec.
     * @param omegaRadPerSec current angular velocity, radians/sec CCW.
     */
    Target compute(double t, Pose2d pose, Translation2d fieldVelocity, double omegaRadPerSec);

    /** Called when the follower starts a path. Clear any per-run state here. */
    default void reset() {}
}
