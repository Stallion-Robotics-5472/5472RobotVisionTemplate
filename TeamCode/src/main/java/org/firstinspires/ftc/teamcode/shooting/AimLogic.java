/*
 * Shoot-on-the-move aiming, for a robot with no turret.
 *
 * Ported from team 5472's FRC turret solution and adapted: there the turret
 * rotated and the chassis did as it pleased, so the output was a turret angle
 * clamped to a mechanical sweep. Here the ROBOT is the turret, so the output is
 * a field-relative heading for the drivetrain plus the angular velocity needed
 * to keep tracking while translating. Nothing is clamped, because the robot can
 * spin freely -- but a robot is far slower to rotate than a turret, which is why
 * the feedforward below matters more here than it did on the FRC bot.
 *
 * WHY AIMING AT THE GOAL IS WRONG WHILE MOVING
 *
 * A game piece leaving a moving robot keeps the robot's velocity. Shoot straight
 * at the goal while strafing right and the piece drifts right and misses -- by
 * roughly (speed x flight time), which at 60 in/s and half a second is about
 * 30 inches. The fix is to aim at a VIRTUAL GOAL, offset from the real one by
 * exactly the distance the robot's motion will carry the piece:
 *
 *     virtualGoal = goal - shooterVelocity * timeOfFlight
 *
 * That is circular: flight time comes from the shot distance, which depends on
 * where the virtual goal is, which depends on the flight time. So it is solved
 * by iterating to a fixed point. The FRC original ran a fixed three passes; this
 * version iterates until the distance stops moving, because a fixed count leaves
 * the reported distance and the virtual goal derived from slightly different
 * flight times, and that inconsistency alone was worth over an inch of miss.
 *
 * STEPS
 *   1. Look ahead by a phase delay, so we aim where the robot will be once the
 *      command actually reaches the motors, not where it was when we read.
 *   2. Find the shooter's own position and velocity. A shooter mounted off the
 *      turn centre sits somewhere else and is swung sideways when the robot
 *      spins, and that motion goes into the piece too.
 *   3. Iterate the virtual goal against flight time until it converges.
 *   4. Turn that into a robot heading, an angular feedforward, and the distance
 *      to look the shooter setpoint up with.
 *
 * Units are INCHES and RADIANS, matching the rest of this template. The FRC
 * original was in metres; every distance here is inches.
 *
 * This class is pure static maths with no hardware, so the offline suite in
 * tools/verify tests it directly -- by simulating the game piece and checking it
 * lands in the goal.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Twist2d;

public final class AimLogic {

    /**
     * Time step used to measure how fast the aim direction is sweeping, seconds.
     * Small enough to be a derivative, large enough to stay clear of rounding.
     */
    private static final double FEEDFORWARD_EPSILON_SECONDS = 1.0e-3;

    private AimLogic() {}

    /**
     * Computes the aiming solution.
     *
     * @param robotPose current fused field pose (inches, radians).
     * @param fieldVelocity robot translational velocity in FIELD coordinates,
     *     inches/sec. Use {@code Localization.getFieldVelocity()}.
     * @param omegaRadPerSec robot angular velocity, radians/sec CCW.
     * @param goal the goal's field position, inches. Alliance-dependent -- run it
     *     through {@code AllianceFlip} before passing it in.
     * @param map the shot table; supplies flight time and the setpoint.
     * @param config geometry and limits, see {@link Config}.
     */
    public static AimSolution calculate(Pose2d robotPose, Translation2d fieldVelocity,
                                        double omegaRadPerSec, Translation2d goal,
                                        ShooterMap map, Config config) {

        // --- 1. Phase-delay lookahead -------------------------------------
        // Everything downstream of this reading takes time: the loop finishes,
        // the command reaches the motor controller, the drivetrain responds. Aim
        // at where the robot will be by then.
        Pose2d aimPose = advance(robotPose, fieldVelocity, omegaRadPerSec,
                config.phaseDelaySeconds);

        // --- 2 & 3. Shooter state and the converged virtual goal ----------
        Geometry now = solve(aimPose, fieldVelocity, omegaRadPerSec, goal, map, config);

        // --- 4a. Heading ---------------------------------------------------
        // The shooter may not fire straight out the robot's front, so rotate the
        // robot such that the SHOOTER points at the goal, not the front bumper.
        double targetHeading = wrapRadians(now.aimAngle - config.shooterYawOffsetRadians);

        // --- 4b. Angular feedforward --------------------------------------
        // While the robot translates past the goal the aim direction keeps
        // sweeping, so a position-only heading controller permanently trails it.
        // This is that sweep rate.
        //
        // Measured rather than differentiated in closed form on purpose: the
        // virtual goal itself drifts as the shot distance changes, and the tidy
        // (vx*dy - vy*dx)/d^2 expression for a static target misses that term --
        // it was off by up to 17% in testing. Re-solving the geometry a
        // millisecond later captures every contribution exactly, and the whole
        // solve is a handful of map lookups.
        double feedforward = 0.0;
        if (fieldVelocity.getNorm() > 1e-9 || Math.abs(omegaRadPerSec) > 1e-9) {
            Pose2d later = advance(aimPose, fieldVelocity, omegaRadPerSec,
                    FEEDFORWARD_EPSILON_SECONDS);
            Geometry next = solve(later, fieldVelocity, omegaRadPerSec, goal, map, config);
            feedforward = wrapRadians(next.aimAngle - now.aimAngle)
                    / FEEDFORWARD_EPSILON_SECONDS;
        }

        // --- 4c. Readiness -------------------------------------------------
        // Tolerance from the goal's physical size: a wide goal close up forgives
        // a lot of heading error, the same goal across the field almost none.
        double tolerance = now.effectiveDistance < 1e-6
                ? config.maxHeadingToleranceRadians
                : Math.atan2(config.goalRadiusInches, now.effectiveDistance);
        tolerance = Math.min(tolerance, config.maxHeadingToleranceRadians);

        boolean inRange = now.effectiveDistance >= config.minShotDistanceInches
                && now.effectiveDistance <= config.maxShotDistanceInches
                && map.covers(now.effectiveDistance);

        return new AimSolution(targetHeading, feedforward, now.effectiveDistance,
                now.actualDistance, now.virtualGoal, now.shooterPosition, tolerance,
                map.setpointAt(now.effectiveDistance), inRange);
    }

    /** Walks a pose forward along its own curved path for {@code dt} seconds. */
    private static Pose2d advance(Pose2d pose, Translation2d fieldVelocity,
                                  double omegaRadPerSec, double dt) {
        if (dt == 0.0) {
            return pose;
        }
        // exp() takes a twist in the robot's own frame, so rotate the field
        // velocity into it first.
        Translation2d robotFrameVelocity =
                fieldVelocity.rotateBy(pose.getRotation().unaryMinus());
        return pose.exp(new Twist2d(
                robotFrameVelocity.getX() * dt,
                robotFrameVelocity.getY() * dt,
                omegaRadPerSec * dt));
    }

    /**
     * The shooter's position and velocity, and the virtual goal iterated to a
     * fixed point against flight time.
     */
    private static Geometry solve(Pose2d pose, Translation2d fieldVelocity,
                                  double omegaRadPerSec, Translation2d goal,
                                  ShooterMap map, Config config) {

        Translation2d rotatedOffset = config.robotToShooter.rotateBy(pose.getRotation());
        Translation2d shooterPosition = pose.getTranslation().plus(rotatedOffset);

        // An off-centre shooter is swung sideways by the robot's rotation: that
        // is omega x r, the offset turned 90 degrees and scaled by omega.
        Translation2d tangentialVelocity =
                rotatedOffset.rotateBy(Rotation2d.fromDegrees(90.0)).times(omegaRadPerSec);
        Translation2d shooterVelocity = fieldVelocity.plus(tangentialVelocity);

        double actualDistance = goal.getDistance(shooterPosition);

        // Fixed-point iteration. Each pass recomputes the offset from the REAL
        // goal with a better flight time, so errors do not compound. On exit
        // `distance` is exactly |virtualGoal - shooterPosition|, which is what
        // keeps the reported distance and the aimed-at point consistent.
        Translation2d virtualGoal = goal;
        double distance = actualDistance;
        for (int pass = 0; pass < config.maxIterations; pass++) {
            double timeOfFlight = map.timeOfFlightAt(distance);
            virtualGoal = goal.minus(shooterVelocity.times(timeOfFlight));
            double next = virtualGoal.getDistance(shooterPosition);
            boolean converged =
                    Math.abs(next - distance) < config.convergenceToleranceInches;
            distance = next;
            if (converged) {
                break;
            }
        }

        Translation2d toGoal = virtualGoal.minus(shooterPosition);
        double aimAngle = Math.atan2(toGoal.getY(), toGoal.getX());

        return new Geometry(shooterPosition, virtualGoal, distance, actualDistance, aimAngle);
    }

    /**
     * Straight-line distance from the shooter to a field point, ignoring motion.
     * Use this while building the shot map: it is the distance you are actually
     * measuring when you take a standing shot.
     */
    public static double shooterDistanceTo(Pose2d robotPose, Translation2d target,
                                           Config config) {
        Translation2d rotatedOffset = config.robotToShooter.rotateBy(robotPose.getRotation());
        return target.getDistance(robotPose.getTranslation().plus(rotatedOffset));
    }

    /** Wraps an angle to [-pi, pi]. */
    public static double wrapRadians(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    // ---------------------------------------------------------------------

    /** Intermediate geometry, shared by the solution and the feedforward probe. */
    private static final class Geometry {
        final Translation2d shooterPosition;
        final Translation2d virtualGoal;
        final double effectiveDistance;
        final double actualDistance;
        final double aimAngle;

        Geometry(Translation2d shooterPosition, Translation2d virtualGoal,
                 double effectiveDistance, double actualDistance, double aimAngle) {
            this.shooterPosition = shooterPosition;
            this.virtualGoal = virtualGoal;
            this.effectiveDistance = effectiveDistance;
            this.actualDistance = actualDistance;
            this.aimAngle = aimAngle;
        }
    }

    /**
     * Robot geometry and shot limits. Build one in ShootingConstants and reuse
     * it; it is immutable.
     */
    public static final class Config {
        /** Shooter position relative to robot centre: +X forward, +Y left, inches. */
        public final Translation2d robotToShooter;

        /**
         * Which way the shooter fires relative to robot forward, radians CCW.
         * 0 = straight ahead, PI = straight out the back.
         */
        public final double shooterYawOffsetRadians;

        /** Lookahead for control latency, seconds. 0.02-0.04 is typical. */
        public final double phaseDelaySeconds;

        /** Cap on virtual-goal passes. Convergence normally takes 3-4. */
        public final int maxIterations;

        /** Stop iterating once the shot distance moves less than this, inches. */
        public final double convergenceToleranceInches;

        /** Effective half-width of the goal opening, inches. Sets the tolerance. */
        public final double goalRadiusInches;

        /** Ceiling on the computed tolerance, so a point-blank shot still aims. */
        public final double maxHeadingToleranceRadians;

        public final double minShotDistanceInches;
        public final double maxShotDistanceInches;

        public Config(Translation2d robotToShooter, double shooterYawOffsetRadians,
                      double phaseDelaySeconds, int maxIterations,
                      double convergenceToleranceInches, double goalRadiusInches,
                      double maxHeadingToleranceRadians, double minShotDistanceInches,
                      double maxShotDistanceInches) {
            if (maxIterations < 1) {
                throw new IllegalArgumentException(
                        "maxIterations must be at least 1, or no moving-shot correction happens");
            }
            if (convergenceToleranceInches <= 0) {
                throw new IllegalArgumentException("convergence tolerance must be positive");
            }
            this.robotToShooter = robotToShooter;
            this.shooterYawOffsetRadians = shooterYawOffsetRadians;
            this.phaseDelaySeconds = phaseDelaySeconds;
            this.maxIterations = maxIterations;
            this.convergenceToleranceInches = convergenceToleranceInches;
            this.goalRadiusInches = goalRadiusInches;
            this.maxHeadingToleranceRadians = maxHeadingToleranceRadians;
            this.minShotDistanceInches = minShotDistanceInches;
            this.maxShotDistanceInches = maxShotDistanceInches;
        }
    }
}
