/*
 * The answer to "where do I point and how hard do I shoot, right now, moving at
 * this speed". Produced by AimLogic.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public final class AimSolution {

    /**
     * Field-relative heading the ROBOT should hold, radians CCW.
     *
     * With a turret this would be a turret angle. Without one the robot itself is
     * the turret, so this goes to the drivetrain's heading controller.
     */
    public final double targetHeadingRadians;

    /**
     * Angular velocity to feed forward, radians/sec CCW.
     *
     * While the robot translates past the goal, the direction to the goal keeps
     * changing, so a position-only heading controller always trails it. This is
     * the rate the aim direction is sweeping at, and adding it to the heading
     * controller's output is what lets the robot actually track while moving
     * rather than chase.
     */
    public final double headingFeedforwardRadPerSec;

    /**
     * Distance from the shooter to the VIRTUAL goal, inches -- the distance the
     * shot has to cover given the robot's motion, not the straight-line distance
     * to the real goal. This is what to look the shooter setpoint up with.
     */
    public final double effectiveDistanceInches;

    /** Straight-line distance from the shooter to the real goal, inches. */
    public final double actualDistanceInches;

    /** Where the shot is actually aimed, in field coordinates. */
    public final Translation2d virtualGoal;

    /** Where the shooter is, in field coordinates, at the lookahead time. */
    public final Translation2d shooterPosition;

    /**
     * Largest heading error that still hits, radians. Derived from the goal's
     * size and the shot distance, so it tightens as you back away -- a fixed
     * angular tolerance would be far too loose up close and too strict far out.
     */
    public final double headingToleranceRadians;

    /** The shooter setpoint for {@link #effectiveDistanceInches}. */
    public final ShooterSetpoint setpoint;

    /**
     * True when the shot is geometrically possible: the effective distance is
     * inside the shooter map's measured range and the configured shot range.
     *
     * This says nothing about whether the robot is currently pointed correctly or
     * the flywheel is up to speed -- see {@link #isAimedFrom(double)} and the
     * shooter's own readiness check.
     */
    public final boolean inRange;

    public AimSolution(double targetHeadingRadians, double headingFeedforwardRadPerSec,
                       double effectiveDistanceInches, double actualDistanceInches,
                       Translation2d virtualGoal, Translation2d shooterPosition,
                       double headingToleranceRadians, ShooterSetpoint setpoint,
                       boolean inRange) {
        this.targetHeadingRadians = targetHeadingRadians;
        this.headingFeedforwardRadPerSec = headingFeedforwardRadPerSec;
        this.effectiveDistanceInches = effectiveDistanceInches;
        this.actualDistanceInches = actualDistanceInches;
        this.virtualGoal = virtualGoal;
        this.shooterPosition = shooterPosition;
        this.headingToleranceRadians = headingToleranceRadians;
        this.setpoint = setpoint;
        this.inRange = inRange;
    }

    public Rotation2d targetHeading() {
        return new Rotation2d(targetHeadingRadians);
    }

    /** Signed heading error from the given actual heading, radians, wrapped. */
    public double headingErrorFrom(double currentHeadingRadians) {
        return AimLogic.wrapRadians(targetHeadingRadians - currentHeadingRadians);
    }

    /** True when the robot is pointed closely enough to hit from this heading. */
    public boolean isAimedFrom(double currentHeadingRadians) {
        return Math.abs(headingErrorFrom(currentHeadingRadians)) <= headingToleranceRadians;
    }

    /** True when the shot is in range AND the robot is pointed correctly. */
    public boolean canShootFrom(double currentHeadingRadians) {
        return inRange && isAimedFrom(currentHeadingRadians);
    }

    @Override
    public String toString() {
        return String.format(
                "AimSolution{heading %.1f deg, ff %.2f rad/s, eff %.1f in "
                        + "(actual %.1f), tol %.1f deg, inRange %b}",
                Math.toDegrees(targetHeadingRadians), headingFeedforwardRadPerSec,
                effectiveDistanceInches, actualDistanceInches,
                Math.toDegrees(headingToleranceRadians), inRange);
    }
}
