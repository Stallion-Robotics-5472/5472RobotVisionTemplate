/*
 * A single path: a Bezier curve plus a rule for what heading the robot should
 * hold along it. Provides the closest-point projection the follower needs to
 * compute its corrective vectors. Original implementation for this template.
 *
 * Heading modes:
 *   TANGENT  - face along the path (optionally reversed to drive backwards).
 *   LINEAR   - interpolate from a start heading to an end heading by t,
 *              taking the shortest angular route.
 *   CONSTANT - hold a fixed heading.
 */
package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public class Path {
    public enum HeadingMode { TANGENT, LINEAR, CONSTANT }

    private final BezierCurve curve;
    private HeadingMode headingMode = HeadingMode.TANGENT;
    private boolean tangentReversed = false;
    private double constantHeading = 0.0;
    private double startHeading = 0.0;
    private double endHeading = 0.0;

    // Resolution of the coarse closest-point scan.
    private int searchSteps = 200;
    // Steps used for remaining-length integration.
    private int lengthSteps = 100;

    public Path(BezierCurve curve) {
        this.curve = curve;
    }

    // ----- heading configuration (chainable) -----

    public Path setTangentHeading() {
        this.headingMode = HeadingMode.TANGENT;
        this.tangentReversed = false;
        return this;
    }

    public Path setReverseTangentHeading() {
        this.headingMode = HeadingMode.TANGENT;
        this.tangentReversed = true;
        return this;
    }

    public Path setLinearHeading(double startHeadingRad, double endHeadingRad) {
        this.headingMode = HeadingMode.LINEAR;
        this.startHeading = startHeadingRad;
        this.endHeading = endHeadingRad;
        return this;
    }

    public Path setConstantHeading(double headingRad) {
        this.headingMode = HeadingMode.CONSTANT;
        this.constantHeading = headingRad;
        return this;
    }

    public Path setSearchSteps(int steps) {
        this.searchSteps = Math.max(2, steps);
        return this;
    }

    // ----- geometry -----

    public BezierCurve getCurve() {
        return curve;
    }

    public Translation2d getPoint(double t) {
        return curve.getPoint(t);
    }

    /** Unit tangent (direction of travel) at t. */
    public Translation2d getUnitTangent(double t) {
        Translation2d d = curve.getDerivative(t);
        double norm = d.getNorm();
        if (norm < 1e-9) {
            return new Translation2d(1, 0);
        }
        return d.times(1.0 / norm);
    }

    public double getCurvature(double t) {
        return curve.getCurvature(t);
    }

    /** Target heading (radians) at parameter t, per the configured mode. */
    public double getHeading(double t) {
        switch (headingMode) {
            case CONSTANT:
                return constantHeading;
            case LINEAR: {
                double delta = shortestAngle(endHeading - startHeading);
                return startHeading + delta * clamp01(t);
            }
            case TANGENT:
            default: {
                Translation2d tan = getUnitTangent(t);
                double heading = Math.atan2(tan.getY(), tan.getX());
                if (tangentReversed) {
                    heading = shortestAngle(heading + Math.PI);
                }
                return heading;
            }
        }
    }

    /**
     * Parameter t of the closest point on the curve to {@code position}, searched
     * only forward from {@code minT} so the projection cannot slip backwards as
     * the robot advances. A coarse scan is refined with a ternary search.
     */
    public double getClosestT(Translation2d position, double minT) {
        minT = clamp01(minT);
        double best = minT;
        double bestDist = Double.MAX_VALUE;
        for (int i = 0; i <= searchSteps; i++) {
            double t = minT + (1.0 - minT) * i / searchSteps;
            double d = curve.getPoint(t).getDistance(position);
            if (d < bestDist) {
                bestDist = d;
                best = t;
            }
        }
        double window = (1.0 - minT) / searchSteps;
        double lo = Math.max(minT, best - window);
        double hi = Math.min(1.0, best + window);
        for (int k = 0; k < 30 && hi - lo > 1e-6; k++) {
            double m1 = lo + (hi - lo) / 3.0;
            double m2 = hi - (hi - lo) / 3.0;
            double d1 = curve.getPoint(m1).getDistance(position);
            double d2 = curve.getPoint(m2).getDistance(position);
            if (d1 < d2) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        return (lo + hi) / 2.0;
    }

    /** Remaining arc length from t to the end of the curve. */
    public double getRemainingLength(double t) {
        return curve.length(clamp01(t), 1.0, lengthSteps);
    }

    public double getTotalLength() {
        return curve.length();
    }

    public Translation2d getEndPoint() {
        return curve.getEndPoint();
    }

    static double shortestAngle(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static double clamp01(double t) {
        return Math.max(0.0, Math.min(1.0, t));
    }
}
