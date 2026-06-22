/*
 * General-degree Bezier curve defined by control points, used as the geometric
 * backbone of a path. Provides the point, first derivative (tangent), second
 * derivative, signed curvature, and arc length via the standard Bernstein-form
 * formulas. Original implementation for this template.
 *
 * Units are inches (the field frame used by the pose estimator).
 */
package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BezierCurve {
    private final List<Translation2d> controlPoints;
    private final int degree;
    private double cachedLength = -1;

    private static final int LENGTH_STEPS = 1000;

    public BezierCurve(Translation2d... points) {
        if (points.length < 2) {
            throw new IllegalArgumentException("A Bezier curve needs at least 2 control points");
        }
        this.controlPoints = new ArrayList<>(Arrays.asList(points));
        this.degree = points.length - 1;
    }

    public int getDegree() {
        return degree;
    }

    public Translation2d getControlPoint(int i) {
        return controlPoints.get(i);
    }

    public Translation2d getEndPoint() {
        return controlPoints.get(degree);
    }

    public Translation2d getStartPoint() {
        return controlPoints.get(0);
    }

    /** Point on the curve at parameter t in [0, 1]. */
    public Translation2d getPoint(double t) {
        t = clamp01(t);
        double x = 0, y = 0;
        for (int i = 0; i <= degree; i++) {
            double b = bernstein(degree, i, t);
            x += b * controlPoints.get(i).getX();
            y += b * controlPoints.get(i).getY();
        }
        return new Translation2d(x, y);
    }

    /** First derivative dP/dt (tangent vector, not normalized). */
    public Translation2d getDerivative(double t) {
        t = clamp01(t);
        double x = 0, y = 0;
        for (int i = 0; i <= degree - 1; i++) {
            double b = bernstein(degree - 1, i, t);
            double dx = degree * (controlPoints.get(i + 1).getX() - controlPoints.get(i).getX());
            double dy = degree * (controlPoints.get(i + 1).getY() - controlPoints.get(i).getY());
            x += b * dx;
            y += b * dy;
        }
        return new Translation2d(x, y);
    }

    /** Second derivative d^2P/dt^2. */
    public Translation2d getSecondDerivative(double t) {
        t = clamp01(t);
        if (degree < 2) {
            return new Translation2d(0, 0);
        }
        double x = 0, y = 0;
        for (int i = 0; i <= degree - 2; i++) {
            double b = bernstein(degree - 2, i, t);
            double ddx = degree * (degree - 1) * (controlPoints.get(i + 2).getX()
                    - 2 * controlPoints.get(i + 1).getX() + controlPoints.get(i).getX());
            double ddy = degree * (degree - 1) * (controlPoints.get(i + 2).getY()
                    - 2 * controlPoints.get(i + 1).getY() + controlPoints.get(i).getY());
            x += b * ddx;
            y += b * ddy;
        }
        return new Translation2d(x, y);
    }

    /**
     * Signed curvature at t. Positive means the curve bends to the left of the
     * tangent direction (counter-clockwise). Returns 0 on a degenerate tangent.
     */
    public double getCurvature(double t) {
        Translation2d d = getDerivative(t);
        Translation2d dd = getSecondDerivative(t);
        double cross = d.getX() * dd.getY() - d.getY() * dd.getX();
        double speed = Math.hypot(d.getX(), d.getY());
        double denom = speed * speed * speed;
        if (denom < 1e-9) {
            return 0.0;
        }
        return cross / denom;
    }

    /** Total arc length of the curve (cached). */
    public double length() {
        if (cachedLength < 0) {
            cachedLength = length(0.0, 1.0, LENGTH_STEPS);
        }
        return cachedLength;
    }

    /** Arc length between two parameter values via Riemann sum. */
    public double length(double t0, double t1, int steps) {
        if (steps < 1) {
            steps = 1;
        }
        double len = 0;
        Translation2d prev = getPoint(t0);
        for (int i = 1; i <= steps; i++) {
            double t = t0 + (t1 - t0) * i / steps;
            Translation2d p = getPoint(t);
            len += p.getDistance(prev);
            prev = p;
        }
        return len;
    }

    private static double bernstein(int n, int i, double t) {
        return binomial(n, i) * Math.pow(1 - t, n - i) * Math.pow(t, i);
    }

    private static double binomial(int n, int k) {
        if (k < 0 || k > n) {
            return 0;
        }
        if (k == 0 || k == n) {
            return 1;
        }
        k = Math.min(k, n - k);
        double result = 1;
        for (int i = 0; i < k; i++) {
            result = result * (n - i) / (i + 1);
        }
        return result;
    }

    private static double clamp01(double t) {
        return Math.max(0.0, Math.min(1.0, t));
    }
}
