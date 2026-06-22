/*
 * Lightweight 2D geometry classes ported from WPILib (BSD-3-Clause, FIRST).
 *
 * A Twist2d is a change in pose along an arc: a forward delta (dx), a sideways
 * delta (dy), and a change in heading (dtheta). It is the "tangent space"
 * representation used by the pose estimator when blending vision and odometry.
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Twist2d {
    public final double dx;
    public final double dy;
    public final double dtheta;

    public Twist2d() {
        this(0.0, 0.0, 0.0);
    }

    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    @Override
    public String toString() {
        return String.format("Twist2d(dX: %.2f, dY: %.2f, dTheta: %.2f)", dx, dy, dtheta);
    }
}
