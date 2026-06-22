/*
 * Lightweight 2D geometry classes ported from WPILib (BSD-3-Clause, FIRST).
 * These mirror the math used by the FRC pose estimator so that the same
 * Kalman-style sensor fusion can be used on an FTC robot.
 *
 * A Rotation2d represents a rotation in 2D space, storing the angle as well as
 * its sine and cosine so that repeated trig is avoided.
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Rotation2d {
    private final double m_value;
    private final double m_cos;
    private final double m_sin;

    /** Constructs a Rotation2d at zero radians. */
    public Rotation2d() {
        this(0.0);
    }

    /** Constructs a Rotation2d from an angle in radians. */
    public Rotation2d(double radians) {
        m_value = radians;
        m_cos = Math.cos(radians);
        m_sin = Math.sin(radians);
    }

    /** Constructs a Rotation2d from an (x, y) vector; the angle is atan2(y, x). */
    public Rotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-9) {
            m_sin = y / magnitude;
            m_cos = x / magnitude;
        } else {
            m_sin = 0.0;
            m_cos = 1.0;
        }
        m_value = Math.atan2(m_sin, m_cos);
    }

    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    /** Adds two rotations together. */
    public Rotation2d plus(Rotation2d other) {
        return rotateBy(other);
    }

    /** Subtracts another rotation from this one. */
    public Rotation2d minus(Rotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    /** Negates this rotation. */
    public Rotation2d unaryMinus() {
        return new Rotation2d(-m_value);
    }

    /** Rotates this rotation by another, composing the two using a rotation matrix. */
    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(
                m_cos * other.m_cos - m_sin * other.m_sin,
                m_cos * other.m_sin + m_sin * other.m_cos);
    }

    public double getRadians() {
        return m_value;
    }

    public double getDegrees() {
        return Math.toDegrees(m_value);
    }

    public double getCos() {
        return m_cos;
    }

    public double getSin() {
        return m_sin;
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", m_value, getDegrees());
    }
}
