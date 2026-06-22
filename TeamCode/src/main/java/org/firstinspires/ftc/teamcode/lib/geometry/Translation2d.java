/*
 * Lightweight 2D geometry classes ported from WPILib (BSD-3-Clause, FIRST).
 *
 * A Translation2d represents a 2D offset (x, y). Units are intentionally
 * unspecified here; this template works in inches to match the FTC field.
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Translation2d {
    private final double m_x;
    private final double m_y;

    public Translation2d() {
        this(0.0, 0.0);
    }

    public Translation2d(double x, double y) {
        m_x = x;
        m_y = y;
    }

    public double getX() {
        return m_x;
    }

    public double getY() {
        return m_y;
    }

    /** Returns the distance of this translation from the origin. */
    public double getNorm() {
        return Math.hypot(m_x, m_y);
    }

    /** Returns the straight-line distance between two translations. */
    public double getDistance(Translation2d other) {
        return Math.hypot(other.m_x - m_x, other.m_y - m_y);
    }

    /** Applies a rotation to this translation about the origin. */
    public Translation2d rotateBy(Rotation2d other) {
        return new Translation2d(
                m_x * other.getCos() - m_y * other.getSin(),
                m_x * other.getSin() + m_y * other.getCos());
    }

    public Translation2d plus(Translation2d other) {
        return new Translation2d(m_x + other.m_x, m_y + other.m_y);
    }

    public Translation2d minus(Translation2d other) {
        return new Translation2d(m_x - other.m_x, m_y - other.m_y);
    }

    public Translation2d unaryMinus() {
        return new Translation2d(-m_x, -m_y);
    }

    public Translation2d times(double scalar) {
        return new Translation2d(m_x * scalar, m_y * scalar);
    }

    @Override
    public String toString() {
        return String.format("Translation2d(X: %.2f, Y: %.2f)", m_x, m_y);
    }
}
