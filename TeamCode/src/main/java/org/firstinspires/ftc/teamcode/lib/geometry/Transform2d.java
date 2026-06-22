/*
 * Lightweight 2D geometry classes ported from WPILib (BSD-3-Clause, FIRST).
 *
 * A Transform2d represents a relative transformation (translation + rotation)
 * between two poses.
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Transform2d {
    private final Translation2d m_translation;
    private final Rotation2d m_rotation;

    public Transform2d(Translation2d translation, Rotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /**
     * Constructs the transform that maps the initial pose to the last pose,
     * expressed in the initial pose's frame.
     */
    public Transform2d(Pose2d initial, Pose2d last) {
        // The difference of the translations, rotated into the initial frame.
        m_translation = last.getTranslation()
                .minus(initial.getTranslation())
                .rotateBy(initial.getRotation().unaryMinus());
        m_rotation = last.getRotation().minus(initial.getRotation());
    }

    public Transform2d() {
        m_translation = new Translation2d();
        m_rotation = new Rotation2d();
    }

    public Translation2d getTranslation() {
        return m_translation;
    }

    public Rotation2d getRotation() {
        return m_rotation;
    }

    public double getX() {
        return m_translation.getX();
    }

    public double getY() {
        return m_translation.getY();
    }

    /**
     * Returns the inverse transform. If this maps frame A to frame B, the
     * inverse maps B back to A. Used to convert a camera's field pose into the
     * robot's field pose when the camera is not at the robot's center.
     */
    public Transform2d inverse() {
        // Rotate the negated translation into the inverse frame.
        return new Transform2d(
                m_translation.unaryMinus().rotateBy(m_rotation.unaryMinus()),
                m_rotation.unaryMinus());
    }

    @Override
    public String toString() {
        return String.format("Transform2d(%s, %s)", m_translation, m_rotation);
    }
}
