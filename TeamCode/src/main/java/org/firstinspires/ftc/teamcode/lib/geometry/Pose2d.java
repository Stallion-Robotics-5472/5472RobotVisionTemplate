/*
 * Lightweight 2D geometry classes ported from WPILib (BSD-3-Clause, FIRST).
 *
 * A Pose2d represents a position and heading on the field. It implements the
 * exp()/log() maps on the SE(2) Lie group, which is what makes the pose
 * estimator able to correctly compose curved odometry deltas with vision
 * corrections.
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Pose2d {
    private final Translation2d m_translation;
    private final Rotation2d m_rotation;

    public Pose2d() {
        m_translation = new Translation2d();
        m_rotation = new Rotation2d();
    }

    public Pose2d(Translation2d translation, Rotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    public Pose2d(double x, double y, Rotation2d rotation) {
        m_translation = new Translation2d(x, y);
        m_rotation = rotation;
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

    /** Heading in radians. */
    public double getHeading() {
        return m_rotation.getRadians();
    }

    /** Transforms this pose by the given relative transform. */
    public Pose2d plus(Transform2d other) {
        return transformBy(other);
    }

    public Pose2d transformBy(Transform2d other) {
        return new Pose2d(
                m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
                other.getRotation().plus(m_rotation));
    }

    /** Returns the transform that maps {@code other} to this pose. */
    public Transform2d minus(Pose2d other) {
        Pose2d pose = this.relativeTo(other);
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    /** Expresses this pose relative to another pose's coordinate frame. */
    public Pose2d relativeTo(Pose2d other) {
        Transform2d transform = new Transform2d(other, this);
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Applies a twist (an arc-shaped delta in the robot's own frame) to this
     * pose, returning the new pose. This is the exponential map on SE(2).
     */
    public Pose2d exp(Twist2d twist) {
        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1e-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }

        Transform2d transform = new Transform2d(
                new Translation2d(dx * s - dy * c, dx * c + dy * s),
                new Rotation2d(cosTheta, sinTheta));

        return this.plus(transform);
    }

    /**
     * Returns the twist that maps this pose to the end pose. This is the
     * logarithmic map on SE(2) and is the inverse of {@link #exp(Twist2d)}.
     */
    public Twist2d log(Pose2d end) {
        Pose2d transform = end.relativeTo(this);
        double dtheta = transform.getRotation().getRadians();
        double halfDtheta = dtheta / 2.0;

        double cosMinusOne = transform.getRotation().getCos() - 1;

        double halfThetaByTanOfHalfDtheta;
        if (Math.abs(cosMinusOne) < 1e-9) {
            halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halfThetaByTanOfHalfDtheta =
                    -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne;
        }

        Translation2d translationPart = transform.getTranslation()
                .rotateBy(new Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta))
                .times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));

        return new Twist2d(translationPart.getX(), translationPart.getY(), dtheta);
    }

    /** Interpolates between this pose and another along the connecting arc. */
    public Pose2d interpolate(Pose2d endValue, double t) {
        if (t < 0) {
            return this;
        } else if (t >= 1) {
            return endValue;
        } else {
            Twist2d twist = this.log(endValue);
            Twist2d scaledTwist = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
            return this.exp(scaledTwist);
        }
    }

    @Override
    public String toString() {
        return String.format("Pose2d(%s, %s)", m_translation, m_rotation);
    }
}
