/*
 * A 3D rigid-body transform (rotation + translation), i.e. an element of SE(3).
 * Used to express the camera's pose relative to the robot center (forward/left/
 * up + roll/pitch/yaw) so a measured camera field pose can be converted to the
 * robot field pose. Original implementation.
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Transform3d {
    private final Translation3d translation;
    private final Rotation3d rotation;

    public Transform3d(Translation3d translation, Rotation3d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    /** @param x forward, @param y left, @param z up (inches); angles in radians. */
    public Transform3d(double x, double y, double z, double roll, double pitch, double yaw) {
        this(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    public Transform3d() {
        this(new Translation3d(), new Rotation3d());
    }

    public Translation3d getTranslation() {
        return translation;
    }

    public Rotation3d getRotation() {
        return rotation;
    }

    /**
     * Inverse transform. For T = (R, t), the inverse is (R^-1, -R^-1 t): if T
     * maps the robot frame to the camera frame, the inverse maps camera back to
     * robot.
     */
    public Transform3d inverse() {
        Rotation3d invRot = rotation.inverse();
        Translation3d invTrans = invRot.rotate(translation).unaryMinus();
        return new Transform3d(invTrans, invRot);
    }

    @Override
    public String toString() {
        return String.format("Transform3d(%s, %s)", translation, rotation);
    }
}
