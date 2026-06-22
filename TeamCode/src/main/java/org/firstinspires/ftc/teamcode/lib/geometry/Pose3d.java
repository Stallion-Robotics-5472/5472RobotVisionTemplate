/*
 * A full 3D pose (element of SE(3)): a Translation3d plus a Rotation3d. The
 * Limelight's AprilTag botpose is inherently 3D; this class lets us compose it
 * with a 3D camera-to-robot transform to recover the robot-center pose when the
 * camera is mounted off-center, raised, tilted (pitch), or rolled. The fused
 * field estimate used for driving is then the 2D projection (x, y, yaw).
 *
 * Original implementation. Angle convention matches the FTC SDK's
 * YawPitchRollAngles (roll about +X, pitch about +Y, yaw about +Z, radians).
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Pose3d {
    private final Translation3d translation;
    private final Rotation3d rotation;

    public Pose3d() {
        this(new Translation3d(), new Rotation3d());
    }

    public Pose3d(Translation3d translation, Rotation3d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Pose3d(double x, double y, double z, double roll, double pitch, double yaw) {
        this(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    public Translation3d getTranslation() {
        return translation;
    }

    public Rotation3d getRotation() {
        return rotation;
    }

    public double getX() {
        return translation.getX();
    }

    public double getY() {
        return translation.getY();
    }

    public double getZ() {
        return translation.getZ();
    }

    public double getRoll() {
        return rotation.getRoll();
    }

    public double getPitch() {
        return rotation.getPitch();
    }

    public double getYaw() {
        return rotation.getYaw();
    }

    public double getRollDegrees() {
        return Math.toDegrees(getRoll());
    }

    public double getPitchDegrees() {
        return Math.toDegrees(getPitch());
    }

    public double getYawDegrees() {
        return Math.toDegrees(getYaw());
    }

    /**
     * Composes this pose with a transform expressed in this pose's own frame.
     * If this is the robot pose and {@code other} is ROBOT_TO_CAMERA, the result
     * is the camera's field pose; applying the inverse transform reverses it.
     */
    public Pose3d transformBy(Transform3d other) {
        Translation3d newTranslation = translation.plus(rotation.rotate(other.getTranslation()));
        Rotation3d newRotation = rotation.times(other.getRotation());
        return new Pose3d(newTranslation, newRotation);
    }

    /** Drops z, roll, and pitch to give the floor-plane pose used for driving. */
    public Pose2d toPose2d() {
        return new Pose2d(getX(), getY(), new Rotation2d(getYaw()));
    }

    @Override
    public String toString() {
        return String.format(
                "Pose3d(x %.2f, y %.2f, z %.2f, roll %.1f, pitch %.1f, yaw %.1f deg)",
                getX(), getY(), getZ(), getRollDegrees(), getPitchDegrees(), getYawDegrees());
    }
}
