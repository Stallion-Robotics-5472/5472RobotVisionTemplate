/*
 * A full 3D pose: translation (x, y, z) plus orientation as roll/pitch/yaw
 * (Tait-Bryan angles, radians). The Limelight's AprilTag botpose is inherently
 * 3D; the fused field estimate used for driving is 2D (the robot lives on the
 * floor), but z/pitch/roll are still useful for diagnostics such as detecting
 * tipping, ramps, or a bad camera-mount estimate.
 *
 * Original implementation for this template. Angle convention matches the FTC
 * SDK's YawPitchRollAngles and the field frame used elsewhere (yaw CCW about
 * +Z, distances in inches).
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Pose3d {
    private final double x;
    private final double y;
    private final double z;
    private final double roll;   // rotation about +X, radians
    private final double pitch;  // rotation about +Y, radians
    private final double yaw;    // rotation about +Z, radians

    public Pose3d() {
        this(0, 0, 0, 0, 0, 0);
    }

    public Pose3d(double x, double y, double z, double roll, double pitch, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getRoll() {
        return roll;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }

    public double getRollDegrees() {
        return Math.toDegrees(roll);
    }

    public double getPitchDegrees() {
        return Math.toDegrees(pitch);
    }

    public double getYawDegrees() {
        return Math.toDegrees(yaw);
    }

    /** Drops z, roll, and pitch to give the floor-plane pose used for driving. */
    public Pose2d toPose2d() {
        return new Pose2d(x, y, new Rotation2d(yaw));
    }

    @Override
    public String toString() {
        return String.format(
                "Pose3d(x %.2f, y %.2f, z %.2f, roll %.1f, pitch %.1f, yaw %.1f deg)",
                x, y, z, getRollDegrees(), getPitchDegrees(), getYawDegrees());
    }
}
