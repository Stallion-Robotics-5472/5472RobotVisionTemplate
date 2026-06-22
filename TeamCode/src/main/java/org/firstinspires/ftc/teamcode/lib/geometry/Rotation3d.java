/*
 * A 3D rotation backed by a 3x3 rotation matrix, built from Tait-Bryan angles
 * (roll about X, pitch about Y, yaw about Z) using the ZYX convention
 * R = Rz(yaw) * Ry(pitch) * Rx(roll). This matches the FTC SDK's
 * YawPitchRollAngles ordering. Part of the minimal SE(3) layer for handling an
 * off-center, tilted, raised camera. Original implementation.
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Rotation3d {
    // Row-major 3x3 rotation matrix.
    private final double[][] m;

    private Rotation3d(double[][] matrix) {
        this.m = matrix;
    }

    public Rotation3d() {
        this(identity());
    }

    /** Builds a rotation from roll/pitch/yaw in radians (ZYX / yaw-pitch-roll). */
    public Rotation3d(double roll, double pitch, double yaw) {
        this(fromRpyMatrix(roll, pitch, yaw));
    }

    public static Rotation3d fromDegrees(double rollDeg, double pitchDeg, double yawDeg) {
        return new Rotation3d(Math.toRadians(rollDeg), Math.toRadians(pitchDeg), Math.toRadians(yawDeg));
    }

    private static double[][] identity() {
        return new double[][]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    }

    private static double[][] fromRpyMatrix(double roll, double pitch, double yaw) {
        double ca = Math.cos(roll), sa = Math.sin(roll);
        double cb = Math.cos(pitch), sb = Math.sin(pitch);
        double cc = Math.cos(yaw), sc = Math.sin(yaw);
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        return new double[][]{
                {cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa},
                {sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa},
                {-sb,     cb * sa,                cb * ca}
        };
    }

    /** Composition: this rotation followed by applying {@code other}. */
    public Rotation3d times(Rotation3d other) {
        double[][] a = this.m, b = other.m, r = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
        return new Rotation3d(r);
    }

    /** Inverse of a rotation matrix is its transpose. */
    public Rotation3d inverse() {
        double[][] t = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                t[i][j] = m[j][i];
            }
        }
        return new Rotation3d(t);
    }

    /** Rotates a 3D vector by this rotation (R * v). */
    public Translation3d rotate(Translation3d v) {
        double x = v.getX(), y = v.getY(), z = v.getZ();
        return new Translation3d(
                m[0][0] * x + m[0][1] * y + m[0][2] * z,
                m[1][0] * x + m[1][1] * y + m[1][2] * z,
                m[2][0] * x + m[2][1] * y + m[2][2] * z);
    }

    /** Yaw (rotation about +Z), radians, extracted from the matrix (ZYX). */
    public double getYaw() {
        // gimbal-lock guard when pitch ~ +/-90 deg (m[2][0] ~ -/+1)
        if (Math.abs(m[2][0]) > 1 - 1e-9) {
            return Math.atan2(-m[0][1], m[1][1]);
        }
        return Math.atan2(m[1][0], m[0][0]);
    }

    /** Pitch (rotation about +Y), radians. */
    public double getPitch() {
        double s = -m[2][0];
        s = Math.max(-1.0, Math.min(1.0, s));
        return Math.asin(s);
    }

    /** Roll (rotation about +X), radians. */
    public double getRoll() {
        if (Math.abs(m[2][0]) > 1 - 1e-9) {
            return 0.0;
        }
        return Math.atan2(m[2][1], m[2][2]);
    }

    public Rotation2d toRotation2d() {
        return new Rotation2d(getYaw());
    }

    @Override
    public String toString() {
        return String.format("Rotation3d(roll %.1f, pitch %.1f, yaw %.1f deg)",
                Math.toDegrees(getRoll()), Math.toDegrees(getPitch()), Math.toDegrees(getYaw()));
    }
}
