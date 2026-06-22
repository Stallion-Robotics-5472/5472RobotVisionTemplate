/*
 * A 3D translation (x, y, z) in inches. Part of the minimal SE(3) layer used to
 * convert the Limelight's 3D camera pose into a robot-center pose when the
 * camera is mounted with a height/pitch/roll offset. Original implementation.
 */
package org.firstinspires.ftc.teamcode.lib.geometry;

public class Translation3d {
    private final double x;
    private final double y;
    private final double z;

    public Translation3d() {
        this(0, 0, 0);
    }

    public Translation3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
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

    public Translation3d plus(Translation3d o) {
        return new Translation3d(x + o.x, y + o.y, z + o.z);
    }

    public Translation3d minus(Translation3d o) {
        return new Translation3d(x - o.x, y - o.y, z - o.z);
    }

    public Translation3d unaryMinus() {
        return new Translation3d(-x, -y, -z);
    }

    public Translation3d times(double s) {
        return new Translation3d(x * s, y * s, z * s);
    }

    /** Applies a 3D rotation to this translation. */
    public Translation3d rotateBy(Rotation3d r) {
        return r.rotate(this);
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(x, y);
    }

    @Override
    public String toString() {
        return String.format("Translation3d(%.2f, %.2f, %.2f)", x, y, z);
    }
}
