/*
 * How a field maps onto itself when you swap alliances.
 *
 * This is the single most important thing to get right when reusing one auto
 * for both alliances: applying the wrong symmetry puts the mirrored path in
 * the wrong place, and it will look almost-correct on a square field.
 *
 * For the 2026-2027 BIOBUZZ field the answer is ROTATIONAL. Verified against
 * the field element coordinates in the Competition Manual: rotating the red
 * GARDEN (x -70.5..-47.4, y -70.0..-68.0) and the red LOADING ZONE
 * (x -70.5..-59.5, y 23.9..46.6) by 180 degrees about the field center lands
 * exactly on their blue counterparts, while neither mirror does.
 *
 * Original implementation for this template.
 */
package org.firstinspires.ftc.teamcode.pathing;

public enum FieldSymmetry {
    /**
     * 180 degree rotation about the field center: (x, y) -> (-x, -y),
     * heading -> heading + 180. This is the correct transform for BIOBUZZ and
     * for most recent FTC fields, whose alliance halves are rotations of each
     * other rather than reflections.
     */
    ROTATIONAL,

    /**
     * Reflection across the Y axis: (x, y) -> (-x, y), heading -> 180 - heading.
     * Only correct on a field whose two alliance halves are genuine left/right
     * mirror images. Note this reverses handedness, so a path that curved left
     * will curve right.
     */
    MIRROR_X,

    /**
     * Reflection across the X axis: (x, y) -> (x, -y), heading -> -heading.
     * Also reverses handedness.
     */
    MIRROR_Y;

    /** Transformed X coordinate (inches). */
    public double x(double x, double y) {
        switch (this) {
            case MIRROR_X:   return -x;
            case MIRROR_Y:   return x;
            case ROTATIONAL:
            default:         return -x;
        }
    }

    /** Transformed Y coordinate (inches). */
    public double y(double x, double y) {
        switch (this) {
            case MIRROR_X:   return y;
            case MIRROR_Y:   return -y;
            case ROTATIONAL:
            default:         return -y;
        }
    }

    /** Transformed heading (radians), wrapped to [-pi, pi]. */
    public double heading(double radians) {
        switch (this) {
            case MIRROR_X:   return Path.shortestAngle(Math.PI - radians);
            case MIRROR_Y:   return Path.shortestAngle(-radians);
            case ROTATIONAL:
            default:         return Path.shortestAngle(radians + Math.PI);
        }
    }
}
