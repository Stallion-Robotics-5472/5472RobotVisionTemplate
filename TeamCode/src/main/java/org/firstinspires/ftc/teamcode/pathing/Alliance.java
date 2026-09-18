/*
 * Which alliance you are playing, and what that changes.
 *
 * It deliberately changes only two things:
 *
 *   1. The driver's point of view in TeleOp. The field coordinate system is
 *      absolute and identical for both alliances, but the two drive teams stand
 *      at opposite ends of the field, so "push the stick away from me" points in
 *      opposite field directions. {@link #driverForward()} supplies that offset.
 *
 *   2. Which side of the field an auto runs on, via {@link AllianceFlip}.
 *
 * It does NOT change the pose frame. Poses, paths and AprilTag results are
 * always in the one absolute field frame (origin at field center, +X right,
 * +Y away from the audience, heading CCW). A robot sitting at a given spot
 * reports the same pose whichever alliance it belongs to.
 *
 * Original implementation for this template.
 */
package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

public enum Alliance {
    RED,
    BLUE;

    /**
     * The field-frame direction the drive team faces when looking at the field,
     * i.e. the field direction the robot travels when the driver pushes the
     * stick straight away from themselves.
     *
     * Derived from the BIOBUZZ ALLIANCE AREA positions: the red area spans
     * x -125.5..-71.5 (outside the -X wall) so red drivers look toward +X, and
     * the blue area spans x 71.5..125.5 so blue drivers look toward -X.
     *
     * If your drive team stands somewhere else, this is the one value to change.
     */
    public Rotation2d driverForward() {
        return this == RED ? new Rotation2d(0.0) : new Rotation2d(Math.PI);
    }

    public Alliance opposite() {
        return this == RED ? BLUE : RED;
    }

    /**
     * True when a plan authored for {@code authoredFor} needs flipping to run
     * on this alliance.
     */
    public boolean needsFlipFrom(Alliance authoredFor) {
        return this != authoredFor;
    }
}
