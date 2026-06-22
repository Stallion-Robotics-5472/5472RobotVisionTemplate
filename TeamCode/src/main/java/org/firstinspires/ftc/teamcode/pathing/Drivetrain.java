/*
 * Drivetrain abstraction the follower commands. The follower works entirely in
 * the field frame; the drivetrain is responsible for converting a field-centric
 * translation + turn request into actual motor outputs given the robot heading.
 * Original implementation for this template.
 */
package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

public interface Drivetrain {
    /**
     * @param fieldXPower translation power along field +X, in [-1, 1].
     * @param fieldYPower translation power along field +Y, in [-1, 1].
     * @param turnPower   rotational power, CCW positive, in [-1, 1].
     * @param robotHeading current robot heading (field frame).
     */
    void driveFieldCentric(double fieldXPower, double fieldYPower, double turnPower,
                           Rotation2d robotHeading);

    void stop();
}
