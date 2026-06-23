/*
 * Tuning constants for the path follower and mecanum drivetrain.
 *
 * The PIDF gains are expressed against field units (inches, radians) and output
 * motor power in [-1, 1]. The defaults are sane starting points; every robot
 * must tune them. Original implementation for this template.
 */
package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class PathConstants {
    private PathConstants() {}

    // ----- Drivetrain motor names (match your robot configuration) -----
    public static final String FRONT_LEFT_MOTOR = "fl";
    public static final String FRONT_RIGHT_MOTOR = "fr";
    public static final String BACK_LEFT_MOTOR = "bl";
    public static final String BACK_RIGHT_MOTOR = "br";

    // Reverse the side whose wheels spin backwards for positive power.
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // ----- Translational controller: pulls the robot onto the path (per inch) -----
    public static final double TRANSLATIONAL_kP = 0.04;
    public static final double TRANSLATIONAL_kI = 0.0;
    public static final double TRANSLATIONAL_kD = 0.010;
    public static final double TRANSLATIONAL_kF = 0.0;

    // ----- Drive controller: moves the robot along the path (per inch remaining) -----
    public static final double DRIVE_kP = 0.040;
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.010;
    public static final double DRIVE_kF = 0.0;

    // ----- Heading controller: holds the target heading (per radian) -----
    public static final double HEADING_kP = 0.30;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.08;
    public static final double HEADING_kF = 0.0;

    /**
     * Centripetal scaling. The centripetal correction magnitude is
     * CENTRIPETAL_SCALE * speed^2 * curvature, nudging the robot toward the
     * inside of a curve so it doesn't drift wide. Keep small; tune up if the
     * robot cuts corners on the outside.
     */
    public static final double CENTRIPETAL_SCALE = 0.0006;

    // ----- Completion tolerances -----
    /** Position tolerance to consider the final path finished (inches). */
    public static final double END_TRANSLATION_TOLERANCE = 1.0;
    /** Heading tolerance to consider finished (radians). */
    public static final double END_HEADING_TOLERANCE = Math.toRadians(2.0);
    /** Speed below which the robot is considered settled (inches/second). */
    public static final double END_VELOCITY_TOLERANCE = 2.0;

    /**
     * When following a chain, advance to the next path once the remaining length
     * of the current path drops below this (inches) or t passes ADVANCE_T.
     */
    public static final double ADVANCE_LENGTH_TOLERANCE = 2.0;
    public static final double ADVANCE_T = 0.99;
}
