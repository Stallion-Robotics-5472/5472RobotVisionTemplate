/*
 * Thin wrapper around the goBILDA Pinpoint odometry computer.
 *
 * The Pinpoint fuses two dead-wheel encoders with its on-board IMU and reports
 * an absolute field pose. This class configures it from VisionConstants and
 * converts between the SDK's Pose2D (inches/radians via DistanceUnit/AngleUnit)
 * and our internal Pose2d geometry type.
 *
 * Configuration mirrors the FTC SensorGoBildaPinpoint sample.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

public class PinpointOdometry {
    private final GoBildaPinpointDriver pinpoint;

    public PinpointOdometry(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, VisionConstants.PINPOINT_NAME);
        configure();
    }

    private void configure() {
        pinpoint.setOffsets(
                VisionConstants.PINPOINT_X_OFFSET_MM,
                VisionConstants.PINPOINT_Y_OFFSET_MM,
                VisionConstants.PINPOINT_OFFSET_UNIT);
        pinpoint.setEncoderResolution(VisionConstants.PINPOINT_POD_TYPE);
        pinpoint.setEncoderDirections(
                VisionConstants.PINPOINT_X_DIRECTION,
                VisionConstants.PINPOINT_Y_DIRECTION);
        // Resets pose to (0,0,0) and recalibrates the IMU. Keep the robot still.
        pinpoint.resetPosAndIMU();
    }

    /** Reads fresh encoder + IMU data from the device. Call once per loop. */
    public void update() {
        pinpoint.update();
    }

    /** Latest fused pose, in inches and radians. Call {@link #update()} first. */
    public Pose2d getPose() {
        Pose2D pose = pinpoint.getPosition();
        return new Pose2d(
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                new Rotation2d(pose.getHeading(AngleUnit.RADIANS)));
    }

    /** Overrides the Pinpoint's internal pose (e.g. a known autonomous start). */
    public void setPose(Pose2d pose) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getRotation().getRadians()));
    }

    /** Direct access to the driver for telemetry or advanced use. */
    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }
}
