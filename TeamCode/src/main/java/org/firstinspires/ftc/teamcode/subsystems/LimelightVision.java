/*
 * Thin wrapper around the Limelight 3A vision sensor.
 *
 * Configures the device from VisionConstants and exposes just what the
 * localization layer needs: pushing the robot's current heading down to the
 * camera (required for MegaTag2) and pulling the latest result back up.
 *
 * Configuration mirrors the FTC SensorLimelight3A sample. See that sample's
 * header for how the Limelight presents itself as a USB/ethernet device and how
 * to name it in the robot configuration.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightVision {
    private final Limelight3A limelight;

    public LimelightVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);
        limelight.setPollRateHz(VisionConstants.LIMELIGHT_POLL_RATE_HZ);
        limelight.pipelineSwitch(VisionConstants.LIMELIGHT_PIPELINE);
        // Begin polling. Without start(), getLatestResult() returns null.
        limelight.start();
    }

    /**
     * Tells the Limelight which way the robot is facing so MegaTag2 can resolve
     * tag geometry. Must be called every loop, before reading the result, with
     * the robot's best heading estimate (degrees, field-relative).
     */
    public void updateRobotOrientation(double yawDegrees) {
        limelight.updateRobotOrientation(yawDegrees);
    }

    /** Most recent pipeline result. May be null if polling hasn't produced one. */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public void stop() {
        limelight.stop();
    }

    public Limelight3A getDevice() {
        return limelight;
    }
}
