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
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

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
     * tag geometry. Must be called every loop, BEFORE reading the result, with
     * the robot's best heading estimate (degrees, field-relative).
     *
     * MegaTag2 uses this yaw to discard the mirror-image solution that makes a
     * single-tag MegaTag1 fix ambiguous. That also means MegaTag2 is only as
     * good as the yaw you give it: feed it a heading that is 180 degrees out
     * and it will return a confidently wrong position. Localization therefore
     * gates MegaTag2 behind a heading-trust check.
     */
    public void updateRobotOrientation(double yawDegrees) {
        limelight.updateRobotOrientation(yawDegrees);
    }

    /** Most recent pipeline result. May be null if polling hasn't produced one. */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * IDs of every AprilTag in the current frame.
     *
     * Separate from the pose solve: a tag can be seen without contributing a
     * usable botpose. Goal selection uses this to tell which goals the robot can
     * actually see. Returns an empty list when there is no frame.
     */
    public static List<Integer> visibleTagIds(LLResult result) {
        List<Integer> ids = new ArrayList<>();
        if (result == null || !result.isValid()) {
            return ids;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return ids;
        }
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            ids.add(fiducial.getFiducialId());
        }
        return ids;
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
