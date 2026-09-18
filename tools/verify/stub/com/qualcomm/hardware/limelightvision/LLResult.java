package com.qualcomm.hardware.limelightvision;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Stub LLResult. Fields are public so tests can synthesise a frame; the real
 * SDK class is read-only and populated from the camera.
 */
public class LLResult {
    public boolean valid = true;
    public int tagCount = 2;
    public long staleness = 0;
    public Pose3D botpose;
    public Pose3D botposeMt2;
    public double avgDist = 30.0;
    public double captureLatency = 10.0;
    public double targetingLatency = 5.0;

    public boolean isValid() { return valid; }
    public int getBotposeTagCount() { return tagCount; }
    public long getStaleness() { return staleness; }
    public Pose3D getBotpose() { return botpose; }
    public Pose3D getBotpose_MT2() { return botposeMt2; }
    public double getBotposeAvgDist() { return avgDist; }
    public double getCaptureLatency() { return captureLatency; }
    public double getTargetingLatency() { return targetingLatency; }
}
