package com.qualcomm.hardware.limelightvision;

/** Stub Limelight3A. Tests set {@link #nextResult} to feed synthetic frames. */
public class Limelight3A {
    public LLResult nextResult = null;
    public double lastYawPushed = Double.NaN;

    public void setPollRateHz(int hz) {}
    public void pipelineSwitch(int i) {}
    public void start() {}
    public void stop() {}
    public void updateRobotOrientation(double yaw) { lastYawPushed = yaw; }
    public LLResult getLatestResult() { return nextResult; }
    public LLStatus getStatus() { return new LLStatus(); }
}
