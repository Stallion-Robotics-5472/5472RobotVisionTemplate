package com.qualcomm.hardware.limelightvision;

/** Stub LLResultTypes. Only the fiducial result is modelled. */
public class LLResultTypes {
    /** One AprilTag detection. Fields are public so tests can build one. */
    public static class FiducialResult {
        public int fiducialId;
        public String family = "tag36h11";
        public double targetXDegrees, targetYDegrees;

        public FiducialResult() {}
        public FiducialResult(int id) { this.fiducialId = id; }

        public int getFiducialId() { return fiducialId; }
        public String getFamily() { return family; }
        public double getTargetXDegrees() { return targetXDegrees; }
        public double getTargetYDegrees() { return targetYDegrees; }
    }
}
