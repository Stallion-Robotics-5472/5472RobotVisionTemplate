package com.qualcomm.hardware.limelightvision;

/** Stub LLResultTypes. Only the fiducial result is modelled. */
public class LLResultTypes {
    /**
     * One AprilTag detection. Fields are public so tests can build one.
     *
     * Mirrors the subset of the real API this codebase uses: id, the crosshair
     * offsets in degrees (tx positive RIGHT, ty positive UP) and the target area.
     */
    public static class FiducialResult {
        public int fiducialId;
        public String family = "tag36h11";
        public double targetXDegrees, targetYDegrees;
        public double targetArea = 1.0;

        public FiducialResult() {}
        public FiducialResult(int id) { this.fiducialId = id; }

        /** Convenience for tests: a detection at a known bearing and elevation. */
        public FiducialResult(int id, double txDegrees, double tyDegrees, double area) {
            this.fiducialId = id;
            this.targetXDegrees = txDegrees;
            this.targetYDegrees = tyDegrees;
            this.targetArea = area;
        }

        public int getFiducialId() { return fiducialId; }
        public String getFamily() { return family; }
        public double getTargetXDegrees() { return targetXDegrees; }
        public double getTargetYDegrees() { return targetYDegrees; }
        public double getTargetArea() { return targetArea; }
    }
}
