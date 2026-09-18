package org.firstinspires.ftc.robotcore.external.navigation;
public class Pose3D {
    private final Position position;
    private final YawPitchRollAngles orientation;
    public Pose3D() { this(new Position(), new YawPitchRollAngles()); }
    public Pose3D(Position p, YawPitchRollAngles o) { position = p; orientation = o; }
    /** Convenience for tests: x/y in inches, yaw in radians. */
    public Pose3D(double x, double y, double yawRad) {
        this(new Position(x, y, 0), new YawPitchRollAngles(yawRad, 0, 0));
    }
    public Position getPosition() { return position; }
    public YawPitchRollAngles getOrientation() { return orientation; }
}
