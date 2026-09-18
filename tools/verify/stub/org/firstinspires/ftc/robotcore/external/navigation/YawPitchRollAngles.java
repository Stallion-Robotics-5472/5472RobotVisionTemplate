package org.firstinspires.ftc.robotcore.external.navigation;
public class YawPitchRollAngles {
    private final double yawRad, pitchRad, rollRad;
    public YawPitchRollAngles() { this(0, 0, 0); }
    public YawPitchRollAngles(double yawRad, double pitchRad, double rollRad) {
        this.yawRad = yawRad; this.pitchRad = pitchRad; this.rollRad = rollRad;
    }
    private static double as(double rad, AngleUnit u) {
        return u == AngleUnit.DEGREES ? Math.toDegrees(rad) : rad;
    }
    public double getYaw(AngleUnit u) { return as(yawRad, u); }
    public double getPitch(AngleUnit u) { return as(pitchRad, u); }
    public double getRoll(AngleUnit u) { return as(rollRad, u); }
}
