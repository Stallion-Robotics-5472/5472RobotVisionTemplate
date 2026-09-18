package org.firstinspires.ftc.robotcore.external.navigation;
public class Pose2D {
    private final double x, y, headingRad;
    public Pose2D(DistanceUnit du, double x, double y, AngleUnit au, double h) {
        this.x = x; this.y = y;
        this.headingRad = au == AngleUnit.DEGREES ? Math.toRadians(h) : h;
    }
    public double getX(DistanceUnit u) { return x; }
    public double getY(DistanceUnit u) { return y; }
    public double getHeading(AngleUnit u) {
        return u == AngleUnit.DEGREES ? Math.toDegrees(headingRad) : headingRad;
    }
}
