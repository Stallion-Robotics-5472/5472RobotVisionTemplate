package org.firstinspires.ftc.robotcore.external.navigation;
public class Position {
    public double x, y, z;
    public Position() {}
    public Position(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    /** The stub works in inches throughout, so conversion is a no-op. */
    public Position toUnit(DistanceUnit u) { return this; }
}
