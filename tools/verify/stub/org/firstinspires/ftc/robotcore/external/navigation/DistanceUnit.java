package org.firstinspires.ftc.robotcore.external.navigation;

/** Stub DistanceUnit with real conversions, so unit handling is actually tested. */
public enum DistanceUnit {
    METER(39.3700787402),
    CM(0.393700787402),
    MM(0.0393700787402),
    INCH(1.0);

    private final double inchesPerUnit;

    DistanceUnit(double inchesPerUnit) {
        this.inchesPerUnit = inchesPerUnit;
    }

    public double toInches(double value) {
        return value * inchesPerUnit;
    }

    public double fromInches(double inches) {
        return inches / inchesPerUnit;
    }

    public double toMeters(double value) {
        return toInches(value) / 39.3700787402;
    }
}
