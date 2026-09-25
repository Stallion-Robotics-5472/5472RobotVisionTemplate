/*
 * Everything the shooter needs to know for one shot: how fast to spin, where to
 * put the hood, and how long the game piece will be in the air.
 *
 * The time of flight is the part that makes shooting on the move possible -- it
 * is what tells the aiming math how far the robot will carry the piece sideways
 * before it lands. See AimLogic.
 */
package org.firstinspires.ftc.teamcode.shooting;

public final class ShooterSetpoint {
    /** Flywheel speed, revolutions per minute. */
    public final double flywheelRpm;

    /** Hood angle, degrees. What this means mechanically is up to your build. */
    public final double hoodDegrees;

    /** How long the game piece spends in the air, seconds. */
    public final double timeOfFlightSeconds;

    public ShooterSetpoint(double flywheelRpm, double hoodDegrees, double timeOfFlightSeconds) {
        this.flywheelRpm = flywheelRpm;
        this.hoodDegrees = hoodDegrees;
        this.timeOfFlightSeconds = timeOfFlightSeconds;
    }

    /** This setpoint with operator trim added, for dialling a shot in mid-match. */
    public ShooterSetpoint withTrim(double rpmTrim, double hoodTrim) {
        return new ShooterSetpoint(
                flywheelRpm + rpmTrim, hoodDegrees + hoodTrim, timeOfFlightSeconds);
    }

    /** Linear blend toward {@code other}, used by ShooterMap to interpolate. */
    public ShooterSetpoint interpolate(ShooterSetpoint other, double t) {
        return new ShooterSetpoint(
                flywheelRpm + t * (other.flywheelRpm - flywheelRpm),
                hoodDegrees + t * (other.hoodDegrees - hoodDegrees),
                timeOfFlightSeconds
                        + t * (other.timeOfFlightSeconds - timeOfFlightSeconds));
    }

    @Override
    public String toString() {
        return String.format("rpm %.0f  hood %.1f deg  tof %.3f s",
                flywheelRpm, hoodDegrees, timeOfFlightSeconds);
    }
}
