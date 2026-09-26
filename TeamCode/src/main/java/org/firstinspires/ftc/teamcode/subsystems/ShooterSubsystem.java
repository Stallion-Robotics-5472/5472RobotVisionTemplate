/*
 * Flywheel + hood + feeder, driven from the shot table.
 *
 * Ported from team 5472's FRC shooter subsystem. Same shape: a velocity-
 * controlled flywheel, a positional hood, a distance-indexed shot map, and
 * readiness checks that gate the feeder.
 *
 * The only method aiming code needs is {@link #setShotForDistance}: hand it the
 * effective distance from the aiming solution and it looks up and applies the
 * flywheel speed and hood angle.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.shooting.ShooterMap;
import org.firstinspires.ftc.teamcode.shooting.ShooterSetpoint;
import org.firstinspires.ftc.teamcode.lib.util.RobotClock;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx flywheel;
    /** Null when the robot has a single-motor flywheel. */
    private final DcMotorEx flywheelFollower;
    private final Servo hood;
    private final DcMotor feeder;

    private final ShooterMap map;

    private double flywheelSetpointRpm = 0.0;
    private double hoodSetpointDeg = ShootingConstants.HOOD_STOWED_DEG;
    private double feederPower = 0.0;

    // Operator trim, dialled in from the gamepad and applied on top of the map.
    private double rpmTrim = 0.0;
    private double hoodTrimDeg = 0.0;

    /** When true the map is bypassed so setpoints can be dialled in by hand. */
    private boolean manualMode = false;

    // Shortfall tracking: a flywheel that cannot reach the speed it was asked for
    // is the failure that looks like bad aim. The shot leaves short, the solution
    // and the pose were both fine, and nothing in the logs says why -- unless the
    // shooter itself notices it never got there and says so.
    private double setpointChangedAt = Double.NEGATIVE_INFINITY;
    private double worstShortfallRpm = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, ShootingConstants.SHOT_MAP);
    }

    public ShooterSubsystem(HardwareMap hardwareMap, ShooterMap map) {
        this.map = map;

        flywheel = hardwareMap.get(DcMotorEx.class, ShootingConstants.FLYWHEEL_MOTOR);
        flywheel.setDirection(ShootingConstants.FLYWHEEL_DIRECTION);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // A flywheel must coast: braking it fights its own inertia and wastes
        // the stored energy that makes the next shot repeatable.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setVelocityPIDFCoefficients(
                ShootingConstants.FLYWHEEL_P, ShootingConstants.FLYWHEEL_I,
                ShootingConstants.FLYWHEEL_D, ShootingConstants.FLYWHEEL_F);

        if (ShootingConstants.FLYWHEEL_FOLLOWER_MOTOR.isEmpty()) {
            flywheelFollower = null;
        } else {
            flywheelFollower = hardwareMap.get(
                    DcMotorEx.class, ShootingConstants.FLYWHEEL_FOLLOWER_MOTOR);
            flywheelFollower.setDirection(ShootingConstants.FLYWHEEL_FOLLOWER_DIRECTION);
            flywheelFollower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelFollower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheelFollower.setVelocityPIDFCoefficients(
                    ShootingConstants.FLYWHEEL_P, ShootingConstants.FLYWHEEL_I,
                    ShootingConstants.FLYWHEEL_D, ShootingConstants.FLYWHEEL_F);
        }

        hood = hardwareMap.get(Servo.class, ShootingConstants.HOOD_SERVO);
        feeder = hardwareMap.get(DcMotor.class, ShootingConstants.FEEDER_MOTOR);
        feeder.setDirection(ShootingConstants.FEEDER_DIRECTION);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stow();
    }

    // ---------------------------------------------------------------------
    // Unit conversion
    // ---------------------------------------------------------------------

    private static double rpmToTicksPerSecond(double rpm) {
        return rpm / 60.0
                * ShootingConstants.FLYWHEEL_TICKS_PER_REV
                / ShootingConstants.FLYWHEEL_GEAR_RATIO;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0
                / ShootingConstants.FLYWHEEL_TICKS_PER_REV
                * ShootingConstants.FLYWHEEL_GEAR_RATIO;
    }

    /** Maps a hood angle onto the servo's 0..1 travel, clamped to its limits. */
    private static double hoodDegreesToServo(double degrees) {
        double span = ShootingConstants.HOOD_DEG_AT_SERVO_1
                - ShootingConstants.HOOD_DEG_AT_SERVO_0;
        if (Math.abs(span) < 1e-9) {
            return 0.0;
        }
        double position = (degrees - ShootingConstants.HOOD_DEG_AT_SERVO_0) / span;
        return Math.max(0.0, Math.min(1.0, position));
    }

    // ---------------------------------------------------------------------
    // Commands from the aiming layer
    // ---------------------------------------------------------------------

    /**
     * Spins up and sets the hood for a shot at {@code distanceInches}, applying
     * operator trim. Pass the aiming solution's EFFECTIVE distance, not the
     * straight-line distance to the goal -- that is the whole point of the
     * moving-shot correction.
     *
     * Does nothing in manual mode, so a tuning session is not fought by the map.
     */
    public void setShotForDistance(double distanceInches) {
        setShotFrom(map, distanceInches);
    }

    /**
     * Same, but from a specific shot table. Goals at different heights need
     * different curves, so the aiming layer passes the selected goal's own map.
     */
    public void setShotFrom(ShooterMap shotMap, double distanceInches) {
        if (manualMode) {
            return;
        }
        ShooterSetpoint setpoint =
                shotMap.setpointAt(distanceInches).withTrim(rpmTrim, hoodTrimDeg);
        setFlywheelRpm(setpoint.flywheelRpm);
        setHoodDegrees(setpoint.hoodDegrees);
    }

    public void setFlywheelRpm(double rpm) {
        // Only a change bigger than the tolerance restarts the clock. Tracking a
        // slowly changing distance nudges the setpoint every loop, and treating
        // each nudge as a new target would reset the grace period forever.
        if (Math.abs(rpm - flywheelSetpointRpm) > ShootingConstants.FLYWHEEL_TOLERANCE_RPM) {
            setpointChangedAt = RobotClock.nowSeconds();
        }
        flywheelSetpointRpm = rpm;
        double ticks = rpmToTicksPerSecond(rpm);
        flywheel.setVelocity(ticks);
        if (flywheelFollower != null) {
            flywheelFollower.setVelocity(ticks);
        }
    }

    public void setHoodDegrees(double degrees) {
        hoodSetpointDeg = degrees;
        hood.setPosition(hoodDegreesToServo(degrees));
    }

    /** Holds the flywheel at idle speed with the hood stowed. */
    public void idle() {
        setFlywheelRpm(ShootingConstants.FLYWHEEL_IDLE_RPM);
        setHoodDegrees(ShootingConstants.HOOD_STOWED_DEG);
        stopFeeder();
    }

    /** Spins the flywheel down and stows the hood. */
    public void stow() {
        setFlywheelRpm(0.0);
        setHoodDegrees(ShootingConstants.HOOD_STOWED_DEG);
        stopFeeder();
    }

    public void runFeeder() {
        feederPower = ShootingConstants.FEEDER_SHOOT_POWER;
        feeder.setPower(feederPower);
    }

    public void stopFeeder() {
        feederPower = 0.0;
        feeder.setPower(0.0);
    }

    /** Stops everything. */
    public void stop() {
        setFlywheelRpm(0.0);
        stopFeeder();
    }

    // ---------------------------------------------------------------------
    // Readiness
    // ---------------------------------------------------------------------

    /** Current average flywheel speed, rpm. */
    public double getFlywheelRpm() {
        double ticks = flywheel.getVelocity();
        if (flywheelFollower != null) {
            ticks = (ticks + flywheelFollower.getVelocity()) / 2.0;
        }
        return ticksPerSecondToRpm(ticks);
    }

    public double getFlywheelSetpointRpm() {
        return flywheelSetpointRpm;
    }

    public double getHoodSetpointDegrees() {
        return hoodSetpointDeg;
    }

    /**
     * True when the flywheel is commanded to a real speed and has reached it.
     *
     * A shoot command should gate on this. Feeding a piece into a flywheel that
     * is still spinning up produces a short shot, and the recovery afterwards
     * makes the next one wrong too.
     */
    public boolean atSpeed() {
        return flywheelSetpointRpm > 0
                && Math.abs(getFlywheelRpm() - flywheelSetpointRpm)
                        < ShootingConstants.FLYWHEEL_TOLERANCE_RPM;
    }

    /**
     * True when the flywheel has been asked for a speed it is not reaching.
     *
     * Distinct from "spinning up": the grace period has already passed and the
     * wheel is still short. Causes, in order of likelihood: a low battery, a shot
     * table whose far end asks for more than the motor can hold, a slipping belt,
     * or FLYWHEEL_F set too low to hold speed under load.
     *
     * Only reports being SHORT. A wheel running fast is a different problem and not
     * one that quietly shortens shots.
     */
    public boolean isStruggling() {
        if (flywheelSetpointRpm <= 0) {
            return false;
        }
        if (RobotClock.nowSeconds() - setpointChangedAt
                < ShootingConstants.FLYWHEEL_SPINUP_GRACE_SECONDS) {
            return false;       // still fairly entitled to be spinning up
        }
        return flywheelSetpointRpm - getFlywheelRpm()
                > ShootingConstants.FLYWHEEL_TOLERANCE_RPM;
    }

    /** How far short the flywheel is right now, rpm. Zero or negative when fine. */
    public double getShortfallRpm() {
        return flywheelSetpointRpm <= 0 ? 0.0 : flywheelSetpointRpm - getFlywheelRpm();
    }

    /** The worst shortfall seen since the last {@link #clearShortfallRecord()}. */
    public double getWorstShortfallRpm() {
        return worstShortfallRpm;
    }

    public void clearShortfallRecord() {
        worstShortfallRpm = 0.0;
    }

    /**
     * Updates the shortfall record. Called from {@link #addTelemetry}, and worth
     * calling once per loop from a subsystem periodic if you want the record to
     * cover loops where telemetry was not drawn.
     */
    public void trackShortfall() {
        if (isStruggling()) {
            worstShortfallRpm = Math.max(worstShortfallRpm, getShortfallRpm());
        }
    }

    // ---------------------------------------------------------------------
    // Trim and manual mode, for tuning
    // ---------------------------------------------------------------------

    public void addRpmTrim(double delta) {
        rpmTrim += delta;
    }

    public void addHoodTrim(double deltaDegrees) {
        hoodTrimDeg += deltaDegrees;
    }

    public void clearTrim() {
        rpmTrim = 0.0;
        hoodTrimDeg = 0.0;
    }

    public double getRpmTrim() {
        return rpmTrim;
    }

    public double getHoodTrim() {
        return hoodTrimDeg;
    }

    /** In manual mode the shot map is ignored; used by the tuning OpMode. */
    public void setManualMode(boolean manual) {
        manualMode = manual;
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public ShooterMap getMap() {
        return map;
    }

    /** What the map alone would ask for at this distance, ignoring trim. */
    public ShooterSetpoint mapSetpointAt(double distanceInches) {
        return map.setpointAt(distanceInches);
    }

    // ---------------------------------------------------------------------

    public void addTelemetry(Telemetry telemetry) {
        trackShortfall();
        telemetry.addData("Flywheel", "%.0f / %.0f rpm  %s",
                getFlywheelRpm(), flywheelSetpointRpm, atSpeed() ? "AT SPEED" : "spinning");
        if (isStruggling()) {
            telemetry.addLine(String.format(
                    "*** FLYWHEEL %.0f RPM SHORT - check battery, or the far end of"
                            + " the shot table is beyond it ***", getShortfallRpm()));
        } else if (worstShortfallRpm > 0.0) {
            telemetry.addData("Flywheel was short by", "%.0f rpm earlier",
                    worstShortfallRpm);
        }
        telemetry.addData("Hood", "%.1f deg", hoodSetpointDeg);
        telemetry.addData("Feeder", "%.2f", feederPower);
        if (rpmTrim != 0.0 || hoodTrimDeg != 0.0) {
            telemetry.addData("Trim", "rpm %+.0f  hood %+.1f deg", rpmTrim, hoodTrimDeg);
        }
        if (manualMode) {
            telemetry.addLine("SHOOTER IN MANUAL MODE (shot map bypassed)");
        }
    }
}
