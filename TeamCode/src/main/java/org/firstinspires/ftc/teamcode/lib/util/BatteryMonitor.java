/*
 * Watches the battery, because almost every "the robot was fine in practice"
 * failure is a voltage story.
 *
 * A 12 V pack at 13.1 V and the same pack at 11.4 V are different robots. The
 * drivetrain accelerates slower, so the follower's braking distance is wrong. The
 * flywheel takes longer to spin up and, at the far end of the shot table, may not
 * reach the setpoint at all. None of that announces itself: the code runs exactly
 * the same and the robot simply misses.
 *
 * What this does NOT do is scale any setpoint. The flywheel runs closed-loop --
 * setVelocity plus the motor controller's PIDF holds rpm, and holding rpm against
 * a falling battery is precisely that loop's job. Multiplying the setpoint by a
 * voltage ratio would be compensating a second time for something already handled,
 * and would make the commanded rpm wrong rather than right. So this measures and
 * reports; the shot table stays in rpm and stays honest.
 *
 * Sampling rate matters. On a Control Hub, reading a VoltageSensor is a bus
 * transaction, not a field read -- doing it every loop spends real milliseconds on
 * a number that moves over seconds. So it is polled on an interval and the last
 * value reused, which keeps the cost off the control loop. See LoopTimer for why
 * that is worth being careful about.
 */
package org.firstinspires.ftc.teamcode.lib.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public final class BatteryMonitor {

    /** Below this, expect the robot to feel sluggish and shots to fall short. */
    public static final double LOW_VOLTAGE = 11.5;

    /** Below this, the Control Hub itself is close to browning out. */
    public static final double CRITICAL_VOLTAGE = 11.0;

    /** A dip this far below the recent average counts as a sag. */
    public static final double SAG_VOLTAGE = 1.0;

    /** How often the sensor is actually read. The battery does not move faster. */
    public static final double SAMPLE_INTERVAL_SECONDS = 0.25;

    /** Seconds the running average covers. */
    private static final double AVERAGE_WINDOW_SECONDS = 3.0;

    private final VoltageSensor sensor;

    private double volts = Double.NaN;
    private double average = Double.NaN;
    private double minimum = Double.NaN;
    private double lastSample = Double.NEGATIVE_INFINITY;
    private long samples = 0;

    private BatteryMonitor(VoltageSensor sensor) {
        this.sensor = sensor;
    }

    /**
     * The robot's first voltage sensor, or an inert monitor if there is none.
     *
     * Never throws: a missing sensor is not a reason for an OpMode to refuse to
     * run, and every reader here handles NaN.
     */
    public static BatteryMonitor from(HardwareMap hardwareMap) {
        try {
            List<VoltageSensor> sensors = hardwareMap.getAll(VoltageSensor.class);
            return new BatteryMonitor(
                    sensors == null || sensors.isEmpty() ? null : sensors.get(0));
        } catch (RuntimeException e) {
            return new BatteryMonitor(null);
        }
    }

    /** Wraps a sensor directly, for tests and for robots with several packs. */
    public static BatteryMonitor of(VoltageSensor sensor) {
        return new BatteryMonitor(sensor);
    }

    /**
     * Call once per loop. Reads the sensor only when the interval has elapsed, so
     * calling it every loop costs a comparison on most loops.
     */
    public void update() {
        update(RobotClock.nowSeconds());
    }

    /** As {@link #update()}, with an explicit timestamp (for offline simulation). */
    public void update(double nowSeconds) {
        if (sensor == null || nowSeconds - lastSample < SAMPLE_INTERVAL_SECONDS) {
            return;
        }
        lastSample = nowSeconds;
        double reading;
        try {
            reading = sensor.getVoltage();
        } catch (RuntimeException e) {
            return;             // a bus hiccup is not worth taking the OpMode down
        }
        if (!(reading > 0.0)) {
            return;             // a disconnected sensor reads 0; that is not a battery
        }
        samples++;
        volts = reading;
        if (Double.isNaN(minimum) || reading < minimum) {
            minimum = reading;
        }
        if (Double.isNaN(average)) {
            average = reading;
        } else {
            // One pole, weighted by how much of the window one sample represents.
            double alpha = SAMPLE_INTERVAL_SECONDS / AVERAGE_WINDOW_SECONDS;
            average += alpha * (reading - average);
        }
    }

    /** Last reading, or NaN before the first sample. */
    public double getVolts() {
        return volts;
    }

    /** Running average over the last few seconds. */
    public double getAverageVolts() {
        return average;
    }

    /** Lowest reading seen this run -- what the robot dipped to under load. */
    public double getMinimumVolts() {
        return minimum;
    }

    public long getSampleCount() {
        return samples;
    }

    public boolean hasSensor() {
        return sensor != null;
    }

    /** True once the pack is low enough to change how the robot behaves. */
    public boolean isLow() {
        return !Double.isNaN(volts) && volts < LOW_VOLTAGE;
    }

    /** True when the Control Hub is close to browning out. */
    public boolean isCritical() {
        return !Double.isNaN(volts) && volts < CRITICAL_VOLTAGE;
    }

    /**
     * True when the reading has dropped well below its own recent average --
     * something on the robot is drawing hard, or a connector is marginal.
     */
    public boolean isSagging() {
        return !Double.isNaN(volts) && !Double.isNaN(average)
                && average - volts > SAG_VOLTAGE;
    }

    /** Null when nothing is wrong, otherwise one line saying what. */
    public String getWarning() {
        if (sensor == null) {
            return null;
        }
        if (isCritical()) {
            return String.format("BATTERY CRITICAL %.2f V - swap it now", volts);
        }
        if (isLow()) {
            return String.format("Battery low %.2f V - shots will fall short", volts);
        }
        if (isSagging()) {
            return String.format("Battery sagging %.2f V (avg %.2f) - check connectors",
                    volts, average);
        }
        return null;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (sensor == null) {
            telemetry.addData("Battery", "no sensor");
            return;
        }
        telemetry.addData("Battery", "%.2f V (low %.2f V this run)", volts, minimum);
        String warning = getWarning();
        if (warning != null) {
            telemetry.addLine("*** " + warning + " ***");
        }
    }
}
