/*
 * Simple PIDF controller with explicit timestep and integral clamping.
 *
 * The error is the quantity to be driven to zero; calculate() returns the
 * control effort. A static feedforward (kF) can be applied in the direction of
 * the error to overcome friction, gated by a deadband so it doesn't chatter
 * around zero. This is an original implementation written for this template.
 */
package org.firstinspires.ftc.teamcode.pathing;

public class PIDFController {
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean hasLast = false;

    private double integralLimit = Double.POSITIVE_INFINITY;
    private double feedforwardDeadband = 1e-6;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** Maximum absolute value the integral term's accumulator may reach. */
    public void setIntegralLimit(double limit) {
        this.integralLimit = Math.abs(limit);
    }

    /** kF is only applied when |error| exceeds this deadband. */
    public void setFeedforwardDeadband(double deadband) {
        this.feedforwardDeadband = Math.abs(deadband);
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        hasLast = false;
    }

    public double calculate(double error, double dt) {
        double derivative = 0.0;
        if (hasLast && dt > 1e-9) {
            derivative = (error - lastError) / dt;
        }
        if (dt > 0) {
            integral += error * dt;
            integral = clamp(integral, -integralLimit, integralLimit);
        }
        lastError = error;
        hasLast = true;

        double feedforward = 0.0;
        if (Math.abs(error) > feedforwardDeadband) {
            feedforward = Math.copySign(kF, error);
        }
        return kP * error + kI * integral + kD * derivative + feedforward;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
