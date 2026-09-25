package com.qualcomm.robotcore.hardware;

/** Stub DcMotorEx. Holds a commanded velocity so tests can read it back. */
public class DcMotorEx implements DcMotor {
    public double velocityTicksPerSecond = 0;
    public double power = 0;
    public Direction direction = Direction.FORWARD;
    public RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    public ZeroPowerBehavior zeroPower = ZeroPowerBehavior.FLOAT;
    public double p, i, d, f;

    /** Tests set this to simulate the flywheel reaching (or not) its setpoint. */
    public double measuredTicksPerSecond = 0;

    public void setVelocity(double ticksPerSecond) {
        velocityTicksPerSecond = ticksPerSecond;
        measuredTicksPerSecond = ticksPerSecond;
    }

    public double getVelocity() {
        return measuredTicksPerSecond;
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        this.p = p; this.i = i; this.d = d; this.f = f;
    }

    @Override public void setDirection(Direction d) { direction = d; }
    @Override public void setPower(double p) { power = p; }
    @Override public double getPower() { return power; }
    @Override public void setMode(RunMode m) { mode = m; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior z) { zeroPower = z; }
}
