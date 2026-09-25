package com.qualcomm.robotcore.hardware;

/**
 * Stub-only concrete DcMotor, so the stub HardwareMap can hand one out when
 * asked for the DcMotor interface. Not part of the real SDK.
 */
public class BasicMotor implements DcMotor {
    public double power = 0;
    public Direction direction = Direction.FORWARD;
    public RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    public ZeroPowerBehavior zeroPower = ZeroPowerBehavior.FLOAT;

    @Override public void setDirection(Direction d) { direction = d; }
    @Override public void setPower(double p) { power = p; }
    @Override public double getPower() { return power; }
    @Override public void setMode(RunMode m) { mode = m; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior z) { zeroPower = z; }
}
