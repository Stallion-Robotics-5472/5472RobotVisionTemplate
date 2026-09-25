package com.qualcomm.robotcore.hardware;

/**
 * Stub DcMotor. An interface in the real SDK; a class here so the stub
 * HardwareMap can instantiate one for tests.
 */
public interface DcMotor extends DcMotorSimple {
    enum RunMode { RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER }
    enum ZeroPowerBehavior { BRAKE, FLOAT, UNKNOWN }
    void setMode(RunMode m);
    void setZeroPowerBehavior(ZeroPowerBehavior z);
}
