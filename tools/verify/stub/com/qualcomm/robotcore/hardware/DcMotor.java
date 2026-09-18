package com.qualcomm.robotcore.hardware;
public interface DcMotor extends DcMotorSimple {
    enum RunMode { RUN_USING_ENCODER, RUN_WITHOUT_ENCODER }
    enum ZeroPowerBehavior { BRAKE, FLOAT }
    void setMode(RunMode m);
    void setZeroPowerBehavior(ZeroPowerBehavior z);
}
