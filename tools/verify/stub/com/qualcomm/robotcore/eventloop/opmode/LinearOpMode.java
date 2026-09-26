package com.qualcomm.robotcore.eventloop.opmode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public abstract class LinearOpMode {
    public HardwareMap hardwareMap = new HardwareMap();
    public Telemetry telemetry;
    public Gamepad gamepad1 = new Gamepad();
    public Gamepad gamepad2 = new Gamepad();
    public abstract void runOpMode() throws InterruptedException;
    public void waitForStart() {}
    public boolean opModeIsActive() { return false; }
    public boolean opModeInInit() { return false; }
    public boolean isStopRequested() { return false; }
    public void sleep(long ms) {}
    public void idle() {}
}
