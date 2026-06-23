/*
 * Plain robot-centric mecanum TeleOp ("drive it like an RC car"): the robot
 * moves relative to its own front, regardless of field orientation. Reuses the
 * existing MecanumDrivetrain (driveRobotCentric) rather than recreating drive
 * logic.
 *
 * Controls (gamepad1):
 *   left stick Y  - forward / back
 *   left stick X  - strafe left / right *    stick X - turn
 *   right bumper  - hold for slow mode
 *
 * Remove or change @Disabled to make it appear on the Driver Station.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pathing.MecanumDrivetrain;

@TeleOp(name = "Robot-Centric Mecanum Drive", group = "Drive")
public class RobotCentricDrive extends LinearOpMode {

    // Scale applied while the slow-mode button is held.
    private static final double SLOW_SCALE = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);

        telemetry.addLine("Robot-centric drive ready. Press play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Map gamepad to robot-frame inputs:
            //   forward    = +X (push stick up -> stick_y is negative)
            //   strafeLeft = +Y (push stick left -> stick_x is negative)
            //   turn       = CCW positive (push right stick left -> turn left)
            double forward = -gamepad1.left_stick_y;
            double strafeLeft = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            drivetrain.driveRobotCentric(forward * scale, strafeLeft * scale, turn * scale);

            telemetry.addData("Mode", gamepad1.right_bumper ? "SLOW" : "normal");
            telemetry.addData("Drive", "fwd %.2f  strafeL %.2f  turn %.2f",
                    forward * scale, strafeLeft * scale, turn * scale);
            telemetry.update();
        }

        drivetrain.stop();
    }
}
