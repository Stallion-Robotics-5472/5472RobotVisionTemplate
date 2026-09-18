/*
 * Drivetrain wiring check. RUN THIS BEFORE ANY OTHER DRIVING OPMODE.
 *
 * Everything else in this template -- field-centric TeleOp, the path follower,
 * the heading controller -- assumes the drivetrain obeys three conventions:
 *
 *   forward power  -> robot drives forward
 *   strafe power   -> robot slides left
 *   turn power     -> robot rotates counter-clockwise (CCW positive)
 *
 * If any of those is inverted, the follower will not merely drive badly, it
 * will drive itself further from the target. The classic failure is setting all
 * four motors to the same Direction: that does not reverse the robot, it swaps
 * translation and rotation, so "drive forward" spins the robot in place. On a
 * mecanum the two sides MUST be opposite.
 *
 * This OpMode drives one motion at a time, slowly, so you can watch the robot
 * and confirm each convention. Put the robot on blocks first if you are unsure.
 *
 * Controls (gamepad1), hold to run:
 *   dpad up     - all wheels forward      -> robot should drive FORWARD
 *   dpad left   - strafe                  -> robot should slide LEFT
 *   dpad right  - strafe                  -> robot should slide RIGHT
 *   X           - turn                    -> robot should rotate CCW (left)
 *   B           - turn                    -> robot should rotate CW (right)
 *   A           - front-left wheel only   -> that wheel should roll FORWARD
 *   Y           - front-right wheel only  -> that wheel should roll FORWARD
 *   left bumper - back-left wheel only    -> that wheel should roll FORWARD
 *   right bumper- back-right wheel only   -> that wheel should roll FORWARD
 *
 * Fixing what you find: the per-wheel tests (A / Y / bumpers) tell you which
 * motor Direction in PathConstants is wrong. Fix those first, then re-check the
 * whole-robot motions. If forward/back is inverted but the wheels are all
 * individually correct, flip BOTH sides together, never one side alone.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pathing.PathConstants;

@TeleOp(name = "Drivetrain Direction Check", group = "Setup")
public class DrivetrainDirectionCheck extends LinearOpMode {

    /** Deliberately slow: this is a diagnostic, not a driving OpMode. */
    private static final double TEST_POWER = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.get(DcMotor.class, PathConstants.FRONT_LEFT_MOTOR);
        DcMotor fr = hardwareMap.get(DcMotor.class, PathConstants.FRONT_RIGHT_MOTOR);
        DcMotor bl = hardwareMap.get(DcMotor.class, PathConstants.BACK_LEFT_MOTOR);
        DcMotor br = hardwareMap.get(DcMotor.class, PathConstants.BACK_RIGHT_MOTOR);

        fl.setDirection(PathConstants.FRONT_LEFT_DIRECTION);
        fr.setDirection(PathConstants.FRONT_RIGHT_DIRECTION);
        bl.setDirection(PathConstants.BACK_LEFT_DIRECTION);
        br.setDirection(PathConstants.BACK_RIGHT_DIRECTION);
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Catch the single most damaging misconfiguration before the robot moves.
        boolean sidesOpposite =
                PathConstants.FRONT_LEFT_DIRECTION != PathConstants.FRONT_RIGHT_DIRECTION
                        && PathConstants.BACK_LEFT_DIRECTION != PathConstants.BACK_RIGHT_DIRECTION;

        telemetry.addLine("Drivetrain Direction Check");
        telemetry.addLine();
        telemetry.addData("FL / BL", "%s / %s",
                PathConstants.FRONT_LEFT_DIRECTION, PathConstants.BACK_LEFT_DIRECTION);
        telemetry.addData("FR / BR", "%s / %s",
                PathConstants.FRONT_RIGHT_DIRECTION, PathConstants.BACK_RIGHT_DIRECTION);
        telemetry.addLine();
        if (sidesOpposite) {
            telemetry.addLine("Left and right sides are opposite. Good.");
        } else {
            telemetry.addLine("*** WARNING: the two sides are NOT opposite. ***");
            telemetry.addLine("On a mecanum this swaps translation and rotation:");
            telemetry.addLine("'drive forward' will spin the robot in place.");
            telemetry.addLine("Fix PathConstants before trusting any auto.");
        }
        telemetry.addLine();
        telemetry.addLine("Put the robot on blocks if unsure. Press play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double f = 0, strafeLeft = 0, turnCCW = 0;
            String expect = "released - all stopped";
            boolean perWheel = false;

            if (gamepad1.dpad_up) {
                f = TEST_POWER;              expect = "robot drives FORWARD";
            } else if (gamepad1.dpad_down) {
                f = -TEST_POWER;             expect = "robot drives BACKWARD";
            } else if (gamepad1.dpad_left) {
                strafeLeft = TEST_POWER;     expect = "robot slides LEFT";
            } else if (gamepad1.dpad_right) {
                strafeLeft = -TEST_POWER;    expect = "robot slides RIGHT";
            } else if (gamepad1.x) {
                turnCCW = TEST_POWER;        expect = "robot rotates CCW (left)";
            } else if (gamepad1.b) {
                turnCCW = -TEST_POWER;       expect = "robot rotates CW (right)";
            } else if (gamepad1.a) {
                perWheel = true; setOnly(fl, fr, bl, br, TEST_POWER, 0, 0, 0);
                expect = "FRONT-LEFT wheel rolls forward";
            } else if (gamepad1.y) {
                perWheel = true; setOnly(fl, fr, bl, br, 0, TEST_POWER, 0, 0);
                expect = "FRONT-RIGHT wheel rolls forward";
            } else if (gamepad1.left_bumper) {
                perWheel = true; setOnly(fl, fr, bl, br, 0, 0, TEST_POWER, 0);
                expect = "BACK-LEFT wheel rolls forward";
            } else if (gamepad1.right_bumper) {
                perWheel = true; setOnly(fl, fr, bl, br, 0, 0, 0, TEST_POWER);
                expect = "BACK-RIGHT wheel rolls forward";
            }

            if (!perWheel) {
                // Same mixing MecanumDrivetrain uses, with turn CCW-positive.
                double right = -strafeLeft;
                double rotate = -turnCCW;
                setOnly(fl, fr, bl, br,
                        f + right + rotate,
                        f - right - rotate,
                        f - right + rotate,
                        f + right - rotate);
            }

            telemetry.addLine("Drivetrain Direction Check");
            telemetry.addData("Expect", expect);
            telemetry.addLine();
            telemetry.addData("Powers", "FL %.2f  FR %.2f  BL %.2f  BR %.2f",
                    fl.getPower(), fr.getPower(), bl.getPower(), br.getPower());
            if (!sidesOpposite) {
                telemetry.addLine();
                telemetry.addLine("*** Sides are not opposite - see init warning. ***");
            }
            telemetry.update();
        }

        setOnly(fl, fr, bl, br, 0, 0, 0, 0);
    }

    private static void setOnly(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br,
                                double flP, double frP, double blP, double brP) {
        fl.setPower(flP);
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);
    }
}
