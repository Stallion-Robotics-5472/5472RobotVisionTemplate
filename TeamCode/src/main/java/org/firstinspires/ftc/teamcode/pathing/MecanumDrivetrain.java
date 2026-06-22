/*
 * Mecanum drivetrain that turns a field-centric translation + turn request into
 * four motor powers. The field vector is rotated into the robot frame using the
 * current heading, then mapped to wheels with the standard mecanum mixing and
 * normalized so no wheel exceeds full power. Original implementation for this
 * template; the wheel mixing matches the FTC field-relative mecanum sample.
 */
package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

public class MecanumDrivetrain implements Drivetrain {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, PathConstants.FRONT_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, PathConstants.FRONT_RIGHT_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, PathConstants.BACK_LEFT_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, PathConstants.BACK_RIGHT_MOTOR);

        frontLeft.setDirection(PathConstants.FRONT_LEFT_DIRECTION);
        frontRight.setDirection(PathConstants.FRONT_RIGHT_DIRECTION);
        backLeft.setDirection(PathConstants.BACK_LEFT_DIRECTION);
        backRight.setDirection(PathConstants.BACK_RIGHT_DIRECTION);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void driveFieldCentric(double fieldXPower, double fieldYPower, double turnPower,
                                  Rotation2d robotHeading) {        double cos = robotHeading.getCos();
        double sin = robotHeading.getSin();

        // Rotate the field vector into the robot frame.
        double forward = fieldXPower * cos + fieldYPower * sin;   // robot +X (forward)
        double right = fieldXPower * sin - fieldYPower * cos;     // robot -Y (strafe right)
        // turnPower is CCW positive; the sample's "rotate" is CW positive.
        double rotate = -turnPower;

        double flPower = forward + right + rotate;
        double frPower = forward - right - rotate;
        double brPower = forward + right - rotate;
        double blPower = forward - right + rotate;

        double max = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(brPower), Math.abs(blPower)))));

        frontLeft.setPower(flPower / max);
        frontRight.setPower(frPower / max);
        backRight.setPower(brPower / max);
        backLeft.setPower(blPower / max);
    }

    /**
     * Robot-centric drive for manual TeleOp. Inputs are in the robot's own
     * frame: +forward, +strafeLeft, +turn is counter-clockwise. Delegates to
     * {@link #driveFieldCentric} with a zero heading, so it reuses the exact
     * same wheel mixing and normalization (no separate drivetrain logic).
     */
    public void driveRobotCentric(double forward, double strafeLeft, double turn) {
        driveFieldCentric(forward, strafeLeft, turn, new Rotation2d());
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
