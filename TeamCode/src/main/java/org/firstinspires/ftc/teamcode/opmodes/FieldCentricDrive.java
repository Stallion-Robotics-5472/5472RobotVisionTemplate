/*
 * Field-centric mecanum TeleOp, correct for both alliances.
 *
 * Field-centric means the stick points at a place on the field, not at a side
 * of the robot: push the stick away from yourself and the robot drives away
 * from you no matter which way it is facing.
 *
 * The subtlety this OpMode exists to handle: the field coordinate system is
 * absolute and the same for both alliances, but the two drive teams stand at
 * opposite ends of the field. So "away from me" is +X for red and -X for blue.
 * The stick vector is therefore rotated twice:
 *
 *   driver frame --(alliance.driverForward())--> field frame --(-heading)--> robot
 *
 * The second rotation happens inside MecanumDrivetrain.driveFieldCentric. The
 * pose estimate itself is never alliance-dependent.
 *
 * Controls (gamepad1):
 *   left stick      - translate, relative to the driver's own point of view
 *   right stick X   - turn (CCW positive)
 *   right bumper    - hold for slow mode
 *   X / B (in init) - select BLUE / RED alliance
 *   back            - re-zero the field heading to the robot's current facing
 *
 * Original implementation for this template.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Localization;

@TeleOp(name = "Field-Centric Mecanum Drive", group = "Drive")
public class FieldCentricDrive extends LinearOpMode {

    private static final double SLOW_SCALE = 0.4;

    /**
     * Where the robot actually sits when the OpMode starts, in absolute field
     * coordinates. Set this to your real starting spot (or carry the pose over
     * from auto) so the heading used for field-centric drive is correct before
     * vision has seen a tag. The field frame does not depend on the alliance,
     * so this is a plain absolute pose.
     */
    private static final Pose2d START_POSE = new Pose2d(0, 0, new Rotation2d(0));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        Localization localization = new Localization(hardwareMap);
        localization.setStartingPose(START_POSE);

        Alliance alliance = Alliance.RED;

        // Alliance selection before start; the frame does not change, only the
        // driver's point of view does.
        while (opModeInInit()) {
            if (gamepad1.b) {
                alliance = Alliance.RED;
            }
            if (gamepad1.x) {
                alliance = Alliance.BLUE;
            }
            telemetry.addLine("Field-Centric Drive");
            telemetry.addData("Alliance", "%s   (B = red, X = blue)", alliance);
            telemetry.addData("Driver looks toward", "%.0f deg in field frame",
                    alliance.driverForward().getDegrees());
            telemetry.addLine("Start pose " + START_POSE);
            telemetry.update();
        }

        waitForStart();

        // Offset applied to the field heading by the "back" re-zero button, so a
        // driver can recover from a bad heading estimate without restarting.
        Rotation2d headingOffset = new Rotation2d(0.0);
        boolean lastBack = false;

        while (opModeIsActive()) {
            localization.update();
            Pose2d pose = localization.getPose();

            if (gamepad1.back && !lastBack) {
                // Treat the robot's current facing as the driver's forward.
                headingOffset = new Rotation2d(pose.getHeading())
                        .minus(alliance.driverForward());
            }
            lastBack = gamepad1.back;

            // Stick in the driver's own frame: +forward is away from the driver,
            // +left is to the driver's left.
            double forward = -gamepad1.left_stick_y;
            double left = -gamepad1.left_stick_x;
            // Negated: turn power is CCW-positive, but pushing the stick right
            // (positive) must turn the robot right, which is CW.
            double turn = -gamepad1.right_stick_x;

            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;

            // Driver frame -> field frame.
            Translation2d fieldVec = new Translation2d(forward * scale, left * scale)
                    .rotateBy(alliance.driverForward());

            // Field frame -> robot frame happens inside the drivetrain.
            Rotation2d heading = new Rotation2d(pose.getHeading()).minus(headingOffset);
            drivetrain.driveFieldCentric(fieldVec.getX(), fieldVec.getY(), turn * scale, heading);

            telemetry.addData("Alliance", alliance);
            telemetry.addData("Mode", gamepad1.right_bumper ? "SLOW" : "normal");
            telemetry.addData("Stick (driver)", "fwd %.2f  left %.2f", forward, left);
            telemetry.addData("Command (field)", "X %.2f  Y %.2f  turn %.2f",
                    fieldVec.getX(), fieldVec.getY(), turn * scale);
            telemetry.addData("Heading used", "%.1f deg", heading.getDegrees());
            localization.addTelemetry(telemetry);
            telemetry.update();
        }

        drivetrain.stop();
        localization.stop();
    }
}
