/*
 * Example OpMode demonstrating the fused localization subsystem.
 *
 * It initializes the Localization subsystem (Pinpoint + Limelight + Kalman pose
 * estimator), seeds a starting pose, then streams the fused pose alongside the
 * raw odometry pose and vision diagnostics so you can watch the estimator
 * correct drift as AprilTags come into view.
 *
 * Remove or change @Disabled to make it appear on the Driver Station.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.subsystems.Localization;

@TeleOp(name = "Localization Test (Pinpoint + Limelight)", group = "Vision")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Localization localization = new Localization(hardwareMap);

        // Seed with a known starting pose. Replace with your real start.
        localization.setStartingPose(new Pose2d(0, 0, new Rotation2d(0)));

        telemetry.addLine("Localization ready. Press play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            localization.update();

            Pose2d fused = localization.getPose();
            Pose2d odo = localization.getOdometryPose();

            telemetry.addData("Fused", "x %.1f  y %.1f  h %.1f deg",
                    fused.getX(), fused.getY(), fused.getRotation().getDegrees());
            telemetry.addData("Odometry", "x %.1f  y %.1f  h %.1f deg",
                    odo.getX(), odo.getY(), odo.getRotation().getDegrees());
            telemetry.addData("Vision enabled", localization.isVisionEnabled());
            telemetry.addData("Vision accepted", localization.wasLastVisionAccepted());
            telemetry.addData("Vision status", localization.getLastVisionReject());
            Pose2d vis = localization.getVisionPose2d();
            telemetry.addData("Vision 2D", "x %.1f  y %.1f  h %.1f deg",
                    vis.getX(), vis.getY(), vis.getRotation().getDegrees());
            telemetry.addData("Tags", localization.getLastTagCount());
            telemetry.addData("Avg tag dist", "%.2f", localization.getLastAvgTagDistance());
            telemetry.addData("Vision 3D", "z %.1f  pitch %.1f  roll %.1f deg",
                    localization.getVisionZ(),
                    Math.toDegrees(localization.getVisionPitch()),
                    Math.toDegrees(localization.getVisionRoll()));
            telemetry.update();
        }

        localization.stop();
    }
}
