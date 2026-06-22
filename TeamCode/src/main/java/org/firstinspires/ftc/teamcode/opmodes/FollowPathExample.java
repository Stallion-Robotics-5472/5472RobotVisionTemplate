/*
 * Example autonomous demonstrating the path follower driving off the fused
 * Kalman pose estimate (Pinpoint + Limelight).
 *
 * It builds a two-segment path: an S-curve out to a point, then a straight
 * segment, finishing facing 90 degrees. The follower pulls its pose from the
 * Localization subsystem every loop.
 *
 * Remove or change @Disabled to make it appear on the Driver Station.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.BezierCurve;
import org.firstinspires.ftc.teamcode.pathing.Follower;
import org.firstinspires.ftc.teamcode.pathing.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pathing.Path;
import org.firstinspires.ftc.teamcode.pathing.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.Localization;

@Autonomous(name = "Follow Path Example", group = "Pathing")
public class FollowPathExample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Localization localization = new Localization(hardwareMap);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        Follower follower = new Follower(localization, drivetrain);

        // Seed the starting pose (must match where the robot physically starts).
        Pose2d start = new Pose2d(0, 0, new Rotation2d(0));
        localization.setStartingPose(start);

        // Segment 1: cubic S-curve from (0,0) to (30,30).
        Path s = new Path(new BezierCurve(
                new Translation2d(0, 0),
                new Translation2d(20, 0),
                new Translation2d(10, 30),
                new Translation2d(30, 30)))
                .setLinearHeading(0, Math.toRadians(90));

        // Segment 2: straight from (30,30) to (30,50), holding 90 degrees.
        Path straight = new Path(new BezierCurve(
                new Translation2d(30, 30),
                new Translation2d(30, 50)))
                .setConstantHeading(Math.toRadians(90));

        PathChain chain = new PathChain(s, straight);

        telemetry.addLine("Path follower ready. Press play.");
        telemetry.update();
        waitForStart();

        follower.followPath(chain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            Pose2d pose = localization.getPose();
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f deg",
                    pose.getX(), pose.getY(), pose.getRotation().getDegrees());
            telemetry.addData("Path", "%d  t=%.2f  remaining=%.1f",
                    follower.getPathIndex(), follower.getClosestT(), follower.getRemainingLength());
            telemetry.addData("Errors", "cross %.2f  heading %.1f deg",
                    follower.getCrossTrackError(), Math.toDegrees(follower.getHeadingError()));
            telemetry.update();
        }

        drivetrain.stop();
        localization.stop();

        telemetry.addLine("Path complete.");
        telemetry.update();
    }
}
