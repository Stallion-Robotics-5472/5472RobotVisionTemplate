/*
 * One auto, both alliances.
 *
 * The plan below is authored once for AUTHORED_FOR. At init the driver picks the
 * alliance actually being played, and AllianceFlip transforms the starting pose
 * and every path onto that side of the field. Nothing else changes: the pose
 * frame, the follower and the gains are all alliance-independent.
 *
 * Controls (during init):
 *   B - red
 *   X - blue
 *
 * Original implementation for this template.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.AllianceFlip;
import org.firstinspires.ftc.teamcode.pathing.BezierCurve;
import org.firstinspires.ftc.teamcode.pathing.Follower;
import org.firstinspires.ftc.teamcode.pathing.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pathing.Path;
import org.firstinspires.ftc.teamcode.pathing.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.Localization;

@Autonomous(name = "Alliance Auto Example", group = "Auto")
public class AllianceAutoExample extends LinearOpMode {

    /** The alliance the coordinates below were drawn for. */
    private static final Alliance AUTHORED_FOR = Alliance.RED;

    /** Starting pose, in absolute field coordinates, for AUTHORED_FOR. */
    private static final Pose2d START = new Pose2d(-58, -58, Rotation2d.fromDegrees(45));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        Localization localization = new Localization(hardwareMap);
        Follower follower = new Follower(localization, drivetrain);

        Alliance alliance = AUTHORED_FOR;
        while (opModeInInit()) {
            if (gamepad1.b) {
                alliance = Alliance.RED;
            }
            if (gamepad1.x) {
                alliance = Alliance.BLUE;
            }
            telemetry.addLine("Alliance Auto Example");
            telemetry.addData("Alliance", "%s   (B = red, X = blue)", alliance);
            telemetry.addData("Authored for", AUTHORED_FOR);
            telemetry.addData("Flipping", alliance.needsFlipFrom(AUTHORED_FOR) ? "YES" : "no");
            telemetry.addData("Start pose",
                    AllianceFlip.forAlliance(START, AUTHORED_FOR, alliance));
            telemetry.update();
        }

        waitForStart();

        // Flip the plan onto whichever side we are actually playing.
        Pose2d start = AllianceFlip.forAlliance(START, AUTHORED_FOR, alliance);
        PathChain plan = AllianceFlip.forAlliance(buildPlan(), AUTHORED_FOR, alliance);

        localization.setStartingPose(start);
        follower.followPath(plan);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Segment", follower.getPathIndex());
            telemetry.addData("Cross-track", "%.2f in", follower.getCrossTrackError());
            telemetry.addData("Remaining", "%.2f in", follower.getRemainingLength());
            telemetry.addData("Pose", localization.getPose());
            telemetry.update();
        }

        follower.stop();
        localization.stop();
    }

    /** The plan, in AUTHORED_FOR's coordinates. */
    private static PathChain buildPlan() {
        Path out = new Path(new BezierCurve(
                new Translation2d(-58, -58),
                new Translation2d(-30, -10),
                new Translation2d(0, 20),
                new Translation2d(34, 40)))
                .setTangentHeading();

        Path park = new Path(new BezierCurve(
                new Translation2d(34, 40),
                new Translation2d(55, 55),
                new Translation2d(62, 20)))
                .setLinearHeading(Math.toRadians(90), Math.toRadians(0));

        return new PathChain(out, park);
    }
}
