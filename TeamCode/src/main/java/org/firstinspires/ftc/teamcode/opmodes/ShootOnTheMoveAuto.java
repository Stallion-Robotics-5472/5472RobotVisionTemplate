/*
 * Autonomous that drives a route while shooting on the move, for either alliance.
 *
 * This is what heading sources bought: the path says where to go, the aiming
 * solution says where to face, and the robot never stops to shoot. The plan is
 * authored once for AUTHORED_FOR and flipped at init.
 *
 * Shape of the routine:
 *   1. leg one   -- drive out of the start, tangent heading, spin up on the way
 *                   (a marker starts the flywheel a third of the way along, so it
 *                   is already at speed when the shooting leg begins)
 *   2. leg two   -- AIMING heading source: track the goal while crossing the field,
 *                   feeding whenever the shot is genuinely ready
 *   3. leg three -- park, tangent heading, shooter stowed
 *
 * Controls (during init):
 *   B - red
 *   X - blue
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPathAndShootCommand;
import org.firstinspires.ftc.teamcode.lib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.command.Commands;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.BatteryMonitor;
import org.firstinspires.ftc.teamcode.logging.MatchRecorder;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.AllianceFlip;
import org.firstinspires.ftc.teamcode.pathing.BezierCurve;
import org.firstinspires.ftc.teamcode.pathing.Path;
import org.firstinspires.ftc.teamcode.pathing.PathChain;
import org.firstinspires.ftc.teamcode.pathing.PathMarker;
import org.firstinspires.ftc.teamcode.shooting.AimAtGoalHeading;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "Shoot On The Move Auto", group = "Auto")
public class ShootOnTheMoveAuto extends CommandOpMode {

    /** The alliance the coordinates below were drawn for. */
    private static final Alliance AUTHORED_FOR = Alliance.RED;

    /** Starting pose in absolute field coordinates, for AUTHORED_FOR. */
    private static final Pose2d START = new Pose2d(-48, -48, Rotation2d.fromDegrees(45));

    /** See ShootOnTheMoveTeleOp: a per-loop CSV of what the robot believed. */
    private static final boolean RECORD_MATCH = true;

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private AimAtGoalHeading aim;
    private FollowPathAndShootCommand routine;
    private MatchRecorder recorder;
    private BatteryMonitor battery;

    private Alliance alliance = AUTHORED_FOR;
    private Alliance seededFor = null;

    @Override
    public void configure() {
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        register(drive, shooter);
        battery = BatteryMonitor.from(hardwareMap);

        whenPressed(() -> gamepad1.b && opModeInInit(),
                Commands.runOnce(() -> alliance = Alliance.RED));
        whenPressed(() -> gamepad1.x && opModeInInit(),
                Commands.runOnce(() -> alliance = Alliance.BLUE));

        // Shooter stays stowed until the routine asks for it.
        setDefaultCommand(shooter,
                Commands.run(shooter::stow, shooter).withName("Stowed"));
    }

    @Override
    public void periodic() {
        battery.update();
        if (opModeInInit()) {
            // Seed on alliance selection, not after start, so the camera can check
            // the seed while the robot sits still. Re-seed only when the choice
            // changes -- setStartingPose resets the heading-trust counters.
            if (alliance != seededFor) {
                drive.getLocalization().setStartingPose(
                        AllianceFlip.forAlliance(START, AUTHORED_FOR, alliance));
                seededFor = alliance;
            }
            telemetry.addLine("Shoot On The Move Auto");
            telemetry.addData("Alliance", "%s   (B = red, X = blue)", alliance);
            telemetry.addData("Flipping", alliance.needsFlipFrom(AUTHORED_FOR) ? "YES" : "no");
            telemetry.addData("Start pose check",
                    drive.getLocalization().getStartPoseCheck());
            if (drive.getLocalization().isStartPoseSuspect()) {
                telemetry.addLine("*** Vision disagrees with the seeded heading ***");
                telemetry.addLine("Check the alliance and which way the robot faces.");
            }
            battery.addTelemetry(telemetry);
            return;
        }

        if (routine != null) {
            telemetry.addData("Routine", routine.getStatus());
            telemetry.addData("Feeding loops", routine.getShotsFedLoops());
        }
        shooter.addTelemetry(telemetry);
        drive.addTelemetry(telemetry);
        battery.addTelemetry(telemetry);
        addLoopTelemetry();
        if (recorder != null) {
            recorder.record();
            recorder.getLog().addTelemetry(telemetry);
        }
    }

    @Override
    public void onStart() {
        aim = new AimAtGoalHeading(
                ShootingConstants.newGoalSelector(),
                () -> alliance,
                drive.getLocalization()::getVisibleTagIds);

        PathChain plan = AllianceFlip.forAlliance(buildPlan(), AUTHORED_FOR, alliance);
        routine = new FollowPathAndShootCommand(drive, shooter, plan, aim);
        schedule(routine);

        if (RECORD_MATCH) {
            recorder = new MatchRecorder("auto", drive, getLoopTimer())
                    .withShooter(shooter)
                    .withAim(aim::getLastSolution, aim::getLastGoal)
                    .withVoltage(battery::getVolts);
        }
    }

    /** The route, in AUTHORED_FOR's coordinates. */
    private PathChain buildPlan() {
        // Leg 1: get out of the start. Plain tangent heading -- nothing to aim at
        // yet. A marker starts the flywheel a third of the way along so it is up to
        // speed by the time the shooting leg starts, rather than spinning up while
        // the shot window is already open.
        Path leaveStart = new Path(new BezierCurve(
                new Translation2d(-48, -48),
                new Translation2d(-30, -20),
                new Translation2d(-24, 6)))
                .setTangentHeading()
                .addMarker(PathMarker.atT(0.33,
                        () -> shooter.setFlywheelRpm(
                                shooter.mapSetpointAt(48.0).flywheelRpm),
                        "spin up"));

        // Leg 2: the shooting leg. The path chooses the route, the aiming solution
        // chooses the heading, and the robot shoots without stopping.
        Path shootWhileCrossing = new Path(new BezierCurve(
                new Translation2d(-24, 6),
                new Translation2d(-4, 18),
                new Translation2d(20, 14),
                new Translation2d(34, 0)))
                .setHeadingSource(aim);

        // Leg 3: park. Back to a plain heading, shooter stowed on arrival.
        Path park = new Path(new BezierCurve(
                new Translation2d(34, 0),
                new Translation2d(46, -18),
                new Translation2d(50, -40)))
                .setTangentHeading()
                .addMarker(PathMarker.withinInchesOfEnd(18.0, shooter::stow, "stow"));

        return new PathChain(leaveStart, shootWhileCrossing, park);
    }

    @Override
    public void onStop() {
        shooter.stop();
        drive.stop();
        drive.getLocalization().stop();
        if (recorder != null) {
            recorder.close();
        }
    }
}
