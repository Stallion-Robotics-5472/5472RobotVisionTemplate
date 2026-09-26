/*
 * Command-based TeleOp with shoot-on-the-move.
 *
 * Hold the aim button and the robot keeps translating wherever the driver puts
 * it while rotating itself to lead the goal. Pull the trigger and it feeds as
 * soon as the shot is genuinely ready. Let go of aim and heading returns to the
 * driver.
 *
 * Controls (gamepad1):
 *   left stick        translate, from the driver's point of view
 *   right stick X     turn (only when not aiming; aiming owns heading)
 *   right bumper      hold for slow mode
 *   LEFT BUMPER       hold to AIM at the goal (shoot on the move)
 *   RIGHT TRIGGER     fire, once the shot is ready
 *   Y                 re-seed the pose from vision (recovers a bad heading)
 *   back              re-zero driver-forward to the robot's current facing
 *   A                 cycle the target goal (pins it; overrides auto-select)
 *   left stick button hand goal choice back to automatic selection
 *   dpad up/down      trim flywheel rpm  +/- 50
 *   dpad left/right   trim hood angle    -/+ 0.5 deg
 *   X                 clear trim
 *   X / B (in init)   select BLUE / RED alliance
 *
 * This is the reference wiring for the command system: build subsystems, give the
 * drivetrain a default command for ordinary driving, then bind everything else to
 * buttons. See lib/command for the framework.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.AimAndShootCommand;
import org.firstinspires.ftc.teamcode.lib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.command.Commands;
import org.firstinspires.ftc.teamcode.lib.command.RunCommand;
import org.firstinspires.ftc.teamcode.lib.util.BatteryMonitor;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.logging.MatchRecorder;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.shooting.GoalSelector;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Shoot On The Move (command-based)", group = "Drive")
public class ShootOnTheMoveTeleOp extends CommandOpMode {

    private static final double SLOW_SCALE = 0.4;

    /**
     * Where the robot physically starts, in absolute field coordinates. Set this
     * or carry it over from auto -- aiming is only as good as the pose, and
     * (0, 0, 0) is the middle of the field facing +X.
     */
    private static final Pose2d START_POSE = new Pose2d(0, 0, new Rotation2d(0));

    /**
     * Records a CSV of what the robot believed, once per loop, to
     * /sdcard/FIRST/matchlogs. Measured at well under 1% of the loop budget, and it
     * cannot block the loop or take the OpMode down -- see logging/MatchLog. Set
     * false if you would rather not write to storage at all.
     */
    private static final boolean RECORD_MATCH = true;

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private AimAndShootCommand aimAndShoot;
    private GoalSelector goalSelector;

    private MatchRecorder recorder;
    private BatteryMonitor battery;

    private Alliance alliance = Alliance.RED;
    private Rotation2d driverForwardOffset = new Rotation2d(0);

    @Override
    public void configure() {
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        register(drive, shooter);

        drive.getLocalization().setStartingPose(START_POSE);
        battery = BatteryMonitor.from(hardwareMap);

        // Alliance select during init. Only the driver's point of view and which
        // goal we aim at change; the pose frame is absolute either way.
        whenPressed(() -> gamepad1.b && opModeInInit(),
                Commands.runOnce(() -> alliance = Alliance.RED));
        whenPressed(() -> gamepad1.x && opModeInInit(),
                Commands.runOnce(() -> alliance = Alliance.BLUE));

        // Default: ordinary driver control of both translation and heading.
        setDefaultCommand(drive, new RunCommand(() -> {
            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            drive.driveDriverRelative(
                    -gamepad1.left_stick_y * scale,
                    -gamepad1.left_stick_x * scale,
                    -gamepad1.right_stick_x * scale,
                    alliance);
        }, drive).withName("DriverControl"));

        // Shooter idles so the next shot only has to make up a small difference.
        setDefaultCommand(shooter, new RunCommand(shooter::idle, shooter).withName("Idle"));

        // The main event. Requires both subsystems, so it displaces both default
        // commands while held and they resume when released.
        goalSelector = ShootingConstants.newGoalSelector();
        aimAndShoot = new AimAndShootCommand(
                drive, shooter,
                () -> -gamepad1.left_stick_y * (gamepad1.right_bumper ? SLOW_SCALE : 1.0),
                () -> -gamepad1.left_stick_x * (gamepad1.right_bumper ? SLOW_SCALE : 1.0),
                () -> alliance,
                () -> gamepad1.right_trigger > 0.5,
                goalSelector);
        whileHeld(() -> gamepad1.left_bumper, aimAndShoot);

        // Goal override. The selector normally picks by AprilTag visibility, but a
        // driver who can see the field knows things the camera does not.
        whenPressed(() -> gamepad1.a && !opModeInInit(),
                Commands.runOnce(goalSelector::cycle));
        whenPressed(() -> gamepad1.left_stick_button,
                Commands.runOnce(() -> goalSelector.auto(ShootingConstants.GOAL_STRATEGY)));

        // Recovery: snap the pose to what the camera sees. Uses MegaTag1, so it
        // can fix a heading the gyro has wrong.
        whenPressed(() -> gamepad1.y, Commands.runOnce(() -> {
            drive.getLocalization().seedFromVision();
            driverForwardOffset = new Rotation2d(0);
        }));

        // Re-zero the driver's forward to wherever the robot is facing now.
        whenPressed(() -> gamepad1.back, Commands.runOnce(() ->
                driverForwardOffset = new Rotation2d(drive.getPose().getHeading())
                        .minus(alliance.driverForward())));

        // Shot trim, for dialling a shot in between matches.
        whenPressed(() -> gamepad1.dpad_up,
                Commands.runOnce(() -> shooter.addRpmTrim(50.0)));
        whenPressed(() -> gamepad1.dpad_down,
                Commands.runOnce(() -> shooter.addRpmTrim(-50.0)));
        whenPressed(() -> gamepad1.dpad_right,
                Commands.runOnce(() -> shooter.addHoodTrim(0.5)));
        whenPressed(() -> gamepad1.dpad_left,
                Commands.runOnce(() -> shooter.addHoodTrim(-0.5)));
        whenPressed(() -> gamepad1.x && !opModeInInit(),
                Commands.runOnce(shooter::clearTrim));
    }

    @Override
    public void onStart() {
        if (RECORD_MATCH) {
            // Shares the OpMode's loop timer, so the log's loop columns are the same
            // numbers the telemetry shows.
            recorder = new MatchRecorder("teleop", drive, getLoopTimer())
                    .withShooter(shooter)
                    .withAim(aimAndShoot::getSolution, goalSelector::getSelected)
                    .withVoltage(battery::getVolts);
        }
    }

    @Override
    public void periodic() {
        battery.update();
        telemetry.addData("Alliance", "%s   (B = red, X = blue)", alliance);

        if (opModeInInit()) {
            telemetry.addData("Start pose check",
                    drive.getLocalization().getStartPoseCheck());
            if (drive.getLocalization().isStartPoseSuspect()) {
                telemetry.addLine("*** Vision disagrees with the seeded heading. ***");
                telemetry.addLine("Check the alliance and which way the robot faces.");
            }
            telemetry.addLine();
            telemetry.addLine("LB = aim  |  RT = fire  |  Y = re-seed from vision");
            telemetry.addData("Goals", "%d configured, strategy %s",
                    ShootingConstants.GOALS.length, ShootingConstants.GOAL_STRATEGY);
            battery.addTelemetry(telemetry);
            return;
        }

        telemetry.addData("Mode", getScheduler().isScheduled(aimAndShoot)
                ? "AIMING" : "driver control");
        telemetry.addData("Goal", goalSelector.describe());
        telemetry.addData("Tags seen", drive.getLocalization().getVisibleTagIds().toString());
        if (getScheduler().isScheduled(aimAndShoot)) {
            telemetry.addData("Shot", aimAndShoot.getStatus());
            if (aimAndShoot.getSolution() != null) {
                telemetry.addData("Aim", "%.1f in effective (%.1f actual)",
                        aimAndShoot.getSolution().effectiveDistanceInches,
                        aimAndShoot.getSolution().actualDistanceInches);
                telemetry.addData("Lead", "heading %.1f deg, sweep %.2f rad/s",
                        Math.toDegrees(aimAndShoot.getSolution().targetHeadingRadians),
                        aimAndShoot.getSolution().headingFeedforwardRadPerSec);
            }
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
    public void onStop() {
        shooter.stop();
        drive.stop();
        drive.getLocalization().stop();
        if (recorder != null) {
            recorder.close();
        }
    }
}
