/*
 * The tool you build the shot table with.
 *
 * The robot auto-aims at the goal the whole time, so the DISTANCE on screen is
 * always the real distance to it -- but the shot map is bypassed and the flywheel
 * and hood come from the numbers you dial in by hand. Park somewhere, adjust
 * until shots go in, write the row down, move, repeat. Five or six rows across
 * your usable range is enough.
 *
 * Ported from the MAP_TUNING state in team 5472's FRC superstructure.
 *
 * HOW TO USE IT
 *   1. Set ShootingConstants.GOALS first. Check each goal by comparing the
 *      DISTANCE readout against a tape measure -- if they disagree, the goal
 *      position (or the start pose) is wrong and nothing else will work.
 *   2. Park at a distance. Hold LB so the robot aims, and let it settle.
 *   3. Dial rpm and hood until shots go in. Fire with RT.
 *   4. Press A to log the row. It appears on screen ready to copy.
 *   5. Move and repeat.
 *   6. Copy the rows into ShootingConstants.SHOT_MAP.
 *
 * FLIGHT TIME is the one column you cannot read off the robot: time it from a
 * slow-motion phone clip of the shot. The logged row leaves a placeholder for it.
 * Do not leave it at zero -- with zero flight time the moving-shot correction does
 * nothing and the robot only shoots well standing still.
 *
 * Controls (gamepad1):
 *   left stick        drive (driver-relative)
 *   LEFT BUMPER       hold to aim at the goal
 *   RIGHT TRIGGER     fire
 *   dpad up/down      flywheel rpm  +/- 25
 *   dpad left/right   hood angle    -/+ 0.5 deg
 *   A                 log the current row
 *   B                 clear the log
 *   right bumper      cycle which goal you are tuning against
 *   X / Y (in init)   select RED / BLUE alliance
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.command.Commands;
import org.firstinspires.ftc.teamcode.lib.command.RunCommand;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.shooting.AimLogic;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
import org.firstinspires.ftc.teamcode.shooting.GoalSelector;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Shooter Map Tuning", group = "Setup")
public class ShooterMapTuning extends CommandOpMode {

    private static final Pose2d START_POSE = new Pose2d(0, 0, new Rotation2d(0));

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;

    private Alliance alliance = Alliance.RED;
    private final GoalSelector goalSelector = ShootingConstants.newGoalSelector();
    private double manualRpm = 2000.0;
    private double manualHoodDeg = ShootingConstants.HOOD_STOWED_DEG;
    private AimSolution solution;
    private final List<String> loggedRows = new ArrayList<>();

    @Override
    public void configure() {
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        register(drive, shooter);

        drive.getLocalization().setStartingPose(START_POSE);

        // The map is bypassed for the whole session; setpoints come from the
        // dials below so a bad map cannot fight the tuning.
        shooter.setManualMode(true);

        whenPressed(() -> gamepad1.x && opModeInInit(),
                Commands.runOnce(() -> alliance = Alliance.RED));
        whenPressed(() -> gamepad1.y && opModeInInit(),
                Commands.runOnce(() -> alliance = Alliance.BLUE));

        setDefaultCommand(drive, new RunCommand(() -> {
            // Aim while LB is held; otherwise plain driving. Aiming is what keeps
            // the DISTANCE readout meaningful, so hold it while taking a shot.
            solution = computeSolution();
            double fwd = -gamepad1.left_stick_y * 0.6;
            double left = -gamepad1.left_stick_x * 0.6;
            if (gamepad1.left_bumper) {
                Translation2d v = new Translation2d(fwd, left)
                        .rotateBy(alliance.driverForward());
                drive.driveWithHeadingLock(v.getX(), v.getY(),
                        solution.targetHeadingRadians,
                        solution.headingFeedforwardRadPerSec);
            } else {
                drive.driveDriverRelative(fwd, left, -gamepad1.right_stick_x * 0.6, alliance);
            }
        }, drive).withName("TuningDrive"));

        // Apply the hand-dialled setpoints every loop.
        setDefaultCommand(shooter, new RunCommand(() -> {
            shooter.setFlywheelRpm(manualRpm);
            shooter.setHoodDegrees(manualHoodDeg);
            if (gamepad1.right_trigger > 0.5 && shooter.atSpeed()) {
                shooter.runFeeder();
            } else {
                shooter.stopFeeder();
            }
        }, shooter).withName("ManualShooter"));

        whenPressed(() -> gamepad1.dpad_up, Commands.runOnce(() -> manualRpm += 25));
        whenPressed(() -> gamepad1.dpad_down, Commands.runOnce(() -> manualRpm -= 25));
        whenPressed(() -> gamepad1.dpad_right, Commands.runOnce(() -> manualHoodDeg += 0.5));
        whenPressed(() -> gamepad1.dpad_left, Commands.runOnce(() -> manualHoodDeg -= 0.5));

        whenPressed(() -> gamepad1.a && !opModeInInit(), Commands.runOnce(this::logRow));
        // Each goal needs its own table, so tune them one at a time.
        whenPressed(() -> gamepad1.right_bumper, Commands.runOnce(goalSelector::cycle));
        whenPressed(() -> gamepad1.b && !opModeInInit(),
                Commands.runOnce(loggedRows::clear));
    }

    private AimSolution computeSolution() {
        goalSelector.update(drive.getPose(),
                drive.getLocalization().getVisibleTagIds(), alliance);
        return AimLogic.calculate(drive.getPose(), drive.getFieldVelocity(),
                drive.getAngularVelocity(), goalTarget(),
                goalSelector.getSelected().getMap(),
                ShootingConstants.AIM_CONFIG.withGoalRadius(
                        goalSelector.getSelected().getRadiusInches()));
    }

    /** The selected goal's position, flipped for the alliance being played. */
    private Translation2d goalTarget() {
        return goalSelector.getTargetPosition(alliance);
    }

    /** Distance from the shooter to the selected goal, standing still. */
    private double standingDistance() {
        return AimLogic.shooterDistanceTo(
                drive.getPose(), goalTarget(), ShootingConstants.AIM_CONFIG);
    }

    /**
     * Records the current distance and setpoints as a pasteable builder row.
     *
     * Uses the STANDING distance, not the moving effective distance: a tuning shot
     * should be taken stationary, and the standing distance is the honest key for
     * the table. Flight time is left as a placeholder because it has to be timed
     * off video.
     */
    private void logRow() {
        // Tagged with the goal, because each goal needs its own table when the
        // heights differ -- rows from two goals must not end up in one map.
        loggedRows.add(String.format("[%s] .add(%8.1f, %6.0f, %6.1f,  /* time me */ 0.00)",
                goalSelector.getSelected().getName(),
                standingDistance(), manualRpm, manualHoodDeg));
    }

    @Override
    public void periodic() {
        if (opModeInInit()) {
            telemetry.addLine("Shooter Map Tuning");
            telemetry.addData("Alliance", "%s   (X = red, Y = blue)", alliance);
            telemetry.addData("Goal", goalSelector.getSelected().toString());
            telemetry.addLine("RB cycles goals (each needs its own table)");
            telemetry.addLine();
            telemetry.addData("Start pose check",
                    drive.getLocalization().getStartPoseCheck());
            telemetry.addLine();
            telemetry.addLine("Check DISTANCE against a tape measure before trusting");
            telemetry.addLine("anything -- if it disagrees, the goal position is wrong.");
            return;
        }

        telemetry.addLine("=== SHOOTER MAP TUNING (shot map bypassed) ===");
        telemetry.addData("Goal", "%s   (RB to cycle)", goalSelector.getSelected().getName());
        telemetry.addData("Tags seen", drive.getLocalization().getVisibleTagIds().toString());
        if (solution != null) {
            telemetry.addData("DISTANCE (standing)", "%.1f in", standingDistance());
            telemetry.addData("Aimed", gamepad1.left_bumper
                    ? String.format("yes, %.1f deg off",
                            Math.toDegrees(solution.headingErrorFrom(
                                    drive.getPose().getHeading())))
                    : "no - hold LB");
        }
        telemetry.addData("Dialled", "rpm %.0f   hood %.1f deg", manualRpm, manualHoodDeg);
        telemetry.addData("Flywheel", "%.0f rpm  %s",
                shooter.getFlywheelRpm(), shooter.atSpeed() ? "AT SPEED" : "spinning");
        telemetry.addLine("dpad up/down = rpm, left/right = hood, A = log row");
        telemetry.addLine();

        if (loggedRows.isEmpty()) {
            telemetry.addLine("No rows logged yet. Press A once a shot goes in.");
        } else {
            telemetry.addLine("Paste into ShootingConstants.SHOT_MAP:");
            for (String row : loggedRows) {
                telemetry.addLine("  " + row);
            }
            telemetry.addLine("(fill in flight time from slow-motion video)");
        }
        telemetry.addLine();
        drive.addTelemetry(telemetry);
    }

    @Override
    public void onStop() {
        shooter.stop();
        drive.stop();
        drive.getLocalization().stop();
    }
}
