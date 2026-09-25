/*
 * Shoot on the move. The command that ties everything together.
 *
 * Every loop it:
 *   1. asks AimLogic where to point, given the pose, velocity and shot table,
 *   2. drives with the DRIVER's translation but the AIMED heading,
 *   3. spins the flywheel and sets the hood for the resulting distance,
 *   4. runs the feeder only once everything genuinely lines up.
 *
 * The robot never has to stop. The driver keeps translating and the aiming
 * solution owns heading, which is the whole point of doing this without a turret.
 *
 * READINESS. The feeder waits for all of:
 *   - the shot is in range and inside the shot table's measured data,
 *   - the robot is pointed within the distance-scaled heading tolerance,
 *   - the flywheel is at speed,
 *   - the pose is trustworthy.
 *
 * That last one matters more than it sounds. Aiming is only as good as the pose,
 * and a pose seeded 180 degrees out looks perfectly healthy from the inside --
 * so this refuses to fire until vision has vouched for the heading. Without that
 * check a mis-seeded robot will confidently shoot at empty field.
 */
package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.lib.command.Command;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.shooting.AimLogic;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
import org.firstinspires.ftc.teamcode.shooting.Goal;
import org.firstinspires.ftc.teamcode.shooting.GoalSelector;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimAndShootCommand extends Command {

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final DoubleSupplier driverForward;
    private final DoubleSupplier driverLeft;
    private final Supplier<Alliance> alliance;
    private final BooleanSupplier fireRequested;

    /** When false, aim and spin up but never feed. */
    private final boolean allowFeed;
    private final GoalSelector goalSelector;

    private AimSolution solution;
    private Goal targetGoal;
    private boolean lastReady = false;

    /**
     * @param drive the drivetrain; this command takes over its heading.
     * @param shooter the shooter.
     * @param driverForward driver's forward stick, [-1, 1]. Translation stays
     *     with the driver throughout.
     * @param driverLeft driver's left stick, [-1, 1].
     * @param alliance which alliance is being played, for flipping the goal.
     * @param fireRequested true while the operator wants pieces fed. Aiming and
     *     spin-up happen regardless, so the shot is ready the instant it is asked
     *     for.
     */
    public AimAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                              DoubleSupplier driverForward, DoubleSupplier driverLeft,
                              Supplier<Alliance> alliance, BooleanSupplier fireRequested) {
        this(drive, shooter, driverForward, driverLeft, alliance, fireRequested, true,
                ShootingConstants.newGoalSelector());
    }

    /** As above, sharing a GoalSelector so the OpMode can read or override it. */
    public AimAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                              DoubleSupplier driverForward, DoubleSupplier driverLeft,
                              Supplier<Alliance> alliance, BooleanSupplier fireRequested,
                              GoalSelector goalSelector) {
        this(drive, shooter, driverForward, driverLeft, alliance, fireRequested, true,
                goalSelector);
    }

    public AimAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                              DoubleSupplier driverForward, DoubleSupplier driverLeft,
                              Supplier<Alliance> alliance, BooleanSupplier fireRequested,
                              boolean allowFeed, GoalSelector goalSelector) {
        this.goalSelector = goalSelector;
        this.drive = drive;
        this.shooter = shooter;
        this.driverForward = driverForward;
        this.driverLeft = driverLeft;
        this.alliance = alliance;
        this.fireRequested = fireRequested;
        this.allowFeed = allowFeed;
        addRequirements(drive, shooter);
        withName("AimAndShoot");
    }

    @Override
    public void initialize() {
        drive.resetHeadingController();
        lastReady = false;
    }

    @Override
    public void execute() {
        Alliance playing = alliance.get();

        // Which goal to shoot at. The selector prefers goals the camera can
        // identify by their AprilTags, and will not switch on a single flickering
        // frame -- on a turretless robot a switch moves the whole chassis.
        targetGoal = goalSelector.update(
                drive.getPose(), drive.getLocalization().getVisibleTagIds(), playing);
        Translation2d goal = GoalSelector.positionFor(targetGoal, playing);

        solution = AimLogic.calculate(
                drive.getPose(),
                drive.getFieldVelocity(),
                drive.getAngularVelocity(),
                goal,
                targetGoal.getMap(),
                // This goal's opening may be a different size from the default.
                ShootingConstants.AIM_CONFIG.withGoalRadius(targetGoal.getRadiusInches()));

        // Driver keeps translation; the solution owns heading. The feedforward is
        // what lets the robot track a sweeping aim instead of trailing it.
        Translation2d fieldVector = new Translation2d(
                driverForward.getAsDouble(), driverLeft.getAsDouble())
                .rotateBy(playing.driverForward());
        drive.driveWithHeadingLock(fieldVector.getX(), fieldVector.getY(),
                solution.targetHeadingRadians, solution.headingFeedforwardRadPerSec);

        // Spin up for the EFFECTIVE distance -- the distance the shot actually
        // has to cover given the robot's motion, not the straight-line distance.
        shooter.setShotFrom(targetGoal.getMap(), solution.effectiveDistanceInches);

        lastReady = isReadyToFire();
        boolean feeding = allowFeed && lastReady && fireRequested.getAsBoolean();
        if (feeding) {
            shooter.runFeeder();
        } else {
            shooter.stopFeeder();
        }

        // Never let the target change out from under a shot in progress.
        goalSelector.freeze(feeding);
    }

    /** Every condition that must hold before a piece is fed. */
    public boolean isReadyToFire() {
        if (solution == null) {
            return false;
        }
        return solution.canShootFrom(drive.getPose().getHeading())
                && shooter.atSpeed()
                && isPoseTrustworthy();
    }

    /**
     * Whether the pose can be trusted enough to shoot on.
     *
     * With vision enabled this demands that MegaTag1 has vouched for the heading,
     * which is what catches a robot seeded backwards. With vision disabled the
     * check cannot be made, so it passes and the shot rests on the seeded pose --
     * as it must, since there is nothing else to go on.
     */
    private boolean isPoseTrustworthy() {
        return !drive.getLocalization().isVisionEnabled()
                || drive.getLocalization().isHeadingTrusted();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFeeder();
        shooter.idle();
        drive.stop();
        goalSelector.freeze(false);
    }

    /** The goal being aimed at. Null before the first loop. */
    public Goal getTargetGoal() {
        return targetGoal;
    }

    public GoalSelector getGoalSelector() {
        return goalSelector;
    }

    /** The most recent solution, for telemetry. Null before the first loop. */
    public AimSolution getSolution() {
        return solution;
    }

    public boolean wasReady() {
        return lastReady;
    }

    /** One line describing what is blocking the shot, for telemetry. */
    public String getStatus() {
        if (solution == null) {
            return "no solution yet";
        }
        if (!solution.inRange) {
            return String.format("%s out of range (%.0f in)",
                    targetGoal.getName(), solution.effectiveDistanceInches);
        }
        if (!isPoseTrustworthy()) {
            return "pose not trusted - vision has not vouched for the heading";
        }
        if (!solution.isAimedFrom(drive.getPose().getHeading())) {
            return String.format("turning (%.1f deg off, need %.1f)",
                    Math.toDegrees(solution.headingErrorFrom(drive.getPose().getHeading())),
                    Math.toDegrees(solution.headingToleranceRadians));
        }
        if (!shooter.atSpeed()) {
            return String.format("spinning up (%.0f / %.0f rpm)",
                    shooter.getFlywheelRpm(), shooter.getFlywheelSetpointRpm());
        }
        return "READY";
    }
}
