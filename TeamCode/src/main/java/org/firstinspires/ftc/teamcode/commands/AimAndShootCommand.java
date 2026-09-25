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
import org.firstinspires.ftc.teamcode.pathing.AllianceFlip;
import org.firstinspires.ftc.teamcode.shooting.AimLogic;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
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

    private AimSolution solution;
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
        this(drive, shooter, driverForward, driverLeft, alliance, fireRequested, true);
    }

    public AimAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                              DoubleSupplier driverForward, DoubleSupplier driverLeft,
                              Supplier<Alliance> alliance, BooleanSupplier fireRequested,
                              boolean allowFeed) {
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
        // The goal lives on one side of the field; flip it for the other.
        Translation2d goal = AllianceFlip.forAlliance(
                ShootingConstants.GOAL_POSITION, ShootingConstants.AUTHORED_FOR,
                alliance.get());

        solution = AimLogic.calculate(
                drive.getPose(),
                drive.getFieldVelocity(),
                drive.getAngularVelocity(),
                goal,
                shooter.getMap(),
                ShootingConstants.AIM_CONFIG);

        // Driver keeps translation; the solution owns heading. The feedforward is
        // what lets the robot track a sweeping aim instead of trailing it.
        Translation2d fieldVector = new Translation2d(
                driverForward.getAsDouble(), driverLeft.getAsDouble())
                .rotateBy(alliance.get().driverForward());
        drive.driveWithHeadingLock(fieldVector.getX(), fieldVector.getY(),
                solution.targetHeadingRadians, solution.headingFeedforwardRadPerSec);

        // Spin up for the EFFECTIVE distance -- the distance the shot actually
        // has to cover given the robot's motion, not the straight-line distance.
        shooter.setShotForDistance(solution.effectiveDistanceInches);

        lastReady = isReadyToFire();
        if (allowFeed && lastReady && fireRequested.getAsBoolean()) {
            shooter.runFeeder();
        } else {
            shooter.stopFeeder();
        }
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
            return String.format("out of range (%.0f in)", solution.effectiveDistanceInches);
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
