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
 * That last one matters more than it sounds. Aiming at a coordinate from the goal
 * table is only as good as the pose, and a pose seeded 180 degrees out looks
 * perfectly healthy from the inside -- so this refuses to fire until vision has
 * vouched for the heading. Without that check a mis-seeded robot will confidently
 * shoot at empty field.
 *
 * It is the TARGET SOURCE that decides whether that gate applies. A source that
 * measures the target relative to the robot (VisionTargetSource) has no dependence
 * on the field pose at all, so demanding a trusted pose would refuse a shot that is
 * actually correct -- in exactly the situation that mode exists for. The two modes
 * are otherwise identical from here down.
 */
package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.lib.command.Command;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.shooting.AimLogic;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
import org.firstinspires.ftc.teamcode.shooting.Goal;
import org.firstinspires.ftc.teamcode.shooting.GoalSelector;
import org.firstinspires.ftc.teamcode.shooting.GoalTargetSource;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;
import org.firstinspires.ftc.teamcode.shooting.TargetSource;
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
    private final TargetSource targetSource;

    private AimSolution solution;
    private TargetSource.Target target;
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
        this(drive, shooter, driverForward, driverLeft, alliance, fireRequested,
                allowFeed, new GoalTargetSource(goalSelector,
                        drive.getLocalization()::getVisibleTagIds));
    }

    /**
     * The general form: any {@link TargetSource}.
     *
     * Pass a {@code VisionTargetSource} to shoot at whatever the camera is looking
     * at instead of at a goal from the table -- no goal coordinates, and no reliance
     * on the field pose being right.
     */
    public AimAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                              DoubleSupplier driverForward, DoubleSupplier driverLeft,
                              Supplier<Alliance> alliance, BooleanSupplier fireRequested,
                              boolean allowFeed, TargetSource targetSource) {
        this.targetSource = targetSource;
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
        targetSource.reset();
        lastReady = false;
    }

    @Override
    public void execute() {
        Alliance playing = alliance.get();

        // What to shoot at. A goal source prefers goals the camera can identify and
        // will not switch on a single flickering frame -- on a turretless robot a
        // switch moves the whole chassis. A vision source ranges off the frame.
        target = targetSource.update(
                drive.getPose(), drive.getFieldVelocity(), drive.getAngularVelocity(),
                playing);

        Translation2d fieldVector = new Translation2d(
                driverForward.getAsDouble(), driverLeft.getAsDouble())
                .rotateBy(playing.driverForward());

        if (target == null) {
            // Nothing to shoot at: hand heading back to nobody in particular and
            // keep driving. Holding the stale aim would point the robot at wherever
            // the target used to be, which is worse than not aiming.
            solution = null;
            drive.driveFieldCentric(fieldVector.getX(), fieldVector.getY(), 0.0);
            shooter.idle();
            shooter.stopFeeder();
            lastReady = false;
            targetSource.freeze(false);
            return;
        }

        solution = AimLogic.calculate(
                drive.getPose(),
                drive.getFieldVelocity(),
                drive.getAngularVelocity(),
                target.fieldPosition,
                target.map,
                // This target's opening may be a different size from the default.
                ShootingConstants.AIM_CONFIG.withGoalRadius(target.radiusInches));

        // Driver keeps translation; the solution owns heading. The feedforward is
        // what lets the robot track a sweeping aim instead of trailing it.
        drive.driveWithHeadingLock(fieldVector.getX(), fieldVector.getY(),
                solution.targetHeadingRadians, solution.headingFeedforwardRadPerSec);

        // Spin up for the EFFECTIVE distance -- the distance the shot actually
        // has to cover given the robot's motion, not the straight-line distance.
        shooter.setShotFrom(target.map, solution.effectiveDistanceInches);

        lastReady = isReadyToFire();
        boolean feeding = allowFeed && lastReady && fireRequested.getAsBoolean();
        if (feeding) {
            shooter.runFeeder();
        } else {
            shooter.stopFeeder();
        }

        // Never let the target change out from under a shot in progress.
        targetSource.freeze(feeding);
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
     * With a goal-table source and vision enabled this demands that MegaTag1 (which
     * is solved without the gyro) has vouched for the heading, which is what catches
     * a robot seeded backwards. With vision disabled the check cannot be made, so it
     * passes and the shot rests on the seeded pose -- as it must, since there is
     * nothing else to go on.
     *
     * A source that does not need the pose is not gated on it. That is not a
     * loosened check: the pose is not an input to that answer, so there is nothing
     * for it to be wrong about.
     */
    private boolean isPoseTrustworthy() {
        return !targetSource.requiresTrustedPose()
                || !drive.getLocalization().isVisionEnabled()
                || drive.getLocalization().isHeadingTrusted();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFeeder();
        shooter.idle();
        drive.stop();
        targetSource.freeze(false);
    }

    /** What is being aimed at, or null when there is nothing to shoot at. */
    public TargetSource.Target getTarget() {
        return target;
    }

    public TargetSource getTargetSource() {
        return targetSource;
    }

    /**
     * The goal being aimed at, or null when the source is not a goal table.
     *
     * Kept for the goal-table path, which is what most OpModes use.
     */
    public Goal getTargetGoal() {
        return targetSource instanceof GoalTargetSource
                ? ((GoalTargetSource) targetSource).getLastGoal() : null;
    }

    /** The goal selector, or null when the source is not a goal table. */
    public GoalSelector getGoalSelector() {
        return targetSource instanceof GoalTargetSource
                ? ((GoalTargetSource) targetSource).getGoalSelector() : null;
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
            String reason = targetSource.getRejectReason();
            return reason == null ? "no solution yet" : "no target: " + reason;
        }
        if (!solution.inRange) {
            return String.format("%s out of range (%.0f in)",
                    target.name, solution.effectiveDistanceInches);
        }
        if (!target.measuredThisLoop) {
            return String.format("%s lost from frame %.2fs ago - holding aim",
                    target.name, target.ageSeconds);
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
