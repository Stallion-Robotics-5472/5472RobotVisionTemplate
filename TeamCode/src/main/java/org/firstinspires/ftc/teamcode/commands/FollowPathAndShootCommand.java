/*
 * Shoot on the move in autonomous: follow a path while tracking the goal.
 *
 * The path decides where the robot GOES; the aiming solution decides where it
 * FACES. Both at once, which is only possible because heading comes from a
 * HeadingSource -- see AimAtGoalHeading.
 *
 *   AimAtGoalHeading aim = new AimAtGoalHeading(selector, () -> alliance,
 *           localization::getVisibleTagIds);
 *   PathChain chain = new PathChain(
 *           new Path(curve).setHeadingSource(aim)
 *                          .addMarker(PathMarker.atT(0.3, () -> {})));
 *   schedule(new FollowPathAndShootCommand(drive, shooter, chain, aim));
 *
 * The solution is computed once, inside the heading source, and read back here --
 * so the shooter spins up for exactly the shot the robot is aiming.
 */
package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.lib.command.Command;
import org.firstinspires.ftc.teamcode.pathing.Follower;
import org.firstinspires.ftc.teamcode.pathing.PathChain;
import org.firstinspires.ftc.teamcode.shooting.AimAtGoalHeading;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class FollowPathAndShootCommand extends Command {

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final PathChain chain;
    private final AimAtGoalHeading aim;
    private final Follower follower;
    private final boolean feedWhenReady;

    private int shotsFedLoops = 0;

    /**
     * @param chain the route. At least one path should use {@code aim} as its
     *     heading source, or the robot will drive without tracking anything.
     * @param aim the shared heading source, so this command reads back the same
     *     solution the heading came from instead of solving twice.
     */
    public FollowPathAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                                     PathChain chain, AimAtGoalHeading aim) {
        this(drive, shooter, chain, aim, true);
    }

    /**
     * @param feedWhenReady false to aim and spin up but never feed -- useful for a
     *     leg that should arrive already up to speed without spending game pieces.
     */
    public FollowPathAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                                     PathChain chain, AimAtGoalHeading aim,
                                     boolean feedWhenReady) {
        this.drive = drive;
        this.shooter = shooter;
        this.chain = chain;
        this.aim = aim;
        this.feedWhenReady = feedWhenReady;

        this.follower = new Follower(drive.getLocalization(), drive.getDrivetrain());
        // DriveSubsystem.periodic() already advances the pose estimate. Letting the
        // follower do it too would feed the estimator two samples microseconds
        // apart with no movement between them, dragging the filtered velocity
        // toward zero -- and the whole moving-shot correction scales with velocity.
        this.follower.setUpdatesLocalizer(false);

        addRequirements(drive, shooter);
        withName("FollowPathAndShoot");
    }

    @Override
    public void initialize() {
        shotsFedLoops = 0;
        follower.followPath(chain);
    }

    @Override
    public void execute() {
        // The follower steers, and asks the heading source for the aim as it goes.
        follower.update();

        // Read back the solution the heading came from.
        AimSolution solution = aim.getLastSolution();
        if (solution == null) {
            // No path on this chain uses the aiming heading source, so there is no
            // shot to prepare. Keep the shooter warm and just drive the route.
            shooter.idle();
            return;
        }

        shooter.setShotFrom(aim.getLastGoal().getMap(), solution.effectiveDistanceInches);

        boolean ready = solution.canShootFrom(drive.getPose().getHeading())
                && shooter.atSpeed()
                && isPoseTrustworthy();
        if (feedWhenReady && ready) {
            shooter.runFeeder();
            shotsFedLoops++;
        } else {
            shooter.stopFeeder();
        }
        // Do not let the target change out from under a shot in progress.
        aim.getGoalSelector().freeze(feedWhenReady && ready);
    }

    /**
     * Same gate the TeleOp command uses: a pose seeded backwards looks perfectly
     * healthy from the inside, so refuse to fire until vision has vouched for the
     * heading. With vision disabled the check cannot be made and passes.
     */
    private boolean isPoseTrustworthy() {
        return !drive.getLocalization().isVisionEnabled()
                || drive.getLocalization().isHeadingTrusted();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        follower.stop();
        shooter.stopFeeder();
        shooter.idle();
        drive.stop();
        aim.getGoalSelector().freeze(false);
    }

    /** Loops spent feeding, as a rough proxy for shots taken. */
    public int getShotsFedLoops() {
        return shotsFedLoops;
    }

    public Follower getFollower() {
        return follower;
    }

    /** One line describing what the command is doing, for telemetry. */
    public String getStatus() {
        AimSolution solution = aim.getLastSolution();
        if (solution == null) {
            return String.format("driving, seg %d (no aim source on this path)",
                    follower.getPathIndex());
        }
        if (!solution.inRange) {
            return String.format("seg %d, %s out of range (%.0f in)",
                    follower.getPathIndex(), aim.getLastGoal().getName(),
                    solution.effectiveDistanceInches);
        }
        if (!isPoseTrustworthy()) {
            return "pose not trusted - vision has not vouched for the heading";
        }
        if (!solution.isAimedFrom(drive.getPose().getHeading())) {
            return String.format("seg %d, turning (%.1f deg off)",
                    follower.getPathIndex(),
                    Math.toDegrees(solution.headingErrorFrom(drive.getPose().getHeading())));
        }
        if (!shooter.atSpeed()) {
            return String.format("seg %d, spinning up (%.0f/%.0f rpm)",
                    follower.getPathIndex(), shooter.getFlywheelRpm(),
                    shooter.getFlywheelSetpointRpm());
        }
        return String.format("seg %d, SHOOTING", follower.getPathIndex());
    }
}
