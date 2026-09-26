/*
 * A path heading rule that keeps the shooter pointed at the goal.
 *
 * Drop this on a Path and the follower will drive the route while the robot
 * rotates to lead the goal -- shooting on the move in autonomous, with the path
 * choosing where to go and the aim choosing where to face. Before heading sources
 * existed this was impossible: the follower took heading from the path and the
 * aiming command took it from the shot solution, and since both required the
 * drivetrain the scheduler only ever ran one.
 *
 *   AimAtGoalHeading aim = new AimAtGoalHeading(selector, () -> alliance);
 *   Path p = new Path(curve).setHeadingSource(aim);
 *   ...
 *   aim.getLastSolution()   // the shot to spin up for, after follower.update()
 *
 * It computes the full aiming solution once per loop and caches it, so the command
 * driving the shooter reads the same solution the heading came from rather than
 * solving it a second time.
 *
 * Alliance handling is its own: the GoalSelector flips the goal at run time, which
 * is why AllianceFlip can carry this across a mirrored path untouched.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.HeadingSource;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class AimAtGoalHeading implements HeadingSource {

    private final GoalSelector goalSelector;
    private final Supplier<Alliance> alliance;
    private final Supplier<List<Integer>> visibleTagIds;

    private AimSolution lastSolution;
    private Goal lastGoal;

    /**
     * @param goalSelector picks which goal to aim at.
     * @param alliance which alliance is being played, for flipping the goal.
     * @param visibleTagIds tags in the current camera frame, for goal selection.
     *     Pass {@code localization::getVisibleTagIds}.
     */
    public AimAtGoalHeading(GoalSelector goalSelector, Supplier<Alliance> alliance,
                            Supplier<List<Integer>> visibleTagIds) {
        this.goalSelector = goalSelector;
        this.alliance = alliance;
        this.visibleTagIds = visibleTagIds;
    }

    /** As above, without tag-based goal selection (geometry only). */
    public AimAtGoalHeading(GoalSelector goalSelector, Supplier<Alliance> alliance) {
        this(goalSelector, alliance, Collections::emptyList);
    }

    @Override
    public Target compute(double t, Pose2d pose, Translation2d fieldVelocity,
                          double omegaRadPerSec) {
        Alliance playing = alliance.get();
        lastGoal = goalSelector.update(
                pose, fieldVelocity, omegaRadPerSec, visibleTagIds.get(), playing);

        lastSolution = AimLogic.calculate(
                pose, fieldVelocity, omegaRadPerSec,
                GoalSelector.positionFor(lastGoal, playing),
                lastGoal.getMap(),
                ShootingConstants.AIM_CONFIG.withGoalRadius(lastGoal.getRadiusInches()));

        return new Target(lastSolution.targetHeadingRadians,
                lastSolution.headingFeedforwardRadPerSec);
    }

    @Override
    public void reset() {
        lastSolution = null;
        lastGoal = null;
    }

    /**
     * The solution behind the current heading. Null until the follower has run a
     * loop. Read this AFTER {@code follower.update()} so it reflects this loop.
     */
    public AimSolution getLastSolution() {
        return lastSolution;
    }

    /** The goal currently being aimed at, or null before the first loop. */
    public Goal getLastGoal() {
        return lastGoal;
    }

    public GoalSelector getGoalSelector() {
        return goalSelector;
    }
}
