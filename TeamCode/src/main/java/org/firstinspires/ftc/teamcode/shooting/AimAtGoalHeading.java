/*
 * A path heading rule that keeps the shooter pointed at a goal from the goal table.
 *
 * This is {@link AimAtTargetHeading} wired to a {@link GoalTargetSource}: the goal
 * is a configured field coordinate, chosen by GoalSelector from what the camera can
 * identify and how far away each one is. Use it when the robot should aim at a goal
 * it may not currently be able to see -- crossing the field while tracking a goal
 * behind it, which the camera-relative mode cannot do.
 *
 *   AimAtGoalHeading aim = new AimAtGoalHeading(selector, () -> alliance,
 *           localization::getVisibleTagIds);
 *   Path p = new Path(curve).setHeadingSource(aim);
 *   ...
 *   aim.getLastSolution()   // the shot to spin up for, after follower.update()
 *
 * For the other mode -- aim at whatever the camera is looking at, no goal table and
 * no reliance on the pose estimate -- use AimAtTargetHeading with a
 * VisionTargetSource. Everything downstream is identical.
 *
 * Alliance handling is its own: the GoalSelector flips the goal at run time, which
 * is why AllianceFlip can carry this across a mirrored path untouched.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.pathing.Alliance;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class AimAtGoalHeading extends AimAtTargetHeading {

    private final GoalTargetSource goalSource;

    /**
     * @param goalSelector picks which goal to aim at.
     * @param alliance which alliance is being played, for flipping the goal.
     * @param visibleTagIds tags in the current camera frame, for goal selection.
     *     Pass {@code localization::getVisibleTagIds}.
     */
    public AimAtGoalHeading(GoalSelector goalSelector, Supplier<Alliance> alliance,
                            Supplier<List<Integer>> visibleTagIds) {
        this(new GoalTargetSource(goalSelector, visibleTagIds), alliance);
    }

    /** As above, without tag-based goal selection (geometry only). */
    public AimAtGoalHeading(GoalSelector goalSelector, Supplier<Alliance> alliance) {
        this(goalSelector, alliance, Collections::emptyList);
    }

    private AimAtGoalHeading(GoalTargetSource goalSource, Supplier<Alliance> alliance) {
        super(goalSource, alliance);
        this.goalSource = goalSource;
    }

    /** The goal currently being aimed at, or null before the first loop. */
    public Goal getLastGoal() {
        return goalSource.getLastGoal();
    }

    public GoalSelector getGoalSelector() {
        return goalSource.getGoalSelector();
    }
}
