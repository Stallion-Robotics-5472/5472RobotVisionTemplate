/*
 * The original way of choosing a target: a goal from the table, picked by
 * GoalSelector and turned into a field coordinate.
 *
 * This is a thin adapter. All of the interesting behaviour -- tag-visibility
 * preference, range filtering, the switch hysteresis that stops a flickering tag
 * swinging a turretless chassis between two headings -- lives in GoalSelector and
 * is unchanged. This exists so the aiming code can take either this or
 * VisionTargetSource without knowing which.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class GoalTargetSource implements TargetSource {

    private final GoalSelector selector;
    private final Supplier<List<Integer>> visibleTagIds;

    private Goal lastGoal;

    /**
     * @param visibleTagIds tags in the current frame, for TAG_VISIBLE selection.
     *     Pass {@code localization::getVisibleTagIds}.
     */
    public GoalTargetSource(GoalSelector selector, Supplier<List<Integer>> visibleTagIds) {
        this.selector = selector;
        this.visibleTagIds = visibleTagIds;
    }

    /** Geometry-only selection, with no tag information. */
    public GoalTargetSource(GoalSelector selector) {
        this(selector, Collections::emptyList);
    }

    @Override
    public Target update(Pose2d pose, Translation2d fieldVelocity, double omegaRadPerSec,
                         Alliance alliance) {
        lastGoal = selector.update(
                pose, fieldVelocity, omegaRadPerSec, visibleTagIds.get(), alliance);
        // A selector always returns something -- it falls back to the nearest goal
        // rather than refusing, so the robot is already pointed when it drives into
        // range. So this source never has "no target".
        return Target.fresh(lastGoal.getName(),
                GoalSelector.positionFor(lastGoal, alliance),
                lastGoal.getMap(),
                lastGoal.getRadiusInches());
    }

    @Override
    public void freeze(boolean frozen) {
        selector.freeze(frozen);
    }

    @Override
    public String describe() {
        return selector.describe();
    }

    /** The goal chosen on the last update, or null before the first one. */
    public Goal getLastGoal() {
        return lastGoal;
    }

    public GoalSelector getGoalSelector() {
        return selector;
    }
}
