/*
 * Decides which goal to shoot at, and holds that decision steady.
 *
 * The strategies are ordinary enough. The important part is the HYSTERESIS.
 *
 * On a robot with no turret, the selected goal sets the whole robot's heading. If
 * selection flickered -- and AprilTag visibility flickers constantly as the robot
 * drives, as a tag clips the edge of frame or a piece passes in front of it -- the
 * robot would swing back and forth between two headings and never settle enough to
 * shoot at either. A turret could get away with that. A chassis cannot.
 *
 * So a switch has to be EARNED: a challenger must beat the incumbent for
 * {@code switchFrames} consecutive updates before it takes over, and while a shot
 * is in progress selection is frozen outright. The cost is a fraction of a second
 * of staleness after the picture genuinely changes, which is far cheaper than a
 * robot oscillating between two goals.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.AllianceFlip;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class GoalSelector {

    /** How the goal is chosen. */
    public enum Strategy {
        /** Always the goal named by {@link #lockTo}, or the first one. */
        FIXED,
        /** The closest goal to the shooter. */
        NEAREST,
        /** The goal needing the least rotation from where the robot points now. */
        LEAST_ROTATION,
        /**
         * Prefer goals the camera can currently identify by their AprilTags,
         * breaking ties by point value then distance. Falls back to NEAREST when no
         * goal's tags are in frame -- not seeing a tag usually just means the camera
         * is pointed elsewhere, which is no reason to refuse to aim.
         */
        TAG_VISIBLE,
        /** The highest-value goal whose shot is actually in range, else nearest. */
        BEST_VALUE
    }

    /**
     * What a tag being in frame tells you about its goal.
     *
     * Which one is right depends on the season. If tags are simply mounted beside
     * each goal, seeing one means "this is that goal" -- VISIBLE_MEANS_AVAILABLE.
     * If instead a tag gets obscured as its goal fills up or is claimed, then a
     * HIDDEN tag is the available one. Check the game manual and set this to match;
     * getting it backwards makes the robot prefer exactly the wrong goals.
     */
    public enum TagMeaning {
        VISIBLE_MEANS_AVAILABLE,
        HIDDEN_MEANS_AVAILABLE
    }

    private final List<Goal> goals;
    private Strategy strategy;
    private TagMeaning tagMeaning = TagMeaning.VISIBLE_MEANS_AVAILABLE;

    /** Consecutive updates a challenger must win before it takes over. */
    private int switchFrames = 12;

    private Goal selected;
    private Goal challenger;
    private int challengerStreak = 0;
    private boolean frozen = false;
    private Translation2d lastFieldVelocity = new Translation2d();
    private double lastOmega = 0.0;
    private String lastReason = "initial";

    public GoalSelector(Strategy strategy, Goal... goals) {
        if (goals.length == 0) {
            throw new IllegalArgumentException("A GoalSelector needs at least one goal");
        }
        this.goals = Collections.unmodifiableList(new ArrayList<>(java.util.Arrays.asList(goals)));
        this.strategy = strategy;
        this.selected = goals[0];
    }

    public GoalSelector(Strategy strategy, List<Goal> goals) {
        this(strategy, goals.toArray(new Goal[0]));
    }

    // ---------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------

    public GoalSelector withStrategy(Strategy strategy) {
        this.strategy = strategy;
        return this;
    }

    public GoalSelector withTagMeaning(TagMeaning meaning) {
        this.tagMeaning = meaning;
        return this;
    }

    /**
     * Consecutive updates a challenger must win before taking over. Higher is
     * steadier but slower to react; 0 switches instantly, which on a turretless
     * robot invites the oscillation described at the top of this file.
     */
    public GoalSelector withSwitchFrames(int frames) {
        this.switchFrames = Math.max(0, frames);
        return this;
    }

    // ---------------------------------------------------------------------
    // Driver overrides
    // ---------------------------------------------------------------------

    /** Pins selection to one goal by name and stops automatic switching. */
    public void lockTo(String goalName) {
        for (Goal goal : goals) {
            if (goal.getName().equalsIgnoreCase(goalName)) {
                selected = goal;
                strategy = Strategy.FIXED;
                challenger = null;
                challengerStreak = 0;
                lastReason = "locked by driver";
                return;
            }
        }
        throw new IllegalArgumentException("No goal named " + goalName);
    }

    /** Steps to the next goal in order and pins to it. Bind to a bumper. */
    public void cycle() {
        int next = (goals.indexOf(selected) + 1) % goals.size();
        selected = goals.get(next);
        strategy = Strategy.FIXED;
        challenger = null;
        challengerStreak = 0;
        lastReason = "cycled by driver";
    }

    /** Hands control back to an automatic strategy. */
    public void auto(Strategy strategy) {
        this.strategy = strategy;
        lastReason = "auto";
    }

    /**
     * Freezes selection, so a shot in progress cannot have the target changed out
     * from under it. Call with true while feeding.
     */
    public void freeze(boolean frozen) {
        this.frozen = frozen;
    }

    // ---------------------------------------------------------------------
    // The decision
    // ---------------------------------------------------------------------

    /**
     * Re-evaluates the selection. Call once per loop, before aiming.
     *
     * @param robotPose current fused pose.
     * @param visibleTagIds tag IDs in the current frame, from
     *     {@code Localization.getVisibleTagIds()}.
     * @param alliance which alliance is being played, for flipping positions.
     * @return the selected goal.
     */
    public Goal update(Pose2d robotPose, Iterable<Integer> visibleTagIds, Alliance alliance) {
        return update(robotPose, new Translation2d(), 0.0, visibleTagIds, alliance);
    }

    /**
     * Re-evaluates the selection, accounting for the robot's motion.
     *
     * Prefer this overload. Range checks then use the same effective distance the
     * aiming solution uses, so the selector cannot keep a goal the solution has
     * already ruled out -- see {@link AimLogic#effectiveDistanceTo}.
     *
     * @param fieldVelocity field-frame velocity, inches/sec.
     * @param omegaRadPerSec angular velocity, radians/sec CCW.
     */
    public Goal update(Pose2d robotPose, Translation2d fieldVelocity, double omegaRadPerSec,
                       Iterable<Integer> visibleTagIds, Alliance alliance) {
        if (frozen || strategy == Strategy.FIXED || goals.size() == 1) {
            return selected;
        }

        this.lastFieldVelocity = fieldVelocity;
        this.lastOmega = omegaRadPerSec;

        Goal best = evaluate(robotPose, visibleTagIds, alliance);
        if (best == selected) {
            challenger = null;
            challengerStreak = 0;
            return selected;
        }

        // A challenger has to win repeatedly before it takes the robot's heading.
        if (best == challenger) {
            challengerStreak++;
        } else {
            challenger = best;
            challengerStreak = 1;
        }
        if (challengerStreak > switchFrames) {
            selected = best;
            challenger = null;
            challengerStreak = 0;
        }
        return selected;
    }

    private Goal evaluate(Pose2d robotPose, Iterable<Integer> visibleTagIds, Alliance alliance) {
        switch (strategy) {
            case NEAREST:
                lastReason = "nearest";
                return nearest(robotPose, alliance, goals);

            case LEAST_ROTATION:
                lastReason = "least rotation";
                return leastRotation(robotPose, alliance);

            case TAG_VISIBLE: {
                List<Goal> available = availableByTag(visibleTagIds);

                // A goal you cannot reach must never win, whatever the camera sees.
                //
                // This is not a detail. Aiming at a goal points the camera AT that
                // goal, so its tags are the ones in frame, so tag visibility keeps
                // choosing it -- a self-reinforcing lock-in that survives the robot
                // driving clean out of range while another goal sits comfortably
                // shootable. Filtering by range first breaks the loop.
                List<Goal> reachable = inRange(available, robotPose, alliance);
                if (!reachable.isEmpty()) {
                    lastReason = tagMeaning == TagMeaning.VISIBLE_MEANS_AVAILABLE
                            ? "tags in frame, in range" : "tags hidden, in range";
                    return bestValueThenNearest(robotPose, alliance, reachable);
                }

                // Nothing the camera can see is reachable. Fall back to geometry
                // over ALL goals -- including ones we are not currently looking at,
                // which is the whole point.
                List<Goal> anyReachable = inRange(goals, robotPose, alliance);
                if (!anyReachable.isEmpty()) {
                    lastReason = available.isEmpty()
                            ? "no tags in frame - nearest in range"
                            : "tagged goal out of range - nearest in range";
                    return bestValueThenNearest(robotPose, alliance, anyReachable);
                }

                // Nothing is reachable at all. Point at the closest thing so the
                // robot is already aimed by the time it drives into range.
                lastReason = "nothing in range - nearest";
                return nearest(robotPose, alliance, goals);
            }

            case BEST_VALUE: {
                List<Goal> reachable = inRange(goals, robotPose, alliance);
                if (reachable.isEmpty()) {
                    lastReason = "nothing in range - nearest";
                    return nearest(robotPose, alliance, goals);
                }
                lastReason = "best value in range";
                return bestValueThenNearest(robotPose, alliance, reachable);
            }

            case FIXED:
            default:
                return selected;
        }
    }

    /**
     * The subset of {@code candidates} the robot could actually shoot at from here:
     * inside the configured shot range AND inside that goal's own measured table.
     */
    private List<Goal> inRange(List<Goal> candidates, Pose2d pose, Alliance alliance) {
        List<Goal> result = new ArrayList<>();
        for (Goal goal : candidates) {
            double distance = shotDistanceTo(goal, pose, alliance);
            if (goal.getMap().covers(distance)
                    && distance >= ShootingConstants.MIN_SHOT_DISTANCE_IN
                    && distance <= ShootingConstants.MAX_SHOT_DISTANCE_IN) {
                result.add(goal);
            }
        }
        return result;
    }

    /** Goals the tag picture says are available, per {@link TagMeaning}. */
    private List<Goal> availableByTag(Iterable<Integer> visibleTagIds) {
        List<Goal> result = new ArrayList<>();
        for (Goal goal : goals) {
            if (goal.getTagIds().isEmpty()) {
                continue;   // no tags: cannot be judged this way
            }
            boolean seen = goal.isSeenAmong(visibleTagIds);
            boolean available = tagMeaning == TagMeaning.VISIBLE_MEANS_AVAILABLE ? seen : !seen;
            if (available) {
                result.add(goal);
            }
        }
        return result;
    }

    private Goal bestValueThenNearest(Pose2d pose, Alliance alliance, List<Goal> candidates) {
        Goal best = candidates.get(0);
        for (Goal goal : candidates) {
            if (goal.getPointValue() > best.getPointValue()) {
                best = goal;
            } else if (goal.getPointValue() == best.getPointValue()
                    && distanceTo(goal, pose, alliance) < distanceTo(best, pose, alliance)) {
                best = goal;
            }
        }
        return best;
    }

    private Goal nearest(Pose2d pose, Alliance alliance, List<Goal> candidates) {
        Goal best = candidates.get(0);
        for (Goal goal : candidates) {
            if (distanceTo(goal, pose, alliance) < distanceTo(best, pose, alliance)) {
                best = goal;
            }
        }
        return best;
    }

    private Goal leastRotation(Pose2d pose, Alliance alliance) {
        Goal best = goals.get(0);
        double bestTurn = Double.MAX_VALUE;
        for (Goal goal : goals) {
            Translation2d to = positionFor(goal, alliance).minus(pose.getTranslation());
            double turn = Math.abs(AimLogic.wrapRadians(
                    Math.atan2(to.getY(), to.getX())
                            - ShootingConstants.AIM_CONFIG.shooterYawOffsetRadians
                            - pose.getHeading()));
            if (turn < bestTurn) {
                bestTurn = turn;
                best = goal;
            }
        }
        return best;
    }

    /** Straight-line distance, for ranking goals by proximity. */
    private double distanceTo(Goal goal, Pose2d pose, Alliance alliance) {
        return AimLogic.shooterDistanceTo(
                pose, positionFor(goal, alliance), ShootingConstants.AIM_CONFIG);
    }

    /**
     * The distance a shot at this goal would actually have to cover, given the
     * robot's current motion. This is the figure the range check must use so the
     * selector agrees with the aiming solution.
     */
    private double shotDistanceTo(Goal goal, Pose2d pose, Alliance alliance) {
        return AimLogic.effectiveDistanceTo(pose, lastFieldVelocity, lastOmega,
                positionFor(goal, alliance), goal.getMap(),
                ShootingConstants.AIM_CONFIG.withGoalRadius(goal.getRadiusInches()));
    }

    // ---------------------------------------------------------------------
    // Readback
    // ---------------------------------------------------------------------

    public Goal getSelected() {
        return selected;
    }

    /** The selected goal's position, already flipped for {@code alliance}. */
    public Translation2d getTargetPosition(Alliance alliance) {
        return positionFor(selected, alliance);
    }

    /** Any goal's position, flipped for {@code alliance}. */
    public static Translation2d positionFor(Goal goal, Alliance alliance) {
        return AllianceFlip.forAlliance(
                goal.getAuthoredPosition(), ShootingConstants.AUTHORED_FOR, alliance);
    }

    public List<Goal> getGoals() {
        return goals;
    }

    public Strategy getStrategy() {
        return strategy;
    }

    /** Why the current goal is selected, for telemetry. */
    public String getReason() {
        return lastReason;
    }

    /** Goal waiting to take over, or null. Useful for seeing switching happen. */
    public Goal getChallenger() {
        return challenger;
    }

    public int getChallengerStreak() {
        return challengerStreak;
    }

    public boolean isFrozen() {
        return frozen;
    }

    /** One line for telemetry. */
    public String describe() {
        StringBuilder sb = new StringBuilder(selected.getName());
        sb.append(" (").append(strategy).append(": ").append(lastReason).append(")");
        if (frozen) {
            sb.append(" FROZEN");
        }
        if (challenger != null) {
            sb.append(String.format("  <- %s %d/%d",
                    challenger.getName(), challengerStreak, switchFrames + 1));
        }
        return sb.toString();
    }
}
