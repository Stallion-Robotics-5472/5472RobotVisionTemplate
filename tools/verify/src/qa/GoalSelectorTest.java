package qa;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.shooting.Goal;
import org.firstinspires.ftc.teamcode.shooting.GoalSelector;
import org.firstinspires.ftc.teamcode.shooting.ShooterMap;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Exercises goal selection, especially the hysteresis.
 *
 * The hysteresis is the part that matters: on a turretless robot the selected goal
 * sets the whole chassis heading, so a selector that flipped on one flickering
 * AprilTag frame would swing the robot between two headings and never settle
 * enough to shoot at either.
 */
public class GoalSelectorTest {
    static int fails = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-56s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    static final ShooterMap LOW_MAP = ShooterMap.builder()
            .add(12, 1500, 15.0, 0.20).add(96, 3000, 30.0, 0.70).build();
    static final ShooterMap HIGH_MAP = ShooterMap.builder()
            .add(12, 2000, 30.0, 0.28).add(96, 3900, 45.0, 0.90).build();

    static Goal left() {
        return Goal.named("left", -40, 60).tags(11, 12).worth(2).map(LOW_MAP).build();
    }

    static Goal right() {
        return Goal.named("right", 40, 60).tags(21, 22).worth(5).map(HIGH_MAP)
                .radius(10.0).build();
    }

    static List<Integer> tags(int... ids) {
        List<Integer> list = new java.util.ArrayList<>();
        for (int id : ids) list.add(id);
        return list;
    }

    static final Pose2d ORIGIN = new Pose2d(0, 0, new Rotation2d(0));

    public static void main(String[] args) {
        System.out.println("=== Goal basics ===");
        Goal l = left();
        check("A goal carries its tag IDs",
                l.getTagIds().contains(11) && l.getTagIds().contains(12), l.toString());
        check("A goal can carry its own shot table", l.getMap() == LOW_MAP, "wrong map");
        check("A goal without a map falls back to the shared default",
                Goal.named("plain", 0, 0).build().getMap() == ShootingConstants.SHOT_MAP,
                "no fallback");
        check("A goal can override the opening size",
                right().getRadiusInches() == 10.0, "" + right().getRadiusInches());
        check("isSeenAmong matches on any of its tags",
                l.isSeenAmong(tags(99, 12)) && !l.isSeenAmong(tags(21, 22)), "wrong");
        check("A goal with no tags is never 'seen'",
                !Goal.named("plain", 0, 0).build().isSeenAmong(tags(1, 2, 3)), "seen");

        System.out.println("\n=== Alliance flipping ===");
        Goal g = Goal.named("g", 30, 60).build();
        Translation2d red = GoalSelector.positionFor(g, Alliance.RED);
        Translation2d blue = GoalSelector.positionFor(g, Alliance.BLUE);
        System.out.printf("   authored (30, 60) -> red (%.0f, %.0f), blue (%.0f, %.0f)%n",
                red.getX(), red.getY(), blue.getX(), blue.getY());
        check("Goal positions flip with the alliance",
                Math.abs(blue.getX() + red.getX()) < 1e-9
                        && Math.abs(blue.getY() + red.getY()) < 1e-9,
                blue.toString());

        System.out.println("\n=== NEAREST ===");
        GoalSelector near = new GoalSelector(GoalSelector.Strategy.NEAREST, left(), right())
                .withSwitchFrames(0);
        Goal pick = near.update(new Pose2d(-50, 30, new Rotation2d(0)),
                Collections.emptyList(), Alliance.RED);
        check("Picks the closer goal", pick.getName().equals("left"), pick.getName());
        pick = near.update(new Pose2d(50, 30, new Rotation2d(0)),
                Collections.emptyList(), Alliance.RED);
        check("Follows the robot across the field", pick.getName().equals("right"),
                pick.getName());

        System.out.println("\n=== TAG_VISIBLE ===");
        GoalSelector byTag = new GoalSelector(GoalSelector.Strategy.TAG_VISIBLE,
                left(), right()).withSwitchFrames(0);
        // Robot sits closer to the LEFT goal, but only the RIGHT goal's tag is seen.
        Pose2d nearLeft = new Pose2d(-50, 20, new Rotation2d(0));
        pick = byTag.update(nearLeft, tags(21), Alliance.RED);
        check("Prefers the goal whose tag is in frame, over the nearer one",
                pick.getName().equals("right"), pick.getName() + " / " + byTag.getReason());
        pick = byTag.update(nearLeft, tags(11), Alliance.RED);
        check("Follows the tag picture", pick.getName().equals("left"),
                pick.getName() + " / " + byTag.getReason());

        // Both visible -> break the tie by point value.
        pick = byTag.update(nearLeft, tags(11, 21), Alliance.RED);
        check("With both tags in frame, prefers the higher-value goal",
                pick.getName().equals("right"), pick.getName());

        // No tags at all must fall back, not refuse: not seeing a tag usually just
        // means the camera is pointed elsewhere.
        GoalSelector noTags = new GoalSelector(GoalSelector.Strategy.TAG_VISIBLE,
                left(), right()).withSwitchFrames(0);
        pick = noTags.update(nearLeft, Collections.emptyList(), Alliance.RED);
        check("With no tags in frame, falls back to nearest",
                pick.getName().equals("left"),
                pick.getName() + " / " + noTags.getReason());
        System.out.printf("   fallback reason: %s%n", noTags.getReason());

        System.out.println("\n=== HIDDEN_MEANS_AVAILABLE (inverted tag semantics) ===");
        GoalSelector inverted = new GoalSelector(GoalSelector.Strategy.TAG_VISIBLE,
                left(), right())
                .withTagMeaning(GoalSelector.TagMeaning.HIDDEN_MEANS_AVAILABLE)
                .withSwitchFrames(0);
        // Right's tag is visible, so under inverted semantics LEFT is the open one.
        pick = inverted.update(new Pose2d(0, 0, new Rotation2d(0)), tags(21), Alliance.RED);
        check("Inverted semantics prefer the goal whose tag is hidden",
                pick.getName().equals("left"), pick.getName());

        System.out.println("\n=== HYSTERESIS (the important one) ===");
        GoalSelector steady = new GoalSelector(GoalSelector.Strategy.TAG_VISIBLE,
                left(), right()).withSwitchFrames(5);
        // Settle on left.
        for (int i = 0; i < 10; i++) {
            steady.update(nearLeft, tags(11), Alliance.RED);
        }
        check("Settles on a goal", steady.getSelected().getName().equals("left"),
                steady.getSelected().getName());

        // A single flickering frame favouring the other goal must NOT move the robot.
        Goal afterOneFlicker = steady.update(nearLeft, tags(21), Alliance.RED);
        check("One flickering frame does not switch the target",
                afterOneFlicker.getName().equals("left"),
                "switched on a single frame - the robot would oscillate");

        // A few more still should not, until the streak is met.
        for (int i = 0; i < 4; i++) {
            steady.update(nearLeft, tags(21), Alliance.RED);
        }
        check("Still held after 5 of 6 required frames",
                steady.getSelected().getName().equals("left"),
                "switched early at streak " + steady.getChallengerStreak());
        Goal afterStreak = steady.update(nearLeft, tags(21), Alliance.RED);
        check("Switches once the challenger has earned it",
                afterStreak.getName().equals("right"), afterStreak.getName());

        // An intermittent challenger never accumulates a streak.
        GoalSelector flaky = new GoalSelector(GoalSelector.Strategy.TAG_VISIBLE,
                left(), right()).withSwitchFrames(5);
        for (int i = 0; i < 10; i++) flaky.update(nearLeft, tags(11), Alliance.RED);
        boolean everSwitched = false;
        for (int i = 0; i < 40; i++) {
            // alternating frames, as a tag clipping the edge of frame would give
            flaky.update(nearLeft, i % 2 == 0 ? tags(21) : tags(11), Alliance.RED);
            if (!flaky.getSelected().getName().equals("left")) everSwitched = true;
        }
        check("An intermittent challenger never takes over", !everSwitched,
                "switched on a flickering tag");

        System.out.println("\n=== Freeze during a shot ===");
        GoalSelector frozen = new GoalSelector(GoalSelector.Strategy.TAG_VISIBLE,
                left(), right()).withSwitchFrames(0);
        for (int i = 0; i < 3; i++) frozen.update(nearLeft, tags(11), Alliance.RED);
        frozen.freeze(true);
        for (int i = 0; i < 20; i++) frozen.update(nearLeft, tags(21), Alliance.RED);
        check("Frozen selection cannot change mid-shot",
                frozen.getSelected().getName().equals("left"),
                "target changed while feeding");
        frozen.freeze(false);
        Goal thawed = frozen.update(nearLeft, tags(21), Alliance.RED);
        check("Unfreezing resumes selection", thawed.getName().equals("right"),
                thawed.getName());

        System.out.println("\n=== Driver override ===");
        GoalSelector driver = new GoalSelector(GoalSelector.Strategy.TAG_VISIBLE,
                left(), right()).withSwitchFrames(0);
        driver.lockTo("right");
        for (int i = 0; i < 20; i++) driver.update(nearLeft, tags(11), Alliance.RED);
        check("A driver lock overrides the tag picture",
                driver.getSelected().getName().equals("right"), driver.getSelected().getName());
        check("Locking switches the strategy to FIXED",
                driver.getStrategy() == GoalSelector.Strategy.FIXED, "" + driver.getStrategy());
        driver.cycle();
        check("cycle() steps to the next goal",
                driver.getSelected().getName().equals("left"), driver.getSelected().getName());
        driver.auto(GoalSelector.Strategy.TAG_VISIBLE);
        Goal back = driver.update(nearLeft, tags(21), Alliance.RED);
        check("auto() hands control back", back.getName().equals("right"), back.getName());
        boolean badName = false;
        try {
            driver.lockTo("nonexistent");
        } catch (IllegalArgumentException e) {
            badName = true;
        }
        check("Locking to an unknown goal is rejected", badName, "accepted");

        System.out.println("\n=== BEST_VALUE respects range ===");
        // Right is worth more, but park far enough that only its map covers us.
        GoalSelector value = new GoalSelector(GoalSelector.Strategy.BEST_VALUE,
                left(), right()).withSwitchFrames(0);
        pick = value.update(new Pose2d(35, 20, new Rotation2d(0)),
                Collections.emptyList(), Alliance.RED);
        check("Picks a reachable goal", pick != null && value.getReason() != null,
                "null");
        System.out.printf("   picked %s (%s)%n", pick.getName(), value.getReason());

        System.out.println("\n=== Single goal degrades to a no-op ===");
        GoalSelector one = new GoalSelector(GoalSelector.Strategy.TAG_VISIBLE, left());
        for (int i = 0; i < 5; i++) {
            check("Single-goal selector always returns it",
                    one.update(ORIGIN, tags(99), Alliance.RED).getName().equals("left"),
                    "changed");
            if (fails > 0) break;
        }
        boolean emptySet = false;
        try {
            new GoalSelector(GoalSelector.Strategy.NEAREST);
        } catch (IllegalArgumentException e) {
            emptySet = true;
        }
        check("A selector with no goals is rejected", emptySet, "accepted");

        System.out.println("\n=== The shipped configuration ===");
        System.out.printf("   %d goal(s), strategy %s, tag meaning %s, switch frames %d%n",
                ShootingConstants.GOALS.length, ShootingConstants.GOAL_STRATEGY,
                ShootingConstants.GOAL_TAG_MEANING, ShootingConstants.GOAL_SWITCH_FRAMES);
        for (Goal goal : ShootingConstants.GOALS) {
            System.out.printf("     %s%n", goal);
        }
        GoalSelector shipped = ShootingConstants.newGoalSelector();
        check("The shipped selector builds", shipped.getSelected() != null, "null");
        check("Switch frames are non-zero, so a flicker cannot move the chassis",
                ShootingConstants.GOAL_SWITCH_FRAMES > 0,
                "0 invites oscillation on a turretless robot");
        boolean anyTags = false;
        for (Goal goal : ShootingConstants.GOALS) {
            if (!goal.getTagIds().isEmpty()) anyTags = true;
        }
        if (!anyTags) {
            System.out.println("   NOTE: no goal has tag IDs yet, so TAG_VISIBLE will");
            System.out.println("   always fall back to NEAREST. Fill them in from the");
            System.out.println("   field drawings to enable tag-based selection.");
        }

        System.out.println(fails == 0
                ? "\nAll goal-selection checks passed."
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }
}
