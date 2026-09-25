/*
 * One place the robot can shoot at.
 *
 * A goal is more than a coordinate. It carries the AprilTags mounted on or beside
 * it (so the robot can tell whether it is looking at this goal), its own shot
 * table (a goal at a different height needs a different flywheel/hood curve), how
 * much it scores, and how forgiving its opening is.
 *
 * Positions are written for ONE alliance -- whichever ShootingConstants.AUTHORED_FOR
 * names -- and flipped at run time by GoalSelector. Inches, field frame.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

public final class Goal {

    private final String name;
    private final Translation2d position;
    private final Set<Integer> tagIds;
    private final double radiusInches;
    private final double pointValue;
    private final ShooterMap map;

    private Goal(Builder builder) {
        this.name = builder.name;
        this.position = builder.position;
        Set<Integer> ids = new LinkedHashSet<>();
        for (int id : builder.tagIds) {
            ids.add(id);
        }
        this.tagIds = Collections.unmodifiableSet(ids);
        this.radiusInches = builder.radiusInches;
        this.pointValue = builder.pointValue;
        this.map = builder.map;
    }

    /**
     * Starts a goal.
     *
     * @param name shown on telemetry and used to select this goal by name.
     * @param xInches field X, for the authored alliance.
     * @param yInches field Y, for the authored alliance.
     */
    public static Builder named(String name, double xInches, double yInches) {
        return new Builder(name, new Translation2d(xInches, yInches));
    }

    public static final class Builder {
        private final String name;
        private final Translation2d position;
        private int[] tagIds = new int[0];
        private double radiusInches = ShootingConstants.GOAL_RADIUS_IN;
        private double pointValue = 1.0;
        private ShooterMap map = null;

        private Builder(String name, Translation2d position) {
            this.name = name;
            this.position = position;
        }

        /**
         * AprilTags that identify this goal. Seeing one of these is how the robot
         * knows it is looking at THIS goal rather than another.
         *
         * Read the IDs off the season's field drawings. A goal with no tags still
         * works -- it is simply never preferred by the tag-visibility strategy and
         * is chosen on geometry alone.
         */
        public Builder tags(int... ids) {
            this.tagIds = ids.clone();
            return this;
        }

        /**
         * Effective half-width of this goal's opening, inches. Sets the heading
         * tolerance, which then tightens with distance. Default is
         * {@link ShootingConstants#GOAL_RADIUS_IN}.
         */
        public Builder radius(double inches) {
            this.radiusInches = inches;
            return this;
        }

        /** What a scored piece is worth here. Used by the best-value strategy. */
        public Builder worth(double points) {
            this.pointValue = points;
            return this;
        }

        /**
         * This goal's own shot table. Give a goal its own map whenever its height
         * or geometry differs -- one curve cannot serve two heights. Omit it and
         * the goal uses {@link ShootingConstants#SHOT_MAP}.
         */
        public Builder map(ShooterMap map) {
            this.map = map;
            return this;
        }

        public Goal build() {
            return new Goal(this);
        }
    }

    public String getName() {
        return name;
    }

    /** Position as authored, i.e. for {@code ShootingConstants.AUTHORED_FOR}. */
    public Translation2d getAuthoredPosition() {
        return position;
    }

    public Set<Integer> getTagIds() {
        return tagIds;
    }

    public double getRadiusInches() {
        return radiusInches;
    }

    public double getPointValue() {
        return pointValue;
    }

    /** This goal's shot table, falling back to the shared default. */
    public ShooterMap getMap() {
        return map != null ? map : ShootingConstants.SHOT_MAP;
    }

    /** True if any of this goal's tags appears in {@code visibleTagIds}. */
    public boolean isSeenAmong(Iterable<Integer> visibleTagIds) {
        if (tagIds.isEmpty()) {
            return false;
        }
        for (Integer id : visibleTagIds) {
            if (tagIds.contains(id)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format("%s at (%.1f, %.1f) tags %s worth %.0f",
                name, position.getX(), position.getY(),
                tagIds.isEmpty() ? "none" : tagIds.toString(), pointValue);
    }
}
