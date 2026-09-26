/*
 * Builds goals from AprilTag positions, so you supply tag numbers instead of
 * hand-measured coordinates.
 *
 * The tags are already surveyed into the field frame -- that is what makes
 * localization work at all -- so a tag next to a goal already tells you where
 * that goal is. Typing the goal's coordinates separately duplicates information
 * the field map already has, and a typo there is silent: aiming is confidently
 * wrong and nothing flags it.
 *
 * WHAT THIS DOES AND DOES NOT SAVE YOU
 *
 * It saves the goal's X and Y. It cannot invent the offset from the tag to the
 * point the game piece must pass through, because that is geometry only the field
 * drawings know: a tag mounted on the face of a goal is not at the middle of its
 * opening. So you still give a small offset per tag -- but that is one measurement
 * off a drawing rather than a coordinate conversion, and it is expressed in terms
 * you can read with a ruler.
 *
 *   TagGoals.from(TAG_FIELD_POSES)
 *           .goal("hive").fromTag(11).outward(6.0).up(0).worth(5).map(HIGH_MAP)
 *           .goal("flower").fromTag(21).outward(4.0).worth(2).map(LOW_MAP)
 *           .build();
 *
 * WHERE THE TAG POSES COME FROM
 *
 * The same place the Limelight gets them: the season's field map. Read the tag
 * positions out of the field drawings (or the .fmap you upload to the Limelight)
 * once, into TAG_FIELD_POSES, and every goal is derived from them. If a tag's
 * position is wrong, the pose estimate and the aiming will disagree with each
 * other in a way you can actually see -- which is a far better failure than a
 * quietly wrong goal coordinate.
 *
 * Positions are in this template's frame: origin at field centre, +X right,
 * +Y away from the audience, inches. Tag yaw is the direction the tag FACES,
 * degrees CCW.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class TagGoals {

    private final Map<Integer, Pose2d> tagPoses;
    private final List<PendingGoal> pending = new ArrayList<>();

    private TagGoals(Map<Integer, Pose2d> tagPoses) {
        this.tagPoses = tagPoses;
    }

    /**
     * Starts building goals from a table of tag field poses.
     *
     * @param tagPoses tag ID to its field pose. The pose's rotation is the
     *     direction the tag faces, which is what makes {@code outward()} and
     *     {@code alongFace()} meaningful.
     */
    public static TagGoals from(Map<Integer, Pose2d> tagPoses) {
        return new TagGoals(tagPoses);
    }

    /** Convenience for declaring a tag table inline. */
    public static Map<Integer, Pose2d> tagTable(Object... idThenPose) {
        if (idThenPose.length % 2 != 0) {
            throw new IllegalArgumentException("tagTable needs id, pose, id, pose, ...");
        }
        Map<Integer, Pose2d> table = new LinkedHashMap<>();
        for (int i = 0; i < idThenPose.length; i += 2) {
            table.put((Integer) idThenPose[i], (Pose2d) idThenPose[i + 1]);
        }
        return table;
    }

    /** A tag pose from x/y inches and the heading the tag faces, in degrees. */
    public static Pose2d tagAt(double xInches, double yInches, double facingDegrees) {
        return new Pose2d(xInches, yInches, Rotation2d.fromDegrees(facingDegrees));
    }

    /** Begins a goal derived from one or more tags. */
    public PendingGoal goal(String name) {
        PendingGoal goal = new PendingGoal(this, name);
        pending.add(goal);
        return goal;
    }

    /** Finishes every declared goal. */
    public Goal[] build() {
        if (pending.isEmpty()) {
            throw new IllegalStateException("No goals declared");
        }
        Goal[] goals = new Goal[pending.size()];
        for (int i = 0; i < goals.length; i++) {
            goals[i] = pending.get(i).toGoal();
        }
        return goals;
    }

    /** The tag poses this was built from, for telemetry and sanity checks. */
    public Map<Integer, Pose2d> getTagPoses() {
        return Collections.unmodifiableMap(tagPoses);
    }

    // ---------------------------------------------------------------------

    /** A goal being described relative to its tags. */
    public static final class PendingGoal {
        private final TagGoals parent;
        private final String name;
        private final List<Integer> tags = new ArrayList<>();
        private double outwardInches = 0.0;
        private double alongFaceInches = 0.0;
        private double radiusInches = ShootingConstants.GOAL_RADIUS_IN;
        private double pointValue = 1.0;
        private ShooterMap map = null;

        private PendingGoal(TagGoals parent, String name) {
            this.parent = parent;
            this.name = name;
        }

        /**
         * The tag this goal is derived from. Call more than once for a goal with
         * several tags: the position is their average and all of them identify it.
         */
        public PendingGoal fromTag(int id) {
            tags.add(id);
            return this;
        }

        /** Several tags at once. */
        public PendingGoal fromTags(int... ids) {
            for (int id : ids) {
                tags.add(id);
            }
            return this;
        }

        /**
         * How far in front of the tag's face the aiming point sits, inches.
         *
         * Positive is away from the tag, along the direction it faces -- i.e.
         * toward the robot. Use this when the piece must pass through a plane in
         * front of the tag rather than at it.
         */
        public PendingGoal outward(double inches) {
            this.outwardInches = inches;
            return this;
        }

        /**
         * How far to slide the aiming point sideways along the tag's face, inches.
         * Positive is to the left as seen from in front of the tag. Use this when
         * the tag is mounted beside the opening rather than centred on it.
         */
        public PendingGoal alongFace(double inches) {
            this.alongFaceInches = inches;
            return this;
        }

        /** Effective half-width of this goal's opening, inches. */
        public PendingGoal radius(double inches) {
            this.radiusInches = inches;
            return this;
        }

        /** What a scored piece is worth here. */
        public PendingGoal worth(double points) {
            this.pointValue = points;
            return this;
        }

        /** This goal's own shot table, for a goal at a different height. */
        public PendingGoal map(ShooterMap map) {
            this.map = map;
            return this;
        }

        /** Declares the next goal. */
        public PendingGoal goal(String nextName) {
            return parent.goal(nextName);
        }

        /** Finishes every declared goal. */
        public Goal[] build() {
            return parent.build();
        }

        private Goal toGoal() {
            if (tags.isEmpty()) {
                throw new IllegalStateException(
                        "Goal '" + name + "' has no tags, so its position cannot be derived. "
                                + "Either add fromTag(...) or declare it with "
                                + "Goal.named(...) and explicit coordinates.");
            }

            // Average the tags, each shifted by the offset in ITS OWN frame, so a
            // goal flanked by two tags lands between them.
            double sumX = 0;
            double sumY = 0;
            for (int id : tags) {
                Pose2d tag = parent.tagPoses.get(id);
                if (tag == null) {
                    throw new IllegalStateException(
                            "Goal '" + name + "' refers to tag " + id
                                    + ", which is not in the tag table. Add it, or "
                                    + "correct the ID.");
                }
                // "Outward" is along the tag's facing; "along face" is 90 degrees
                // to its left.
                Translation2d offset = new Translation2d(outwardInches, alongFaceInches)
                        .rotateBy(tag.getRotation());
                sumX += tag.getX() + offset.getX();
                sumY += tag.getY() + offset.getY();
            }
            double x = sumX / tags.size();
            double y = sumY / tags.size();

            Goal.Builder builder = Goal.named(name, x, y)
                    .radius(radiusInches)
                    .worth(pointValue);
            int[] ids = new int[tags.size()];
            for (int i = 0; i < ids.length; i++) {
                ids[i] = tags.get(i);
            }
            builder.tags(ids);
            if (map != null) {
                builder.map(map);
            }
            return builder.build();
        }
    }
}
