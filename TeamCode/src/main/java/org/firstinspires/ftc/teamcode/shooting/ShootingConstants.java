/*
 * Everything you have to measure before shoot-on-the-move will work: where the
 * goal is, where the shooter sits on your robot, your shot table, and the
 * hardware that drives it.
 *
 * All distances are INCHES, all angles are degrees at this boundary and radians
 * inside the maths. The field frame is the one the whole template uses: origin at
 * the field centre, +X right, +Y away from the audience, heading CCW.
 */
package org.firstinspires.ftc.teamcode.shooting;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public final class ShootingConstants {
    private ShootingConstants() {}

    // =====================================================================
    // 1. THE GOALS  -- YOU MUST FILL THESE IN FROM THE GAME MANUAL
    // =====================================================================
    /**
     * The alliance every position in this file is written for. Positions are
     * flipped to the other alliance at run time by GoalSelector.
     */
    public static final org.firstinspires.ftc.teamcode.pathing.Alliance AUTHORED_FOR =
            org.firstinspires.ftc.teamcode.pathing.Alliance.RED;

    /**
     * Where each AprilTag sits in the field frame: ID -> pose, with the pose's
     * rotation being the direction the tag FACES. Inches, degrees.
     *
     * FILL THIS IN FROM THE FIELD DRAWINGS (or the .fmap you upload to the
     * Limelight -- same numbers, converted into this frame: origin at field
     * centre, +X right, +Y away from the audience).
     *
     * Only the tags on or beside the goals you shoot at are needed. Enter them for
     * ONE alliance; {@link #AUTHORED_FOR} says which, and everything is flipped at
     * run time.
     *
     * The values below are placeholders. They are a coherent example, not your
     * field.
     */
    public static final java.util.Map<Integer, Pose2d> TAG_FIELD_POSES = TagGoals.tagTable(
            //        id,            x,     y,  facing (deg)
            11, TagGoals.tagAt(  -36.0,  66.0,  -90.0),
            21, TagGoals.tagAt(   36.0,  66.0,  -90.0));

    /**
     * Every goal the robot can shoot at, derived from the tags above.
     *
     * Deriving beats typing coordinates twice: the tags are already surveyed into
     * the field frame, so a tag beside a goal already says where that goal is. A
     * typo in a separately-entered goal coordinate is silent -- aiming is
     * confidently wrong and nothing flags it.
     *
     * What still has to come off the drawings is the small offset from the tag to
     * the point the piece must pass through, because a tag on a goal's face is not
     * at the middle of its opening:
     *
     *   outward(n)    n inches in front of the tag's face (toward the robot)
     *   alongFace(n)  n inches sideways along the face, positive to the tag's left
     *
     * THESE ROWS ARE PLACEHOLDERS and will aim at the wrong place until the tag
     * table and the offsets are real. Check each one with the "Shooter Map Tuning"
     * OpMode: park somewhere and compare the DISTANCE readout to a tape measure.
     *
     * A goal with no tag can still be declared the long way, with
     * {@code Goal.named("x", xIn, yIn)...build()}, and mixed into this array.
     */
    public static final Goal[] GOALS = TagGoals.from(TAG_FIELD_POSES)
            .goal("hive").fromTag(11).outward(6.0).radius(7.0).worth(5)
            .goal("flower").fromTag(21).outward(6.0).radius(9.0).worth(2)
            .build();

    /**
     * How the robot picks between {@link #GOALS}.
     *
     * TAG_VISIBLE prefers goals whose AprilTags the camera can currently identify,
     * falling back to the nearest when none are in frame. With one goal any
     * strategy behaves identically.
     */
    public static final GoalSelector.Strategy GOAL_STRATEGY =
            GoalSelector.Strategy.TAG_VISIBLE;

    /**
     * What a tag being in frame means about its goal. CHECK THE MANUAL: if tags sit
     * beside each goal, seeing one identifies it (VISIBLE_MEANS_AVAILABLE); if a
     * tag is instead covered as its goal fills or is claimed, the HIDDEN one is the
     * available one. Backwards, the robot prefers exactly the wrong goals.
     */
    public static final GoalSelector.TagMeaning GOAL_TAG_MEANING =
            GoalSelector.TagMeaning.VISIBLE_MEANS_AVAILABLE;

    /**
     * Consecutive updates a different goal must win before the robot switches to
     * it. Without this the robot swings between headings every time a tag flickers
     * at the edge of frame -- see GoalSelector. About 12 loops is a fifth of a
     * second at typical FTC loop rates.
     */
    public static final int GOAL_SWITCH_FRAMES = 12;

    /** Builds a selector over {@link #GOALS} with the settings above. */
    public static GoalSelector newGoalSelector() {
        return new GoalSelector(GOAL_STRATEGY, GOALS)
                .withTagMeaning(GOAL_TAG_MEANING)
                .withSwitchFrames(GOAL_SWITCH_FRAMES);
    }

    /**
     * Default effective half-width of a goal opening, inches -- how far off-centre
     * a shot can land and still score. Sets the heading tolerance, which then
     * tightens automatically with distance. A goal may override it with
     * {@code Goal.Builder.radius(...)}.
     *
     * Use something smaller than the true half-width: the piece has size, your
     * pose estimate has error, and the shot has spread. Two thirds is a reasonable
     * starting point.
     */
    public static final double GOAL_RADIUS_IN = 6.0;

    // =====================================================================
    // 2. WHERE THE SHOOTER IS ON YOUR ROBOT  -- MEASURE THIS
    // =====================================================================
    /**
     * Shooter exit position relative to the robot's centre of rotation, inches:
     * +X forward, +Y left.
     *
     * This matters more than it looks. An off-centre shooter is swung sideways
     * whenever the robot rotates, and the game piece inherits that motion too --
     * which is exactly the sort of error that makes a robot shoot well standing
     * still and miss while turning.
     */
    public static final double SHOOTER_FORWARD_OFFSET_IN = 0.0;
    public static final double SHOOTER_LEFT_OFFSET_IN = 0.0;

    /**
     * Which way the shooter fires, relative to robot forward, degrees CCW.
     * 0 = out the front, 180 = out the back, 90 = out the left side.
     *
     * The aiming maths rotates the whole robot so the SHOOTER faces the goal, so
     * getting this wrong points the robot the wrong way by exactly this much.
     */
    public static final double SHOOTER_YAW_OFFSET_DEG = 0.0;

    // =====================================================================
    // 3. AIMING BEHAVIOUR
    // =====================================================================
    /**
     * Lookahead for control latency, seconds. Covers the time between reading
     * the pose and the drivetrain actually responding. 0.02-0.04 is typical;
     * raise it if the robot consistently lags behind a moving aim.
     */
    public static final double PHASE_DELAY_SECONDS = 0.02;

    /** Cap on virtual-goal passes. Convergence normally takes 3-4. */
    public static final int MAX_AIM_ITERATIONS = 10;

    /** Stop iterating once the shot distance moves less than this, inches. */
    public static final double AIM_CONVERGENCE_TOLERANCE_IN = 0.01;

    /** Widest heading tolerance allowed, however close the goal is. */
    public static final double MAX_HEADING_TOLERANCE_DEG = 20.0;

    /**
     * Shot distance limits, inches. Outside these the aiming solution reports
     * itself out of range and the shoot command refuses to fire.
     *
     * THESE MUST SIT INSIDE SHOT_MAP's measured range, and they default to
     * matching it exactly. A shot is only in range when BOTH this window and
     * the table allow it, so setting these wider than the table does not extend
     * the robot's reach -- it just makes these numbers lie about it. Widen the
     * table first, then widen these. (The offline suite checks this holds.)
     */
    public static final double MIN_SHOT_DISTANCE_IN = 18.0;
    public static final double MAX_SHOT_DISTANCE_IN = 96.0;

    /** Assembled config for {@link AimLogic}. */
    public static final AimLogic.Config AIM_CONFIG = new AimLogic.Config(
            new Translation2d(SHOOTER_FORWARD_OFFSET_IN, SHOOTER_LEFT_OFFSET_IN),
            Math.toRadians(SHOOTER_YAW_OFFSET_DEG),
            PHASE_DELAY_SECONDS,
            MAX_AIM_ITERATIONS,
            AIM_CONVERGENCE_TOLERANCE_IN,
            GOAL_RADIUS_IN,
            Math.toRadians(MAX_HEADING_TOLERANCE_DEG),
            MIN_SHOT_DISTANCE_IN,
            MAX_SHOT_DISTANCE_IN);

    // =====================================================================
    // 4. THE SHOT TABLE  -- BUILD THIS FROM REAL SHOTS
    // =====================================================================
    /**
     * Distance -> flywheel speed, hood angle and flight time.
     *
     * THE SHIPPED ROWS ARE PLACEHOLDERS with a plausible shape, not your robot's
     * numbers. Replace them using the "Shooter Map Tuning" OpMode: park at a
     * distance, dial rpm and hood until shots go in, write the row down, move,
     * repeat. Five or six rows across your usable range is plenty.
     *
     * Flight time is the row teams skip, and it is the one shoot-on-the-move
     * depends on -- with it left at zero the moving correction does nothing at
     * all and the robot only shoots well standing still. Time it from match video
     * or a phone slow-motion clip; even a rough value is far better than zero.
     */
    public static final ShooterMap SHOT_MAP = ShooterMap.builder()
            //      distance(in)   rpm   hood(deg)  flight(s)
            .add(           18,   1900,     20.0,      0.26)
            .add(           36,   2350,     27.0,      0.40)
            .add(           54,   2800,     32.0,      0.53)
            .add(           72,   3250,     36.0,      0.66)
            .add(           96,   3800,     40.0,      0.84)
            .build();

    // =====================================================================
    // 5. SHOOTER HARDWARE
    // =====================================================================
    /** Hardware-map name of the flywheel motor. */
    public static final String FLYWHEEL_MOTOR = "flywheel";

    /**
     * Second flywheel motor, or "" if you only have one. A two-motor flywheel
     * usually needs this one reversed; see FLYWHEEL_FOLLOWER_DIRECTION.
     */
    public static final String FLYWHEEL_FOLLOWER_MOTOR = "";

    public static final DcMotorSimple.Direction FLYWHEEL_DIRECTION =
            DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction FLYWHEEL_FOLLOWER_DIRECTION =
            DcMotorSimple.Direction.REVERSE;

    /**
     * Encoder ticks per revolution of the MOTOR (not the flywheel). A bare
     * goBILDA 5202/5203 or REV HD Hex motor reads 28 ticks per rev; a geared
     * motor multiplies that by its gearbox ratio.
     */
    public static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    /**
     * Flywheel revolutions per motor revolution. 1.0 for a direct drive; 2.0 if
     * the flywheel is geared to spin twice per motor turn.
     */
    public static final double FLYWHEEL_GEAR_RATIO = 1.0;

    /**
     * Velocity PIDF for the flywheel, in the SDK's units (ticks/sec).
     *
     * F is the important one and should be roughly 32767 / maxTicksPerSecond --
     * it does most of the work, with P cleaning up the rest. Tune F first, then
     * add P until recovery after a shot is quick without oscillating.
     */
    public static final double FLYWHEEL_P = 12.0;
    public static final double FLYWHEEL_I = 0.0;
    public static final double FLYWHEEL_D = 0.0;
    public static final double FLYWHEEL_F = 14.0;

    /** Flywheel is "at speed" within this many rpm of the setpoint. */
    public static final double FLYWHEEL_TOLERANCE_RPM = 75.0;

    /**
     * Speed the flywheel holds while waiting for a shot, rpm. Keeping it spinning
     * means the next shot only has to make up a small difference instead of
     * spinning up from nothing.
     */
    public static final double FLYWHEEL_IDLE_RPM = 1200.0;

    /** Hardware-map name of the hood servo. */
    public static final String HOOD_SERVO = "hood";

    /**
     * Hood angles at the servo's travel limits, degrees. The subsystem maps an
     * angle onto servo position 0..1 with these, so the shot table can be written
     * in real angles rather than servo units.
     */
    public static final double HOOD_DEG_AT_SERVO_0 = 15.0;
    public static final double HOOD_DEG_AT_SERVO_1 = 45.0;

    /** Hood angle to sit at when idle or stowed, degrees. */
    public static final double HOOD_STOWED_DEG = 15.0;

    // =====================================================================
    // 6. FEEDER HARDWARE (what pushes a piece into the flywheel)
    // =====================================================================
    public static final String FEEDER_MOTOR = "feeder";
    public static final DcMotorSimple.Direction FEEDER_DIRECTION =
            DcMotorSimple.Direction.FORWARD;

    /** Feeder power while shooting. */
    public static final double FEEDER_SHOOT_POWER = 1.0;

    /** How long one shot takes to clear the feeder, seconds. */
    public static final double FEED_TIME_SECONDS = 0.35;
}
