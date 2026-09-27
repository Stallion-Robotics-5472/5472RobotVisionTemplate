/*
 * Runs both targeting modes and lets the driver -- or the situation -- choose.
 *
 * The two modes fail in opposite ways, which is the whole reason to have both:
 *
 *   GOAL_TABLE     can aim at a goal the camera cannot see, so the robot can track
 *                  a goal behind it while crossing the field. Rests on the goal
 *                  coordinates and the pose estimate both being right.
 *   VISION_TARGET  needs no goal coordinates and no field pose, so it still works
 *                  on a robot that was never seeded or was seeded wrong. Can only
 *                  shoot at something it can currently see.
 *   AUTO           vision while it has a target, the goal table otherwise. Usually
 *                  what you want: the camera's own measurement when it is
 *                  available, and the table to fall back on when the target leaves
 *                  frame.
 *
 * AUTO is not a fudge. The pose-trust gate follows whichever source actually
 * answered: while vision is ranging, the shot does not depend on the pose and is not
 * gated on it; the moment it falls back to the table, the gate comes back. Each
 * shot is judged by what it actually rests on.
 *
 * Switching mode resets the source being switched TO, so a mode coming back does not
 * start from a stale held measurement. A switch is refused while frozen, because
 * frozen means a shot is in progress and on a turretless robot a change of target
 * moves the whole chassis.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;

public final class SwitchableTargetSource implements TargetSource {

    public enum Mode {
        /** Always the goal table. */
        GOAL_TABLE,
        /** Always what the camera sees; no target when it sees nothing. */
        VISION_TARGET,
        /** Vision when it has a target, the goal table otherwise. */
        AUTO
    }

    private final TargetSource goalSource;
    private final TargetSource visionSource;

    private Mode mode;
    private boolean frozen = false;
    /** Which source actually produced the last target. */
    private TargetSource answered;

    public SwitchableTargetSource(TargetSource goalSource, TargetSource visionSource,
                                  Mode mode) {
        this.goalSource = goalSource;
        this.visionSource = visionSource;
        this.mode = mode;
        this.answered = mode == Mode.GOAL_TABLE ? goalSource : visionSource;
    }

    @Override
    public Target update(Pose2d pose, Translation2d fieldVelocity, double omegaRadPerSec,
                         Alliance alliance) {
        switch (mode) {
            case GOAL_TABLE:
                answered = goalSource;
                return goalSource.update(pose, fieldVelocity, omegaRadPerSec, alliance);

            case VISION_TARGET:
                answered = visionSource;
                return visionSource.update(pose, fieldVelocity, omegaRadPerSec, alliance);

            case AUTO:
            default: {
                // Both are updated every loop, not just the one that wins. The goal
                // selector's hysteresis and range filtering are built out of
                // CONSECUTIVE frames, and a selector fed only the frames where
                // vision happened to fail would never accumulate the streak a
                // switch requires -- it would sit on a stale choice and then jump.
                Target fromTable = goalSource.update(
                        pose, fieldVelocity, omegaRadPerSec, alliance);
                Target seen = visionSource.update(
                        pose, fieldVelocity, omegaRadPerSec, alliance);
                if (seen != null) {
                    answered = visionSource;
                    return seen;
                }
                answered = goalSource;
                return fromTable;
            }
        }
    }

    /** Switches mode, unless a shot is in progress. Returns the mode now in use. */
    public Mode setMode(Mode newMode) {
        if (frozen || newMode == mode) {
            return mode;
        }
        mode = newMode;
        // Whichever source is coming back must not start from a stale hold.
        (newMode == Mode.GOAL_TABLE ? goalSource : visionSource).reset();
        return mode;
    }

    /** Steps GOAL_TABLE -> VISION_TARGET -> AUTO -> GOAL_TABLE. */
    public Mode cycleMode() {
        Mode[] all = Mode.values();
        return setMode(all[(mode.ordinal() + 1) % all.length]);
    }

    public Mode getMode() {
        return mode;
    }

    @Override
    public void reset() {
        goalSource.reset();
        visionSource.reset();
        frozen = false;
    }

    @Override
    public void freeze(boolean frozen) {
        this.frozen = frozen;
        goalSource.freeze(frozen);
        visionSource.freeze(frozen);
    }

    @Override
    public String describe() {
        String which = answered == visionSource ? "VISION" : "GOAL TABLE";
        if (mode == Mode.AUTO) {
            return String.format("AUTO -> %s | %s", which, answered.describe());
        }
        return String.format("%s | %s", mode, answered.describe());
    }

    @Override
    public String getRejectReason() {
        return answered.getRejectReason();
    }

    /**
     * Whatever the source that actually answered requires -- so in AUTO the pose
     * gate applies exactly on the loops where the answer rests on the pose.
     */
    @Override
    public boolean requiresTrustedPose() {
        return answered.requiresTrustedPose();
    }

    /** The source that produced the last target. */
    public TargetSource getAnsweringSource() {
        return answered;
    }

    public TargetSource getGoalSource() {
        return goalSource;
    }

    public TargetSource getVisionSource() {
        return visionSource;
    }
}
