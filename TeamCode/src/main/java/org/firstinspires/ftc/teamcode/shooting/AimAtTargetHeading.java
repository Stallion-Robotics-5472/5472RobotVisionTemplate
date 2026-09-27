/*
 * A path heading rule that keeps the shooter pointed at whatever the TargetSource
 * says to shoot at.
 *
 * Drop this on a Path and the follower drives the route while the robot rotates to
 * lead the target -- shooting on the move in autonomous, with the path choosing
 * where to go and the aim choosing where to face.
 *
 *   AimAtTargetHeading aim = new AimAtTargetHeading(source);
 *   Path p = new Path(curve).setHeadingSource(aim);
 *   ...
 *   aim.getLastSolution()   // the shot to spin up for, after follower.update()
 *
 * It solves once per loop and caches, so the command driving the shooter reads the
 * same solution the heading came from rather than solving a second time and
 * possibly disagreeing with itself.
 *
 * Works with either source. With a goal table it aims at a configured coordinate;
 * with VisionTargetSource it aims at whatever the camera is looking at, and needs
 * no goal coordinates at all. The path is authored the same way either way.
 *
 * When the source has nothing to shoot at -- a vision source with an empty frame --
 * the last target heading is held rather than snapping somewhere arbitrary, and
 * getLastSolution() goes null so the shooter's command refuses to fire.
 */
package org.firstinspires.ftc.teamcode.shooting;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.HeadingSource;

import java.util.function.Supplier;

public class AimAtTargetHeading implements HeadingSource {

    private final TargetSource targetSource;
    private final Supplier<Alliance> alliance;

    private AimSolution lastSolution;
    private TargetSource.Target lastTarget;
    private double heldHeadingRadians = Double.NaN;

    /**
     * @param targetSource where the thing being shot at is.
     * @param alliance which alliance is being played, for flipping a goal. A vision
     *     source ignores it -- what the camera sees is already alliance-correct.
     */
    public AimAtTargetHeading(TargetSource targetSource, Supplier<Alliance> alliance) {
        this.targetSource = targetSource;
        this.alliance = alliance;
    }

    /** For a source that does not care about the alliance, such as a vision one. */
    public AimAtTargetHeading(TargetSource targetSource) {
        this(targetSource, () -> Alliance.RED);
    }

    @Override
    public Target compute(double t, Pose2d pose, Translation2d fieldVelocity,
                          double omegaRadPerSec) {
        Alliance playing = alliance.get();
        lastTarget = targetSource.update(pose, fieldVelocity, omegaRadPerSec, playing);

        if (lastTarget == null) {
            // Nothing to shoot at. Hold the last heading rather than snapping to
            // some default: the robot was probably nearly aimed, and the target is
            // most likely about to come back into frame.
            lastSolution = null;
            return Target.of(Double.isNaN(heldHeadingRadians)
                    ? pose.getHeading() : heldHeadingRadians);
        }

        lastSolution = AimLogic.calculate(
                pose, fieldVelocity, omegaRadPerSec,
                lastTarget.fieldPosition,
                lastTarget.map,
                ShootingConstants.AIM_CONFIG.withGoalRadius(lastTarget.radiusInches));

        heldHeadingRadians = lastSolution.targetHeadingRadians;
        return new Target(lastSolution.targetHeadingRadians,
                lastSolution.headingFeedforwardRadPerSec);
    }

    @Override
    public void reset() {
        lastSolution = null;
        lastTarget = null;
        heldHeadingRadians = Double.NaN;
        targetSource.reset();
    }

    /**
     * The solution behind the current heading, or null when there is nothing to
     * shoot at. Read this AFTER {@code follower.update()} so it reflects this loop.
     */
    public AimSolution getLastSolution() {
        return lastSolution;
    }

    /** What is being aimed at, or null when there is nothing. */
    public TargetSource.Target getLastTarget() {
        return lastTarget;
    }

    public TargetSource getTargetSource() {
        return targetSource;
    }
}
