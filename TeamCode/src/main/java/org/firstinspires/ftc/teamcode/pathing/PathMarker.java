/*
 * An action fired partway along a path.
 *
 * Without these, anything the robot does can only happen BETWEEN path segments --
 * so "start spinning up two thirds of the way down the field" or "drop the intake
 * just before the corner" means chopping the path into pieces purely to get a
 * callback, which changes how the follower drives it.
 *
 *   Path p = new Path(curve)
 *           .setTangentHeading()
 *           .addMarker(PathMarker.atT(0.6, shooter::spinUp))
 *           .addMarker(PathMarker.withinInchesOfEnd(12.0, intake::deploy));
 *
 * Each marker fires ONCE per run of the path. The follower's projection only moves
 * forward, but a marker still cannot be allowed to re-fire if the robot is nudged
 * back across the threshold -- a feeder that restarts every loop near the boundary
 * would be worse than no marker at all.
 *
 * Actions run inside the follower's update, on the OpMode thread, so keep them
 * short: set a setpoint or flip a flag. Anything that blocks steals loop time from
 * the control loop that is steering the robot.
 */
package org.firstinspires.ftc.teamcode.pathing;

public final class PathMarker {

    /** What the marker's threshold is measured against. */
    public enum Trigger {
        /** Fires once progress along the path reaches a given t (0..1). */
        AT_T,
        /** Fires once the remaining arc length drops below a given distance. */
        REMAINING_DISTANCE
    }

    private final Trigger trigger;
    private final double threshold;
    private final Runnable action;
    private final String name;
    private boolean fired = false;

    private PathMarker(Trigger trigger, double threshold, Runnable action, String name) {
        this.trigger = trigger;
        this.threshold = threshold;
        this.action = action;
        this.name = name;
    }

    /** Fires when the path parameter reaches {@code t}. */
    public static PathMarker atT(double t, Runnable action) {
        return new PathMarker(Trigger.AT_T, Math.max(0.0, Math.min(1.0, t)), action,
                String.format("t>=%.2f", t));
    }

    /** Fires when fewer than {@code inches} of the path remain. */
    public static PathMarker withinInchesOfEnd(double inches, Runnable action) {
        return new PathMarker(Trigger.REMAINING_DISTANCE, Math.max(0.0, inches), action,
                String.format("remaining<=%.0fin", inches));
    }

    /** Named variants, so telemetry says which marker fired. */
    public static PathMarker atT(double t, Runnable action, String name) {
        return new PathMarker(Trigger.AT_T, Math.max(0.0, Math.min(1.0, t)), action, name);
    }

    public static PathMarker withinInchesOfEnd(double inches, Runnable action, String name) {
        return new PathMarker(Trigger.REMAINING_DISTANCE, Math.max(0.0, inches), action, name);
    }

    /**
     * Fires the action if this marker's threshold has been crossed and it has not
     * already run.
     *
     * @return true if the action ran on this call.
     */
    boolean poll(double t, double remainingInches) {
        if (fired) {
            return false;
        }
        boolean crossed = trigger == Trigger.AT_T
                ? t >= threshold
                : remainingInches <= threshold;
        if (!crossed) {
            return false;
        }
        fired = true;
        action.run();
        return true;
    }

    /** Re-arms the marker. The follower calls this when a path starts. */
    void rearm() {
        fired = false;
    }

    public boolean hasFired() {
        return fired;
    }

    public Trigger getTrigger() {
        return trigger;
    }

    public double getThreshold() {
        return threshold;
    }

    /** The action, so AllianceFlip can copy a marker onto a mirrored path. */
    Runnable getAction() {
        return action;
    }

    public String getName() {
        return name;
    }

    @Override
    public String toString() {
        return name + (fired ? " (fired)" : "");
    }
}
