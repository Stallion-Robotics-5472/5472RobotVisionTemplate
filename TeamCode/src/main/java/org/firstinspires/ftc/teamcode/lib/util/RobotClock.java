/*
 * The one place robot code asks what time it is.
 *
 * Every control loop in this template -- the pose estimator's latency
 * compensation, the path follower's PIDs, the heading-lock controller, command
 * timeouts -- needs a timestamp. They all come through here.
 *
 * Why bother instead of calling System.nanoTime() in each place: it makes the
 * whole stack testable off-robot. An offline match simulation runs thousands of
 * loops in a fraction of a second, so a wall clock hands every controller a dt of
 * a few microseconds. A derivative term divided by that explodes, the sim
 * saturates, and whatever it then shows you is about the sim rather than the
 * robot. With one clock the simulation can step time in realistic 20 ms slices
 * and the controllers behave the way they will on the field.
 *
 * Robot code never touches the setters. Defaults to the monotonic system clock.
 */
package org.firstinspires.ftc.teamcode.lib.util;

import java.util.function.DoubleSupplier;

public final class RobotClock {

    private static final DoubleSupplier SYSTEM_CLOCK = () -> System.nanoTime() / 1.0e9;

    private static DoubleSupplier source = SYSTEM_CLOCK;

    private RobotClock() {}

    /** Seconds on a monotonic clock. Only differences between calls are meaningful. */
    public static double nowSeconds() {
        return source.getAsDouble();
    }

    /** Test seam: supply a controllable time source. */
    public static void setSource(DoubleSupplier newSource) {
        source = newSource == null ? SYSTEM_CLOCK : newSource;
    }

    /** Restores the real monotonic clock. */
    public static void useSystemClock() {
        source = SYSTEM_CLOCK;
    }

    /** True when the real clock is in use. */
    public static boolean isSystemClock() {
        return source == SYSTEM_CLOCK;
    }
}
