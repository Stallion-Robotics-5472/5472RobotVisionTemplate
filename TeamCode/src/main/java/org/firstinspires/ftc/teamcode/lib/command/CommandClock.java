/*
 * The clock the command system measures time with.
 *
 * A thin alias for {@link org.firstinspires.ftc.teamcode.lib.util.RobotClock} so
 * that command timeouts, the pose estimator and the controllers all share one
 * time source -- otherwise a test could freeze one and not the others, and the
 * halves of the robot would disagree about how much time had passed.
 *
 * Robot code never needs to touch this.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import org.firstinspires.ftc.teamcode.lib.util.RobotClock;

import java.util.function.DoubleSupplier;

public final class CommandClock {

    private CommandClock() {}

    public static double nowSeconds() {
        return RobotClock.nowSeconds();
    }

    /** Test seam: supply a controllable time source. */
    public static void setSource(DoubleSupplier newSource) {
        RobotClock.setSource(newSource);
    }

    /** Restores the real monotonic clock. */
    public static void useSystemClock() {
        RobotClock.useSystemClock();
    }
}
