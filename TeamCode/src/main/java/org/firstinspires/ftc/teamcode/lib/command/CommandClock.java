/*
 * The clock the command system measures time with.
 *
 * Defaults to the monotonic system clock. Tests replace the source so timeouts
 * and waits can be stepped deterministically instead of by sleeping -- see
 * tools/verify. Robot code never needs to touch this.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.function.DoubleSupplier;

public final class CommandClock {
    private static DoubleSupplier source = () -> System.nanoTime() / 1.0e9;

    private CommandClock() {}

    public static double nowSeconds() {
        return source.getAsDouble();
    }

    /** Test seam: supply a controllable time source. */
    public static void setSource(DoubleSupplier newSource) {
        source = newSource;
    }

    /** Restores the real monotonic clock. */
    public static void useSystemClock() {
        source = () -> System.nanoTime() / 1.0e9;
    }
}
