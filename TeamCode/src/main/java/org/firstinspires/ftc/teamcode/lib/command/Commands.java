/*
 * Static factories for building commands inline, so simple behaviour does not
 * need its own class. Modeled on WPILib's Commands.
 *
 *   Commands.sequence(
 *       Commands.runOnce(shooter::spinUp, shooter),
 *       Commands.waitUntil(shooter::atSetpoint),
 *       Commands.run(feeder::feed, feeder).withTimeout(1.5));
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.function.BooleanSupplier;

public final class Commands {
    private Commands() {}

    /** Runs an action once and ends. */
    public static Command runOnce(Runnable action, Subsystem... requirements) {
        return new InstantCommand(action, requirements);
    }

    /** Runs an action every loop; never ends on its own. */
    public static Command run(Runnable action, Subsystem... requirements) {
        return new RunCommand(action, requirements);
    }

    /** Runs one action on start and another on end. */
    public static Command startEnd(Runnable onStart, Runnable onEnd, Subsystem... requirements) {
        return new StartEndCommand(onStart, onEnd, requirements);
    }

    /** Does nothing and ends immediately. */
    public static Command none() {
        return new InstantCommand();
    }

    /** Does nothing for a duration. */
    public static Command waitSeconds(double seconds) {
        return new WaitCommand(seconds);
    }

    /** Ends once the condition is true. */
    public static Command waitUntil(BooleanSupplier condition) {
        return new WaitUntilCommand(condition);
    }

    /** Runs the commands in order. */
    public static Command sequence(Command... commands) {
        return new SequentialCommandGroup(commands);
    }

    /** Runs the commands together, ending when all have ended. */
    public static Command parallel(Command... commands) {
        return new ParallelCommandGroup(commands);
    }

    /** Runs the commands together, ending when the first one ends. */
    public static Command race(Command... commands) {
        return new ParallelRaceGroup(commands);
    }

    /** Runs the others alongside the deadline, ending when the deadline ends. */
    public static Command deadline(Command deadline, Command... others) {
        return new ParallelDeadlineGroup(deadline, others);
    }

    /** Picks between two commands when scheduled. */
    public static Command either(Command onTrue, Command onFalse, BooleanSupplier condition) {
        return new ConditionalCommand(onTrue, onFalse, condition);
    }

    /** Holds a subsystem doing nothing, so nothing else can take it. */
    public static Command idle(Subsystem... requirements) {
        return new RunCommand(() -> {}, requirements);
    }
}
