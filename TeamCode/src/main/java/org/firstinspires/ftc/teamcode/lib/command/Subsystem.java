/*
 * A mechanism the command system can hand out exclusively.
 *
 * Modeled on WPILib's Subsystem. Registering a subsystem with the scheduler
 * gets its periodic() called once per loop, and commands that require it are
 * guaranteed never to run at the same time as each other -- which is what
 * stops two commands fighting over the same motor.
 */
package org.firstinspires.ftc.teamcode.lib.command;

public interface Subsystem {
    /**
     * Called once per scheduler loop, before commands execute. Put sensor reads,
     * state updates and telemetry here -- not motor commands, which belong in
     * commands so the scheduler can arbitrate them.
     */
    default void periodic() {}

    /** Human-readable name, used in scheduler telemetry. */
    default String getName() {
        return getClass().getSimpleName();
    }
}
