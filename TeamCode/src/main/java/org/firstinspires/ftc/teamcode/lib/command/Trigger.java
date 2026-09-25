/*
 * Binds commands to a boolean condition. Modeled on WPILib's Trigger.
 *
 * A Trigger wraps any BooleanSupplier -- a gamepad button, a sensor, a
 * subsystem's state -- and schedules commands on its edges. Bindings are
 * registered with a scheduler and polled once per {@code run()}.
 *
 *   new Trigger(scheduler, () -> gamepad1.a).whenPressed(shootCommand);
 *   new Trigger(scheduler, shooter::atSetpoint).whileTrue(feedCommand);
 *
 * Because bindings are polled rather than interrupt-driven, an edge is detected
 * only if the condition changes between two scheduler loops. That is the same
 * limitation WPILib has and is not a problem at FTC loop rates.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.function.BooleanSupplier;

public class Trigger {
    private final CommandScheduler scheduler;
    private final BooleanSupplier condition;

    public Trigger(CommandScheduler scheduler, BooleanSupplier condition) {
        this.scheduler = scheduler;
        this.condition = condition;
    }

    public boolean get() {
        return condition.getAsBoolean();
    }

    /** Schedules the command on the rising edge. */
    public Trigger whenPressed(Command command) {
        scheduler.addBinding(new Runnable() {
            private boolean last = condition.getAsBoolean();

            @Override
            public void run() {
                boolean now = condition.getAsBoolean();
                if (now && !last) {
                    scheduler.schedule(command);
                }
                last = now;
            }
        });
        return this;
    }

    /** Schedules the command on the falling edge. */
    public Trigger whenReleased(Command command) {
        scheduler.addBinding(new Runnable() {
            private boolean last = condition.getAsBoolean();

            @Override
            public void run() {
                boolean now = condition.getAsBoolean();
                if (!now && last) {
                    scheduler.schedule(command);
                }
                last = now;
            }
        });
        return this;
    }

    /**
     * Runs the command while the condition holds, cancelling it on the falling
     * edge. The command's {@code end(true)} runs on release, which is where a
     * "hold to shoot" command should stop the feeder.
     */
    public Trigger whileTrue(Command command) {
        scheduler.addBinding(new Runnable() {
            private boolean last = condition.getAsBoolean();

            @Override
            public void run() {
                boolean now = condition.getAsBoolean();
                if (now && !last) {
                    scheduler.schedule(command);
                } else if (!now && last) {
                    scheduler.cancel(command);
                }
                last = now;
            }
        });
        return this;
    }

    /** Alias for {@link #whileTrue} reading naturally for held buttons. */
    public Trigger whileHeld(Command command) {
        return whileTrue(command);
    }

    /** Starts the command on the rising edge and cancels it on the next one. */
    public Trigger toggleWhenPressed(Command command) {
        scheduler.addBinding(new Runnable() {
            private boolean last = condition.getAsBoolean();

            @Override
            public void run() {
                boolean now = condition.getAsBoolean();
                if (now && !last) {
                    if (scheduler.isScheduled(command)) {
                        scheduler.cancel(command);
                    } else {
                        scheduler.schedule(command);
                    }
                }
                last = now;
            }
        });
        return this;
    }

    /** Cancels the command on the rising edge. */
    public Trigger cancelWhenPressed(Command command) {
        scheduler.addBinding(new Runnable() {
            private boolean last = condition.getAsBoolean();

            @Override
            public void run() {
                boolean now = condition.getAsBoolean();
                if (now && !last) {
                    scheduler.cancel(command);
                }
                last = now;
            }
        });
        return this;
    }

    // ----- composition -----

    public Trigger and(BooleanSupplier other) {
        return new Trigger(scheduler, () -> condition.getAsBoolean() && other.getAsBoolean());
    }

    public Trigger or(BooleanSupplier other) {
        return new Trigger(scheduler, () -> condition.getAsBoolean() || other.getAsBoolean());
    }

    public Trigger negate() {
        return new Trigger(scheduler, () -> !condition.getAsBoolean());
    }
}
