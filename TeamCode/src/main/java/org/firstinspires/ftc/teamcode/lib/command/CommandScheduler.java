/*
 * Runs commands, arbitrates subsystems, and polls triggers. Modeled on WPILib's
 * CommandScheduler.
 *
 * One deliberate difference from WPILib: this is an ORDINARY OBJECT, not a
 * static singleton. An FTC Robot Controller app runs many OpModes inside one
 * process, so a static scheduler would carry commands, bindings and stale
 * hardware handles from one OpMode run into the next -- the classic symptom
 * being a robot that behaves correctly the first time an OpMode is run after a
 * restart and strangely every time after. Each CommandOpMode owns its own
 * scheduler, which is created fresh and thrown away with the OpMode.
 *
 * Each call to {@link #run()}:
 *   1. runs every registered subsystem's periodic()
 *   2. polls every trigger binding
 *   3. executes each scheduled command, ending the finished ones
 *   4. schedules default commands for any idle subsystem
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class CommandScheduler {

    /** Commands currently running, in scheduling order. */
    private final Set<Command> scheduled = new LinkedHashSet<>();

    /** Which command currently owns each subsystem. */
    private final Map<Subsystem, Command> requirements = new LinkedHashMap<>();

    /** Registered subsystems and their default commands (value may be null). */
    private final Map<Subsystem, Command> defaultCommands = new LinkedHashMap<>();

    /** Trigger bindings, polled once per run(). */
    private final List<Runnable> bindings = new ArrayList<>();

    // Scheduling from inside a command's execute()/end() would mutate the
    // collections we are iterating, so during run() those calls are buffered.
    private boolean inRunLoop = false;
    private final List<Command> toSchedule = new ArrayList<>();
    private final List<Command> toCancel = new ArrayList<>();

    private boolean disabled = false;

    // ---------------------------------------------------------------------
    // Registration
    // ---------------------------------------------------------------------

    /** Registers subsystems so their periodic() runs each loop. */
    public void registerSubsystem(Subsystem... subsystems) {
        for (Subsystem s : subsystems) {
            if (s != null && !defaultCommands.containsKey(s)) {
                defaultCommands.put(s, null);
            }
        }
    }

    public void unregisterSubsystem(Subsystem... subsystems) {
        for (Subsystem s : subsystems) {
            defaultCommands.remove(s);
        }
    }

    /**
     * Sets the command that runs on {@code subsystem} whenever nothing else
     * requires it -- typically the driver-control command for a drivetrain.
     * The command must require the subsystem and must never finish on its own.
     */
    public void setDefaultCommand(Subsystem subsystem, Command command) {
        if (!command.getRequirements().contains(subsystem)) {
            throw new IllegalArgumentException(
                    "Default command " + command.getName() + " must require "
                            + subsystem.getName());
        }
        if (command.isFinished()) {
            throw new IllegalArgumentException(
                    "Default command " + command.getName()
                            + " must not be finished when set; it would be rescheduled forever");
        }
        registerSubsystem(subsystem);
        defaultCommands.put(subsystem, command);
    }

    public Command getDefaultCommand(Subsystem subsystem) {
        return defaultCommands.get(subsystem);
    }

    /** Adds a binding polled once per run(); used by {@link Trigger}. */
    public void addBinding(Runnable binding) {
        bindings.add(binding);
    }

    // ---------------------------------------------------------------------
    // Scheduling
    // ---------------------------------------------------------------------

    /**
     * Schedules commands to run. A command whose requirements are held by a
     * command with {@link Command.InterruptionBehavior#CANCEL_INCOMING} is
     * refused; otherwise the holders are cancelled and replaced.
     */
    public void schedule(Command... commands) {
        for (Command command : commands) {
            scheduleSingle(command);
        }
    }

    private void scheduleSingle(Command command) {
        if (command == null || disabled) {
            return;
        }
        if (inRunLoop) {
            toSchedule.add(command);
            return;
        }
        if (scheduled.contains(command)) {
            return;
        }

        // Refuse if any requirement is held by a command that will not yield.
        for (Subsystem required : command.getRequirements()) {
            Command holder = requirements.get(required);
            if (holder != null && holder != command
                    && holder.getInterruptionBehavior()
                            == Command.InterruptionBehavior.CANCEL_INCOMING) {
                return;
            }
        }

        // Cancel the commands currently holding what we need.
        for (Subsystem required : command.getRequirements()) {
            Command holder = requirements.get(required);
            if (holder != null && holder != command) {
                cancelSingle(holder);
            }
        }

        scheduled.add(command);
        for (Subsystem required : command.getRequirements()) {
            requirements.put(required, command);
        }
        command.initialize();
    }

    /** Cancels commands, calling end(true) on each. */
    public void cancel(Command... commands) {
        for (Command command : commands) {
            if (command == null) {
                continue;
            }
            if (inRunLoop) {
                toCancel.add(command);
            } else {
                cancelSingle(command);
            }
        }
    }

    private void cancelSingle(Command command) {
        if (!scheduled.contains(command)) {
            return;
        }
        command.end(true);
        scheduled.remove(command);
        requirements.values().removeIf(holder -> holder == command);
    }

    /** Cancels everything currently running. */
    public void cancelAll() {
        for (Command command : new ArrayList<>(scheduled)) {
            cancelSingle(command);
        }
    }

    public boolean isScheduled(Command command) {
        return scheduled.contains(command);
    }

    /** The command currently using {@code subsystem}, or null. */
    public Command requiring(Subsystem subsystem) {
        return requirements.get(subsystem);
    }

    public Collection<Command> getScheduledCommands() {
        return Collections.unmodifiableCollection(scheduled);
    }

    public Collection<Subsystem> getRegisteredSubsystems() {
        return Collections.unmodifiableCollection(defaultCommands.keySet());
    }

    /** Stops the scheduler accepting or running commands. */
    public void disable() {
        disabled = true;
    }

    public void enable() {
        disabled = false;
    }

    // ---------------------------------------------------------------------
    // The loop
    // ---------------------------------------------------------------------

    public void run() {
        if (disabled) {
            return;
        }

        // 1. Subsystems read sensors and update state first, so commands that
        //    execute below see this loop's data rather than last loop's.
        for (Subsystem subsystem : defaultCommands.keySet()) {
            subsystem.periodic();
        }

        // 2. Poll bindings. These may schedule or cancel, which is buffered.
        inRunLoop = true;
        for (Runnable binding : bindings) {
            binding.run();
        }

        // 3. Run the scheduled commands.
        for (Command command : new ArrayList<>(scheduled)) {
            if (!scheduled.contains(command)) {
                continue;   // cancelled earlier in this same loop
            }
            command.execute();
            if (command.isFinished()) {
                command.end(false);
                scheduled.remove(command);
                requirements.values().removeIf(holder -> holder == command);
            }
        }
        inRunLoop = false;

        // Apply anything buffered while we were iterating.
        for (Command command : toCancel) {
            cancelSingle(command);
        }
        toCancel.clear();
        for (Command command : toSchedule) {
            scheduleSingle(command);
        }
        toSchedule.clear();

        // 4. Anything idle falls back to its default command.
        for (Map.Entry<Subsystem, Command> entry : defaultCommands.entrySet()) {
            Command fallback = entry.getValue();
            if (fallback != null && !requirements.containsKey(entry.getKey())) {
                scheduleSingle(fallback);
            }
        }
    }

    /**
     * Cancels everything and forgets all state. Call when an OpMode stops so a
     * command cannot outlive the hardware it was driving.
     */
    public void reset() {
        cancelAll();
        scheduled.clear();
        requirements.clear();
        defaultCommands.clear();
        bindings.clear();
        toSchedule.clear();
        toCancel.clear();
        disabled = false;
        inRunLoop = false;
    }
}
