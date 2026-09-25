/*
 * Runs commands at the same time, finishing when they all have.
 * WPILib's ParallelCommandGroup.
 *
 * Children may not share a subsystem with each other -- that would be two
 * commands driving one mechanism, which is exactly what the scheduler exists to
 * prevent, so it is rejected up front rather than silently.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class ParallelCommandGroup extends Command {
    private final List<Command> commands = new ArrayList<>();
    private final List<Boolean> done = new ArrayList<>();
    private boolean running = false;

    public ParallelCommandGroup(Command... commands) {
        addCommands(commands);
    }

    public final void addCommands(Command... toAdd) {
        if (running) {
            throw new IllegalStateException("Cannot add commands to a running group");
        }
        requireUngrouped(toAdd);
        for (Command c : toAdd) {
            Set<Subsystem> overlap = new HashSet<>(c.getRequirements());
            overlap.retainAll(getRequirements());
            if (!overlap.isEmpty()) {
                throw new IllegalArgumentException(
                        "Parallel commands cannot share a subsystem: " + c.getName()
                                + " conflicts on " + overlap.iterator().next().getName());
            }
            commands.add(c);
            done.add(false);
            addRequirements(c.getRequirements().toArray(new Subsystem[0]));
        }
    }

    @Override
    public void initialize() {
        running = true;
        for (int i = 0; i < commands.size(); i++) {
            commands.get(i).initialize();
            done.set(i, false);
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < commands.size(); i++) {
            if (done.get(i)) {
                continue;
            }
            Command c = commands.get(i);
            c.execute();
            if (c.isFinished()) {
                c.end(false);
                done.set(i, true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            for (int i = 0; i < commands.size(); i++) {
                if (!done.get(i)) {
                    commands.get(i).end(true);
                }
            }
        }
        running = false;
    }

    @Override
    public boolean isFinished() {
        return !done.contains(false);
    }
}
