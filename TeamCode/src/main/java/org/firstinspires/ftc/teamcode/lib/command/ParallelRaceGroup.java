/*
 * Runs commands at the same time and ends them all as soon as any one finishes.
 * WPILib's ParallelRaceGroup. This is what withTimeout() and until() are built
 * from.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class ParallelRaceGroup extends Command {
    private final List<Command> commands = new ArrayList<>();
    private boolean finished = false;

    public ParallelRaceGroup(Command... commands) {
        addCommands(commands);
    }

    public final void addCommands(Command... toAdd) {
        requireUngrouped(toAdd);
        for (Command c : toAdd) {
            Set<Subsystem> overlap = new HashSet<>(c.getRequirements());
            overlap.retainAll(getRequirements());
            if (!overlap.isEmpty()) {
                throw new IllegalArgumentException(
                        "Racing commands cannot share a subsystem: " + c.getName()
                                + " conflicts on " + overlap.iterator().next().getName());
            }
            commands.add(c);
            addRequirements(c.getRequirements().toArray(new Subsystem[0]));
        }
    }

    @Override
    public void initialize() {
        finished = false;
        for (Command c : commands) {
            c.initialize();
        }
    }

    @Override
    public void execute() {
        for (Command c : commands) {
            c.execute();
            if (c.isFinished()) {
                finished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Every child is cut short, including the one that finished: the race is
        // over for all of them, so they all get a chance to stop their hardware.
        for (Command c : commands) {
            c.end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
