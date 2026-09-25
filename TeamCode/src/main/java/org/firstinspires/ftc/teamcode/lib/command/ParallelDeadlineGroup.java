/*
 * Runs commands alongside a deadline command and cancels the rest when the
 * deadline finishes. WPILib's ParallelDeadlineGroup.
 *
 * Useful for "do this while that happens": e.g. run the feeder for as long as
 * the drive-to-position command takes.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class ParallelDeadlineGroup extends Command {
    private final Command deadline;
    private final List<Command> others = new ArrayList<>();
    private final List<Boolean> done = new ArrayList<>();

    public ParallelDeadlineGroup(Command deadline, Command... others) {
        requireUngrouped(deadline);
        this.deadline = deadline;
        addRequirements(deadline.getRequirements().toArray(new Subsystem[0]));
        addCommands(others);
    }

    public final void addCommands(Command... toAdd) {
        requireUngrouped(toAdd);
        for (Command c : toAdd) {
            Set<Subsystem> overlap = new HashSet<>(c.getRequirements());
            overlap.retainAll(getRequirements());
            if (!overlap.isEmpty()) {
                throw new IllegalArgumentException(
                        "Deadline-group commands cannot share a subsystem: " + c.getName()
                                + " conflicts on " + overlap.iterator().next().getName());
            }
            others.add(c);
            done.add(false);
            addRequirements(c.getRequirements().toArray(new Subsystem[0]));
        }
    }

    @Override
    public void initialize() {
        deadline.initialize();
        for (int i = 0; i < others.size(); i++) {
            others.get(i).initialize();
            done.set(i, false);
        }
    }

    @Override
    public void execute() {
        deadline.execute();
        for (int i = 0; i < others.size(); i++) {
            if (done.get(i)) {
                continue;
            }
            Command c = others.get(i);
            c.execute();
            if (c.isFinished()) {
                c.end(false);
                done.set(i, true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        deadline.end(interrupted);
        for (int i = 0; i < others.size(); i++) {
            if (!done.get(i)) {
                others.get(i).end(true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return deadline.isFinished();
    }
}
