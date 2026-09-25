/*
 * Runs commands one after another. WPILib's SequentialCommandGroup.
 *
 * The group requires the union of its children's subsystems for its whole
 * duration, so nothing else can grab a mechanism halfway through a sequence.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequentialCommandGroup extends Command {
    private final List<Command> commands = new ArrayList<>();
    private int index = -1;
    private boolean finished = true;

    public SequentialCommandGroup(Command... commands) {
        addCommands(commands);
    }

    public final void addCommands(Command... toAdd) {
        if (index != -1) {
            throw new IllegalStateException("Cannot add commands to a running group");
        }
        requireUngrouped(toAdd);
        commands.addAll(Arrays.asList(toAdd));
        for (Command c : toAdd) {
            addRequirements(c.getRequirements().toArray(new Subsystem[0]));
        }
    }

    @Override
    public void initialize() {
        index = 0;
        finished = commands.isEmpty();
        if (!finished) {
            commands.get(0).initialize();
        }
    }

    @Override
    public void execute() {
        if (finished) {
            return;
        }
        Command current = commands.get(index);
        current.execute();
        if (current.isFinished()) {
            current.end(false);
            index++;
            if (index < commands.size()) {
                commands.get(index).initialize();
            } else {
                finished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Only the command that was mid-flight needs ending; earlier ones
        // already ended normally and later ones never started.
        if (interrupted && !finished && index >= 0 && index < commands.size()) {
            commands.get(index).end(true);
        }
        index = -1;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    /** Index of the currently running child, or -1 when not running. */
    public int getCurrentIndex() {
        return index;
    }
}
