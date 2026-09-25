/*
 * Restarts the wrapped command each time it finishes, so it runs forever.
 * WPILib's RepeatCommand.
 */
package org.firstinspires.ftc.teamcode.lib.command;

public class RepeatCommand extends Command {
    private final Command inner;
    private boolean innerFinished = false;

    public RepeatCommand(Command inner) {
        requireUngrouped(inner);
        this.inner = inner;
        addRequirements(inner.getRequirements().toArray(new Subsystem[0]));
        withName(inner.getName() + "/repeatedly");
    }

    @Override
    public void initialize() {
        innerFinished = false;
        inner.initialize();
    }

    @Override
    public void execute() {
        if (innerFinished) {
            innerFinished = false;
            inner.initialize();
        }
        inner.execute();
        if (inner.isFinished()) {
            inner.end(false);
            innerFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!innerFinished) {
            inner.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
