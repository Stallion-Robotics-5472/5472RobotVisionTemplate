/*
 * Runs an action once and finishes immediately. WPILib's InstantCommand.
 */
package org.firstinspires.ftc.teamcode.lib.command;

public class InstantCommand extends Command {
    private final Runnable action;

    /** A command that does nothing and ends at once. */
    public InstantCommand() {
        this(() -> {});
    }

    public InstantCommand(Runnable action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        action.run();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
