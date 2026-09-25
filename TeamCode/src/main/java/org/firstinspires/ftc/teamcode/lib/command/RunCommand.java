/*
 * Runs an action every loop and never finishes on its own. WPILib's RunCommand.
 * Typically used as a subsystem's default command.
 */
package org.firstinspires.ftc.teamcode.lib.command;

public class RunCommand extends Command {
    private final Runnable action;

    public RunCommand(Runnable action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    @Override
    public void execute() {
        action.run();
    }
}
