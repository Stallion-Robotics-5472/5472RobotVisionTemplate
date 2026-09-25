/*
 * Runs one action on start and another on end, running in between.
 * WPILib's StartEndCommand. Useful for "hold to run" behaviour.
 */
package org.firstinspires.ftc.teamcode.lib.command;

public class StartEndCommand extends Command {
    private final Runnable onInit;
    private final Runnable onEnd;

    public StartEndCommand(Runnable onInit, Runnable onEnd, Subsystem... requirements) {
        this.onInit = onInit;
        this.onEnd = onEnd;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        onInit.run();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.run();
    }
}
