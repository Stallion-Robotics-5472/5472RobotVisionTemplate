/*
 * Does nothing for a fixed duration, then finishes. WPILib's WaitCommand.
 * Requires no subsystems, so it can pad out a sequence without blocking a
 * mechanism.
 */
package org.firstinspires.ftc.teamcode.lib.command;

public class WaitCommand extends Command {
    private final double durationSeconds;
    private double startSeconds;

    public WaitCommand(double durationSeconds) {
        this.durationSeconds = durationSeconds;
        withName("Wait(" + durationSeconds + "s)");
    }

    @Override
    public void initialize() {
        startSeconds = CommandClock.nowSeconds();
    }

    @Override
    public boolean isFinished() {
        return CommandClock.nowSeconds() - startSeconds >= durationSeconds;
    }
}
