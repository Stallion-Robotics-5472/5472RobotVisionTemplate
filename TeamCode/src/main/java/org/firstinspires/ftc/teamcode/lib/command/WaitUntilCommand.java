/*
 * Finishes once a condition becomes true. WPILib's WaitUntilCommand.
 *
 * Note it requires no subsystems, so on its own it will not stop a mechanism
 * from being taken by another command while it waits.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.function.BooleanSupplier;

public class WaitUntilCommand extends Command {
    private final BooleanSupplier condition;

    public WaitUntilCommand(BooleanSupplier condition) {
        this.condition = condition;
        withName("WaitUntil");
    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }
}
