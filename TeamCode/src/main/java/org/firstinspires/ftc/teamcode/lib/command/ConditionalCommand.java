/*
 * Picks one of two commands when scheduled, based on a condition evaluated
 * once at that moment. WPILib's ConditionalCommand.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.function.BooleanSupplier;

public class ConditionalCommand extends Command {
    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;
    private Command selected;

    public ConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        requireUngrouped(onTrue, onFalse);
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = condition;
        addRequirements(onTrue.getRequirements().toArray(new Subsystem[0]));
        addRequirements(onFalse.getRequirements().toArray(new Subsystem[0]));
        withName("Conditional");
    }

    @Override
    public void initialize() {
        selected = condition.getAsBoolean() ? onTrue : onFalse;
        selected.initialize();
    }

    @Override
    public void execute() {
        selected.execute();
    }

    @Override
    public void end(boolean interrupted) {
        selected.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return selected.isFinished();
    }
}
