/*
 * A command assembled from lambdas for each lifecycle stage, for behaviour not
 * worth its own class. WPILib's FunctionalCommand.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class FunctionalCommand extends Command {
    private final Runnable onInit;
    private final Runnable onExecute;
    private final Consumer<Boolean> onEnd;
    private final BooleanSupplier finished;

    public FunctionalCommand(Runnable onInit, Runnable onExecute, Consumer<Boolean> onEnd,
                             BooleanSupplier finished, Subsystem... requirements) {
        this.onInit = onInit;
        this.onExecute = onExecute;
        this.onEnd = onEnd;
        this.finished = finished;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        onInit.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.accept(interrupted);
    }

    @Override
    public boolean isFinished() {
        return finished.getAsBoolean();
    }
}
