/*
 * A unit of robot behaviour with a lifecycle, modeled on WPILib's Command.
 *
 * The scheduler calls, in order:
 *
 *   initialize()            once, when the command is scheduled
 *   execute()               every loop while it runs
 *   isFinished()            every loop; true ends the command
 *   end(interrupted)        once, when it finishes or is cancelled
 *
 * A command declares the subsystems it needs via {@link #addRequirements}. The
 * scheduler guarantees two running commands never require the same subsystem,
 * which is what keeps two pieces of code from fighting over one motor.
 *
 * The decorators at the bottom (withTimeout, andThen, alongWith, ...) build
 * composed commands, so most robot behaviour can be written as one expression.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

public abstract class Command {

    /** What happens when another command wants a subsystem this one is using. */
    public enum InterruptionBehavior {
        /** Default: this command is cancelled and the new one runs. */
        CANCEL_SELF,
        /** This command keeps the subsystem and the new command is refused. */
        CANCEL_INCOMING
    }

    private final Set<Subsystem> requirements = new LinkedHashSet<>();
    private String name = getClass().getSimpleName();

    /** Run once when the command is scheduled. */
    public void initialize() {}

    /** Run every scheduler loop while the command is active. */
    public void execute() {}

    /**
     * Run once when the command stops.
     *
     * @param interrupted true if it was cancelled rather than finishing on its
     *     own. Commands that hold a mechanism should stop it here so an
     *     interrupted command cannot leave a motor running.
     */
    public void end(boolean interrupted) {}

    /** True when the command has achieved its goal. Default never finishes. */
    public boolean isFinished() {
        return false;
    }

    public Set<Subsystem> getRequirements() {
        return Collections.unmodifiableSet(requirements);
    }

    /** Declares the subsystems this command needs exclusive use of. */
    public final void addRequirements(Subsystem... subsystems) {
        requirements.addAll(Arrays.asList(subsystems));
    }

    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.CANCEL_SELF;
    }

    public String getName() {
        return name;
    }

    public Command withName(String name) {
        this.name = name;
        return this;
    }

    // ---------------------------------------------------------------------
    // Decorators. Each returns a NEW command wrapping this one.
    // ---------------------------------------------------------------------

    /** Ends this command after {@code seconds}, whether or not it finished. */
    public Command withTimeout(double seconds) {
        return raceWith(new WaitCommand(seconds)).withName(getName() + "/timeout");
    }

    /** Ends this command early once {@code condition} becomes true. */
    public Command until(BooleanSupplier condition) {
        return raceWith(new WaitUntilCommand(condition)).withName(getName() + "/until");
    }

    /** Ends this command early once {@code condition} becomes false. */
    public Command onlyWhile(BooleanSupplier condition) {
        return until(() -> !condition.getAsBoolean());
    }

    /** Runs this command, then the given ones, in order. */
    public Command andThen(Command... next) {
        SequentialCommandGroup group = new SequentialCommandGroup(this);
        group.addCommands(next);
        return group;
    }

    /** Runs this command alongside the others; ends when all have ended. */
    public Command alongWith(Command... others) {
        ParallelCommandGroup group = new ParallelCommandGroup(this);
        group.addCommands(others);
        return group;
    }

    /** Runs this command alongside the others; ends as soon as any one ends. */
    public Command raceWith(Command... others) {
        ParallelRaceGroup group = new ParallelRaceGroup(this);
        group.addCommands(others);
        return group;
    }

    /**
     * Runs this command as the deadline for the others: the others are cancelled
     * when this one finishes.
     */
    public Command deadlineFor(Command... others) {
        return new ParallelDeadlineGroup(this, others);
    }

    /** Restarts this command every time it finishes. Never ends on its own. */
    public Command repeatedly() {
        return new RepeatCommand(this);
    }

    /** Runs {@code action} when this command ends, however it ends. */
    public Command finallyDo(Runnable action) {
        Command inner = this;
        return new Command() {
            {
                addRequirements(inner.getRequirements().toArray(new Subsystem[0]));
                withName(inner.getName() + "/finallyDo");
            }

            @Override
            public void initialize() {
                inner.initialize();
            }

            @Override
            public void execute() {
                inner.execute();
            }

            @Override
            public boolean isFinished() {
                return inner.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                inner.end(interrupted);
                action.run();
            }
        };
    }

    /** Runs this command only if {@code condition} is false when scheduled. */
    public Command unless(BooleanSupplier condition) {
        return new ConditionalCommand(new InstantCommand(), this, condition)
                .withName(getName() + "/unless");
    }

    /** Runs this command only if {@code condition} is true when scheduled. */
    public Command onlyIf(BooleanSupplier condition) {
        return unless(() -> !condition.getAsBoolean());
    }

    /** This command with the given interruption behaviour. */
    public Command withInterruptBehavior(InterruptionBehavior behavior) {
        Command inner = this;
        return new Command() {
            {
                addRequirements(inner.getRequirements().toArray(new Subsystem[0]));
                withName(inner.getName());
            }

            @Override
            public void initialize() {
                inner.initialize();
            }

            @Override
            public void execute() {
                inner.execute();
            }

            @Override
            public boolean isFinished() {
                return inner.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                inner.end(interrupted);
            }

            @Override
            public InterruptionBehavior getInterruptionBehavior() {
                return behavior;
            }
        };
    }

    /**
     * Guards against a command being used in two places at once, which would
     * run one instance's lifecycle twice over and is almost always a bug.
     * Command groups call this on their children.
     */
    static void requireUngrouped(Command... commands) {
        Set<Command> seen = new HashSet<>();
        for (Command c : commands) {
            if (c == null) {
                throw new IllegalArgumentException("null command in a group");
            }
            if (!seen.add(c)) {
                throw new IllegalArgumentException(
                        "Command " + c.getName() + " was added to the same group twice");
            }
            if (c.grouped) {
                throw new IllegalArgumentException(
                        "Command " + c.getName() + " is already part of another command group. "
                                + "Build a second instance instead of reusing this one.");
            }
        }
        for (Command c : commands) {
            c.grouped = true;
        }
    }

    /** True once this command has been placed inside a command group. */
    boolean grouped = false;

    @Override
    public String toString() {
        return getName();
    }
}
