/*
 * Base OpMode for command-based robot code. The FTC counterpart of WPILib's
 * TimedRobot + RobotContainer.
 *
 * Subclass it and override {@link #configure()} to build subsystems, set default
 * commands and bind buttons. The OpMode then runs the scheduler every loop until
 * stopped, and tears everything down afterwards.
 *
 *   @TeleOp(name = "My Robot")
 *   public class MyOpMode extends CommandOpMode {
 *       @Override
 *       public void configure() {
 *           DriveSubsystem drive = new DriveSubsystem(hardwareMap);
 *           register(drive);
 *           setDefaultCommand(drive, new RunCommand(
 *                   () -> drive.driveFieldCentric(-gamepad1.left_stick_y, ...), drive));
 *           whenPressed(() -> gamepad1.a, new MyCommand(drive));
 *       }
 *   }
 *
 * The scheduler is created per OpMode run and reset on exit, so no command or
 * binding survives into the next run -- the failure mode where an OpMode works
 * once after a Robot Controller restart and misbehaves every time after.
 */
package org.firstinspires.ftc.teamcode.lib.command;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.util.LoopTimer;

import java.util.function.BooleanSupplier;

public abstract class CommandOpMode extends LinearOpMode {

    private final CommandScheduler scheduler = new CommandScheduler();

    /**
     * Measures the loop rate for free.
     *
     * Every gain in this template was tuned at some loop rate, and nothing
     * otherwise warns you when that rate changes. Add a per-loop String.format
     * here and an inline file flush there and 50 Hz quietly becomes 15 Hz -- the
     * heading PID damps less, the follower brakes later, the shot solution is
     * stale. Cheap enough to always run: a few doubles per loop, formatted only
     * when {@link #addLoopTelemetry()} asks.
     */
    private final LoopTimer loopTimer = new LoopTimer();

    /** Build subsystems, default commands and bindings here. */
    public abstract void configure();

    /** Optional hook run every loop after the scheduler, for telemetry. */
    public void periodic() {}

    /** Optional hook run once after the driver presses play. */
    public void onStart() {}

    /** Optional hook run once when the OpMode is stopping. */
    public void onStop() {}

    @Override
    public final void runOpMode() throws InterruptedException {
        // The clock is a static test seam; make sure a previous test or OpMode
        // cannot leave a frozen clock behind.
        CommandClock.useSystemClock();

        configure();

        // Run the scheduler during init too, so subsystems keep their sensors
        // and telemetry live while the driver is still setting up.
        while (opModeInInit()) {
            scheduler.run();
            periodic();
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            scheduler.reset();
            return;
        }

        onStart();
        // Init can sit for minutes; those gaps say nothing about the loop rate.
        loopTimer.reset();

        try {
            while (opModeIsActive()) {
                loopTimer.tick();
                scheduler.run();
                periodic();
                telemetry.update();
            }
        } finally {
            // Always tear down, even if a command threw: leaving the scheduler
            // populated would strand commands holding real hardware.
            onStop();
            scheduler.reset();
        }
    }

    // ---------------------------------------------------------------------
    // Conveniences so subclasses rarely touch the scheduler directly
    // ---------------------------------------------------------------------

    public CommandScheduler getScheduler() {
        return scheduler;
    }

    /** Loop timing for this run. Reset at the start of the active period. */
    public LoopTimer getLoopTimer() {
        return loopTimer;
    }

    /** Adds the loop rate to telemetry. Call it from {@link #periodic()}. */
    public void addLoopTelemetry() {
        loopTimer.addTelemetry(telemetry);
    }

    public void register(Subsystem... subsystems) {
        scheduler.registerSubsystem(subsystems);
    }

    public void setDefaultCommand(Subsystem subsystem, Command command) {
        scheduler.setDefaultCommand(subsystem, command);
    }

    public void schedule(Command... commands) {
        scheduler.schedule(commands);
    }

    public void cancel(Command... commands) {
        scheduler.cancel(commands);
    }

    /** A trigger bound to this OpMode's scheduler. */
    public Trigger trigger(BooleanSupplier condition) {
        return new Trigger(scheduler, condition);
    }

    public Trigger whenPressed(BooleanSupplier condition, Command command) {
        return trigger(condition).whenPressed(command);
    }

    public Trigger whileHeld(BooleanSupplier condition, Command command) {
        return trigger(condition).whileHeld(command);
    }

    public Trigger toggleWhenPressed(BooleanSupplier condition, Command command) {
        return trigger(condition).toggleWhenPressed(command);
    }
}
