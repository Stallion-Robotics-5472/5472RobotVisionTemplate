# Command-Based Programming

A WPILib-shaped command framework for FTC, written for this template. If you have
written FRC code, this will read exactly as you expect.

Nothing external is required — no FTCLib, no SolversLib, no extra Gradle
dependency. Because it lives in the repo it is covered by
`./tools/verify/run.sh`, which runs 38 checks on the semantics below.

---

## The idea

Two things fight in ordinary OpMode code: the `while (opModeIsActive())` loop
wants to do one thing at a time, and a robot wants to do several. Command-based
programming splits robot code into:

- **Subsystems** — a mechanism (drivetrain, shooter). Exactly one command may use
  a subsystem at a time, so two pieces of code can never fight over one motor.
- **Commands** — a behaviour with a lifecycle (`initialize` → `execute` …
  `isFinished` → `end`). Commands compose into sequences and parallels.
- **The scheduler** — runs it all, one pass per loop.

```java
@TeleOp(name = "My Robot")
public class MyOpMode extends CommandOpMode {
    @Override
    public void configure() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        register(drive, shooter);

        // What the drivetrain does when nothing else asks for it.
        setDefaultCommand(drive, new RunCommand(
                () -> drive.driveFieldCentric(
                        -gamepad1.left_stick_y, -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x),
                drive));

        // Hold A to shoot; releasing it restores the default command.
        whileHeld(() -> gamepad1.a, Commands.sequence(
                Commands.runOnce(() -> shooter.setFlywheelRpm(2500), shooter),
                Commands.waitUntil(shooter::atSpeed),
                Commands.run(shooter::runFeeder, shooter).withTimeout(1.5)));
    }
}
```

---

## One difference from WPILib, and why

**The scheduler is an ordinary object, not a static singleton.**

In WPILib you write `CommandScheduler.getInstance()`. Here each `CommandOpMode`
owns its own scheduler.

This is not stylistic. An FTC Robot Controller app runs *many OpModes inside one
process*. A static scheduler would carry commands, button bindings and stale
hardware handles from one OpMode run into the next — which shows up as the
maddening failure where an OpMode behaves correctly the first time you run it
after a Robot Controller restart and strangely every time after. `CommandOpMode`
creates a scheduler per run and calls `reset()` on the way out, so nothing
survives.

For the same reason `SubsystemBase` does **not** self-register. Call
`register(...)` in `configure()`.

---

## Subsystems

```java
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void periodic() {
        // Runs once per loop, BEFORE commands execute -- so commands see this
        // loop's sensor data, not last loop's. Reads and telemetry go here;
        // motor commands belong in commands, so the scheduler can arbitrate.
    }

    public void run(double power) { motor.setPower(power); }
    public void stop() { motor.setPower(0); }
}
```

## Commands

```java
public class SpinUpCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double rpm;

    public SpinUpCommand(ShooterSubsystem shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);      // claim the subsystem
    }

    @Override public void initialize() { shooter.setFlywheelRpm(rpm); }
    @Override public boolean isFinished() { return shooter.atSpeed(); }
    @Override public void end(boolean interrupted) {
        // ALWAYS stop your mechanism here. `interrupted` is true when the
        // command was cancelled -- if you skip this, an interrupted command
        // leaves a motor running.
        if (interrupted) shooter.stow();
    }
}
```

For anything simple, skip the class:

```java
Commands.runOnce(shooter::stow, shooter);            // once, then done
Commands.run(intake::forward, intake);               // every loop, forever
Commands.startEnd(intake::forward, intake::stop, intake);  // hold-to-run
Commands.waitUntil(shooter::atSpeed);                // gate a sequence
Commands.waitSeconds(0.5);
```

## Composing

```java
Commands.sequence(a, b, c)          // one after another
Commands.parallel(a, b)             // together, ends when all end
Commands.race(a, b)                 // together, ends when the FIRST ends
Commands.deadline(a, b, c)          // b and c run while a does; a decides the end
Commands.either(a, b, () -> cond)   // picks when scheduled
```

or as decorators, which read better inline:

```java
shootCommand.withTimeout(2.0)                  // give up after 2 s
driveCommand.until(() -> sensor.triggered())   // end early on a condition
spinUp.andThen(feed)                           // sequence
aim.alongWith(spinUp)                          // parallel
feed.repeatedly()                              // restart forever
intake.finallyDo(() -> led.off())              // cleanup however it ends
shoot.onlyIf(shooter::atSpeed)                 // skip unless ready
```

## Button bindings

From inside a `CommandOpMode`:

```java
whenPressed(() -> gamepad1.a, command);        // rising edge
whileHeld(() -> gamepad1.left_bumper, command);// runs while held, cancels on release
toggleWhenPressed(() -> gamepad1.x, command);  // press on, press off

trigger(() -> gamepad1.right_trigger > 0.5)
        .and(shooter::atSpeed)                 // compose conditions
        .whileHeld(feedCommand);
```

`whileHeld` cancels on release, which calls `end(true)` — that is where a
hold-to-shoot command stops the feeder.

Triggers are not limited to buttons. Any `BooleanSupplier` works:

```java
trigger(shooter::atSpeed).whenPressed(Commands.runOnce(() -> led.green()));
```

---

## The rules the scheduler enforces

Each `run()`:

1. every registered subsystem's `periodic()`
2. every trigger binding
3. every scheduled command's `execute()`, ending the finished ones
4. a default command for any subsystem left idle

**Subsystem exclusivity.** Scheduling a command whose subsystem is already taken
**cancels the holder** and takes over. To protect a command that must not be
displaced:

```java
criticalCommand.withInterruptBehavior(
        Command.InterruptionBehavior.CANCEL_INCOMING);
```

**Default commands** must require their subsystem and must not finish
immediately — a command that finishes at once would be rescheduled every loop
forever, so that is rejected when you set it.

**Groups claim everything.** A group requires the union of its children's
subsystems for its whole duration, so nothing can grab a mechanism halfway
through a sequence.

Two guards reject mistakes up front rather than letting them misbehave:

- Two commands in one `parallel`/`race` requiring the **same subsystem** — that
  is two commands driving one mechanism, exactly what the scheduler exists to
  prevent.
- Reusing one command **instance** in two groups — its lifecycle would run twice
  over. Build a second instance.

**Scheduling from inside a command** is safe. Calls made during `run()` are
buffered and applied after the loop, so the scheduler never mutates a collection
it is iterating.

---

## What is in here

| File | What it is |
|---|---|
| `Subsystem`, `SubsystemBase` | mechanism base |
| `Command` | lifecycle + decorators |
| `CommandScheduler` | the loop and the arbitration |
| `CommandOpMode` | `LinearOpMode` that runs the scheduler |
| `Commands` | static factories |
| `InstantCommand`, `RunCommand`, `StartEndCommand`, `FunctionalCommand` | the simple ones |
| `WaitCommand`, `WaitUntilCommand` | timing and gating |
| `SequentialCommandGroup`, `ParallelCommandGroup`, `ParallelRaceGroup`, `ParallelDeadlineGroup` | composition |
| `ConditionalCommand`, `RepeatCommand` | control flow |
| `Trigger` | condition → command bindings |
| `CommandClock` | the clock waits measure against; a test seam, robot code ignores it |

## Worked example

[`ShootOnTheMoveTeleOp`](../../opmodes/ShootOnTheMoveTeleOp.java) is the
reference: two subsystems, a default drive command, a default shooter idle
command, an aim-and-shoot command bound to a held bumper, and trim bound to the
dpad. See [`../../shooting/SHOOTING.md`](../../shooting/SHOOTING.md) for what it
is doing.

## Moving to SolversLib later

FTCLib is effectively unmaintained (last release 2.1.1); **SolversLib** is the
active fork and is API-compatible with it. If you later want the full library,
the concepts here map one-to-one and most code ports by changing imports —
`Subsystem`, `SubsystemBase`, `Command`, `CommandScheduler`,
`SequentialCommandGroup` and friends all carry the same names and semantics. The
one thing to redo is scheduler ownership: those libraries use a static singleton,
so you must reset it yourself between OpMode runs (see the note above for why
that matters on FTC).
