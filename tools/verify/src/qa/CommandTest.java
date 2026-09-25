package qa;

import org.firstinspires.ftc.teamcode.lib.command.Command;
import org.firstinspires.ftc.teamcode.lib.command.CommandClock;
import org.firstinspires.ftc.teamcode.lib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.command.Commands;
import org.firstinspires.ftc.teamcode.lib.command.RunCommand;
import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.lib.command.Trigger;

import java.util.ArrayList;
import java.util.List;

/**
 * Exercises the command framework's semantics: subsystem exclusivity, default
 * commands, interruption, group behaviour, and the decorators.
 *
 * These are the rules that stop two commands fighting over one motor, so they are
 * worth pinning down rather than assuming.
 */
public class CommandTest {
    static int fails = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-56s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    /** Records its lifecycle so tests can assert on the order of events. */
    static class Recorder extends Command {
        final String id;
        final List<String> log;
        boolean finish = false;
        int executes = 0;

        Recorder(String id, List<String> log, SubsystemBase... reqs) {
            this.id = id;
            this.log = log;
            addRequirements(reqs);
            withName(id);
        }

        @Override public void initialize() { log.add(id + ":init"); }
        @Override public void execute() { executes++; log.add(id + ":exec"); }
        @Override public void end(boolean interrupted) {
            log.add(id + (interrupted ? ":interrupted" : ":end"));
        }
        @Override public boolean isFinished() { return finish; }
    }

    static class Motor extends SubsystemBase {
        int periodics = 0;
        @Override public void periodic() { periodics++; }
    }

    public static void main(String[] args) {
        // Deterministic clock so timeouts do not depend on wall time.
        final double[] clock = {0.0};
        CommandClock.setSource(() -> clock[0]);

        System.out.println("=== Scheduler basics ===");
        CommandScheduler s = new CommandScheduler();
        Motor motor = new Motor();
        s.registerSubsystem(motor);
        List<String> log = new ArrayList<>();

        Recorder a = new Recorder("A", log, motor);
        s.schedule(a);
        check("Scheduling calls initialize", log.contains("A:init"), log.toString());
        s.run();
        check("run() executes the command", a.executes == 1, "executes " + a.executes);
        check("run() calls subsystem periodic", motor.periodics == 1,
                "periodics " + motor.periodics);

        a.finish = true;
        s.run();
        check("Finishing calls end(false)", log.contains("A:end"), log.toString());
        check("Finished command is unscheduled", !s.isScheduled(a), "still scheduled");

        // Subsystem exclusivity: the core guarantee.
        System.out.println();
        log.clear();
        Recorder first = new Recorder("FIRST", log, motor);
        Recorder second = new Recorder("SECOND", log, motor);
        s.schedule(first);
        s.run();
        s.schedule(second);
        check("A new command displaces the one holding the subsystem",
                log.contains("FIRST:interrupted") && s.isScheduled(second),
                log.toString());
        check("Only one command holds a subsystem at a time",
                s.requiring(motor) == second, "wrong holder");
        s.cancel(second);

        // CANCEL_INCOMING protects a command that must not be displaced.
        System.out.println();
        log.clear();
        Recorder holder = new Recorder("HOLD", log, motor);
        Command protectedHolder =
                holder.withInterruptBehavior(Command.InterruptionBehavior.CANCEL_INCOMING);
        s.schedule(protectedHolder);
        Recorder intruder = new Recorder("INTRUDE", log, motor);
        s.schedule(intruder);
        check("CANCEL_INCOMING refuses the new command",
                s.isScheduled(protectedHolder) && !s.isScheduled(intruder),
                log.toString());
        s.cancel(protectedHolder);

        // Default commands fill idle subsystems, and yield to real ones.
        System.out.println();
        log.clear();
        CommandScheduler ds = new CommandScheduler();
        Motor dm = new Motor();
        Recorder def = new Recorder("DEFAULT", log, dm);
        ds.setDefaultCommand(dm, def);
        ds.run();
        check("Default command is scheduled when idle", ds.isScheduled(def),
                log.toString());
        Recorder override = new Recorder("OVERRIDE", log, dm);
        ds.schedule(override);
        check("Default command yields to a real command",
                ds.isScheduled(override) && !ds.isScheduled(def), log.toString());
        override.finish = true;
        ds.run();
        ds.run();
        check("Default command resumes afterwards", ds.isScheduled(def), log.toString());

        // A default command that finishes instantly would be rescheduled forever.
        boolean rejected = false;
        try {
            ds.setDefaultCommand(dm, Commands.runOnce(() -> {}, dm));
        } catch (IllegalArgumentException e) {
            rejected = true;
        }
        check("A finished command is rejected as a default", rejected, "accepted");

        // Sequential groups.
        System.out.println();
        log.clear();
        CommandScheduler gs = new CommandScheduler();
        Motor gm = new Motor();
        gs.registerSubsystem(gm);
        Recorder s1 = new Recorder("S1", log, gm);
        Recorder s2 = new Recorder("S2", log, gm);
        Command seq = Commands.sequence(s1, s2);
        gs.schedule(seq);
        gs.run();
        check("Sequence starts only its first command",
                log.contains("S1:exec") && !log.contains("S2:init"), log.toString());
        s1.finish = true;
        gs.run();
        check("Sequence advances when a step finishes",
                log.contains("S1:end") && log.contains("S2:init"), log.toString());
        s2.finish = true;
        gs.run();
        check("Sequence ends after its last command",
                !gs.isScheduled(seq) && log.contains("S2:end"), log.toString());

        // Interrupting a sequence must only interrupt the in-flight step.
        log.clear();
        Recorder t1 = new Recorder("T1", log, gm);
        Recorder t2 = new Recorder("T2", log, gm);
        Command seq2 = Commands.sequence(t1, t2);
        gs.schedule(seq2);
        gs.run();
        gs.cancel(seq2);
        check("Cancelling a sequence interrupts only the running step",
                log.contains("T1:interrupted") && !log.contains("T2:init"),
                log.toString());

        // Parallel groups.
        System.out.println();
        log.clear();
        Motor m1 = new Motor();
        Motor m2 = new Motor();
        CommandScheduler ps = new CommandScheduler();
        ps.registerSubsystem(m1, m2);
        Recorder p1 = new Recorder("P1", log, m1);
        Recorder p2 = new Recorder("P2", log, m2);
        Command par = Commands.parallel(p1, p2);
        ps.schedule(par);
        ps.run();
        check("Parallel runs both at once",
                log.contains("P1:exec") && log.contains("P2:exec"), log.toString());
        p1.finish = true;
        ps.run();
        check("Parallel waits for the slower command", ps.isScheduled(par),
                "ended early");
        p2.finish = true;
        ps.run();
        check("Parallel ends when all have ended", !ps.isScheduled(par), log.toString());

        // Two commands wanting the same subsystem inside one parallel group is a
        // guaranteed conflict, so it must be refused at build time.
        boolean conflict = false;
        try {
            Commands.parallel(new Recorder("X", log, m1), new Recorder("Y", log, m1));
        } catch (IllegalArgumentException e) {
            conflict = true;
        }
        check("Parallel rejects commands sharing a subsystem", conflict, "accepted");

        // Reusing one command instance in two groups would run its lifecycle
        // twice over.
        boolean reuse = false;
        try {
            Recorder shared = new Recorder("SHARED", log, m1);
            Commands.sequence(shared);
            Commands.sequence(shared);
        } catch (IllegalArgumentException e) {
            reuse = true;
        }
        check("Reusing a command in two groups is rejected", reuse, "accepted");

        // Race groups.
        System.out.println();
        log.clear();
        Recorder r1 = new Recorder("R1", log, m1);
        Recorder r2 = new Recorder("R2", log, m2);
        Command race = Commands.race(r1, r2);
        ps.schedule(race);
        ps.run();
        r1.finish = true;
        ps.run();
        check("Race ends as soon as one command finishes", !ps.isScheduled(race),
                log.toString());
        check("Race stops the loser too", log.contains("R2:interrupted"),
                log.toString());

        // withTimeout, on the deterministic clock.
        System.out.println();
        log.clear();
        clock[0] = 100.0;
        CommandScheduler ts = new CommandScheduler();
        Motor tm = new Motor();
        ts.registerSubsystem(tm);
        Recorder forever = new Recorder("FOREVER", log, tm);
        Command timed = forever.withTimeout(1.5);
        ts.schedule(timed);
        ts.run();
        check("Timed command runs before the timeout", ts.isScheduled(timed), "ended early");
        clock[0] = 101.0;
        ts.run();
        check("Timed command still runs at 1.0s of 1.5s", ts.isScheduled(timed),
                "ended early");
        clock[0] = 101.6;
        ts.run();
        check("Timed command ends after the timeout", !ts.isScheduled(timed),
                log.toString());
        check("Timed-out command is interrupted, so it can stop its hardware",
                log.contains("FOREVER:interrupted"), log.toString());

        // until()
        log.clear();
        final boolean[] gate = {false};
        Recorder gated = new Recorder("GATED", log, tm);
        Command untilCmd = gated.until(() -> gate[0]);
        ts.schedule(untilCmd);
        ts.run();
        check("until() keeps running while false", ts.isScheduled(untilCmd), "ended");
        gate[0] = true;
        ts.run();
        check("until() ends when the condition becomes true", !ts.isScheduled(untilCmd),
                log.toString());

        // Triggers.
        System.out.println();
        log.clear();
        CommandScheduler bs = new CommandScheduler();
        Motor bm = new Motor();
        bs.registerSubsystem(bm);
        final boolean[] button = {false};
        Recorder held = new Recorder("HELD", log, bm);
        new Trigger(bs, () -> button[0]).whileHeld(held);
        bs.run();
        check("whileHeld does not fire while released", !bs.isScheduled(held), "fired");
        button[0] = true;
        bs.run();
        check("whileHeld schedules on press", bs.isScheduled(held), log.toString());
        button[0] = false;
        bs.run();
        check("whileHeld cancels on release", !bs.isScheduled(held), log.toString());
        check("Released command is interrupted, so it can stop its hardware",
                log.contains("HELD:interrupted"), log.toString());

        // Scheduling from inside a running command must not corrupt the loop.
        System.out.println();
        log.clear();
        CommandScheduler ns = new CommandScheduler();
        Motor nm1 = new Motor();
        Motor nm2 = new Motor();
        ns.registerSubsystem(nm1, nm2);
        Recorder target = new Recorder("TARGET", log, nm2);
        Command scheduler = new RunCommand(() -> ns.schedule(target), nm1);
        ns.schedule(scheduler);
        boolean threw = false;
        try {
            ns.run();
            ns.run();
        } catch (RuntimeException e) {
            threw = true;
            System.out.println("   threw: " + e);
        }
        check("Scheduling from inside a command does not throw", !threw,
                "concurrent modification");
        check("The command scheduled from inside actually ran",
                ns.isScheduled(target), log.toString());

        // reset() must leave nothing behind for the next OpMode run.
        System.out.println();
        log.clear();
        Recorder lingering = new Recorder("LINGER", log, nm1);
        ns.schedule(lingering);
        ns.reset();
        check("reset() interrupts everything still running",
                log.contains("LINGER:interrupted"), log.toString());
        check("reset() clears the schedule", ns.getScheduledCommands().isEmpty(),
                "commands remain");
        check("reset() clears registered subsystems",
                ns.getRegisteredSubsystems().isEmpty(), "subsystems remain");

        CommandClock.useSystemClock();
        System.out.println(fails == 0
                ? "\nAll command-framework checks passed."
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }
}
