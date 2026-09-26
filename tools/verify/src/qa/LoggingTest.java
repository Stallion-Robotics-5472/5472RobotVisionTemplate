package qa;

import org.firstinspires.ftc.teamcode.lib.util.LoopTimer;
import org.firstinspires.ftc.teamcode.logging.MatchLog;
import org.firstinspires.ftc.teamcode.logging.MatchRecorder;

import java.io.File;
import java.io.IOException;
import java.io.Writer;
import java.nio.file.Files;
import java.util.List;

/**
 * Checks that logging records what it claims to and cannot stall the control loop.
 *
 * The claim being tested is specific: a control loop can hand a row to MatchLog
 * and carry on, whatever storage is doing. A logger that usually costs nothing and
 * occasionally costs 80 ms is worse than no logger, because the cost lands during
 * a match and presents as bad driving rather than as a logging problem. So the
 * interesting cases here are the bad ones -- a sink far slower than the loop, a
 * ring that has filled, a log that could not be opened at all.
 */
public class LoggingTest {
    static int fails = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-58s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    /** A writer that takes a fixed time per row, standing in for slow storage. */
    static final class SlowWriter extends Writer {
        final StringBuilder captured = new StringBuilder();
        final long millisPerWrite;

        SlowWriter(long millisPerWrite) {
            this.millisPerWrite = millisPerWrite;
        }

        @Override
        public void write(char[] buffer, int off, int len) {
            try {
                Thread.sleep(millisPerWrite);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            captured.append(buffer, off, len);
        }

        @Override
        public void flush() {}

        @Override
        public void close() {}
    }

    /** A writer that fails on the first row, standing in for a full or dead disk. */
    static final class BrokenWriter extends Writer {
        int writes = 0;

        @Override
        public void write(char[] buffer, int off, int len) throws IOException {
            if (++writes > 1) {           // let the header through, fail on data
                throw new IOException("disk full");
            }
        }

        @Override
        public void flush() {}

        @Override
        public void close() {}
    }

    public static void main(String[] args) throws Exception {
        numberFormatting();
        roundTrip();
        boundedUnderSlowStorage();
        perRowCost();
        storageFailureIsNotFatal();
        loopTimer();
        recorderSchema();

        System.out.println(fails == 0
                ? "\nAll logging checks passed."
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }

    // -----------------------------------------------------------------
    // The CSV cells themselves
    // -----------------------------------------------------------------

    static void numberFormatting() {
        System.out.println("=== Number formatting ===");
        check("Whole numbers", cell(42.0).equals("42"), cell(42.0));
        check("One decimal", cell(1.5).equals("1.5"), cell(1.5));
        check("Trailing zeros trimmed", cell(1.2500).equals("1.25"), cell(1.25));
        check("Leading zeros kept", cell(1.0005).equals("1.0005"), cell(1.0005));
        check("Tenths after a zero", cell(1.05).equals("1.05"), cell(1.05));
        check("Hundredths after a zero", cell(1.005).equals("1.005"), cell(1.005));
        check("Small values", cell(0.0001).equals("0.0001"), cell(0.0001));
        check("Negatives", cell(-12.75).equals("-12.75"), cell(-12.75));
        check("Negative fractions keep their sign", cell(-0.5).equals("-0.5"), cell(-0.5));
        check("Zero", cell(0.0).equals("0"), cell(0.0));
        check("NaN is an empty cell, not the text 'NaN'", cell(Double.NaN).isEmpty(),
                cell(Double.NaN));
        check("Infinity does not become a huge number",
                cell(Double.POSITIVE_INFINITY).equals("inf"),
                cell(Double.POSITIVE_INFINITY));
        check("Rounds rather than truncating", cell(0.99999).equals("1"), cell(0.99999));

        // The reason this is hand-rolled instead of String.format: a device set to
        // a comma-decimal locale would write "1,2345" into a comma-separated file.
        java.util.Locale original = java.util.Locale.getDefault();
        try {
            java.util.Locale.setDefault(java.util.Locale.GERMANY);
            check("A comma-decimal device still writes a dot",
                    cell(1.2345).equals("1.2345"), cell(1.2345));
        } finally {
            java.util.Locale.setDefault(original);
        }
    }

    static String cell(double value) {
        StringBuilder out = new StringBuilder();
        // Package-private on purpose: the formatting is an internal detail, but it
        // is the detail that makes a log readable, so it is tested directly.
        callAppend(out, value);
        return out.toString();
    }

    static void callAppend(StringBuilder out, double value) {
        try {
            java.lang.reflect.Method m = MatchLog.class
                    .getDeclaredMethod("append", StringBuilder.class, double.class);
            m.setAccessible(true);
            m.invoke(null, out, value);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("MatchLog.append is gone", e);
        }
    }

    // -----------------------------------------------------------------
    // Does a written row come back out of the file unchanged
    // -----------------------------------------------------------------

    static void roundTrip() throws Exception {
        System.out.println("\n=== Round trip through a real file ===");
        File dir = Files.createTempDirectory("matchlog").toFile();
        MatchLog log = MatchLog.open(dir, "round trip/test", 512, "t", "x", "y");
        check("A log opens without error", log.getError() == null, "" + log.getError());
        check("An unsafe label cannot escape the directory",
                log.getFile().getName().startsWith("round_trip_test-")
                        && log.getFile().getParentFile().equals(dir),
                log.getFile().getPath());

        int rows = 500;
        for (int i = 0; i < rows; i++) {
            double[] row = log.claim();
            if (row == null) continue;
            row[0] = i * 0.02;
            row[1] = i;
            row[2] = -i * 0.5;
            log.commit();
            // A real loop is 20 ms; 1 ms is 20x faster than storage ever sees.
            Thread.sleep(1);
        }
        log.close();

        List<String> lines = Files.readAllLines(log.getFile().toPath());
        check("The header names the columns",
                lines.get(0).equals("t,x,y"), lines.get(0));
        check("Every row reached the file", lines.size() == rows + 1,
                lines.size() + " lines for " + rows + " rows");
        check("Nothing was dropped at loop speed", log.getRowsDropped() == 0,
                log.getRowsDropped() + " dropped");
        check("The first row is intact", lines.get(1).equals("0,0,0"), lines.get(1));
        check("A later row is intact", lines.get(100).equals("1.98,99,-49.5"),
                lines.get(100));
        System.out.printf("   %d rows, peak queue depth %d of %d%n",
                log.getRowsWritten(), log.getPeakQueueDepth(), log.getCapacity());

        // A short row must not inherit numbers from whatever used the slot before.
        MatchLog shortRows = MatchLog.open(dir, "short", 64, "a", "b", "c");
        shortRows.write(1, 2, 3);
        shortRows.write(9);
        shortRows.close();
        List<String> shortLines = Files.readAllLines(shortRows.getFile().toPath());
        check("A short row blanks the columns it did not fill",
                shortLines.get(2).equals("9,,"), shortLines.get(2));
    }

    // -----------------------------------------------------------------
    // The case it exists for: storage far slower than the control loop
    // -----------------------------------------------------------------

    static void boundedUnderSlowStorage() {
        System.out.println("\n=== Storage 100x slower than the loop ===");
        // 2 ms per row against a loop that offers rows as fast as it can: the ring
        // must fill, and rows must be dropped rather than the producer waiting.
        SlowWriter slow = new SlowWriter(2);
        int capacity = 64;
        MatchLog log = MatchLog.into("slow", slow, capacity, "t", "x");

        int attempts = 2000;
        int accepted = 0;
        long worstNanos = 0;
        boolean overCapacity = false;
        for (int i = 0; i < attempts; i++) {
            long before = System.nanoTime();
            double[] row = log.claim();
            if (row != null) {
                row[0] = i;
                row[1] = i * 2.0;
                log.commit();
                accepted++;
            }
            long elapsed = System.nanoTime() - before;
            if (elapsed > worstNanos) worstNanos = elapsed;
            if (log.getQueueDepth() > capacity) overCapacity = true;
        }
        long dropped = log.getRowsDropped();
        System.out.printf("   %d attempts -> %d accepted, %d dropped;"
                        + " worst claim+commit %.3f ms%n",
                attempts, accepted, dropped, worstNanos / 1e6);

        check("Rows are dropped rather than the loop waiting", dropped > 0,
                "nothing dropped, so the producer may have been blocking");
        check("Every attempt is either kept or counted",
                accepted + dropped == attempts,
                accepted + " + " + dropped + " != " + attempts);
        check("Memory stays bounded by the ring", !overCapacity,
                "the queue grew past its capacity");
        check("A full ring costs the loop under 0.5 ms",
                worstNanos < 500_000L,
                String.format("worst was %.3f ms", worstNanos / 1e6));
        log.close();
    }

    // -----------------------------------------------------------------
    // What a row actually costs the loop
    // -----------------------------------------------------------------

    static void perRowCost() throws Exception {
        System.out.println("\n=== Per-row cost in the control loop ===");
        File dir = Files.createTempDirectory("matchlogcost").toFile();
        // 33 columns, the same width as MatchRecorder's schema.
        String[] columns = new String[MatchRecorder.COLUMNS.length];
        System.arraycopy(MatchRecorder.COLUMNS, 0, columns, 0, columns.length);
        MatchLog log = MatchLog.open(dir, "cost", 8192, columns);

        int iterations = 200_000;
        // Warm up, so the measurement is of steady-state code rather than of the
        // JIT compiling it.
        for (int i = 0; i < 20_000; i++) fillOne(log, i);

        long start = System.nanoTime();
        for (int i = 0; i < iterations; i++) fillOne(log, i);
        double perRowMicros = (System.nanoTime() - start) / 1000.0 / iterations;

        System.out.printf("   %.3f us per row of %d columns (%.4f%% of a 20 ms loop)%n",
                perRowMicros, columns.length, perRowMicros / 200.0);
        check("A row costs well under 1% of the loop budget",
                perRowMicros < 200.0 * 1.0,
                String.format("%.1f us is %.1f%% of 20 ms", perRowMicros,
                        perRowMicros / 200.0));
        log.close();
    }

    static void fillOne(MatchLog log, int i) {
        double[] row = log.claim();
        if (row == null) return;
        for (int c = 0; c < row.length; c++) {
            row[c] = i + c * 0.25;
        }
        log.commit();
    }

    // -----------------------------------------------------------------
    // Failures must be visible, not fatal
    // -----------------------------------------------------------------

    static void storageFailureIsNotFatal() {
        System.out.println("\n=== When storage fails ===");
        MatchLog broken = MatchLog.into("broken", new BrokenWriter(), 32, "a");
        for (int i = 0; i < 200; i++) {
            broken.write(i);
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        check("A write failure is reported, not thrown",
                broken.getError() != null, "no error recorded");
        check("The log turns itself off rather than filling memory",
                !broken.isRunning(), "still running after a write failure");
        check("claim() refuses once the log is off", broken.claim() == null, "handed out a row");
        System.out.printf("   reported: %s%n", broken.getError());
        broken.close();

        // A path that cannot be created at all: the OpMode must survive it.
        MatchLog impossible = MatchLog.open(
                new File("/proc/this/cannot/exist"), "nope", 32, "a");
        check("An unopenable log is inert, not an exception",
                impossible.getError() != null && impossible.claim() == null,
                "" + impossible.getError());
        check("write() on a dead log returns false rather than throwing",
                !impossible.write(1.0), "claimed to have written");
        impossible.close();
        System.out.printf("   reported: %s%n", impossible.getError());
    }

    // -----------------------------------------------------------------
    // Loop timing
    // -----------------------------------------------------------------

    static void loopTimer() {
        System.out.println("\n=== Loop timer ===");
        LoopTimer timer = new LoopTimer(0.020);
        check("Reads zero before it has seen anything", timer.getLoops() == 0,
                "" + timer.getLoops());

        // 100 loops at 10 ms, then 10 at 30 ms.
        double t = 0.0;
        timer.tick(t);
        for (int i = 0; i < 100; i++) {
            t += 0.010;
            timer.tick(t);
        }
        for (int i = 0; i < 10; i++) {
            t += 0.030;
            timer.tick(t);
        }
        System.out.printf("   %s; 95th pct <= %.0f ms; %.0f Hz%n",
                timer, timer.getPercentileMs(0.95), timer.getHz());
        check("Counts every loop", timer.getLoops() == 110, "" + timer.getLoops());
        check("Mean is between the two rates",
                timer.getMeanMs() > 10.0 && timer.getMeanMs() < 13.0,
                String.format("%.2f ms", timer.getMeanMs()));
        check("Worst loop is the worst loop",
                Math.abs(timer.getWorstMs() - 30.0) < 1e-6,
                String.format("%.2f ms", timer.getWorstMs()));
        check("Only the slow loops count as overruns", timer.getOverruns() == 10,
                "" + timer.getOverruns());
        // The tail is the point: 9% of loops are 3x the budget, and the mean of
        // 11.8 ms hides it completely.
        check("The percentile shows the tail the mean hides",
                timer.getPercentileMs(0.95) >= 30.0,
                String.format("95th pct reads %.0f ms", timer.getPercentileMs(0.95)));

        // A long pause is a stall, not a slow loop, and must not poison the mean.
        LoopTimer stalled = new LoopTimer(0.020);
        double s = 0.0;
        stalled.tick(s);
        for (int i = 0; i < 50; i++) {
            s += 0.010;
            stalled.tick(s);
        }
        s += 5.0;                  // e.g. sitting in init
        stalled.tick(s);
        for (int i = 0; i < 50; i++) {
            s += 0.010;
            stalled.tick(s);
        }
        System.out.printf("   after a 5 s pause: mean %.2f ms, %d stalls%n",
                stalled.getMeanMs(), stalled.getStalls());
        check("A 5 s pause is counted as a stall", stalled.getStalls() == 1,
                "" + stalled.getStalls());
        check("A stall does not wreck the mean",
                Math.abs(stalled.getMeanMs() - 10.0) < 0.01,
                String.format("%.2f ms", stalled.getMeanMs()));
        check("A stall is not counted as a loop", stalled.getLoops() == 100,
                "" + stalled.getLoops());

        LoopTimer reset = new LoopTimer();
        reset.tick(0.0);
        reset.tick(0.1);
        reset.reset();
        check("reset() clears everything",
                reset.getLoops() == 0 && reset.getWorstMs() == 0.0
                        && reset.getOverruns() == 0,
                "" + reset);
        check("The default budget is the 50 Hz the gains assume",
                Math.abs(new LoopTimer().getBudgetMs() - 20.0) < 1e-9,
                "" + new LoopTimer().getBudgetMs());
    }

    // -----------------------------------------------------------------
    // The recorder's schema
    // -----------------------------------------------------------------

    static void recorderSchema() {
        System.out.println("\n=== Recorder schema ===");
        check("Column names are unique",
                new java.util.HashSet<>(java.util.Arrays.asList(MatchRecorder.COLUMNS))
                        .size() == MatchRecorder.COLUMNS.length,
                "a duplicate column name means two things share a column");
        boolean noCommas = true;
        for (String column : MatchRecorder.COLUMNS) {
            if (column.contains(",") || column.trim().isEmpty()) noCommas = false;
        }
        check("No column name contains a comma", noCommas, "a name would split a cell");

        // The diagnosis these logs exist for needs all of these; losing one quietly
        // is how a log turns out to be useless after the match rather than during it.
        String[] required = {
                "t", "x", "y", "heading_deg", "odo_x", "odo_y",
                "vx", "vy", "omega_dps",
                "vision_ok", "tags", "heading_trusted",
                "shot_dist_in", "heading_err_deg", "in_range", "aimed",
                "fly_set_rpm", "fly_rpm", "at_speed",
                "loop_ms",
        };
        List<String> columns = java.util.Arrays.asList(MatchRecorder.COLUMNS);
        StringBuilder missing = new StringBuilder();
        for (String name : required) {
            if (!columns.contains(name)) missing.append(name).append(' ');
        }
        check("Everything a shot depends on is recorded", missing.length() == 0,
                "missing: " + missing);
        System.out.printf("   %d columns%n", MatchRecorder.COLUMNS.length);
    }
}
