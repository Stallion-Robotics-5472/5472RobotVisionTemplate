/*
 * A CSV log the control loop can write to without ever waiting for storage.
 *
 * The obvious way to log a match is to open a writer and call println() in the
 * loop. That works right up until the flush lands on a slow write, and then one
 * loop takes 80 ms. The control gains were tuned at 50 Hz; a loop that
 * occasionally takes four times as long does not announce itself, it just makes
 * the robot worse. And a log that costs you the match it was recording is not a
 * diagnostic tool.
 *
 * So writing is split in two:
 *
 *   - The control loop copies numbers into a pre-allocated ring buffer. No file
 *     access, no allocation, no locks, no formatting -- it is a handful of array
 *     stores and one volatile write, and it cannot block.
 *   - A background daemon thread drains the ring, formats it, and writes it out.
 *     If storage is slow, the ring fills.
 *
 * And when the ring fills, rows are DROPPED and counted. That is the whole point:
 * the alternative is blocking the control loop, which is the thing we are trying
 * to avoid. The drop count is part of the telemetry so a log with holes in it
 * cannot be mistaken for a complete one.
 *
 * Nothing here throws into the control loop. A storage failure -- no permission,
 * disk full, path missing -- sets an error message, stops the writer, and turns
 * write() into a no-op. A team's OpMode must not crash because logging broke.
 *
 *   MatchLog log = MatchLog.open("teleop", "t", "x", "y", "heading");
 *   ...
 *   double[] row = log.claim();
 *   if (row != null) {
 *       row[0] = t; row[1] = x; row[2] = y; row[3] = heading;
 *       log.commit();
 *   }
 *   ...
 *   log.close();
 *
 * Threading: exactly one thread may call claim/commit -- the control loop. The
 * ring is single-producer, single-consumer, which is what lets it avoid locks.
 */
package org.firstinspires.ftc.teamcode.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Locale;

public final class MatchLog {

    /**
     * Rows held in memory. At 50 Hz this is about 80 seconds, comfortably longer
     * than a match, so a log only develops holes if storage genuinely stalls for
     * that long.
     */
    public static final int DEFAULT_CAPACITY_ROWS = 4096;

    /** Where logs go on a Control Hub. Visible over USB and in the file manager. */
    public static final String DEFAULT_DIRECTORY = "/sdcard/FIRST/matchlogs";

    /** Logs to keep in the directory. Older ones are deleted when a new one opens. */
    public static final int KEEP_FILES = 25;

    /** How long the writer sleeps when it has caught up. */
    private static final long IDLE_SLEEP_MS = 10;

    /** Rows between flushes. A flush is the expensive part, so batch them. */
    private static final int ROWS_PER_FLUSH = 100;

    private final String[] columns;
    private final int capacity;
    private final double[][] ring;
    private final File file;

    /** Rows published by the control loop. Written only by the producer. */
    private volatile long head = 0;
    /** Rows consumed by the writer thread. Written only by the consumer. */
    private volatile long tail = 0;

    private long dropped = 0;
    private long peakDepth = 0;
    private volatile String error = null;
    private volatile boolean running = false;
    private Thread writerThread;

    private MatchLog(File file, int capacityRows, String[] columns) {
        this.file = file;
        this.capacity = Math.max(16, capacityRows);
        this.columns = columns.clone();
        this.ring = new double[this.capacity][columns.length];
    }

    // ---------------------------------------------------------------------
    // Opening
    // ---------------------------------------------------------------------

    /**
     * Opens a log named after {@code label} and the current time, in
     * {@link #DEFAULT_DIRECTORY}.
     *
     * Never throws. If the file cannot be opened the returned log is inert: its
     * {@link #claim()} returns null forever and {@link #getError()} says why.
     */
    public static MatchLog open(String label, String... columns) {
        return open(new File(DEFAULT_DIRECTORY), label, DEFAULT_CAPACITY_ROWS, columns);
    }

    /** As {@link #open(String, String...)}, into a directory of your choosing. */
    public static MatchLog open(File directory, String label, int capacityRows,
                                String... columns) {
        String stamp = new java.text.SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US)
                .format(new java.util.Date());
        String safeLabel = label == null ? "log" : label.replaceAll("[^A-Za-z0-9_-]", "_");
        File target = new File(directory, safeLabel + "-" + stamp + ".csv");

        MatchLog log = new MatchLog(target, capacityRows, columns);
        try {
            if (!directory.exists() && !directory.mkdirs()) {
                throw new IOException("could not create " + directory);
            }
            pruneOldLogs(directory);
            log.start();
        } catch (IOException | RuntimeException e) {
            // Deliberately swallowed: a logging failure must not take an OpMode
            // down. The error surfaces through getError() and addTelemetry().
            log.error = describe(e);
            log.running = false;
        }
        return log;
    }

    /**
     * Streams to a writer you supply instead of a file, for a log that should go
     * somewhere else -- and for the offline tests, which use a deliberately slow
     * writer to prove the control loop still does not wait.
     *
     * The writer is closed when the log is closed.
     */
    public static MatchLog into(String name, Writer sink, int capacityRows,
                                String... columns) {
        MatchLog log = new MatchLog(new File(name), capacityRows, columns);
        try {
            log.start(sink);
        } catch (IOException | RuntimeException e) {
            log.error = describe(e);
            log.running = false;
        }
        return log;
    }

    /**
     * Deletes the oldest logs beyond {@link #KEEP_FILES}.
     *
     * Storage on a Control Hub is small and nobody remembers to clear it out. A
     * logger that fills the device is worse than no logger, because it takes the
     * rest of the robot's storage with it.
     */
    private static void pruneOldLogs(File directory) {
        File[] existing = directory.listFiles((dir, name) -> name.endsWith(".csv"));
        if (existing == null || existing.length <= KEEP_FILES) {
            return;
        }
        Arrays.sort(existing, Comparator.comparingLong(File::lastModified));
        for (int i = 0; i < existing.length - KEEP_FILES; i++) {
            // Best effort: a file we cannot delete is not worth failing over.
            //noinspection ResultOfMethodCallIgnored
            existing[i].delete();
        }
    }

    private void start() throws IOException {
        start(new BufferedWriter(new FileWriter(file), 1 << 16));
    }

    private void start(final Writer writer) throws IOException {
        // Write the header on THIS thread, so a sink that cannot be written fails
        // now and reports an error, rather than silently doing nothing on a
        // background thread nobody is watching.
        StringBuilder header = new StringBuilder();
        for (int i = 0; i < columns.length; i++) {
            if (i > 0) header.append(',');
            header.append(columns[i]);
        }
        writer.write(header.toString());
        writer.write('\n');
        writer.flush();

        running = true;
        writerThread = new Thread(() -> drainLoop(writer), "MatchLog");
        // Daemon: an OpMode that forgets to close() must not keep the app alive.
        writerThread.setDaemon(true);
        // Below the control loop, so logging loses the CPU race rather than winning it.
        writerThread.setPriority(Thread.MIN_PRIORITY);
        writerThread.start();
    }

    // ---------------------------------------------------------------------
    // The control loop's side: claim, fill, commit
    // ---------------------------------------------------------------------

    /**
     * The next row to fill in, or null if there is no room (the row is dropped and
     * counted) or the log is not running.
     *
     * Call {@link #commit()} once the row is filled. Do not hold the array: it is
     * reused, and the writer thread reads it after commit.
     */
    public double[] claim() {
        if (!running) {
            return null;
        }
        // Reading the consumer's tail is the only cross-thread read here, and a
        // stale (low) value only ever makes us conservative: we drop a row we could
        // have kept, never overwrite one the writer has not consumed.
        if (head - tail >= capacity) {
            dropped++;
            return null;
        }
        long depth = head - tail;
        if (depth > peakDepth) peakDepth = depth;
        return ring[(int) (head % capacity)];
    }

    /**
     * Publishes the row from {@link #claim()}.
     *
     * The volatile write is what makes the row visible to the writer thread: every
     * array store before it happens-before the writer's read of head.
     */
    public void commit() {
        head = head + 1;
    }

    /**
     * Convenience for callers that would rather not manage the row buffer. Costs
     * one small array per call (the varargs), which is why the hot path uses
     * {@link #claim()} instead.
     *
     * @return false if the row was dropped.
     */
    public boolean write(double... values) {
        double[] row = claim();
        if (row == null) {
            return false;
        }
        int n = Math.min(row.length, values.length);
        System.arraycopy(values, 0, row, 0, n);
        // A short row leaves stale numbers from a previous use of this slot, which
        // would read as real data. Blank the rest.
        for (int i = n; i < row.length; i++) {
            row[i] = Double.NaN;
        }
        commit();
        return true;
    }

    // ---------------------------------------------------------------------
    // The writer thread's side
    // ---------------------------------------------------------------------

    private void drainLoop(Writer writer) {
        StringBuilder line = new StringBuilder(256);
        int sinceFlush = 0;
        try {
            while (true) {
                boolean wrote = false;
                while (tail < head) {
                    double[] row = ring[(int) (tail % capacity)];
                    line.setLength(0);
                    for (int i = 0; i < row.length; i++) {
                        if (i > 0) line.append(',');
                        append(line, row[i]);
                    }
                    line.append('\n');
                    writer.write(line.toString());
                    // Only now is the slot free for the producer to reuse.
                    tail = tail + 1;
                    wrote = true;
                    if (++sinceFlush >= ROWS_PER_FLUSH) {
                        writer.flush();
                        sinceFlush = 0;
                    }
                }
                if (!running && tail >= head) {
                    break;
                }
                if (!wrote) {
                    try {
                        Thread.sleep(IDLE_SLEEP_MS);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }
            }
            writer.flush();
        } catch (IOException e) {
            error = describe(e);
            running = false;
        } finally {
            try {
                writer.close();
            } catch (IOException ignored) {
                // Nothing useful to do; the error above already says what happened.
            }
        }
    }

    /**
     * Four decimal places, always with a dot.
     *
     * String.format would follow the device's locale, and on a Control Hub set to
     * a comma-decimal locale that writes "1,2345" into a comma-separated file --
     * every row silently gains a column and the log is unreadable. Doing the digits
     * by hand also keeps the writer thread off the formatter entirely.
     */
    static void append(StringBuilder out, double value) {
        if (Double.isNaN(value)) {
            return;             // empty cell reads as "no data" in every tool
        }
        if (Double.isInfinite(value)) {
            out.append(value > 0 ? "inf" : "-inf");
            return;
        }
        long scaled = Math.round(value * 10000.0);
        if (scaled < 0) {
            out.append('-');
            scaled = -scaled;
        }
        out.append(scaled / 10000);
        long fraction = scaled % 10000;
        if (fraction != 0) {
            out.append('.');
            // Leading zeros, then trailing zeros trimmed: 0.5 -> "0.5", not "0.5000".
            if (fraction < 10) out.append("000");
            else if (fraction < 100) out.append("00");
            else if (fraction < 1000) out.append('0');
            while (fraction % 10 == 0) fraction /= 10;
            out.append(fraction);
        }
    }

    private static String describe(Exception e) {
        String message = e.getMessage();
        return e.getClass().getSimpleName() + (message == null ? "" : ": " + message);
    }

    // ---------------------------------------------------------------------
    // Closing and status
    // ---------------------------------------------------------------------

    /**
     * Stops accepting rows and waits briefly for the queue to reach storage.
     *
     * Bounded on purpose: an OpMode's stop path has limited time before the SDK
     * kills it, so this gives up rather than hanging. Rows still queued when it
     * gives up are lost, and {@link #getQueueDepth()} says how many.
     */
    public void close() {
        running = false;
        Thread thread = writerThread;
        if (thread != null) {
            try {
                thread.join(400);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /** True while rows are being accepted. */
    public boolean isRunning() {
        return running;
    }

    /** Rows handed over by the control loop. */
    public long getRowsWritten() {
        return head;
    }

    /** Rows written out to the file so far. */
    public long getRowsFlushed() {
        return tail;
    }

    /** Rows refused because the ring was full. Non-zero means the log has holes. */
    public long getRowsDropped() {
        return dropped;
    }

    /** Rows waiting to be written. */
    public long getQueueDepth() {
        return head - tail;
    }

    /** The deepest the queue ever got -- how close logging came to dropping rows. */
    public long getPeakQueueDepth() {
        return peakDepth;
    }

    public int getCapacity() {
        return capacity;
    }

    /** Why logging is off, or null if it is fine. */
    public String getError() {
        return error;
    }

    public File getFile() {
        return file;
    }

    public String[] getColumns() {
        return columns.clone();
    }

    /** One line when healthy; says plainly when the log is incomplete. */
    public void addTelemetry(Telemetry telemetry) {
        if (error != null) {
            telemetry.addData("Match log", "OFF - %s", error);
            return;
        }
        if (dropped > 0) {
            telemetry.addData("Match log", "%d rows, %d DROPPED (log has holes)",
                    head, dropped);
        } else {
            telemetry.addData("Match log", "%d rows, queue %d/%d (peak %d)",
                    head, getQueueDepth(), capacity, peakDepth);
        }
    }
}
