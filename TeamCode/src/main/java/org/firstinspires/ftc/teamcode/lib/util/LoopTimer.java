/*
 * Measures how long the control loop actually takes.
 *
 * Every control gain in this template is tuned at some loop rate, and nothing
 * warns you when that rate changes. Add a bit of telemetry here, a per-loop
 * String.format there, a logger that flushes to storage inline, and the loop
 * quietly drops from 50 Hz to 15 Hz. Nothing throws. The robot just gets worse:
 * the heading PID's derivative term is computed over a longer dt so it damps less,
 * the follower's braking feedforward updates three times less often and overshoots,
 * and the shot solution is stale by the time it is used.
 *
 * So the loop rate is measured and shown, and code that might cost something --
 * the match logger especially -- is judged against this rather than assumed cheap.
 *
 * Costs nothing to run: a handful of doubles and one array increment per loop, no
 * allocation, no formatting. The formatting happens only when telemetry asks.
 */
package org.firstinspires.ftc.teamcode.lib.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class LoopTimer {

    /**
     * What a loop is expected to fit inside, in seconds.
     *
     * 20 ms (50 Hz) is what a simple FTC loop with one Pinpoint read and a
     * Limelight poll comfortably achieves, and what the control gains here assume.
     * A loop past this is not an error -- it is a signal that something in the loop
     * has started costing real time.
     */
    public static final double DEFAULT_BUDGET_SECONDS = 0.020;

    /**
     * A gap longer than this is a stall, not a slow loop: init pauses, the first
     * loop after start, a garbage collection freeze. Counted separately so one
     * 800 ms hiccup cannot make the average look terrible.
     */
    private static final double STALL_SECONDS = 0.250;

    /** 1 ms buckets. The last one is "this or worse". */
    private static final int BUCKETS = 40;

    private final double budgetSeconds;

    private final int[] histogram = new int[BUCKETS];
    private double lastTime = Double.NaN;
    private double lastDt = 0.0;
    private double totalTime = 0.0;
    private double worstDt = 0.0;
    private long loops = 0;
    private long overruns = 0;
    private long stalls = 0;

    public LoopTimer() {
        this(DEFAULT_BUDGET_SECONDS);
    }

    public LoopTimer(double budgetSeconds) {
        this.budgetSeconds = budgetSeconds;
    }

    /** Call exactly once per control loop. */
    public void tick() {
        tick(RobotClock.nowSeconds());
    }

    /** As {@link #tick()}, with an explicit timestamp (for offline simulation). */
    public void tick(double nowSeconds) {
        if (Double.isNaN(lastTime)) {
            lastTime = nowSeconds;
            return;
        }
        double dt = nowSeconds - lastTime;
        lastTime = nowSeconds;
        if (dt < 0) {
            return;     // clock went backwards; nothing useful to record
        }
        lastDt = dt;

        if (dt > STALL_SECONDS) {
            stalls++;
            return;
        }

        loops++;
        totalTime += dt;
        if (dt > worstDt) worstDt = dt;
        if (dt > budgetSeconds) overruns++;

        int bucket = (int) (dt * 1000.0);
        if (bucket < 0) bucket = 0;
        if (bucket >= BUCKETS) bucket = BUCKETS - 1;
        histogram[bucket]++;
    }

    /** The most recent loop, in milliseconds. */
    public double getLastMs() {
        return lastDt * 1000.0;
    }

    /** Mean loop time in milliseconds, stalls excluded. */
    public double getMeanMs() {
        return loops == 0 ? 0.0 : (totalTime / loops) * 1000.0;
    }

    /** Loop rate in Hz, from the mean. */
    public double getHz() {
        double mean = getMeanMs();
        return mean <= 0 ? 0.0 : 1000.0 / mean;
    }

    /** The single worst loop in milliseconds, stalls excluded. */
    public double getWorstMs() {
        return worstDt * 1000.0;
    }

    /**
     * An upper bound on the given fraction of loops, in whole milliseconds.
     *
     * The tail is what matters: a mean of 12 ms with one loop in twenty taking
     * 40 ms is a robot that stutters, and the mean alone will not show it.
     *
     * @param fraction e.g. 0.95 for the 95th percentile.
     * @return the top of the bucket that fraction falls in, so "at or under N ms".
     */
    public double getPercentileMs(double fraction) {
        if (loops == 0) return 0.0;
        long target = (long) Math.ceil(fraction * loops);
        long seen = 0;
        for (int i = 0; i < BUCKETS; i++) {
            seen += histogram[i];
            if (seen >= target) {
                return i + 1.0;
            }
        }
        return BUCKETS;
    }

    public long getLoops() {
        return loops;
    }

    /** Loops that took longer than the budget. */
    public long getOverruns() {
        return overruns;
    }

    /** Gaps too long to be a slow loop -- init pauses, GC freezes. */
    public long getStalls() {
        return stalls;
    }

    public double getOverrunFraction() {
        return loops == 0 ? 0.0 : (double) overruns / loops;
    }

    public double getBudgetMs() {
        return budgetSeconds * 1000.0;
    }

    public void reset() {
        lastTime = Double.NaN;
        lastDt = 0.0;
        totalTime = 0.0;
        worstDt = 0.0;
        loops = 0;
        overruns = 0;
        stalls = 0;
        java.util.Arrays.fill(histogram, 0);
    }

    /** One line when healthy, and the detail only when there is something to see. */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Loop", "%.1f ms (%.0f Hz), worst %.1f",
                getMeanMs(), getHz(), getWorstMs());
        if (overruns > 0) {
            telemetry.addData("Loop overruns", "%d of %d (%.1f%%), 95th pct <= %.0f ms",
                    overruns, loops, getOverrunFraction() * 100.0,
                    getPercentileMs(0.95));
        }
        if (stalls > 0) {
            telemetry.addData("Loop stalls", "%d gaps over %.0f ms",
                    stalls, STALL_SECONDS * 1000.0);
        }
    }

    @Override
    public String toString() {
        return String.format("%.1f ms mean, %.1f worst, %d/%d over %.0f ms",
                getMeanMs(), getWorstMs(), overruns, loops, getBudgetMs());
    }
}
