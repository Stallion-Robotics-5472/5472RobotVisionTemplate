package qa;

import org.firstinspires.ftc.teamcode.opmodes.DrivetrainCharacterization;
import org.firstinspires.ftc.teamcode.opmodes.DrivetrainCharacterization.WindowedRate;

/**
 * Checks the arithmetic behind the characterization OpMode.
 *
 * The OpMode itself needs a robot, but the parts that can be quietly wrong do
 * not: the coast-down formula, the feedforward inversion, the heading unwrap that
 * lets a spin past 180 degrees be counted, and the sliding-window differentiator
 * that every rate reading comes through. A characterization tool that reports a
 * confident wrong number is worse than none at all -- the team pastes it into
 * PathConstants and then tunes around it.
 */
public class CharacterizationTest {
    static int fails = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-58s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    public static void main(String[] args) {
        System.out.println("=== Coast deceleration ===");
        // v = sqrt(2*a*d): entering at 50 in/s and stopping in 41.67 in is 30 in/s^2.
        double a = DrivetrainCharacterization.decelRateFrom(50.0, 41.666667);
        System.out.printf("   50 in/s over 41.67 in -> %.2f in/s^2%n", a);
        check("Recovers the textbook deceleration", Math.abs(a - 30.0) < 0.01,
                "" + a);

        // The measurement this has to survive: integrate a real coast-down at a
        // known rate, then see whether the formula recovers that rate from just
        // the entry speed and the total distance, which is all the OpMode has.
        double truth = 42.0;
        double v = 60.0;
        double x = 0.0;
        double dt = 0.02;
        while (v > 0) {
            v -= truth * dt;
            if (v < 0) v = 0;
            x += v * dt;
        }
        double recovered = DrivetrainCharacterization.decelRateFrom(60.0, x);
        System.out.printf("   simulated coast at %.0f in/s^2 -> measured %.2f "
                + "(stopped in %.2f in)%n", truth, recovered, x);
        check("Recovers a simulated coast-down within 5%",
                Math.abs(recovered - truth) / truth < 0.05,
                String.format("%.2f vs %.2f", recovered, truth));

        check("A robot that did not move gives no answer, not infinity",
                Double.isNaN(DrivetrainCharacterization.decelRateFrom(50.0, 0.0)),
                "returned a number for a zero-length coast");
        check("A standing start gives no answer",
                Double.isNaN(DrivetrainCharacterization.decelRateFrom(0.0, 10.0)),
                "returned a number for zero entry speed");

        System.out.println("\n=== Turn feedforward ===");
        double ff = DrivetrainCharacterization.turnPowerPerRadPerSec(6.0);
        System.out.printf("   6.00 rad/s at full power -> %.4f power per rad/s%n", ff);
        check("Inverts the measured turn rate", Math.abs(ff - 1.0 / 6.0) < 1e-9, "" + ff);
        check("The shipped constant matches a 6 rad/s robot",
                Math.abs(ff - org.firstinspires.ftc.teamcode.pathing.PathConstants
                        .TURN_POWER_PER_RAD_PER_SEC) < 0.001,
                "the documented provenance of TURN_POWER_PER_RAD_PER_SEC no longer"
                        + " matches what this formula produces");
        check("A robot that did not turn gives no answer",
                Double.isNaN(DrivetrainCharacterization.turnPowerPerRadPerSec(0.0)),
                "returned a number for zero turn rate");

        System.out.println("\n=== Heading unwrap ===");
        // Two full turns, sampled coarsely enough to cross the wrap several times.
        double accumulated = 0.0;
        double last = 0.0;
        for (int i = 1; i <= 72; i++) {
            double trueAngle = Math.toRadians(i * 10.0);
            double reported = DrivetrainCharacterization.wrapRadians(trueAngle);
            accumulated += DrivetrainCharacterization.wrapRadians(reported - last);
            last = reported;
        }
        System.out.printf("   two full turns accumulate to %.1f deg%n",
                Math.toDegrees(accumulated));
        check("Accumulated rotation counts past 180 degrees",
                Math.abs(Math.toDegrees(accumulated) - 720.0) < 1e-6,
                Math.toDegrees(accumulated) + " deg");

        // And the other direction, because a sign slip here would read a CW spin
        // as a CCW one and invert the feedforward.
        accumulated = 0.0;
        last = 0.0;
        for (int i = 1; i <= 72; i++) {
            double reported = DrivetrainCharacterization.wrapRadians(
                    Math.toRadians(-i * 10.0));
            accumulated += DrivetrainCharacterization.wrapRadians(reported - last);
            last = reported;
        }
        check("A clockwise spin accumulates negative",
                Math.abs(Math.toDegrees(accumulated) + 720.0) < 1e-6,
                Math.toDegrees(accumulated) + " deg");

        System.out.println("\n=== Sliding-window rate ===");
        WindowedRate rate = new WindowedRate(0.15);
        check("Reads zero before it has two samples", rate.get() == 0.0, "" + rate.get());
        rate.add(0.0, 0.0);
        check("Still zero with one sample", rate.get() == 0.0, "" + rate.get());

        // A clean ramp: 30 in/s sampled every 20 ms.
        rate = new WindowedRate(0.15);
        for (int i = 0; i <= 50; i++) {
            rate.add(i * 0.02, i * 0.02 * 30.0);
        }
        System.out.printf("   clean 30 in/s ramp -> %.3f in/s over %d samples%n",
                rate.get(), rate.size());
        check("Measures a clean ramp exactly", Math.abs(rate.get() - 30.0) < 1e-9,
                "" + rate.get());
        check("The window really spans the requested time",
                rate.size() >= 8,
                "collapsed to " + rate.size() + " samples, so it is differentiating"
                        + " over roughly one loop");

        // The case it exists for. Odometry reports a quantized position; at 20 ms
        // and 30 in/s the robot moves 0.6 in per loop, so 0.1 in quantization is
        // a sixth of the signal. Differencing one loop at a time turns that into
        // huge frame-to-frame swings; a window averages it out.
        double quantum = 0.1;
        WindowedRate windowed = new WindowedRate(0.15);
        double worstWindowed = 0.0;
        double worstNaive = 0.0;
        double prevT = 0.0;
        double prevQ = 0.0;
        for (int i = 0; i <= 100; i++) {
            double t = i * 0.02;
            double trueX = t * 30.0;
            double q = Math.floor(trueX / quantum) * quantum;
            windowed.add(t, q);
            if (i > 10) {
                worstWindowed = Math.max(worstWindowed, Math.abs(windowed.get() - 30.0));
                double naive = (q - prevQ) / (t - prevT);
                worstNaive = Math.max(worstNaive, Math.abs(naive - 30.0));
            }
            prevT = t;
            prevQ = q;
        }
        System.out.printf("   with %.1f in quantization: windowed worst error %.2f in/s,"
                + " per-loop worst error %.2f in/s%n", quantum, worstWindowed, worstNaive);
        check("A window beats a per-loop difference on quantized odometry",
                worstWindowed < worstNaive / 3.0,
                String.format("%.2f vs %.2f -- the window is not buying anything",
                        worstWindowed, worstNaive));
        check("Windowed error is small enough to read a plateau off",
                worstWindowed < 1.0,
                String.format("%.2f in/s of noise would trigger the 2%% plateau test"
                        + " at random", worstWindowed));

        // Deceleration: the rate must go negative promptly once the signal turns
        // around, or the coast measurement never notices the robot stopped.
        WindowedRate stopping = new WindowedRate(0.15);
        double pos = 0.0;
        double speed = 40.0;
        int loopsUntilRead = -1;
        int actuallyStopped = -1;
        for (int i = 0; i <= 120; i++) {
            double t = i * 0.02;
            stopping.add(t, pos);
            if (i >= 10) {
                speed = Math.max(0.0, speed - 40.0 * 0.02);
            }
            if (speed == 0.0 && actuallyStopped < 0) actuallyStopped = i;
            pos += speed * 0.02;
            if (actuallyStopped >= 0 && loopsUntilRead < 0
                    && Math.abs(stopping.get()) < 1.0) {
                loopsUntilRead = i;
            }
        }
        System.out.printf("   robot stops at loop %d, window reads stopped at loop %d"
                        + " (%d loops of lag)%n",
                actuallyStopped, loopsUntilRead, loopsUntilRead - actuallyStopped);
        // The lag cannot be better than the window: a window still holding samples
        // from when the robot was moving reports it as moving. What matters is that
        // the lag is about the window and not several times it, since the coast
        // measurement waits on this reading before recording the distance.
        check("Notices a stop within about one window of it happening",
                loopsUntilRead > 0 && loopsUntilRead - actuallyStopped <= 10,
                "took " + (loopsUntilRead - actuallyStopped) + " loops, window is 8");

        System.out.println(fails == 0
                ? "\nAll characterization checks passed."
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }
}
