/*
 * Measures the four drivetrain constants the path follower and the heading
 * controller depend on, and prints them ready to paste into PathConstants.
 *
 * Why this exists: the numbers shipped in PathConstants are estimates for a
 * generic 15-inch mecanum. They are in the right ballpark and nothing more. The
 * follower uses MAX_ROBOT_SPEED and ZERO_POWER_DECEL_RATE to decide how early to
 * start braking for the end of a path, and the aiming feedforward uses
 * TURN_POWER_PER_RAD_PER_SEC to keep up with a sweeping target. If those are
 * wrong the robot overshoots path endpoints and permanently trails a moving aim --
 * both of which look like "the PID needs tuning" and are not.
 *
 * Each test measures one thing and reports it. Nothing is written to disk: read
 * the numbers off the Driver Station and edit PathConstants by hand, so a bad run
 * cannot silently poison your constants.
 *
 * WHAT YOU NEED
 *   - about 10 feet of clear floor, on the same surface you compete on (carpet
 *     and tile give genuinely different numbers)
 *   - a freshly charged battery: every one of these numbers moves with voltage,
 *     which is why the battery reading is shown next to each result
 *   - nothing on the robot that can fall off under full-power acceleration
 *
 * CONTROLS (press once, during the run; the robot moves on its own)
 *   dpad up     - straight-line test  -> MAX_ROBOT_SPEED, ZERO_POWER_DECEL_RATE
 *                 Needs a clear runway AHEAD. Robot accelerates to full speed,
 *                 then coasts to a stop.
 *   dpad right  - turn test           -> TURN_POWER_PER_RAD_PER_SEC
 *                 Robot spins in place. Needs about 2 feet of clearance.
 *   dpad left   - stiction test       -> the smallest turn power that moves it
 *                 Ramps turn power up slowly until the robot starts to rotate.
 *   Y           - flywheel test       -> spin-up time, and whether the top of
 *                 your shot table is actually reachable. Skipped if no flywheel.
 *   A           - show the results as pasteable constants
 *   B           - clear the results
 *   BACK        - abort a running test immediately
 *
 * READING THE RESULTS -- see MANUAL.md section 8a, "Characterize the
 * drivetrain", for what each number means, how to tell a bad run from a bad
 * robot, and what to do about each result.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.RobotClock;
import org.firstinspires.ftc.teamcode.pathing.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pathing.PathConstants;
import org.firstinspires.ftc.teamcode.shooting.ShooterMap;
import org.firstinspires.ftc.teamcode.subsystems.PinpointOdometry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.List;

@TeleOp(name = "Drivetrain Characterization", group = "Setup")
public class DrivetrainCharacterization extends LinearOpMode {

    // ----- Safety limits. A characterization run must not end in a wall. -----

    /** Abort the straight-line test if the robot travels further than this. */
    private static final double MAX_TRAVEL_IN = 108.0;
    /** Stop accelerating once this far out, leaving room to coast. */
    private static final double ACCEL_DISTANCE_IN = 48.0;
    /** Give up on reaching a plateau after this long. */
    private static final double ACCEL_TIMEOUT_S = 4.0;
    /** Stop watching the coast after this long. */
    private static final double COAST_TIMEOUT_S = 3.0;
    /** Below this the robot counts as stopped (inches/sec). */
    private static final double STOPPED_SPEED_IN_S = 1.0;

    /** How long to hold full turn power, to be sure the turn rate has settled. */
    private static final double TURN_HOLD_S = 1.6;
    /** Only the last part of the turn is averaged -- the rest is spin-up. */
    private static final double TURN_SETTLE_S = 0.6;
    /** Turn rate that counts as "it is definitely rotating" (rad/sec). */
    private static final double MOVING_OMEGA = 0.35;

    /** Window the rate estimators differentiate over. */
    private static final double RATE_WINDOW_S = 0.15;

    private MecanumDrivetrain drivetrain;
    private PinpointOdometry odometry;
    private VoltageSensor battery;

    // Results, NaN until measured.
    private double maxSpeed = Double.NaN;
    private double decelRate = Double.NaN;
    private double coastEntrySpeed = Double.NaN;
    private double coastDistance = Double.NaN;
    private double fullPowerOmega = Double.NaN;
    private double lockPowerOmega = Double.NaN;
    private double stictionTurnPower = Double.NaN;
    private double spinUpSeconds = Double.NaN;
    private double spinUpCommandedRpm = Double.NaN;
    private double spinUpReachedRpm = Double.NaN;
    private double voltageAtStart = Double.NaN;
    private double lowestVoltage = Double.NaN;
    private String lastNote = "";

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrivetrain(hardwareMap);
        odometry = new PinpointOdometry(hardwareMap);
        battery = firstVoltageSensor();

        telemetry.addLine("Drivetrain Characterization");
        telemetry.addLine();
        telemetry.addLine("Run 'Drivetrain Direction Check' FIRST. These tests");
        telemetry.addLine("assume forward is forward and turn is CCW; if it is");
        telemetry.addLine("wired backwards the numbers will be nonsense.");
        telemetry.addLine();
        telemetry.addLine("Clear about 10 ft ahead of the robot.");
        telemetry.addData("Battery", "%.2f V", voltage());
        telemetry.update();

        waitForStart();
        voltageAtStart = voltage();
        lowestVoltage = voltageAtStart;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                straightLineTest();
            } else if (gamepad1.dpad_right) {
                turnTest();
            } else if (gamepad1.dpad_left) {
                stictionTest();
            } else if (gamepad1.y) {
                flywheelTest();
            } else if (gamepad1.b) {
                clearResults();
            }

            odometry.update();
            trackVoltage();
            showMenu();
            idleLoop();
        }

        drivetrain.stop();
    }

    // ---------------------------------------------------------------------
    // Test 1: straight-line top speed and coast deceleration
    // ---------------------------------------------------------------------

    /**
     * Accelerates at full power until the speed stops rising, then cuts power and
     * watches the robot stop.
     *
     * Both numbers come out of one run on purpose: the deceleration has to be
     * measured from the speed the robot actually reaches, and measuring them
     * separately invites reading a coast-down that started from some other speed.
     */
    private void straightLineTest() {
        odometry.update();
        odometry.setPose(new Pose2d(0, 0, new Rotation2d(0)));
        WindowedRate speed = new WindowedRate(RATE_WINDOW_S);

        double start = RobotClock.nowSeconds();
        double peak = 0.0;
        double travelled = 0.0;
        boolean plateau = false;
        double plateauSince = Double.NaN;

        // --- accelerate ---
        while (opModeIsActive() && !gamepad1.back) {
            odometry.update();
            double now = RobotClock.nowSeconds();
            travelled = distanceFromOrigin();
            speed.add(now, travelled);
            double v = speed.get();
            peak = Math.max(peak, v);
            trackVoltage();

            // A plateau is the honest end of the acceleration phase: the entry
            // speed for the coast measurement has to be a settled speed, not
            // whatever the robot happened to be doing when a timer expired.
            if (v > STOPPED_SPEED_IN_S && v >= peak * 0.98) {
                if (Double.isNaN(plateauSince)) plateauSince = now;
                if (now - plateauSince > 0.30) plateau = true;
            } else {
                plateauSince = Double.NaN;
            }

            if (plateau || travelled >= ACCEL_DISTANCE_IN
                    || now - start > ACCEL_TIMEOUT_S) {
                break;
            }

            drivetrain.driveRobotCentric(1.0, 0.0, 0.0);
            telemetry.addLine("STRAIGHT TEST - accelerating");
            telemetry.addData("Travelled", "%.1f in", travelled);
            telemetry.addData("Speed", "%.1f in/s (peak %.1f)", v, peak);
            telemetry.addData("Abort", "BACK");
            telemetry.update();
        }

        double entrySpeed = speed.get();
        double cutAt = travelled;
        drivetrain.stop();

        // --- coast ---
        double coastStart = RobotClock.nowSeconds();
        double stopped = travelled;
        while (opModeIsActive() && !gamepad1.back) {
            odometry.update();
            double now = RobotClock.nowSeconds();
            stopped = distanceFromOrigin();
            speed.add(now, stopped);
            trackVoltage();

            if (Math.abs(speed.get()) < STOPPED_SPEED_IN_S
                    || now - coastStart > COAST_TIMEOUT_S
                    || stopped > MAX_TRAVEL_IN) {
                break;
            }
            telemetry.addLine("STRAIGHT TEST - coasting");
            telemetry.addData("Coasted", "%.1f in", stopped - cutAt);
            telemetry.addData("Speed", "%.1f in/s", speed.get());
            telemetry.update();
        }
        drivetrain.stop();

        if (gamepad1.back) {
            lastNote = "straight test aborted - results not recorded";
            waitForRelease();
            return;
        }

        maxSpeed = peak;
        coastEntrySpeed = entrySpeed;
        coastDistance = stopped - cutAt;
        decelRate = decelRateFrom(coastEntrySpeed, coastDistance);

        if (!plateau) {
            lastNote = "speed never plateaued - runway too short, MAX_ROBOT_SPEED"
                    + " is a floor not a measurement";
        } else if (coastDistance < 1.0) {
            lastNote = "coasted under an inch: the motors brake hard, so"
                    + " ZERO_POWER_DECEL_RATE is huge and imprecise";
        } else {
            lastNote = "straight test OK";
        }
        waitForRelease();
    }

    // ---------------------------------------------------------------------
    // Test 2: turn rate
    // ---------------------------------------------------------------------

    /**
     * Spins in place at full turn power, then at MAX_HEADING_LOCK_TURN, and
     * reports the settled turn rate for each.
     *
     * The second figure is the one that matters in a match: the heading lock and
     * the follower both clamp turn power to MAX_HEADING_LOCK_TURN to leave power
     * for translation, so that is the fastest the robot ever actually pivots
     * while aiming.
     */
    private void turnTest() {
        fullPowerOmega = measureTurnRate(1.0, "full power");
        if (Double.isNaN(fullPowerOmega)) return;

        // Let it settle between the two runs.
        drivetrain.stop();
        holdStill(0.5);

        lockPowerOmega = measureTurnRate(PathConstants.MAX_HEADING_LOCK_TURN,
                String.format("%.2f power", PathConstants.MAX_HEADING_LOCK_TURN));
        lastNote = Double.isNaN(lockPowerOmega) ? "turn test aborted" : "turn test OK";
        waitForRelease();
    }

    /** Holds {@code power} as a pure turn and returns the settled rate, rad/sec. */
    private double measureTurnRate(double power, String label) {
        odometry.update();
        double lastHeading = odometry.getPose().getHeading();
        double accumulated = 0.0;

        double start = RobotClock.nowSeconds();
        double settleStartAngle = Double.NaN;
        double settleStartTime = Double.NaN;

        while (opModeIsActive() && !gamepad1.back) {
            odometry.update();
            double now = RobotClock.nowSeconds();
            double heading = odometry.getPose().getHeading();
            accumulated += wrapRadians(heading - lastHeading);
            lastHeading = heading;
            trackVoltage();

            double elapsed = now - start;
            if (elapsed >= TURN_HOLD_S - TURN_SETTLE_S
                    && Double.isNaN(settleStartAngle)) {
                settleStartAngle = accumulated;
                settleStartTime = now;
            }
            if (elapsed > TURN_HOLD_S) break;

            drivetrain.driveRobotCentric(0.0, 0.0, power);
            telemetry.addLine("TURN TEST - " + label);
            telemetry.addData("Rotated", "%.0f deg", Math.toDegrees(accumulated));
            telemetry.addData("Elapsed", "%.2f / %.2f s", elapsed, TURN_HOLD_S);
            telemetry.addData("Abort", "BACK");
            telemetry.update();
        }
        drivetrain.stop();

        if (gamepad1.back || Double.isNaN(settleStartAngle)) {
            waitForRelease();
            return Double.NaN;
        }
        double span = RobotClock.nowSeconds() - settleStartTime;
        if (span < 0.1) return Double.NaN;
        return Math.abs(accumulated - settleStartAngle) / span;
    }

    // ---------------------------------------------------------------------
    // Test 3: stiction
    // ---------------------------------------------------------------------

    /**
     * Ramps turn power up from zero until the robot starts rotating.
     *
     * This is the number behind "the robot sits 3 degrees off target and never
     * closes the gap". A heading PID whose output for a small error is below this
     * power commands a rotation the robot physically will not perform, and no
     * amount of waiting fixes it -- the fix is more kP, or accepting a wider
     * tolerance.
     */
    private void stictionTest() {
        odometry.update();
        double lastHeading = odometry.getPose().getHeading();
        WindowedRate omega = new WindowedRate(RATE_WINDOW_S);
        double accumulated = 0.0;

        double power = 0.0;
        double start = RobotClock.nowSeconds();
        double found = Double.NaN;

        while (opModeIsActive() && !gamepad1.back && power < 0.85) {
            odometry.update();
            double now = RobotClock.nowSeconds();
            double heading = odometry.getPose().getHeading();
            accumulated += wrapRadians(heading - lastHeading);
            lastHeading = heading;
            omega.add(now, accumulated);
            trackVoltage();

            // Ramp slowly: a fast ramp measures inertia, not stiction.
            power = 0.06 * (now - start);
            drivetrain.driveRobotCentric(0.0, 0.0, power);

            if (Math.abs(omega.get()) > MOVING_OMEGA) {
                found = power;
                break;
            }
            telemetry.addLine("STICTION TEST - ramping turn power");
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Turn rate", "%.2f rad/s (need %.2f)",
                    omega.get(), MOVING_OMEGA);
            telemetry.addData("Abort", "BACK");
            telemetry.update();
        }
        drivetrain.stop();

        if (!Double.isNaN(found)) {
            stictionTurnPower = found;
            lastNote = String.format("stiction OK: kP must exceed %.2f to move a"
                    + " %.0f deg error", found / Math.toRadians(5.0), 5.0);
        } else {
            lastNote = gamepad1.back ? "stiction test aborted"
                    : "never started rotating below 0.85 power - check the wiring";
        }
        waitForRelease();
    }

    // ---------------------------------------------------------------------
    // Test 4: flywheel spin-up
    // ---------------------------------------------------------------------

    /**
     * Commands the fastest shot in the table from a dead stop and times how long
     * it takes to get there, then checks it actually gets there.
     *
     * Two things come out of this. The spin-up time tells you how early a marker
     * has to start the flywheel in autonomous -- a shooting leg that opens before
     * the wheel is up to speed wastes the whole leg. And a wheel that never
     * reaches the top of the table means the far end of your shot map is fiction,
     * whatever the numbers in it say.
     */
    private void flywheelTest() {
        ShooterSubsystem shooter;
        try {
            shooter = new ShooterSubsystem(hardwareMap);
        } catch (RuntimeException e) {
            lastNote = "no flywheel configured - skipped";
            waitForRelease();
            return;
        }

        ShooterMap map = shooter.getMap();
        double target = shooter.mapSetpointAt(map.maxDistance()).flywheelRpm;
        spinUpCommandedRpm = target;

        shooter.setFlywheelRpm(target);
        double start = RobotClock.nowSeconds();
        double reachedAt = Double.NaN;
        double best = 0.0;

        while (opModeIsActive() && !gamepad1.back) {
            double now = RobotClock.nowSeconds();
            best = Math.max(best, shooter.getFlywheelRpm());
            trackVoltage();

            if (Double.isNaN(reachedAt) && shooter.atSpeed()) {
                reachedAt = now;
            }
            // Keep going a little past first contact so a wheel that touches the
            // setpoint and falls back is not recorded as having reached it.
            if (now - start > 6.0 || (!Double.isNaN(reachedAt) && now - reachedAt > 1.5)) {
                break;
            }
            telemetry.addLine("FLYWHEEL TEST - spinning up");
            telemetry.addData("Target", "%.0f rpm", target);
            telemetry.addData("Now", "%.0f rpm", shooter.getFlywheelRpm());
            telemetry.addData("Elapsed", "%.2f s", now - start);
            telemetry.addData("Abort", "BACK");
            telemetry.update();
            idleLoop();
        }
        shooter.stop();

        spinUpSeconds = Double.isNaN(reachedAt) ? Double.NaN : reachedAt - start;
        spinUpReachedRpm = best;
        if (Double.isNaN(spinUpSeconds)) {
            lastNote = String.format("flywheel never reached %.0f rpm (best %.0f)"
                    + " - the far end of the shot map is not real", target, best);
        } else {
            lastNote = String.format("flywheel OK: %.2f s to %.0f rpm",
                    spinUpSeconds, target);
        }
        waitForRelease();
    }

    // ---------------------------------------------------------------------
    // Pure arithmetic, kept separate so it can be checked offline
    // ---------------------------------------------------------------------

    /**
     * Coast deceleration from the speed it started at and how far it took to
     * stop: v = sqrt(2*a*d), so a = v^2 / (2*d).
     *
     * @return inches/sec^2, or NaN if the coast distance is too small to divide by.
     */
    public static double decelRateFrom(double entrySpeedInPerSec, double coastInches) {
        if (!(coastInches > 0.05) || !(entrySpeedInPerSec > 0)) {
            return Double.NaN;
        }
        return (entrySpeedInPerSec * entrySpeedInPerSec) / (2.0 * coastInches);
    }

    /**
     * The feedforward constant from a measured turn rate: turn power needed per
     * radian/sec of commanded rotation, i.e. 1 / (rate at full power).
     */
    public static double turnPowerPerRadPerSec(double omegaAtFullPower) {
        if (!(omegaAtFullPower > 0.01)) {
            return Double.NaN;
        }
        return 1.0 / omegaAtFullPower;
    }

    /** Wraps an angle to [-pi, pi]. */
    public static double wrapRadians(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /**
     * Differentiates a signal over a sliding time window.
     *
     * A per-loop difference is far too noisy to read a plateau off: odometry
     * quantization over a 20 ms step is a large fraction of the movement in that
     * step. Differencing across a window of samples instead trades a little lag
     * for a number that holds still enough to compare against itself.
     */
    public static final class WindowedRate {
        private final double window;
        private final java.util.ArrayDeque<double[]> samples = new java.util.ArrayDeque<>();

        public WindowedRate(double windowSeconds) {
            this.window = windowSeconds;
        }

        public void add(double time, double value) {
            samples.addLast(new double[]{time, value});
            // Keep one sample older than the window, so the span stays >= window
            // instead of collapsing toward a single-loop difference.
            while (samples.size() > 2 && time - samples.peekFirst()[0] > window) {
                double[] oldest = samples.removeFirst();
                if (time - samples.peekFirst()[0] < window) {
                    samples.addFirst(oldest);
                    break;
                }
            }
        }

        /** Rate of change over the window, or 0 before there is anything to divide. */
        public double get() {
            if (samples.size() < 2) return 0.0;
            double[] first = samples.peekFirst();
            double[] last = samples.peekLast();
            double dt = last[0] - first[0];
            if (dt <= 1e-6) return 0.0;
            return (last[1] - first[1]) / dt;
        }

        public void clear() {
            samples.clear();
        }

        public int size() {
            return samples.size();
        }
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    private void showMenu() {
        telemetry.addLine("Drivetrain Characterization");
        telemetry.addLine("dpad UP straight | RIGHT turn | LEFT stiction");
        telemetry.addLine("Y flywheel | A results | B clear");
        telemetry.addLine();
        telemetry.addData("Battery", "%.2f V now, %.2f V lowest seen",
                voltage(), lowestVoltage);
        if (!lastNote.isEmpty()) {
            telemetry.addLine();
            telemetry.addData("Last", lastNote);
        }
        telemetry.addLine();

        if (gamepad1.a) {
            addResults();
        } else {
            telemetry.addData("MAX_ROBOT_SPEED", format(maxSpeed, "%.1f in/s"));
            telemetry.addData("ZERO_POWER_DECEL_RATE", format(decelRate, "%.1f in/s2"));
            telemetry.addData("TURN_POWER_PER_RAD_PER_SEC",
                    format(turnPowerPerRadPerSec(fullPowerOmega), "%.3f"));
            telemetry.addData("Stiction turn power", format(stictionTurnPower, "%.3f"));
            telemetry.addData("Flywheel spin-up", format(spinUpSeconds, "%.2f s"));
            telemetry.addLine();
            telemetry.addLine("Hold A for the pasteable constants.");
        }
        telemetry.update();
    }

    /** The results as PathConstants lines, alongside what is in the file now. */
    private void addResults() {
        telemetry.addLine("--- paste into PathConstants ---");
        if (!Double.isNaN(maxSpeed)) {
            telemetry.addLine(String.format(
                    "MAX_ROBOT_SPEED = %.1f;   // was %.1f",
                    maxSpeed, PathConstants.MAX_ROBOT_SPEED));
        }
        if (!Double.isNaN(decelRate)) {
            telemetry.addLine(String.format(
                    "ZERO_POWER_DECEL_RATE = %.1f;   // was %.1f",
                    decelRate, PathConstants.ZERO_POWER_DECEL_RATE));
            telemetry.addLine(String.format("  (from %.1f in/s over %.1f in)",
                    coastEntrySpeed, coastDistance));
        }
        double ff = turnPowerPerRadPerSec(fullPowerOmega);
        if (!Double.isNaN(ff)) {
            telemetry.addLine(String.format(
                    "TURN_POWER_PER_RAD_PER_SEC = %.3f;   // was %.3f",
                    ff, PathConstants.TURN_POWER_PER_RAD_PER_SEC));
            telemetry.addLine(String.format("  (%.2f rad/s at full power)",
                    fullPowerOmega));
        }
        if (!Double.isNaN(lockPowerOmega)) {
            telemetry.addLine(String.format(
                    "  at MAX_HEADING_LOCK_TURN=%.2f: %.2f rad/s (%.0f deg/s)",
                    PathConstants.MAX_HEADING_LOCK_TURN, lockPowerOmega,
                    Math.toDegrees(lockPowerOmega)));
        }
        if (!Double.isNaN(spinUpSeconds)) {
            telemetry.addLine(String.format(
                    "flywheel: %.2f s to %.0f rpm (reached %.0f)",
                    spinUpSeconds, spinUpCommandedRpm, spinUpReachedRpm));
        }
        telemetry.addLine(String.format("measured at %.2f V (low %.2f V)",
                voltageAtStart, lowestVoltage));
        telemetry.addLine("Re-run each test twice. If two runs disagree by more");
        telemetry.addLine("than ~10%, believe neither.");
    }

    private static String format(double value, String pattern) {
        return Double.isNaN(value) ? "not measured" : String.format(pattern, value);
    }

    // ---------------------------------------------------------------------
    // Plumbing
    // ---------------------------------------------------------------------

    private void clearResults() {
        maxSpeed = decelRate = coastEntrySpeed = coastDistance = Double.NaN;
        fullPowerOmega = lockPowerOmega = stictionTurnPower = Double.NaN;
        spinUpSeconds = spinUpCommandedRpm = spinUpReachedRpm = Double.NaN;
        lowestVoltage = voltage();
        lastNote = "results cleared";
        waitForRelease();
    }

    private double distanceFromOrigin() {
        return odometry.getPose().getTranslation().getNorm();
    }

    private void holdStill(double seconds) {
        double until = RobotClock.nowSeconds() + seconds;
        while (opModeIsActive() && RobotClock.nowSeconds() < until) {
            odometry.update();
            idleLoop();
        }
    }

    /**
     * Waits for the buttons to come up, so one press does not immediately start
     * the next test when a test finishes with the button still held.
     */
    private void waitForRelease() {
        drivetrain.stop();
        while (opModeIsActive() && (gamepad1.a || gamepad1.b || gamepad1.y
                || gamepad1.back || gamepad1.dpad_up || gamepad1.dpad_down
                || gamepad1.dpad_left || gamepad1.dpad_right)) {
            telemetry.addLine("Release the buttons.");
            telemetry.addData("Last", lastNote);
            telemetry.update();
            idleLoop();
        }
    }

    private VoltageSensor firstVoltageSensor() {
        List<VoltageSensor> sensors = hardwareMap.getAll(VoltageSensor.class);
        return sensors == null || sensors.isEmpty() ? null : sensors.get(0);
    }

    private double voltage() {
        return battery == null ? Double.NaN : battery.getVoltage();
    }

    private void trackVoltage() {
        double v = voltage();
        if (!Double.isNaN(v) && (Double.isNaN(lowestVoltage) || v < lowestVoltage)) {
            lowestVoltage = v;
        }
    }

    /** One loop's worth of yielding, without pretending to a fixed rate. */
    private void idleLoop() {
        // LinearOpMode.idle() yields to the SDK; a sleep here would stretch the
        // control loop and skew every rate this OpMode measures.
        idle();
    }
}
