/*
 * Records what the robot believed, once per loop, to a CSV you can plot afterwards.
 *
 * This exists because of how vision and shooting failures present. "It shot three
 * and missed two" tells you nothing. The pose at the moment of each shot, whether
 * vision had vouched for the heading, how many tags were in frame, what distance
 * the solution used and what the flywheel was actually doing -- those tell you
 * which of half a dozen things went wrong, and they are gone the moment the match
 * ends unless something wrote them down.
 *
 * Everything that feeds a decision is logged, which is a deliberately wide net:
 * the fused pose AND the raw odometry pose (so drift is visible as the gap between
 * them), velocity as well as position (the moving-shot correction scales with it),
 * both the effective and standing shot distance, and the readiness gates
 * individually rather than one "ready" flag, because knowing WHICH gate was open
 * is the entire diagnosis.
 *
 * What it costs: reading the values already computed this loop and copying them
 * into an array. No formatting, no file access, no allocation -- see MatchLog for
 * why that matters and what happens when storage cannot keep up. LoggingTest
 * measures the per-loop cost; MatchTest runs a whole simulated match with the
 * recorder attached and checks the loop budget still holds.
 *
 *   recorder = new MatchRecorder("teleop", drive)
 *           .withShooter(shooter)
 *           .withAim(command::getLastSolution, selector::getSelected);
 *   // once per loop, after the subsystems have updated:
 *   recorder.record();
 *   // in stop():
 *   recorder.close();
 *
 * The CSV's first column is seconds since the recorder was created, so two logs
 * from the same match line up on it.
 */
package org.firstinspires.ftc.teamcode.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.LoopTimer;
import org.firstinspires.ftc.teamcode.lib.util.RobotClock;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
import org.firstinspires.ftc.teamcode.shooting.Goal;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Localization;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.io.File;
import java.util.function.Supplier;

public class MatchRecorder {

    /**
     * The schema. Order matters: it is the CSV's column order, and the indices
     * below index into it.
     *
     * Flags are logged as 1/0 rather than left out when false, so a gap in the
     * data is distinguishable from a false.
     */
    public static final String[] COLUMNS = {
            "t",                    // seconds since the recorder started
            "loop_ms",              // this loop's duration
            "loop_worst_ms",        // worst loop so far
            "loop_overruns",        // loops over the budget so far

            "x", "y", "heading_deg",                 // fused estimate
            "odo_x", "odo_y", "odo_heading_deg",     // odometry alone, for drift
            "vx", "vy", "speed", "omega_dps",

            "vision_ok",            // last frame accepted into the estimate
            "tags",                 // tags in that frame
            "tag_dist_in",          // average tag distance, inches
            "heading_trusted",      // vision has vouched for the heading
            "heading_disagree_deg", // vision heading vs odometry heading
            "tag_dist_bad",         // the distance reading itself looked implausible

            "goal",                 // index into ShootingConstants.GOALS, -1 unknown
            "shot_dist_in",         // effective (moving) distance the solution used
            "stand_dist_in",        // straight-line distance, for comparison
            "target_heading_deg",
            "heading_err_deg",
            "heading_tol_deg",
            "in_range",
            "aimed",

            "fly_set_rpm", "fly_rpm", "at_speed", "hood_deg",
            "volts",
    };

    private static final int I_T = 0;
    private static final int I_LOOP_MS = 1;
    private static final int I_LOOP_WORST = 2;
    private static final int I_LOOP_OVERRUNS = 3;
    private static final int I_X = 4;
    private static final int I_Y = 5;
    private static final int I_HEADING = 6;
    private static final int I_ODO_X = 7;
    private static final int I_ODO_Y = 8;
    private static final int I_ODO_HEADING = 9;
    private static final int I_VX = 10;
    private static final int I_VY = 11;
    private static final int I_SPEED = 12;
    private static final int I_OMEGA = 13;
    private static final int I_VISION_OK = 14;
    private static final int I_TAGS = 15;
    private static final int I_TAG_DIST = 16;
    private static final int I_TRUSTED = 17;
    private static final int I_DISAGREE = 18;
    private static final int I_TAG_DIST_BAD = 19;
    private static final int I_GOAL = 20;
    private static final int I_SHOT_DIST = 21;
    private static final int I_STAND_DIST = 22;
    private static final int I_TARGET_HEADING = 23;
    private static final int I_HEADING_ERR = 24;
    private static final int I_HEADING_TOL = 25;
    private static final int I_IN_RANGE = 26;
    private static final int I_AIMED = 27;
    private static final int I_FLY_SET = 28;
    private static final int I_FLY_RPM = 29;
    private static final int I_AT_SPEED = 30;
    private static final int I_HOOD = 31;
    private static final int I_VOLTS = 32;

    private final MatchLog log;
    private final LoopTimer loopTimer;
    private final DriveSubsystem drive;
    private final boolean ownsLoopTimer;
    private final double startTime;

    private ShooterSubsystem shooter;
    private Supplier<AimSolution> solutionSource;
    private Supplier<Goal> goalSource;
    private Supplier<Double> voltageSource;

    public MatchRecorder(String label, DriveSubsystem drive) {
        this(MatchLog.open(label, COLUMNS), drive, new LoopTimer());
    }

    /**
     * Shares an existing loop timer -- pass {@code getLoopTimer()} from a
     * CommandOpMode, which already ticks one, so the loop is not timed twice and
     * the log's loop columns match the telemetry.
     */
    public MatchRecorder(String label, DriveSubsystem drive, LoopTimer loopTimer) {
        this(MatchLog.open(label, COLUMNS), drive, loopTimer, false);
    }

    /** Logs into a directory of your choosing -- used by the offline tests. */
    public MatchRecorder(File directory, String label, DriveSubsystem drive) {
        this(MatchLog.open(directory, label, MatchLog.DEFAULT_CAPACITY_ROWS, COLUMNS),
                drive, new LoopTimer());
    }

    public MatchRecorder(MatchLog log, DriveSubsystem drive, LoopTimer loopTimer) {
        this(log, drive, loopTimer, true);
    }

    /**
     * @param ownsLoopTimer false when something else already calls
     *     {@code tick()} on this timer once per loop -- CommandOpMode does. Ticking
     *     it twice would halve every dt it reports.
     */
    public MatchRecorder(MatchLog log, DriveSubsystem drive, LoopTimer loopTimer,
                         boolean ownsLoopTimer) {
        this.log = log;
        this.drive = drive;
        this.loopTimer = loopTimer;
        this.ownsLoopTimer = ownsLoopTimer;
        this.startTime = RobotClock.nowSeconds();
    }

    /** Also record the flywheel and hood. */
    public MatchRecorder withShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
        return this;
    }

    /**
     * Also record the aiming solution and the goal it was for.
     *
     * Both suppliers may return null -- before the first solution, or on a path
     * with no aiming heading source -- and those columns are then left empty.
     */
    public MatchRecorder withAim(Supplier<AimSolution> solutionSource,
                                 Supplier<Goal> goalSource) {
        this.solutionSource = solutionSource;
        this.goalSource = goalSource;
        return this;
    }

    /** Also record battery voltage. Pass {@code () -> sensor.getVoltage()}. */
    public MatchRecorder withVoltage(Supplier<Double> voltageSource) {
        this.voltageSource = voltageSource;
        return this;
    }

    /**
     * Records one row. Call once per loop, after the subsystems have updated, so
     * the row is what the robot decided with rather than what it had last loop.
     *
     * Ticks the loop timer even when the log is off, so loop-rate telemetry works
     * whether or not anything is being recorded.
     */
    public void record() {
        if (ownsLoopTimer) {
            loopTimer.tick();
        }

        double[] row = log.claim();
        if (row == null) {
            return;             // dropped, or logging is off. Counted in MatchLog.
        }
        // Every column is written every row: a slot is reused, so a value left
        // alone would be last time's number wearing this row's timestamp.
        java.util.Arrays.fill(row, Double.NaN);

        row[I_T] = RobotClock.nowSeconds() - startTime;
        row[I_LOOP_MS] = loopTimer.getLastMs();
        row[I_LOOP_WORST] = loopTimer.getWorstMs();
        row[I_LOOP_OVERRUNS] = loopTimer.getOverruns();

        Localization localization = drive.getLocalization();
        Pose2d pose = localization.getPose();
        row[I_X] = pose.getX();
        row[I_Y] = pose.getY();
        row[I_HEADING] = Math.toDegrees(pose.getHeading());

        Pose2d odometry = localization.getOdometryPose();
        row[I_ODO_X] = odometry.getX();
        row[I_ODO_Y] = odometry.getY();
        row[I_ODO_HEADING] = Math.toDegrees(odometry.getHeading());

        Translation2d velocity = localization.getFieldVelocity();
        row[I_VX] = velocity.getX();
        row[I_VY] = velocity.getY();
        row[I_SPEED] = localization.getSpeed();
        row[I_OMEGA] = Math.toDegrees(localization.getAngularVelocity());

        row[I_VISION_OK] = flag(localization.wasLastVisionAccepted());
        row[I_TAGS] = localization.getLastTagCount();
        row[I_TAG_DIST] = localization.getLastAvgTagDistance();
        row[I_TRUSTED] = flag(localization.isHeadingTrusted());
        row[I_DISAGREE] = localization.getHeadingDisagreementDegrees();
        row[I_TAG_DIST_BAD] = flag(localization.isAvgTagDistanceImplausible());

        if (goalSource != null) {
            row[I_GOAL] = goalIndex(goalSource.get());
        }
        if (solutionSource != null) {
            AimSolution solution = solutionSource.get();
            if (solution != null) {
                row[I_SHOT_DIST] = solution.effectiveDistanceInches;
                row[I_STAND_DIST] = solution.actualDistanceInches;
                row[I_TARGET_HEADING] = Math.toDegrees(solution.targetHeadingRadians);
                row[I_HEADING_ERR] = Math.toDegrees(
                        solution.headingErrorFrom(pose.getHeading()));
                row[I_HEADING_TOL] = Math.toDegrees(solution.headingToleranceRadians);
                row[I_IN_RANGE] = flag(solution.inRange);
                row[I_AIMED] = flag(solution.isAimedFrom(pose.getHeading()));
            }
        }

        if (shooter != null) {
            row[I_FLY_SET] = shooter.getFlywheelSetpointRpm();
            row[I_FLY_RPM] = shooter.getFlywheelRpm();
            row[I_AT_SPEED] = flag(shooter.atSpeed());
            row[I_HOOD] = shooter.getHoodSetpointDegrees();
        }

        if (voltageSource != null) {
            Double volts = voltageSource.get();
            if (volts != null) row[I_VOLTS] = volts;
        }

        log.commit();
    }

    private static double flag(boolean value) {
        return value ? 1.0 : 0.0;
    }

    /** The goal's index in the shipped list, or -1 if it is not one of them. */
    private static double goalIndex(Goal goal) {
        if (goal == null) {
            return -1.0;
        }
        for (int i = 0; i < ShootingConstants.GOALS.length; i++) {
            if (ShootingConstants.GOALS[i].getName().equals(goal.getName())) {
                return i;
            }
        }
        return -1.0;
    }

    /** Flushes what is queued and stops the writer. Call from the OpMode's stop. */
    public void close() {
        log.close();
    }

    public MatchLog getLog() {
        return log;
    }

    public LoopTimer getLoopTimer() {
        return loopTimer;
    }

    /** Loop rate and log health, two lines. */
    public void addTelemetry(Telemetry telemetry) {
        loopTimer.addTelemetry(telemetry);
        log.addTelemetry(telemetry);
    }
}
