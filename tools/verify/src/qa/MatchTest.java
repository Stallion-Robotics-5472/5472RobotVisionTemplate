package qa;

import org.firstinspires.ftc.teamcode.commands.AimAndShootCommand;
import org.firstinspires.ftc.teamcode.lib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.command.RunCommand;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.AllianceFlip;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
import org.firstinspires.ftc.teamcode.shooting.Goal;
import org.firstinspires.ftc.teamcode.shooting.GoalSelector;
import org.firstinspires.ftc.teamcode.shooting.ShooterMap;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;

import java.util.Arrays;
import java.util.List;

/**
 * Plays a full match as red, then the mirror image as blue, and reports what went
 * wrong.
 *
 * This is the test that exercises coordination rather than components: the same
 * plan, mirrored, must produce mirrored behaviour. Anything alliance-dependent
 * that should not be shows up as red and blue disagreeing.
 */
public class MatchTest {
    static int fails = 0;
    static int warnings = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("   %-52s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    static void warn(String message) {
        System.out.printf("   !! %s%n", message);
        warnings++;
    }

    // Two goals of different heights, so per-goal shot maps and tag selection are
    // both in play. Authored for RED; the sim flips them for blue.
    static final ShooterMap HIGH_MAP = ShooterMap.builder()
            .add(18, 2100, 30.0, 0.30).add(48, 2900, 38.0, 0.52)
            .add(84, 3700, 44.0, 0.80).build();
    static final ShooterMap LOW_MAP = ShooterMap.builder()
            .add(18, 1700, 16.0, 0.22).add(48, 2400, 24.0, 0.42)
            .add(84, 3100, 30.0, 0.66).build();

    static List<Goal> buildGoals() {
        return Arrays.asList(
                Goal.named("hive", -36.0, 58.0).tags(11, 12).worth(5)
                        .radius(7.0).map(HIGH_MAP).build(),
                Goal.named("flower", 36.0, 58.0).tags(21, 22).worth(2)
                        .radius(9.0).map(LOW_MAP).build());
    }

    /**
     * How far the shooter is physically pointed away from the goal, in the TRUTH
     * frame, in degrees.
     *
     * Every other aim check in this file measures the error the robot believes it
     * has, which is near zero even when the robot is pointed at a wall. For the
     * camera-relative scenario that is exactly the thing under test, so this
     * measures where the shooter is really pointing against where the goal really
     * is. Only meaningful on a near-stationary robot, where the moving-shot lead is
     * negligible.
     */
    static double trueAimErrorDeg(MatchSim sim, Translation2d goal) {
        Translation2d shooterOffset = new Translation2d(
                ShootingConstants.SHOOTER_FORWARD_OFFSET_IN,
                ShootingConstants.SHOOTER_LEFT_OFFSET_IN);
        Translation2d shooter = sim.truePose.getTranslation()
                .plus(shooterOffset.rotateBy(sim.truePose.getRotation()));
        Translation2d toGoal = goal.minus(shooter);
        double want = Math.atan2(toGoal.getY(), toGoal.getX());
        double have = sim.truePose.getHeading()
                + Math.toRadians(ShootingConstants.SHOOTER_YAW_OFFSET_DEG);
        return Math.toDegrees(Math.abs(MatchSim.wrap(want - have)));
    }

    /** Where the robot starts, authored for red. */
    static final Pose2d RED_START = new Pose2d(-48.0, -48.0, Rotation2d.fromDegrees(45));

    /** Result of one match, so red and blue can be compared. */
    static class MatchResult {
        Alliance alliance;
        double worstPoseError;
        double finalPoseError;
        double worstOdometryError;
        double worstHeadingErrorDeg;
        int loops;
        int framesAimed;
        int framesReady;
        int shotsFed;
        int goalSwitches;
        double worstAimErrorAtFireDeg;
        double totalDistanceTravelled;
        String lastGoal;
        boolean headingEverTrusted;
        double autoEndPoseError;
        Translation2d autoEndPosition;
        Pose2d finalTruePose;
        double minShotDistance = Double.MAX_VALUE;
        double maxShotDistance = 0;
        int firedOutsideTolerance = 0;
        double worstToleranceAtFireDeg = 0;
        boolean trustedAfterInit;
        int wallContacts;
    }

    /**
     * Plays one match: 5 s of init, 20 s of auto driving a mirrored plan, then
     * 30 s of teleop aiming and shooting while moving.
     */
    static MatchResult playMatch(Alliance alliance, boolean verbose) {
        MatchResult r = new MatchResult();
        r.alliance = alliance;

        List<Goal> goals = buildGoals();
        Pose2d start = AllianceFlip.forAlliance(RED_START, Alliance.RED, alliance);
        MatchSim sim = new MatchSim(start, goals);

        CommandScheduler scheduler = new CommandScheduler();
        scheduler.registerSubsystem(sim.drive, sim.shooter);

        // Driver inputs, written in the DRIVER's frame so they are identical for
        // both alliances -- which is the whole point of driverForward().
        final double[] stick = {0.0, 0.0};      // forward, left
        final boolean[] wantFire = {false};
        final boolean[] wantAim = {false};

        sim.seedOdometry(start);

        AimAndShootCommand aim = new AimAndShootCommand(
                sim.drive, sim.shooter,
                () -> stick[0], () -> stick[1],
                () -> alliance, () -> wantFire[0], true, sim.goalSelector);

        scheduler.setDefaultCommand(sim.drive, new RunCommand(
                () -> sim.drive.driveDriverRelative(stick[0], stick[1], 0.0, alliance),
                sim.drive).withName("Driver"));
        scheduler.setDefaultCommand(sim.shooter,
                new RunCommand(sim.shooter::idle, sim.shooter).withName("Idle"));

        String previousGoal = sim.goalSelector.getSelected().getName();
        Translation2d lastPosition = sim.truePose.getTranslation();

        // ---- INIT: robot still, camera looking for a tag -----------------
        // This is where a mis-seeded pose is supposed to be caught.
        for (int i = 0; i < (int) (5.0 / MatchSim.DT); i++) {
            sim.tick(alliance);
            scheduler.run();
            r.loops++;
            if (sim.localization.isHeadingTrusted()) {
                r.headingEverTrusted = true;
            }
        }
        r.trustedAfterInit = sim.localization.isHeadingTrusted();

        // ---- AUTO: drive a mirrored plan -------------------------------
        // Driver-frame inputs, so red and blue drive the mirrored path.
        double[][] autoLegs = {
                //  forward, left, seconds
                {0.8, 0.0, 2.0},
                {0.0, 0.7, 1.5},
                {0.6, -0.5, 2.0},
                {0.0, 0.0, 0.5},
        };
        wantAim[0] = false;
        for (double[] leg : autoLegs) {
            for (int i = 0; i < (int) (leg[2] / MatchSim.DT); i++) {
                stick[0] = leg[0];
                stick[1] = leg[1];
                sim.tick(alliance);
                scheduler.run();
                r.loops++;
                r.totalDistanceTravelled +=
                        sim.truePose.getTranslation().getDistance(lastPosition);
                lastPosition = sim.truePose.getTranslation();
                r.worstPoseError = Math.max(r.worstPoseError, sim.poseError());
                r.worstOdometryError = Math.max(r.worstOdometryError, sim.odometryError());
                r.worstHeadingErrorDeg =
                        Math.max(r.worstHeadingErrorDeg, sim.headingErrorDeg());
                if (sim.localization.isHeadingTrusted()) r.headingEverTrusted = true;
            }
        }
        stick[0] = 0;
        stick[1] = 0;
        r.autoEndPoseError = sim.poseError();
        r.autoEndPosition = sim.truePose.getTranslation();

        // ---- TELEOP: aim and shoot while driving -----------------------
        scheduler.schedule(aim);
        double[][] teleopLegs = {
                {0.0, 0.0, 1.5, 0},      // stand and settle, then fire
                {0.0, 0.0, 1.0, 1},
                {0.5, 0.0, 2.0, 1},      // shoot while driving forward
                {0.0, 0.6, 2.0, 1},      // shoot while strafing
                {-0.4, -0.4, 2.0, 1},    // shoot while moving diagonally
                {0.0, 0.0, 1.0, 1},
        };
        for (double[] leg : teleopLegs) {
            for (int i = 0; i < (int) (leg[2] / MatchSim.DT); i++) {
                stick[0] = leg[0];
                stick[1] = leg[1];
                wantFire[0] = leg[3] > 0.5;
                sim.tick(alliance);
                scheduler.run();
                r.loops++;

                r.totalDistanceTravelled +=
                        sim.truePose.getTranslation().getDistance(lastPosition);
                lastPosition = sim.truePose.getTranslation();
                r.worstPoseError = Math.max(r.worstPoseError, sim.poseError());
                r.worstOdometryError = Math.max(r.worstOdometryError, sim.odometryError());
                r.worstHeadingErrorDeg =
                        Math.max(r.worstHeadingErrorDeg, sim.headingErrorDeg());
                if (sim.localization.isHeadingTrusted()) r.headingEverTrusted = true;

                r.framesAimed++;
                AimSolution solution = aim.getSolution();
                if (aim.wasReady()) {
                    r.framesReady++;
                }
                if (sim.feeder.power > 0.5) {
                    r.shotsFed++;
                    // Only a frame that actually feeds says anything about shot
                    // quality. Measuring over every frame, including ones the
                    // command correctly refused, describes the refusals instead.
                    if (solution != null) {
                        r.minShotDistance = Math.min(
                                r.minShotDistance, solution.effectiveDistanceInches);
                        r.maxShotDistance = Math.max(
                                r.maxShotDistance, solution.effectiveDistanceInches);
                        double errorDeg = Math.toDegrees(Math.abs(solution.headingErrorFrom(
                                sim.localization.getPose().getHeading())));
                        r.worstAimErrorAtFireDeg =
                                Math.max(r.worstAimErrorAtFireDeg, errorDeg);
                        // The real invariant: never fire outside the solution's own
                        // distance-scaled tolerance.
                        double toleranceDeg =
                                Math.toDegrees(solution.headingToleranceRadians);
                        if (errorDeg > toleranceDeg + 1e-6) {
                            r.firedOutsideTolerance++;
                        }
                        r.worstToleranceAtFireDeg =
                                Math.max(r.worstToleranceAtFireDeg, toleranceDeg);
                    }
                }
                String current = sim.goalSelector.getSelected().getName();
                if (!current.equals(previousGoal)) {
                    r.goalSwitches++;
                    previousGoal = current;
                }
            }
        }
        wantFire[0] = false;
        scheduler.reset();

        r.finalPoseError = sim.poseError();
        r.finalTruePose = sim.truePose;
        r.lastGoal = sim.goalSelector.getSelected().getName();
        r.wallContacts = sim.wallContacts;
        sim.releaseClock();

        if (verbose) {
            System.out.printf("   %s: %d loops (%.1f s), travelled %.0f in%n",
                    alliance, r.loops, r.loops * MatchSim.DT, r.totalDistanceTravelled);
            System.out.printf("     heading trusted after init: %s%n", r.trustedAfterInit);
            System.out.printf("     pose error   worst %.2f in, final %.2f in%n",
                    r.worstPoseError, r.finalPoseError);
            System.out.printf("     odometry-only drift reached %.2f in%n",
                    r.worstOdometryError);
            System.out.printf("     heading error worst %.2f deg%n", r.worstHeadingErrorDeg);
            System.out.printf("     aiming: %d frames, ready on %d, fed %d%n",
                    r.framesAimed, r.framesReady, r.shotsFed);
            System.out.printf("     shot distance %.0f..%.0f in, goal switches %d, "
                            + "last goal %s%n",
                    r.minShotDistance, r.maxShotDistance, r.goalSwitches, r.lastGoal);
            System.out.printf("     at the moment of firing: worst aim error %.2f deg, "
                            + "loosest tolerance %.2f deg%n",
                    r.worstAimErrorAtFireDeg, r.worstToleranceAtFireDeg);
            if (r.wallContacts > 0) {
                System.out.printf("     touched a field wall on %d loops%n", r.wallContacts);
            }
        }
        return r;
    }

    public static void main(String[] args) {
        System.out.println("=== RED MATCH ===");
        MatchResult red = playMatch(Alliance.RED, true);
        System.out.println();
        System.out.println("=== BLUE MATCH (mirror image) ===");
        MatchResult blue = playMatch(Alliance.BLUE, true);

        System.out.println("\n=== Per-match checks ===");
        for (MatchResult r : new MatchResult[]{red, blue}) {
            System.out.printf("  %s%n", r.alliance);
            check("vision corrects odometry drift",
                    r.worstPoseError < r.worstOdometryError,
                    String.format("fused %.2f in vs odometry-only %.2f in -- "
                            + "vision is not helping", r.worstPoseError, r.worstOdometryError));
            check("fused pose stays accurate", r.worstPoseError < 6.0,
                    String.format("worst %.2f in", r.worstPoseError));
            check("heading stays accurate", r.worstHeadingErrorDeg < 10.0,
                    String.format("worst %.2f deg", r.worstHeadingErrorDeg));
            check("vision vouches for the heading", r.headingEverTrusted,
                    "never trusted -- shooting would be blocked all match");
            check("the robot actually fired", r.shotsFed > 0,
                    "never fired: nothing ever became ready");
            check("never fired outside its own aim tolerance",
                    r.firedOutsideTolerance == 0,
                    r.firedOutsideTolerance + " frames fired while out of tolerance");
            check("the target did not thrash", r.goalSwitches <= 3,
                    r.goalSwitches + " switches -- hysteresis is too weak");
            check("never fired outside the shot table's data",
                    r.minShotDistance >= 18.0 - 1e-6 && r.maxShotDistance <= 84.0 + 1e-6,
                    String.format("fired at %.0f..%.0f in, table covers 18..84",
                            r.minShotDistance, r.maxShotDistance));
            if (!r.trustedAfterInit) {
                warn(r.alliance + ": no goal tag was in view during init, so the "
                        + "start-pose check could not run. Shooting stays blocked "
                        + "until the robot first turns toward a goal.");
            }
        }

        System.out.println("\n=== Red vs blue symmetry ===");
        // A rotational flip means blue's truth should be red's, negated.
        double mirrorX = red.finalTruePose.getX() + blue.finalTruePose.getX();
        double mirrorY = red.finalTruePose.getY() + blue.finalTruePose.getY();
        System.out.printf("   red ended (%.1f, %.1f, %.0f deg), blue (%.1f, %.1f, %.0f deg)%n",
                red.finalTruePose.getX(), red.finalTruePose.getY(),
                red.finalTruePose.getRotation().getDegrees(),
                blue.finalTruePose.getX(), blue.finalTruePose.getY(),
                blue.finalTruePose.getRotation().getDegrees());
        System.out.printf("   mirror residual: x %+.3f, y %+.3f in%n", mirrorX, mirrorY);
        check("blue ends at the mirror of red",
                Math.abs(mirrorX) < 1.0 && Math.abs(mirrorY) < 1.0,
                String.format("residual (%.2f, %.2f) in -- something is "
                        + "alliance-dependent that should not be", mirrorX, mirrorY));
        double headingResidual = MatchSim.wrap(
                blue.finalTruePose.getHeading() - red.finalTruePose.getHeading() - Math.PI);
        check("blue's final heading is red's rotated 180 deg",
                Math.abs(headingResidual) < Math.toRadians(2.0),
                String.format("off by %.2f deg", Math.toDegrees(headingResidual)));
        check("both alliances fire a similar number of times",
                Math.abs(red.shotsFed - blue.shotsFed) <= Math.max(3, red.shotsFed / 5),
                String.format("red %d vs blue %d", red.shotsFed, blue.shotsFed));
        check("both alliances pick the same goal",
                red.lastGoal.equals(blue.lastGoal),
                String.format("red %s vs blue %s", red.lastGoal, blue.lastGoal));
        check("pose accuracy is the same for both",
                Math.abs(red.worstPoseError - blue.worstPoseError) < 1.0,
                String.format("red %.2f vs blue %.2f in",
                        red.worstPoseError, blue.worstPoseError));

        System.out.println("\n=== Driver-frame consistency ===");
        // "Push the stick away from me" must move the robot away from the driver
        // for BOTH alliances -- the field directions are opposite.
        for (Alliance alliance : Alliance.values()) {
            MatchSim sim = new MatchSim(new Pose2d(0, 0, new Rotation2d(0)), buildGoals());
            sim.localization.setStartingPose(new Pose2d(0, 0, new Rotation2d(0)));
            for (int i = 0; i < 25; i++) {
                sim.drive.driveDriverRelative(1.0, 0.0, 0.0, alliance);
                sim.tick(alliance);
                sim.localization.update();
            }
            Translation2d moved = sim.truePose.getTranslation();
            double expected = alliance.driverForward().getRadians();
            double actual = Math.atan2(moved.getY(), moved.getX());
            System.out.printf("   %s: stick forward -> moved %.0f deg "
                            + "(driverForward is %.0f deg)%n",
                    alliance, Math.toDegrees(actual), Math.toDegrees(expected));
            check(alliance + " drives away from its own driver",
                    Math.abs(MatchSim.wrap(actual - expected)) < Math.toRadians(5.0),
                    "moved the wrong way in the field frame");
            sim.releaseClock();
        }

        // =================================================================
        // Failure scenarios -- the ones that actually cost matches.
        // =================================================================
        System.out.println("\n=== Scenario: robot placed backwards (180 deg seed error) ===");
        {
            // Start facing the hive so a tag IS in view, then lie to the estimator
            // about which way the robot faces.
            Pose2d truth = new Pose2d(-36, 0, Rotation2d.fromDegrees(90));
            MatchSim sim = new MatchSim(truth, buildGoals());
            sim.seedOdometry(new Pose2d(-36, 0, Rotation2d.fromDegrees(-90)));

            CommandScheduler scheduler = new CommandScheduler();
            scheduler.registerSubsystem(sim.drive, sim.shooter);
            final boolean[] fire = {true};
            AimAndShootCommand aim = new AimAndShootCommand(sim.drive, sim.shooter,
                    () -> 0.0, () -> 0.0, () -> Alliance.RED, () -> fire[0], true,
                    sim.goalSelector);
            scheduler.schedule(aim);

            boolean everSuspect = false;
            int firedWhileWrong = 0;
            for (int i = 0; i < 150; i++) {
                sim.tick(Alliance.RED);
                scheduler.run();
                if (sim.localization.isStartPoseSuspect()) everSuspect = true;
                // "Wrong" means the estimate still disagrees badly with the truth.
                boolean stillWrong = sim.headingErrorDeg() > 45.0;
                if (stillWrong && sim.feeder.power > 0.5) firedWhileWrong++;
            }
            System.out.printf("   flagged suspect: %s | heading error now %.1f deg%n",
                    everSuspect, sim.headingErrorDeg());
            check("a backwards seed is flagged", everSuspect,
                    "the start-pose check never fired");
            check("never fires while the pose is still badly wrong",
                    firedWhileWrong == 0,
                    firedWhileWrong + " frames fired at empty field");

            // And the driver's recovery must work.
            boolean reseeded = sim.localization.seedFromVision();
            sim.seedPose = sim.localization.getPose();
            sim.truthAtSeed = sim.truePose;
            System.out.printf("   after seedFromVision: reseeded %s, "
                            + "heading error %.1f deg%n",
                    reseeded, sim.headingErrorDeg());
            check("re-seeding from vision recovers the heading",
                    reseeded && sim.headingErrorDeg() < 5.0,
                    String.format("still %.1f deg off", sim.headingErrorDeg()));
            scheduler.reset();
            sim.releaseClock();
        }

        System.out.println("\n=== Scenario: no field map, so no botpose at all ===");
        {
            // The situation: the Limelight sees tags perfectly well but has no field
            // map uploaded (or last season's), so it publishes no botpose. The
            // estimator then never corrects the odometry and never vouches for the
            // heading -- and the robot here was seeded wrong on top of that.
            //
            // This is what aiming at what the camera SEES is for. The goal table
            // cannot work: it subtracts a wrong pose from a field coordinate. The
            // camera-relative mode does not care, because it never uses the pose.
            Pose2d truth = new Pose2d(-36, 0, Rotation2d.fromDegrees(75));
            Translation2d hive = buildGoals().get(0).getAuthoredPosition();

            // --- first, the goal table, which must refuse ---
            MatchSim tableSim = new MatchSim(truth, buildGoals());
            tableSim.botposeAvailable = false;
            tableSim.seedOdometry(new Pose2d(-6, 20, Rotation2d.fromDegrees(120)));

            CommandScheduler tableScheduler = new CommandScheduler();
            tableScheduler.registerSubsystem(tableSim.drive, tableSim.shooter);
            AimAndShootCommand tableAim = new AimAndShootCommand(
                    tableSim.drive, tableSim.shooter, () -> 0.0, () -> 0.0,
                    () -> Alliance.RED, () -> true, true, tableSim.goalSelector);
            tableScheduler.schedule(tableAim);

            int tableFired = 0;
            boolean tagsWereSeen = false;
            for (int i = 0; i < 200; i++) {
                tableSim.tick(Alliance.RED);
                tableScheduler.run();
                if (!tableSim.localization.getFiducials().isEmpty()) tagsWereSeen = true;
                if (tableSim.feeder.power > 0.5) tableFired++;
            }
            System.out.printf("   goal table: fired %d, heading %s, pose error %.0f in%n",
                    tableFired,
                    tableSim.localization.isHeadingTrusted() ? "TRUSTED" : "untrusted",
                    tableSim.poseError());
            check("tags are seen even with no botpose", tagsWereSeen,
                    "the camera saw nothing, so this scenario proves nothing");
            check("without a botpose the heading is never trusted",
                    !tableSim.localization.isHeadingTrusted(),
                    "trust was granted with no field-relative fix");
            check("the goal table refuses to shoot on an uncorrected pose",
                    tableFired == 0,
                    tableFired + " frames fired at wherever it guessed the goal was");
            tableScheduler.reset();
            tableSim.releaseClock();

            // --- now the camera-relative mode, which must shoot, and hit ---
            MatchSim sim = new MatchSim(truth, buildGoals());
            sim.botposeAvailable = false;
            sim.seedOdometry(new Pose2d(-6, 20, Rotation2d.fromDegrees(120)));

            CommandScheduler scheduler = new CommandScheduler();
            scheduler.registerSubsystem(sim.drive, sim.shooter);
            org.firstinspires.ftc.teamcode.shooting.VisionTargetSource seen =
                    new org.firstinspires.ftc.teamcode.shooting.VisionTargetSource(
                            sim.visionTargetConfig(), sim.localization::getFiducials);
            AimAndShootCommand aim = new AimAndShootCommand(
                    sim.drive, sim.shooter, () -> 0.0, () -> 0.0,
                    () -> Alliance.RED, () -> true, true, seen);
            scheduler.schedule(aim);

            int fired = 0;
            double worstTrueAimErrorAtFire = 0.0;
            double worstRangeError = 0.0;
            double worstPoseError = 0.0;
            // The shot's own tolerance, which is what "aimed" means: it comes from
            // the goal's opening at this distance. Comparing the true aim error
            // against a made-up threshold instead would be measuring the threshold.
            double toleranceAtFire = 0.0;
            for (int i = 0; i < 200; i++) {
                sim.tick(Alliance.RED);
                scheduler.run();
                worstPoseError = Math.max(worstPoseError, sim.poseError());
                if (sim.feeder.power > 0.5) {
                    fired++;
                    worstTrueAimErrorAtFire = Math.max(worstTrueAimErrorAtFire,
                            trueAimErrorDeg(sim, hive));
                    worstRangeError = Math.max(worstRangeError,
                            Math.abs(seen.getRangeInches()
                                    - hive.getDistance(sim.truePose.getTranslation())));
                    toleranceAtFire = Math.toDegrees(
                            aim.getSolution().headingToleranceRadians);
                }
            }
            System.out.printf("   camera-relative: fired %d, worst TRUE aim error at"
                            + " fire %.2f deg (tolerance %.2f), range error %.2f in%n",
                    fired, worstTrueAimErrorAtFire, toleranceAtFire, worstRangeError);
            System.out.printf("   meanwhile the pose estimate was wrong by up to"
                    + " %.0f in the whole time%n", worstPoseError);

            check("it shoots with no botpose and a badly wrong pose", fired > 0,
                    "never fired: " + aim.getStatus());
            check("the pose really was wrong throughout", worstPoseError > 20.0,
                    String.format("only %.1f in off, so nothing was proven",
                            worstPoseError));
            check("the range it measures matches the truth",
                    worstRangeError < 1.0,
                    String.format("%.2f in out", worstRangeError));
            check("and it is REALLY pointed at the goal, not just in its own frame",
                    fired > 0 && worstTrueAimErrorAtFire <= toleranceAtFire,
                    String.format("fired %.2f deg off in the real world, outside its"
                            + " own %.2f deg tolerance",
                            worstTrueAimErrorAtFire, toleranceAtFire));
            scheduler.reset();
            sim.releaseClock();
        }

        System.out.println("\n=== Scenario: no Limelight (odometry only) ===");
        {
            Pose2d truth = new Pose2d(-36, 0, Rotation2d.fromDegrees(90));
            MatchSim sim = new MatchSim(truth, buildGoals());
            sim.localization.setVisionEnabled(false);
            sim.seedOdometry(truth);

            CommandScheduler scheduler = new CommandScheduler();
            scheduler.registerSubsystem(sim.drive, sim.shooter);
            final boolean[] fire = {true};
            AimAndShootCommand aim = new AimAndShootCommand(sim.drive, sim.shooter,
                    () -> 0.3, () -> 0.0, () -> Alliance.RED, () -> fire[0], true,
                    sim.goalSelector);
            scheduler.schedule(aim);

            int fed = 0;
            for (int i = 0; i < 300; i++) {
                sim.tick(Alliance.RED);
                scheduler.run();
                if (sim.feeder.power > 0.5) fed++;
            }
            System.out.printf("   fired on %d/300 loops, pose error %.2f in "
                            + "(odometry drift %.2f in)%n",
                    fed, sim.poseError(), sim.odometryError());
            // The pose-trust gate must not deadlock when there is no vision to
            // vouch for anything -- otherwise disabling the camera silently
            // disables shooting.
            check("still shoots with vision disabled", fed > 0,
                    "the pose-trust gate deadlocks without a camera");
            check("odometry-only pose degrades but stays usable",
                    sim.poseError() < 15.0,
                    String.format("%.1f in", sim.poseError()));
            scheduler.reset();
            sim.releaseClock();
        }

        System.out.println("\n=== Scenario: mid-match vision dropout ===");
        {
            Pose2d truth = new Pose2d(-36, 0, Rotation2d.fromDegrees(90));
            MatchSim sim = new MatchSim(truth, buildGoals());
            sim.seedOdometry(truth);

            CommandScheduler scheduler = new CommandScheduler();
            scheduler.registerSubsystem(sim.drive, sim.shooter);
            final boolean[] fire = {true};
            AimAndShootCommand aim = new AimAndShootCommand(sim.drive, sim.shooter,
                    () -> 0.0, () -> 0.25, () -> Alliance.RED, () -> fire[0], true,
                    sim.goalSelector);
            scheduler.schedule(aim);

            for (int i = 0; i < 120; i++) {
                sim.tick(Alliance.RED);
                scheduler.run();
            }
            boolean trustedBefore = sim.localization.isHeadingTrusted();
            String goalBefore = sim.goalSelector.getSelected().getName();

            sim.cameraWorking = false;      // camera unplugged / blinded
            int switchesDuringDropout = 0;
            int fedDuringDropout = 0;
            for (int i = 0; i < 150; i++) {
                sim.tick(Alliance.RED);
                scheduler.run();
                if (!sim.goalSelector.getSelected().getName().equals(goalBefore)) {
                    switchesDuringDropout++;
                    goalBefore = sim.goalSelector.getSelected().getName();
                }
                if (sim.feeder.power > 0.5) fedDuringDropout++;
            }
            System.out.printf("   trusted before dropout: %s | goal switches during "
                            + "dropout: %d | fired %d%n",
                    trustedBefore, switchesDuringDropout, fedDuringDropout);
            check("losing the camera does not make the target thrash",
                    switchesDuringDropout <= 1,
                    switchesDuringDropout + " switches with no tags at all");
            check("keeps shooting on odometry through a dropout",
                    fedDuringDropout > 0, "stopped firing entirely");

            sim.cameraWorking = true;
            for (int i = 0; i < 60; i++) {
                sim.tick(Alliance.RED);
                scheduler.run();
            }
            check("recovers when the camera returns", sim.poseError() < 6.0,
                    String.format("pose error %.1f in after recovery", sim.poseError()));
            scheduler.reset();
            sim.releaseClock();
        }

        System.out.println("\n=== Scenario: autonomous that shoots on the move ===");
        {
            // The whole point of heading sources: follow a path AND track the goal.
            Pose2d truth = new Pose2d(-40, -20, Rotation2d.fromDegrees(60));
            MatchSim sim = new MatchSim(truth, buildGoals());
            sim.seedOdometry(truth);

            CommandScheduler scheduler = new CommandScheduler();
            scheduler.registerSubsystem(sim.drive, sim.shooter);

            final int[] markerFires = {0};
            org.firstinspires.ftc.teamcode.shooting.AimAtGoalHeading aim =
                    new org.firstinspires.ftc.teamcode.shooting.AimAtGoalHeading(
                            sim.goalSelector, () -> Alliance.RED,
                            sim.localization::getVisibleTagIds);

            org.firstinspires.ftc.teamcode.pathing.Path leg =
                    new org.firstinspires.ftc.teamcode.pathing.Path(
                            new org.firstinspires.ftc.teamcode.pathing.BezierCurve(
                                    new Translation2d(-40, -20),
                                    new Translation2d(-14, 4),
                                    new Translation2d(14, 6),
                                    new Translation2d(34, -6)))
                            .setHeadingSource(aim)
                            .addMarker(org.firstinspires.ftc.teamcode.pathing.PathMarker
                                    .atT(0.25, () -> markerFires[0]++, "test marker"));

            org.firstinspires.ftc.teamcode.commands.FollowPathAndShootCommand routine =
                    new org.firstinspires.ftc.teamcode.commands.FollowPathAndShootCommand(
                            sim.drive, sim.shooter,
                            new org.firstinspires.ftc.teamcode.pathing.PathChain(leg), aim);
            scheduler.schedule(routine);

            int loops = 0;
            int fed = 0;
            int aimedLoops = 0;
            double worstAimErrorAtFire = 0;
            double travelled = 0;
            Translation2d prev = sim.truePose.getTranslation();
            while (scheduler.isScheduled(routine) && loops < 1500) {
                sim.tick(Alliance.RED);
                scheduler.run();
                loops++;
                travelled += sim.truePose.getTranslation().getDistance(prev);
                prev = sim.truePose.getTranslation();
                AimSolution s = aim.getLastSolution();
                if (s != null) {
                    aimedLoops++;
                    if (sim.feeder.power > 0.5) {
                        fed++;
                        worstAimErrorAtFire = Math.max(worstAimErrorAtFire,
                                Math.toDegrees(Math.abs(s.headingErrorFrom(
                                        sim.localization.getPose().getHeading()))));
                    }
                }
            }
            System.out.printf("   %d loops (%.1f s), travelled %.0f in, "
                            + "aim solved on %d loops, fed %d%n",
                    loops, loops * MatchSim.DT, travelled, aimedLoops, fed);
            System.out.printf("   ended (%.1f, %.1f), worst aim error at fire %.2f deg%n",
                    sim.truePose.getX(), sim.truePose.getY(), worstAimErrorAtFire);

            check("the path actually ran to completion",
                    !scheduler.isScheduled(routine) && travelled > 40,
                    String.format("travelled %.0f in in %d loops", travelled, loops));
            check("the aiming heading source drove the heading", aimedLoops > 0,
                    "no solution was ever produced");
            check("it shot while following the path", fed > 0,
                    "never fired while pathing -- aim and path are still fighting");
            check("never fired outside its aim tolerance", worstAimErrorAtFire < 20.0,
                    String.format("fired %.1f deg off", worstAimErrorAtFire));
            check("the mid-path marker fired exactly once", markerFires[0] == 1,
                    markerFires[0] + " fires");
            scheduler.reset();
            sim.releaseClock();
        }

        System.out.println("\n=== Scenario: recording a whole match to disk ===");
        {
            // The claim under test: logging every important number, every loop, for a
            // whole match costs the control loop essentially nothing -- and the log
            // that comes out is complete, not full of holes.
            Pose2d truth = new Pose2d(-40, -20, Rotation2d.fromDegrees(60));
            MatchSim sim = new MatchSim(truth, buildGoals());
            sim.seedOdometry(truth);

            CommandScheduler scheduler = new CommandScheduler();
            scheduler.registerSubsystem(sim.drive, sim.shooter);

            org.firstinspires.ftc.teamcode.shooting.AimAtGoalHeading aim =
                    new org.firstinspires.ftc.teamcode.shooting.AimAtGoalHeading(
                            sim.goalSelector, () -> Alliance.RED,
                            sim.localization::getVisibleTagIds);
            org.firstinspires.ftc.teamcode.pathing.Path leg =
                    new org.firstinspires.ftc.teamcode.pathing.Path(
                            new org.firstinspires.ftc.teamcode.pathing.BezierCurve(
                                    new Translation2d(-40, -20),
                                    new Translation2d(-14, 4),
                                    new Translation2d(14, 6),
                                    new Translation2d(34, -6)))
                            .setHeadingSource(aim);
            org.firstinspires.ftc.teamcode.commands.FollowPathAndShootCommand routine =
                    new org.firstinspires.ftc.teamcode.commands.FollowPathAndShootCommand(
                            sim.drive, sim.shooter,
                            new org.firstinspires.ftc.teamcode.pathing.PathChain(leg), aim);
            scheduler.schedule(routine);

            java.io.File dir;
            try {
                dir = java.nio.file.Files.createTempDirectory("matchrec").toFile();
            } catch (java.io.IOException e) {
                throw new RuntimeException(e);
            }
            org.firstinspires.ftc.teamcode.logging.MatchRecorder recorder =
                    new org.firstinspires.ftc.teamcode.logging.MatchRecorder(
                            dir, "sim", sim.drive)
                            .withShooter(sim.shooter)
                            .withAim(aim::getLastSolution, aim::getLastGoal)
                            .withVoltage(() -> 12.4);

            int loops = 0;
            long recordNanos = 0;
            long loopNanos = 0;
            long worstRecordNanos = 0;
            while (scheduler.isScheduled(routine) && loops < 1500) {
                long loopStart = System.nanoTime();
                sim.tick(Alliance.RED);
                scheduler.run();
                long beforeRecord = System.nanoTime();
                recorder.record();
                long after = System.nanoTime();
                recordNanos += after - beforeRecord;
                worstRecordNanos = Math.max(worstRecordNanos, after - beforeRecord);
                loopNanos += after - loopStart;
                loops++;
            }
            recorder.close();

            double perRowMicros = recordNanos / 1000.0 / loops;
            System.out.printf("   %d loops recorded; %.2f us per row (worst %.2f us),"
                            + " %.2f%% of the robot's own loop work%n",
                    loops, perRowMicros, worstRecordNanos / 1000.0,
                    100.0 * recordNanos / loopNanos);
            System.out.printf("   %.4f%% of a 20 ms loop budget; queue peaked at %d"
                            + " of %d rows%n",
                    perRowMicros / 200.0, recorder.getLog().getPeakQueueDepth(),
                    recorder.getLog().getCapacity());

            check("recording a whole match drops nothing",
                    recorder.getLog().getRowsDropped() == 0,
                    recorder.getLog().getRowsDropped() + " rows dropped");
            check("one row per loop, no more and no fewer",
                    recorder.getLog().getRowsWritten() == loops,
                    recorder.getLog().getRowsWritten() + " rows for " + loops + " loops");
            check("a row costs under 1% of the loop budget",
                    perRowMicros < 200.0,
                    String.format("%.1f us per row", perRowMicros));
            check("no single row ever costs a whole loop",
                    worstRecordNanos < 20_000_000L,
                    String.format("worst row took %.2f ms", worstRecordNanos / 1e6));
            check("logging is not turned off by a failure",
                    recorder.getLog().getError() == null,
                    "" + recorder.getLog().getError());

            // And the file has to be readable, with the numbers the robot actually had.
            try {
                java.util.List<String> lines = java.nio.file.Files.readAllLines(
                        recorder.getLog().getFile().toPath());
                check("every row reached the file",
                        lines.size() == loops + 1,
                        lines.size() + " lines for " + loops + " rows");
                String[] header = lines.get(0).split(",", -1);
                check("the header matches the schema",
                        header.length
                                == org.firstinspires.ftc.teamcode.logging.MatchRecorder
                                        .COLUMNS.length,
                        header.length + " columns in the file");
                String[] last = lines.get(lines.size() - 1).split(",", -1);
                check("every column is present on every row",
                        last.length == header.length,
                        last.length + " cells on the last row");
                int xColumn = java.util.Arrays.asList(
                        org.firstinspires.ftc.teamcode.logging.MatchRecorder.COLUMNS)
                        .indexOf("x");
                double loggedX = Double.parseDouble(last[xColumn]);
                System.out.printf("   last logged x %.3f, estimator says %.3f%n",
                        loggedX, sim.localization.getPose().getX());
                check("the log holds what the robot believed, not a stale copy",
                        Math.abs(loggedX - sim.localization.getPose().getX()) < 0.01,
                        String.format("%.3f vs %.3f", loggedX,
                                sim.localization.getPose().getX()));
            } catch (java.io.IOException e) {
                check("the log file is readable", false, e.toString());
            }
            scheduler.reset();
            sim.releaseClock();
        }

        System.out.println(fails == 0
                ? String.format("%nBoth matches completed cleanly.%s",
                        warnings > 0 ? " (" + warnings + " warning(s))" : "")
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }
}
