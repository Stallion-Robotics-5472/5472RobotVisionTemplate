package qa;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.AllianceFlip;
import org.firstinspires.ftc.teamcode.shooting.AimLogic;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
import org.firstinspires.ftc.teamcode.shooting.ShooterMap;

/**
 * Verifies the shoot-on-the-move aiming maths by SIMULATING THE GAME PIECE.
 *
 * The aiming solution is only correct if a piece launched along it actually lands
 * in the goal, so rather than assert on intermediate numbers, this flies the
 * piece: launch it from the shooter at the speed implied by the shot table,
 * add the robot's velocity (which the piece inherits), let it fly for the
 * table's time of flight, and measure where it lands.
 */
public class AimLogicTest {
    static int fails = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-56s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    /** A plausible shot table, in inches / rpm / degrees / seconds. */
    static final ShooterMap MAP = ShooterMap.builder()
            .add(12, 1800, 18.0, 0.22)
            .add(24, 2200, 24.0, 0.32)
            .add(48, 2700, 31.0, 0.48)
            .add(72, 3200, 36.0, 0.64)
            .add(96, 3700, 40.0, 0.82)
            .build();

    /** Shooter 6 in behind centre, firing forward. */
    static final AimLogic.Config CONFIG = new AimLogic.Config(
            new Translation2d(-6.0, 0.0),   // robotToShooter
            0.0,                             // fires straight ahead
            0.0,                             // no lookahead, so the sim is exact
            10,                              // max virtual-goal passes
            0.005,                           // converge the shot distance to 0.005 in
            8.0,                             // goal radius, inches
            Math.toRadians(25.0),            // tolerance ceiling
            6.0, 120.0);                     // shot distance range

    static final Translation2d GOAL = new Translation2d(60.0, 40.0);

    /**
     * Flies the piece and returns where it lands.
     *
     * The piece leaves the shooter along the aim direction, fast enough to cover
     * the aimed distance within the flight time, and carries the robot's velocity
     * with it.
     */
    static Translation2d simulateLanding(AimSolution s, Translation2d fieldVelocity,
                                         double aimHeadingRad) {
        double tof = s.setpoint.timeOfFlightSeconds;
        double speed = s.effectiveDistanceInches / tof;           // in/sec along the aim
        Translation2d launch = new Translation2d(
                Math.cos(aimHeadingRad) * speed, Math.sin(aimHeadingRad) * speed);
        Translation2d total = launch.plus(fieldVelocity);          // piece inherits robot motion
        return s.shooterPosition.plus(total.times(tof));
    }

    /** Where a naive "point straight at the goal" shot would land. */
    static Translation2d simulateNaive(Pose2d pose, Translation2d fieldVelocity) {
        Translation2d shooterPos = pose.getTranslation()
                .plus(CONFIG.robotToShooter.rotateBy(pose.getRotation()));
        double dist = GOAL.getDistance(shooterPos);
        double tof = MAP.timeOfFlightAt(dist);
        Translation2d toGoal = GOAL.minus(shooterPos);
        double angle = Math.atan2(toGoal.getY(), toGoal.getX());
        Translation2d launch = new Translation2d(
                Math.cos(angle) * dist / tof, Math.sin(angle) * dist / tof);
        return shooterPos.plus(launch.plus(fieldVelocity).times(tof));
    }

    public static void main(String[] args) {
        System.out.println("=== Shoot-on-the-move: does the piece land in the goal? ===");
        System.out.printf("   goal at (%.0f, %.0f), shooter %.0f in behind robot centre%n%n",
                GOAL.getX(), GOAL.getY(), -CONFIG.robotToShooter.getX());

        // Sweep a range of robot positions, headings and velocities. For each,
        // aim with the solution and fly the piece.
        double worstMiss = 0;
        double worstNaiveMiss = 0;
        int cases = 0;
        for (double rx : new double[]{0, -30, 20}) {
            for (double ry : new double[]{0, -25, 30}) {
                for (double headingDeg : new double[]{0, 35, -80, 170}) {
                    for (double[] vel : new double[][]{
                            {0, 0}, {40, 0}, {0, 40}, {-30, 25}, {55, -20}}) {
                        Pose2d pose = new Pose2d(rx, ry, Rotation2d.fromDegrees(headingDeg));
                        Translation2d v = new Translation2d(vel[0], vel[1]);

                        AimSolution s = AimLogic.calculate(pose, v, 0.0, GOAL, MAP, CONFIG);
                        if (!s.inRange) continue;
                        cases++;

                        // The robot is assumed to have reached the commanded heading.
                        Translation2d landing = simulateLanding(s, v, s.targetHeadingRadians);
                        worstMiss = Math.max(worstMiss, landing.getDistance(GOAL));
                        worstNaiveMiss = Math.max(worstNaiveMiss,
                                simulateNaive(pose, v).getDistance(GOAL));
                    }
                }
            }
        }
        System.out.printf("   %d in-range cases swept%n", cases);
        System.out.printf("   worst miss WITH moving-shot correction: %.4f in%n", worstMiss);
        System.out.printf("   worst miss aiming straight at the goal: %.2f in%n",
                worstNaiveMiss);
        check("Corrected shots land in the goal", worstMiss < 0.01,
                String.format("worst miss %.3f in", worstMiss));
        check("Naive aiming misses badly (so the correction earns its keep)",
                worstNaiveMiss > 10.0,
                String.format("only %.2f in -- test velocities may be too low",
                        worstNaiveMiss));

        // A standing shot must need no correction at all.
        System.out.println();
        Pose2d still = new Pose2d(0, 0, Rotation2d.fromDegrees(20));
        AimSolution stat = AimLogic.calculate(
                still, new Translation2d(), 0.0, GOAL, MAP, CONFIG);
        double straightAngle = Math.atan2(
                GOAL.getY() - stat.shooterPosition.getY(),
                GOAL.getX() - stat.shooterPosition.getX());
        check("Stationary: aims straight at the goal",
                Math.abs(AimLogic.wrapRadians(stat.targetHeadingRadians - straightAngle)) < 1e-9,
                "heading " + Math.toDegrees(stat.targetHeadingRadians));
        check("Stationary: no heading feedforward",
                Math.abs(stat.headingFeedforwardRadPerSec) < 1e-12,
                "" + stat.headingFeedforwardRadPerSec);
        check("Stationary: effective distance == actual distance",
                Math.abs(stat.effectiveDistanceInches - stat.actualDistanceInches) < 1e-9,
                stat.effectiveDistanceInches + " vs " + stat.actualDistanceInches);

        // Lead direction: strafing one way must aim the other way.
        System.out.println();
        Pose2d atOrigin = new Pose2d(0, 0, new Rotation2d(0));
        Translation2d goalAhead = new Translation2d(60, 0);
        AimSolution movingLeft = AimLogic.calculate(
                atOrigin, new Translation2d(0, 40), 0.0, goalAhead, MAP, CONFIG);
        System.out.printf("   goal dead ahead, strafing LEFT at 40 in/s -> aim %.1f deg%n",
                Math.toDegrees(movingLeft.targetHeadingRadians));
        check("Strafing left aims right of the goal",
                movingLeft.targetHeadingRadians < -1e-3,
                "aim " + Math.toDegrees(movingLeft.targetHeadingRadians) + " deg");
        AimSolution movingRight = AimLogic.calculate(
                atOrigin, new Translation2d(0, -40), 0.0, goalAhead, MAP, CONFIG);
        check("Strafing right aims left of the goal",
                movingRight.targetHeadingRadians > 1e-3,
                "aim " + Math.toDegrees(movingRight.targetHeadingRadians) + " deg");

        // The heading feedforward must equal the true rate the aim is sweeping,
        // which is what makes tracking-while-moving work instead of trailing.
        System.out.println();
        double dt = 1e-4;
        double worstFfError = 0;
        for (double[] vel : new double[][]{{40, 0}, {0, 40}, {-30, 25}, {55, -20}}) {
            Translation2d v = new Translation2d(vel[0], vel[1]);
            Pose2d p0 = new Pose2d(-20, -15, Rotation2d.fromDegrees(30));
            Pose2d p1 = new Pose2d(p0.getX() + v.getX() * dt, p0.getY() + v.getY() * dt,
                    p0.getRotation());
            AimSolution s0 = AimLogic.calculate(p0, v, 0.0, GOAL, MAP, CONFIG);
            AimSolution s1 = AimLogic.calculate(p1, v, 0.0, GOAL, MAP, CONFIG);
            double numeric = AimLogic.wrapRadians(
                    s1.targetHeadingRadians - s0.targetHeadingRadians) / dt;
            worstFfError = Math.max(worstFfError,
                    Math.abs(numeric - s0.headingFeedforwardRadPerSec));
            System.out.printf("   v=(%.0f,%.0f): reported ff %+.4f rad/s, "
                            + "numerical %+.4f rad/s%n",
                    vel[0], vel[1], s0.headingFeedforwardRadPerSec, numeric);
        }
        check("Heading feedforward matches the true sweep rate", worstFfError < 2e-3,
                String.format("worst error %.5f rad/s", worstFfError));

        // The virtual goal must be self-consistent with the flight time it implies.
        System.out.println();
        Translation2d v = new Translation2d(45, -25);
        AimSolution it = AimLogic.calculate(
                new Pose2d(-10, 10, new Rotation2d(0)), v, 0.0, GOAL, MAP, CONFIG);
        Translation2d consistent = GOAL.minus(v.times(it.setpoint.timeOfFlightSeconds));
        check("Virtual-goal iteration converged",
                it.virtualGoal.getDistance(consistent) < 0.05,
                String.format("residual %.4f in", it.virtualGoal.getDistance(consistent)));

        // Tolerance must tighten with distance, or far shots are gated too loosely.
        System.out.println();
        AimSolution near = AimLogic.calculate(new Pose2d(48, 40, new Rotation2d(0)),
                new Translation2d(), 0.0, GOAL, MAP, CONFIG);
        AimSolution far = AimLogic.calculate(new Pose2d(-30, 40, new Rotation2d(0)),
                new Translation2d(), 0.0, GOAL, MAP, CONFIG);
        System.out.printf("   %.0f in -> tol %.1f deg | %.0f in -> tol %.1f deg%n",
                near.effectiveDistanceInches, Math.toDegrees(near.headingToleranceRadians),
                far.effectiveDistanceInches, Math.toDegrees(far.headingToleranceRadians));
        check("Heading tolerance tightens with distance",
                near.headingToleranceRadians > far.headingToleranceRadians,
                "tolerance does not scale");
        check("Tolerance is capped close in",
                near.headingToleranceRadians <= CONFIG.maxHeadingToleranceRadians + 1e-12,
                "cap not applied");

        // Out-of-range shots must be refused rather than silently clamped.
        System.out.println();
        AimSolution tooFar = AimLogic.calculate(
                new Pose2d(-72, -72, new Rotation2d(0)), new Translation2d(), 0.0,
                new Translation2d(72, 72), MAP, CONFIG);
        System.out.printf("   corner-to-corner distance %.0f in (map covers to %.0f)%n",
                tooFar.effectiveDistanceInches, MAP.maxDistance());
        check("A shot beyond the map's data is not in range", !tooFar.inRange,
                "claimed in range at " + tooFar.effectiveDistanceInches + " in");

        // A rotating robot with an off-centre shooter still carries the piece.
        System.out.println();
        AimSolution spinning = AimLogic.calculate(
                new Pose2d(0, 0, new Rotation2d(0)), new Translation2d(), 2.0,
                goalAhead, MAP, CONFIG);
        check("Rotation with an offset shooter shifts the aim",
                Math.abs(spinning.targetHeadingRadians) > 1e-4,
                "omega x r term appears to be missing");
        System.out.printf("   spinning 2 rad/s with a 6 in offset -> aim %.2f deg%n",
                Math.toDegrees(spinning.targetHeadingRadians));

        // The goal is alliance-dependent; flipping it must mirror the aim.
        System.out.println();
        Translation2d redGoal = GOAL;
        Translation2d blueGoal = AllianceFlip.forAlliance(redGoal, Alliance.RED, Alliance.BLUE);
        check("Flipped goal is the rotated goal",
                Math.abs(blueGoal.getX() + redGoal.getX()) < 1e-9
                        && Math.abs(blueGoal.getY() + redGoal.getY()) < 1e-9,
                blueGoal.toString());
        // A rotational flip mirrors the ROBOT as well as the goal. With an
        // off-centre shooter the robot's own pose matters, so mirror all of it:
        // position, heading and velocity.
        Pose2d redPose = new Pose2d(-15, 8, Rotation2d.fromDegrees(25));
        Pose2d bluePose = AllianceFlip.forAlliance(redPose, Alliance.RED, Alliance.BLUE);
        Translation2d redVel = new Translation2d(30, -12);
        Translation2d blueVel = redVel.times(-1.0);
        AimSolution redAim = AimLogic.calculate(redPose, redVel, 0.0, redGoal, MAP, CONFIG);
        AimSolution blueAim = AimLogic.calculate(bluePose, blueVel, 0.0, blueGoal, MAP, CONFIG);
        check("Mirrored situation gives the mirrored aim",
                Math.abs(AimLogic.wrapRadians(
                        blueAim.targetHeadingRadians - redAim.targetHeadingRadians - Math.PI))
                        < 1e-6,
                String.format("red %.2f vs blue %.2f deg",
                        Math.toDegrees(redAim.targetHeadingRadians),
                        Math.toDegrees(blueAim.targetHeadingRadians)));

        // A shooter firing out the back must turn the robot around.
        System.out.println();
        AimLogic.Config backwards = new AimLogic.Config(
                new Translation2d(-6, 0), Math.PI, 0.0, 10, 0.005, 8.0,
                Math.toRadians(25), 6.0, 120.0);
        AimSolution back = AimLogic.calculate(new Pose2d(0, 0, new Rotation2d(0)),
                new Translation2d(), 0.0, goalAhead, MAP, backwards);
        check("Rear-facing shooter points the robot away from the goal",
                Math.abs(Math.abs(back.targetHeadingRadians) - Math.PI) < 1e-9,
                "heading " + Math.toDegrees(back.targetHeadingRadians) + " deg");

        System.out.println(fails == 0
                ? "\nAll aiming checks passed."
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }
}
