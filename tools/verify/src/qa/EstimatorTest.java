package qa;

import org.firstinspires.ftc.teamcode.lib.estimator.PoseEstimator;
import org.firstinspires.ftc.teamcode.lib.geometry.*;
import org.firstinspires.ftc.teamcode.pathing.*;

public class EstimatorTest {
    static int fails = 0;
    static void check(String n, boolean ok, String d) {
        System.out.printf("%-56s %s%s%n", n, ok ? "PASS" : "**FAIL**", ok ? "" : "   " + d);
        if (!ok) fails++;
    }
    static boolean near(double a, double b, double tol) { return Math.abs(a - b) <= tol; }
    static double wrap(double r) { return Math.atan2(Math.sin(r), Math.cos(r)); }

    /** The gain PoseEstimator derives from odometry std q and vision std r. */
    static double kalmanGain(double odoStd, double visionStd) {
        double q = odoStd * odoStd, r = visionStd * visionStd;
        return q / (q + Math.sqrt(q * r));
    }

    static final double VISION_XY_COEFF =
            org.firstinspires.ftc.teamcode.subsystems.VisionConstants
                    .VISION_XY_STD_DEV_COEFFICIENT;

    // Mirrors VisionConstants (which can't be loaded here - it imports the SDK).
    static final double[] ODO_STD = {0.5, 0.5, Math.toRadians(2.0)};
    static final double[] VIS_STD = {2.0, 2.0, Math.toRadians(30.0)};
    static final double VISION_HEADING_STD = Math.toRadians(45.0);

    public static void main(String[] a) {
        System.out.println("=== PoseEstimator ===");
        PoseEstimator pe = new PoseEstimator(ODO_STD, VIS_STD);
        pe.resetPose(new Pose2d(0, 0, new Rotation2d(0)));

        // Odometry drives straight along +X, but is drifting 10 in short.
        double t = 0;
        for (int i = 1; i <= 50; i++) { t = i * 0.02; pe.updateWithTime(t, new Pose2d(i * 0.2, 0, new Rotation2d(0))); }
        Pose2d beforeVision = pe.getEstimatedPosition();
        check("Pure odometry passes through untouched",
                near(beforeVision.getX(), 10.0, 1e-9), beforeVision.toString());

        // One vision frame says we are really at x=20 (10 in of drift).
        double[] vstd = {2.0, 2.0, VISION_HEADING_STD};
        pe.addVisionMeasurement(new Pose2d(20, 0, new Rotation2d(0)), t, vstd);
        Pose2d afterOne = pe.getEstimatedPosition();
        double gainXY = (afterOne.getX() - 10.0) / 10.0;
        System.out.printf("   one frame moved the estimate %.2f in of the 10 in error "
                + "(effective XY gain %.3f)%n", afterOne.getX() - 10.0, gainXY);
        check("Single vision frame nudges, does not snap",
                gainXY > 0.05 && gainXY < 0.5, "gain " + gainXY);

        // Repeated consistent frames should converge toward the vision pose.
        for (int i = 51; i <= 300; i++) {
            t = i * 0.02;
            pe.updateWithTime(t, new Pose2d(i * 0.2, 0, new Rotation2d(0)));
            pe.addVisionMeasurement(new Pose2d(i * 0.2 + 10, 0, new Rotation2d(0)), t, vstd);
        }
        double residual = pe.getEstimatedPosition().getX() - (300 * 0.2 + 10);
        System.out.printf("   after 250 consistent frames residual = %.3f in%n", residual);
        check("Converges to a consistent vision pose", Math.abs(residual) < 0.5,
                "residual " + residual);

        // Heading gain must be small (gyro-dominant) per the documented design.
        PoseEstimator ph = new PoseEstimator(ODO_STD, VIS_STD);
        ph.resetPose(new Pose2d());
        double t2 = 0;
        for (int i = 1; i <= 50; i++) { t2 = i * 0.02; ph.updateWithTime(t2, new Pose2d(0, 0, new Rotation2d(0))); }
        ph.addVisionMeasurement(new Pose2d(0, 0, Rotation2d.fromDegrees(20)), t2, vstd);
        double hGain = ph.getEstimatedPosition().getRotation().getDegrees() / 20.0;
        System.out.printf("   heading Kalman gain = %.4f (docs claim ~0.04)%n", hGain);
        check("Heading is gyro-dominant as documented", hGain > 0.01 && hGain < 0.10,
                "gain " + hGain);

        // Latency compensation: a delayed frame must not rewrite current odometry motion.
        PoseEstimator pl = new PoseEstimator(ODO_STD, VIS_STD);
        pl.resetPose(new Pose2d());
        for (int i = 1; i <= 100; i++) pl.updateWithTime(i * 0.02, new Pose2d(i * 0.5, 0, new Rotation2d(0)));
        double before = pl.getEstimatedPosition().getX();          // 50.0
        pl.addVisionMeasurement(new Pose2d(37.5, 0, new Rotation2d(0)), 1.5, vstd); // odometry at t=1.5 was x=37.5
        double after = pl.getEstimatedPosition().getX();
        System.out.printf("   agreeing frame delayed 0.5s: %.2f -> %.2f in%n", before, after);
        check("Delayed but agreeing frame leaves the estimate alone",
                near(after, before, 0.05), before + " -> " + after);

        // A frame older than the 1.5 s buffer must be dropped, not applied.
        PoseEstimator pd = new PoseEstimator(ODO_STD, VIS_STD);
        pd.resetPose(new Pose2d());
        for (int i = 1; i <= 200; i++) pd.updateWithTime(i * 0.02, new Pose2d(i * 0.5, 0, new Rotation2d(0)));
        double b2 = pd.getEstimatedPosition().getX();
        pd.addVisionMeasurement(new Pose2d(-500, 0, new Rotation2d(0)), 0.1, vstd);
        check("Frame older than the buffer is rejected",
                near(pd.getEstimatedPosition().getX(), b2, 1e-9),
                b2 + " -> " + pd.getEstimatedPosition().getX());

        // ---- The std devs the PIPELINE actually produces ----
        // The checks above feed hand-picked std devs, which cannot catch a
        // mis-scaled VISION_XY_STD_DEV_COEFFICIENT. This one uses the real
        // formula, because a coefficient in the wrong units silently reduces
        // the gain to zero and switches vision fusion off without any
        // outward sign -- odometry keeps the pose looking plausible.
        System.out.println("\n=== Vision std devs from the real formula ===");
        System.out.printf("   VISION_XY_STD_DEV_COEFFICIENT = %s%n",
                VISION_XY_COEFF);
        boolean gainsSane = true;
        for (double[] cfg : new double[][]{{18, 2}, {40, 1}, {60, 2}, {100, 1}}) {
            double dist = cfg[0];
            int tags = (int) cfg[1];
            double xyStd = VISION_XY_COEFF * dist * dist / tags;
            double g = kalmanGain(ODO_STD[0], xyStd);
            System.out.printf("   %3.0f in, %d tag(s) -> std %7.2f in, gain %.3f%n",
                    dist, tags, xyStd, g);
            if (g < 0.02 || g > 0.95) gainsSane = false;
        }
        check("Real-formula gains stay in a usable range", gainsSane,
                "a frame either does nothing or snaps the pose - check the "
                        + "coefficient's units (inches, not metres)");

        // Close multi-tag must be trusted more than far single-tag.
        double closeGain = kalmanGain(ODO_STD[0], VISION_XY_COEFF * 18 * 18 / 2);
        double farGain = kalmanGain(ODO_STD[0], VISION_XY_COEFF * 100 * 100 / 1);
        check("Close multi-tag outranks far single-tag", closeGain > farGain * 3,
                String.format("close %.3f vs far %.3f", closeGain, farGain));

        System.out.println("\n=== AllianceFlip / FieldSymmetry ===");
        Pose2d red = new Pose2d(-58, -58, Rotation2d.fromDegrees(45));
        Pose2d blue = AllianceFlip.forAlliance(red, Alliance.RED, Alliance.BLUE);
        System.out.printf("   red %s  ->  blue (%.1f, %.1f, %.0fdeg)%n", red,
                blue.getX(), blue.getY(), blue.getRotation().getDegrees());
        check("ROTATIONAL flip negates x and y",
                near(blue.getX(), 58, 1e-9) && near(blue.getY(), 58, 1e-9), blue.toString());
        check("ROTATIONAL flip rotates heading 180",
                near(blue.getRotation().getDegrees(), -135, 1e-9), blue.toString());
        Pose2d back = AllianceFlip.flip(blue);
        check("Flip is an involution (flip twice == identity)",
                near(back.getX(), red.getX(), 1e-9) && near(back.getY(), red.getY(), 1e-9)
                        && near(back.getRotation().getDegrees(), 45, 1e-9), back.toString());
        check("No flip when running the authored alliance",
                AllianceFlip.forAlliance(red, Alliance.RED, Alliance.RED) == red, "copied");

        // A flipped TANGENT path must have headings consistent with its geometry.
        Path p = new Path(new BezierCurve(new Translation2d(0, 0), new Translation2d(24, 0),
                new Translation2d(24, 24))).setTangentHeading();
        Path fp = AllianceFlip.flip(p);
        boolean tangentOk = true;
        for (double tt = 0; tt <= 1.0; tt += 0.1) {
            double expect = wrap(p.getHeading(tt) + Math.PI);
            if (!near(wrap(fp.getHeading(tt) - expect), 0, 1e-9)) tangentOk = false;
        }
        check("Flipped TANGENT heading matches flipped geometry", tangentOk, "mismatch");

        Path lp = new Path(new BezierCurve(new Translation2d(0, 0), new Translation2d(10, 10)))
                .setLinearHeading(Math.toRadians(0), Math.toRadians(90));
        Path flp = AllianceFlip.flip(lp);
        check("Flipped LINEAR heading endpoints rotate 180",
                near(wrap(flp.getHeading(0) - Math.PI), 0, 1e-9)
                        && near(wrap(flp.getHeading(1.0) - Math.toRadians(-90)), 0, 1e-9),
                Math.toDegrees(flp.getHeading(0)) + " / " + Math.toDegrees(flp.getHeading(1)));

        System.out.println(fails == 0 ? "\nAll estimator/flip checks passed."
                : "\n" + fails + " FAILURE(S)");
    }
}
