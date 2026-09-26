package qa;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.subsystems.Localization;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;

/**
 * Drives the real Localization subsystem against synthetic Limelight frames to
 * exercise the heading-trust gate, the MegaTag1 -> MegaTag2 switchover, and the
 * outlier rejection.
 *
 * This is the logic that protects against a flipped or mis-seeded start pose,
 * so it is worth testing directly rather than reasoning about.
 */
public class HeadingTrustTest {
    static int fails = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-56s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    private final HardwareMap map = new HardwareMap();
    private final Limelight3A camera = new Limelight3A();
    private final GoBildaPinpointDriver pinpoint = new GoBildaPinpointDriver();
    private final Localization localization;

    HeadingTrustTest() {
        map.put(VisionConstants.LIMELIGHT_NAME, camera);
        map.put(VisionConstants.PINPOINT_NAME, pinpoint);
        localization = new Localization(map, true);
    }

    /** Places the simulated robot, as odometry would report it. */
    void setOdometry(double x, double y, double headingDeg) {
        pinpoint.x = x;
        pinpoint.y = y;
        pinpoint.headingRad = Math.toRadians(headingDeg);
    }

    /** Feeds one synthetic frame where both solvers agree on the same pose. */
    void feed(double x, double y, double headingDeg) {
        feed(x, y, headingDeg, x, y);
    }

    /** Feeds a frame with separate MegaTag1 and MegaTag2 positions. */
    void feed(double mt1X, double mt1Y, double mt1HeadingDeg, double mt2X, double mt2Y) {
        LLResult r = new LLResult();
        r.botpose = new Pose3D(mt1X, mt1Y, Math.toRadians(mt1HeadingDeg));
        r.botposeMt2 = new Pose3D(mt2X, mt2Y, Math.toRadians(mt1HeadingDeg));
        camera.nextResult = r;
        localization.update();
    }

    /** Feeds a frame with an explicit average tag distance, in the reported unit. */
    void feedWithDistance(double x, double y, double headingDeg, double avgDistInReportedUnit) {
        LLResult r = new LLResult();
        r.botpose = new Pose3D(x, y, Math.toRadians(headingDeg));
        r.botposeMt2 = new Pose3D(x, y, Math.toRadians(headingDeg));
        r.avgDist = avgDistInReportedUnit;
        camera.nextResult = r;
        localization.update();
    }

    void run(int frames, double x, double y, double headingDeg) {
        for (int i = 0; i < frames; i++) {
            feed(x, y, headingDeg);
        }
    }

    public static void main(String[] args) {
        System.out.println("=== Heading trust gate ===");
        System.out.printf("   PREFER_MEGATAG2=%s  trust after %d frames  "
                        + "tolerance %.0f deg  distrust %.0f deg%n%n",
                VisionConstants.PREFER_MEGATAG2, VisionConstants.HEADING_TRUST_FRAMES,
                Math.toDegrees(VisionConstants.HEADING_TRUST_TOLERANCE),
                Math.toDegrees(VisionConstants.HEADING_DISTRUST_THRESHOLD));

        // ---- 1. A correctly seeded robot earns trust and moves to MegaTag2 ----
        HeadingTrustTest t = new HeadingTrustTest();
        t.setOdometry(24, -12, 90);
        t.localization.setStartingPose(new Pose2d(24, -12, Rotation2d.fromDegrees(90)));
        check("Heading starts unverified", !t.localization.isHeadingTrusted(), "");

        t.run(VisionConstants.HEADING_TRUST_FRAMES + 2, 24, -12, 90);
        check("Agreeing frames earn heading trust", t.localization.isHeadingTrusted(),
                "gap " + t.localization.getHeadingDisagreementDegrees());
        check("Position source switches to MegaTag2",
                "MegaTag2".equals(t.localization.getLastVisionSource()),
                "source = " + t.localization.getLastVisionSource());
        check("Correct seed is not flagged suspect",
                !t.localization.isStartPoseSuspect(), t.localization.getStartPoseCheck());

        // ---- 2. The 180-degree case: wrong alliance / robot placed backwards ----
        System.out.println();
        HeadingTrustTest flipped = new HeadingTrustTest();
        flipped.setOdometry(24, -12, 90);
        // Seeded facing 90, but the camera says the robot is really facing -90.
        flipped.localization.setStartingPose(new Pose2d(24, -12, Rotation2d.fromDegrees(90)));
        flipped.run(VisionConstants.HEADING_TRUST_FRAMES + 2, 24, -12, -90);

        System.out.printf("   seeded 90 deg, vision says -90 -> gap %.0f deg%n",
                flipped.localization.getHeadingDisagreementDegrees());
        check("180-degree seed error is flagged suspect",
                flipped.localization.isStartPoseSuspect(),
                flipped.localization.getStartPoseCheck());
        check("180-degree seed error never earns trust",
                !flipped.localization.isHeadingTrusted(), "trusted despite disagreeing");
        check("Position stays on MegaTag1 while heading is unverified",
                "MegaTag1".equals(flipped.localization.getLastVisionSource()),
                "source = " + flipped.localization.getLastVisionSource()
                        + " -- MegaTag2 would inherit the bad heading");
        System.out.printf("   check reads: %s%n", flipped.localization.getStartPoseCheck());

        // MegaTag1 heading fusion does slowly drag a flipped estimate around on
        // its own -- but at a 0.04 gain it takes seconds, during which the pose
        // and field-centric drive are both wrong. That slow self-heal is a
        // safety net, not the plan; the plan is to catch it at init.
        double gapAfter12 = Math.abs(flipped.localization.getHeadingDisagreementDegrees());
        check("A flipped seed is still badly wrong after 12 frames",
                gapAfter12 > 90, String.format("gap already down to %.0f deg", gapAfter12));

        // ---- 3. Recovery: seedFromVision snaps the pose to what the camera sees ----
        System.out.println();
        boolean reseeded = flipped.localization.seedFromVision();
        check("seedFromVision accepts a fresh fix", reseeded, "refused");
        double headingAfter = flipped.localization.getPose().getRotation().getDegrees();
        System.out.printf("   heading after re-seed: %.0f deg (want -90)%n", headingAfter);
        check("Re-seed recovers the true heading",
                Math.abs(headingAfter - (-90)) < 1.0, "got " + headingAfter);
        // After re-seeding, odometry still reports the old heading, so move the
        // simulated robot to match what we just told it.
        flipped.setOdometry(24, -12, -90);
        flipped.run(VisionConstants.HEADING_TRUST_FRAMES + 2, 24, -12, -90);
        check("Trust is earned again after re-seeding",
                flipped.localization.isHeadingTrusted(), "still unverified");

        // ---- 4. Trust is revoked if vision later disagrees badly ----
        System.out.println();
        // Note: (0, 0) is the Limelight's "no fix" sentinel, so these cases sit
        // somewhere else on the field.
        HeadingTrustTest drift = new HeadingTrustTest();
        drift.setOdometry(10, 10, 0);
        drift.localization.setStartingPose(new Pose2d(10, 10, new Rotation2d(0)));
        drift.run(VisionConstants.HEADING_TRUST_FRAMES + 2, 10, 10, 0);
        check("Trust established before the disagreement",
                drift.localization.isHeadingTrusted(), "");
        drift.feed(10, 10, 120);   // vision suddenly says we are 120 deg out
        check("Gross disagreement revokes trust immediately",
                !drift.localization.isHeadingTrusted(), "still trusted");
        check("Position falls back to MegaTag1 on distrust",
                "MegaTag1".equals(drift.localization.getLastVisionSource()),
                "source = " + drift.localization.getLastVisionSource());

        // ---- 5. Outlier rejection, and its escape hatch ----
        System.out.println();
        HeadingTrustTest jump = new HeadingTrustTest();
        jump.setOdometry(10, 10, 0);
        jump.localization.setStartingPose(new Pose2d(10, 10, new Rotation2d(0)));
        jump.run(VisionConstants.MIN_FRAMES_BEFORE_JUMP_REJECT + 2, 10, 10, 0);

        // One frame teleports across the field: an ambiguous single-tag solve.
        jump.feed(10, 10, 0, 60, 60);
        check("Implausible position jump is rejected",
                !jump.localization.wasLastVisionAccepted(),
                "accepted: " + jump.localization.getLastVisionReject());
        System.out.printf("   reject reason: %s%n", jump.localization.getLastVisionReject());

        // But a sustained disagreement must eventually win, or a wrong estimate
        // could reject every correction forever. Once it gives up, it must stay
        // given up -- releasing one frame in every JUMP_REJECT_LIMIT would take
        // many seconds to converge.
        for (int i = 0; i < VisionConstants.JUMP_REJECT_LIMIT + 1; i++) {
            jump.feed(10, 10, 0, 60, 60);
        }
        check("Sustained disagreement eventually breaks through",
                jump.localization.wasLastVisionAccepted(),
                "estimate would be stuck forever");

        int acceptedInARow = 0;
        for (int i = 0; i < 10; i++) {
            jump.feed(10, 10, 0, 60, 60);
            if (jump.localization.wasLastVisionAccepted()) acceptedInARow++;
        }
        check("Recovery is not throttled once it gives up", acceptedInARow == 10,
                acceptedInARow + "/10 frames accepted");

        double err = Math.hypot(jump.localization.getPose().getX() - 60,
                jump.localization.getPose().getY() - 60);
        System.out.printf("   estimate is now %.1f in from the insisted position%n", err);
        check("Estimate actually converges on the camera", err < 20.0,
                String.format("still %.1f in away", err));

        // ---- 5b. The average tag distance is converted, and a wrong unit caught --
        // getBotposeAvgDist() is a bare double with no unit attached. If the
        // configured unit is wrong, the distance is off by ~39x, the std dev by
        // ~1550x, and the Kalman gain pins to 1.0 -- vision snaps the pose every
        // frame and the distance weighting is gone. So the value is range-checked.
        System.out.println();
        System.out.printf("   BOTPOSE_AVG_DIST_UNIT = %s%n",
                VisionConstants.BOTPOSE_AVG_DIST_UNIT);

        HeadingTrustTest units = new HeadingTrustTest();
        units.setOdometry(10, 10, 0);
        units.localization.setStartingPose(new Pose2d(10, 10, new Rotation2d(0)));
        // A believable frame: about 40 inches, expressed in the reported unit.
        units.feedWithDistance(10, 10, 0,
                VisionConstants.BOTPOSE_AVG_DIST_UNIT.fromInches(40.0));
        System.out.printf("   40 in reported as %.4f %s -> read back %.1f in%n",
                VisionConstants.BOTPOSE_AVG_DIST_UNIT.fromInches(40.0),
                VisionConstants.BOTPOSE_AVG_DIST_UNIT,
                units.localization.getLastAvgTagDistance());
        check("Tag distance is converted into inches",
                Math.abs(units.localization.getLastAvgTagDistance() - 40.0) < 0.01,
                "" + units.localization.getLastAvgTagDistance());
        check("A believable distance is not flagged",
                !units.localization.isAvgTagDistanceImplausible(), "flagged wrongly");

        // Now the misconfiguration: a value that is ~39x too small once converted,
        // which is exactly what metres-read-as-inches looks like.
        units.feedWithDistance(10, 10, 0,
                VisionConstants.BOTPOSE_AVG_DIST_UNIT.fromInches(40.0) / 39.37);
        check("A distance ~39x too small is flagged",
                units.localization.isAvgTagDistanceImplausible(),
                "a wrong unit would pass unnoticed");

        // And the other direction.
        units.feedWithDistance(10, 10, 0,
                VisionConstants.BOTPOSE_AVG_DIST_UNIT.fromInches(40.0) * 39.37);
        check("A distance ~39x too large is flagged",
                units.localization.isAvgTagDistanceImplausible(),
                "a wrong unit would pass unnoticed");

        // ---- 6. The gyro heading really is pushed to the camera for MegaTag2 ----
        System.out.println();
        check("Robot yaw is pushed to the camera each loop",
                !Double.isNaN(t.camera.lastYawPushed),
                "updateRobotOrientation never called -- MegaTag2 cannot work");

        System.out.println(fails == 0
                ? "\nAll heading-trust checks passed."
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }
}
