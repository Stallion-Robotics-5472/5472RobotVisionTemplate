package qa;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.RobotClock;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.shooting.AimLogic;
import org.firstinspires.ftc.teamcode.shooting.AimSolution;
import org.firstinspires.ftc.teamcode.shooting.Goal;
import org.firstinspires.ftc.teamcode.shooting.GoalSelector;
import org.firstinspires.ftc.teamcode.shooting.GoalTargetSource;
import org.firstinspires.ftc.teamcode.shooting.ShooterMap;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;
import org.firstinspires.ftc.teamcode.shooting.SwitchableTargetSource;
import org.firstinspires.ftc.teamcode.shooting.TargetSource;
import org.firstinspires.ftc.teamcode.shooting.VisionTargetSource;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;

/**
 * Checks aiming at what the camera sees, rather than at a coordinate from a table.
 *
 * The claim that has to hold, and that most of this file is about: this mode does
 * not depend on the robot's field position OR its heading being right. If that
 * claim is false the mode is worse than the one it complements, because it would be
 * wrong in exactly the situation it was added for -- a robot whose pose has gone
 * bad. So the pose is deliberately displaced and rotated, and the PHYSICAL aim is
 * compared.
 *
 * The rest is the things that silently produce a confident wrong number: the trig,
 * the tx sign, an unmeasured camera height, a mis-signed camera pitch, and a
 * flickering detection swinging a turretless chassis.
 */
public class VisionTargetTest {
    static int fails = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-58s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    static final ShooterMap MAP = ShooterMap.builder()
            .add(12, 1800, 20.0, 0.25)
            .add(120, 3400, 40.0, 0.80)
            .build();

    // The sim's own geometry: camera low, target high, camera angled up.
    static final double CAMERA_HEIGHT = 12.0;
    static final double TARGET_HEIGHT = 44.0;
    static final double CAMERA_ELEVATION_DEG = 18.0;

    static double simTime = 0.0;

    static VisionTargetSource.Config config() {
        return new VisionTargetSource.Config(
                CAMERA_HEIGHT, TARGET_HEIGHT, Math.toRadians(CAMERA_ELEVATION_DEG),
                ShootingConstants.VISION_TARGET_TX_SIGN, MAP, 6.0, 0.5,
                new HashSet<>(Arrays.asList(11, 12, 21)), 4);
    }

    /**
     * The camera frame a tag at this range and bearing would produce, in the
     * camera's own conventions: tx positive RIGHT, ty positive UP from boresight.
     */
    static LLResultTypes.FiducialResult tagAt(int id, double rangeInches,
                                              double bearingRadiansCCW) {
        double elevation = Math.toDegrees(
                Math.atan2(TARGET_HEIGHT - CAMERA_HEIGHT, rangeInches));
        return new LLResultTypes.FiducialResult(
                id,
                -Math.toDegrees(bearingRadiansCCW),
                elevation - CAMERA_ELEVATION_DEG,
                400.0 / (rangeInches * rangeInches));
    }

    static List<LLResultTypes.FiducialResult> frame(LLResultTypes.FiducialResult... tags) {
        return new ArrayList<>(Arrays.asList(tags));
    }

    public static void main(String[] args) {
        RobotClock.setSource(() -> simTime);
        try {
            ranging();
            poseIndependence();
            holding();
            refusals();
            stickiness();
            switching();
            shippedConstants();
        } finally {
            RobotClock.useSystemClock();
        }

        System.out.println(fails == 0
                ? "\nAll vision-target checks passed."
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }

    // -----------------------------------------------------------------
    // The trig, and the sign of tx
    // -----------------------------------------------------------------

    static void ranging() {
        System.out.println("=== Ranging off the frame ===");
        final List<LLResultTypes.FiducialResult> view = new ArrayList<>();
        VisionTargetSource source = new VisionTargetSource(config(), () -> view);

        Pose2d atOrigin = new Pose2d(0, 0, new Rotation2d(0));

        view.clear();
        view.addAll(frame(tagAt(11, 60.0, 0.0)));
        TargetSource.Target target = source.update(
                atOrigin, new Translation2d(), 0.0, Alliance.RED);
        check("A tag dead ahead produces a target", target != null,
                "" + source.getRejectReason());
        System.out.printf("   60 in dead ahead -> range %.2f in, target at (%.2f, %.2f)%n",
                source.getRangeInches(), target.fieldPosition.getX(),
                target.fieldPosition.getY());
        check("Range comes back out of the elevation angle",
                Math.abs(source.getRangeInches() - 60.0) < 0.01,
                String.format("%.3f in", source.getRangeInches()));
        check("A target dead ahead is straight along +X from the robot",
                Math.abs(target.fieldPosition.getX() - 60.0) < 0.01
                        && Math.abs(target.fieldPosition.getY()) < 0.01,
                target.fieldPosition.toString());
        check("A fresh measurement is marked fresh",
                target.measuredThisLoop && target.ageSeconds == 0.0, target.toString());
        check("The target carries the configured shot table and radius",
                target.map == MAP && target.radiusInches == 6.0, target.toString());

        // The whole range sweep, not one point: a trig error that happens to work at
        // one distance is the kind that survives a test.
        double worst = 0.0;
        for (double range = 24.0; range <= 200.0; range += 8.0) {
            view.clear();
            view.addAll(frame(tagAt(11, range, 0.0)));
            source.update(atOrigin, new Translation2d(), 0.0, Alliance.RED);
            worst = Math.max(worst, Math.abs(source.getRangeInches() - range));
        }
        System.out.printf("   worst range error over 24-200 in: %.4f in%n", worst);
        check("Ranging is exact across the whole usable span", worst < 0.01,
                String.format("%.4f in", worst));

        // The sign that cannot be checked offline, checked against its own
        // convention: Limelight's tx is positive RIGHT, and right is -Y here.
        view.clear();
        view.addAll(frame(tagAt(11, 60.0, Math.toRadians(-15.0))));   // 15 deg right
        target = source.update(atOrigin, new Translation2d(), 0.0, Alliance.RED);
        System.out.printf("   a tag 15 deg to the RIGHT lands at (%.1f, %.1f)%n",
                target.fieldPosition.getX(), target.fieldPosition.getY());
        check("A tag to the right of the crosshair is to the robot's right",
                target.fieldPosition.getY() < -10.0,
                "a right-hand target came out on the left: VISION_TARGET_TX_SIGN");
        check("Bearing is reported CCW-positive, so right is negative",
                Math.abs(Math.toDegrees(source.getBearingRadians()) + 15.0) < 0.01,
                String.format("%.2f deg", Math.toDegrees(source.getBearingRadians())));

        // A rotated robot: the same frame must put the target somewhere else.
        Pose2d facingLeft = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
        view.clear();
        view.addAll(frame(tagAt(11, 60.0, 0.0)));
        target = source.update(facingLeft, new Translation2d(), 0.0, Alliance.RED);
        check("The measurement is rotated into the field by the robot's heading",
                Math.abs(target.fieldPosition.getX()) < 0.01
                        && Math.abs(target.fieldPosition.getY() - 60.0) < 0.01,
                target.fieldPosition.toString());
    }

    // -----------------------------------------------------------------
    // The claim: the pose does not matter
    // -----------------------------------------------------------------

    /**
     * The same physical situation, described from two very different (and one very
     * wrong) pose estimates, must produce the same physical aim.
     *
     * "Physical aim" is the heading RELATIVE TO WHERE THE ROBOT IS POINTING, because
     * that is what the robot actually does: the heading controller drives the
     * measured heading to the commanded one, and both are expressed in the same
     * (possibly wrong) frame. An absolute comparison would prove nothing.
     */
    static void poseIndependence() {
        System.out.println("\n=== The pose estimate does not matter ===");
        final List<LLResultTypes.FiducialResult> view = new ArrayList<>();
        VisionTargetSource source = new VisionTargetSource(config(), () -> view);

        // One physical situation: target 70 in away, 12 degrees to the left, robot
        // moving at 40 in/s forwards and 10 in/s to its own left.
        double range = 70.0;
        double bearing = Math.toRadians(12.0);
        Translation2d robotFrameVelocity = new Translation2d(40.0, 10.0);

        double reference = Double.NaN;
        double worstDifference = 0.0;
        Pose2d[] poses = {
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(60, -40, Rotation2d.fromDegrees(35)),
                new Pose2d(-120, 96, Rotation2d.fromDegrees(-170)),
                // The failure this mode exists for: a pose seeded 180 degrees out.
                new Pose2d(24, 24, Rotation2d.fromDegrees(180)),
        };
        for (Pose2d pose : poses) {
            view.clear();
            view.addAll(frame(tagAt(11, range, bearing)));
            // The same physical motion, expressed in this pose's frame.
            Translation2d fieldVelocity = robotFrameVelocity.rotateBy(pose.getRotation());

            TargetSource.Target target =
                    source.update(pose, fieldVelocity, 0.0, Alliance.RED);
            AimSolution solution = AimLogic.calculate(
                    pose, fieldVelocity, 0.0, target.fieldPosition, target.map,
                    ShootingConstants.AIM_CONFIG.withGoalRadius(target.radiusInches));

            double relativeAim = AimLogic.wrapRadians(
                    solution.targetHeadingRadians - pose.getHeading());
            System.out.printf("   pose (%6.1f,%6.1f,%7.1f deg) -> aim %+7.3f deg off the"
                            + " nose, shot %.2f in%n",
                    pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
                    Math.toDegrees(relativeAim), solution.effectiveDistanceInches);
            if (Double.isNaN(reference)) {
                reference = relativeAim;
            } else {
                worstDifference = Math.max(worstDifference,
                        Math.abs(AimLogic.wrapRadians(relativeAim - reference)));
            }
        }
        System.out.printf("   worst disagreement between poses: %.6f deg%n",
                Math.toDegrees(worstDifference));
        check("The physical aim is the same from any pose, right or wrong",
                Math.toDegrees(worstDifference) < 0.01,
                String.format("%.4f deg apart -- the pose is leaking into the answer",
                        Math.toDegrees(worstDifference)));

        // And the distance it spins up for must match too, since that is what the
        // shot table is indexed on.
        view.clear();
        view.addAll(frame(tagAt(11, range, bearing)));
        TargetSource.Target a = source.update(
                new Pose2d(0, 0, new Rotation2d(0)), new Translation2d(), 0.0,
                Alliance.RED);
        double distanceFromGood = AimLogic.effectiveDistanceTo(
                new Pose2d(0, 0, new Rotation2d(0)), new Translation2d(), 0.0,
                a.fieldPosition, a.map, ShootingConstants.AIM_CONFIG);
        view.clear();
        view.addAll(frame(tagAt(11, range, bearing)));
        Pose2d wrong = new Pose2d(24, 24, Rotation2d.fromDegrees(180));
        TargetSource.Target b = source.update(wrong, new Translation2d(), 0.0,
                Alliance.RED);
        double distanceFromWrong = AimLogic.effectiveDistanceTo(
                wrong, new Translation2d(), 0.0, b.fieldPosition, b.map,
                ShootingConstants.AIM_CONFIG);
        System.out.printf("   shot distance: good pose %.3f in, 180-deg-wrong pose"
                + " %.3f in%n", distanceFromGood, distanceFromWrong);
        check("The shot distance is the same from a badly wrong pose",
                Math.abs(distanceFromGood - distanceFromWrong) < 0.01,
                String.format("%.3f vs %.3f", distanceFromGood, distanceFromWrong));

        // Contrast, so the difference between the two modes is on the record: the
        // goal table from the same wrong pose aims somewhere else entirely.
        Goal goal = Goal.named("g", 70, 0).tags(11).map(MAP).build();
        GoalTargetSource table = new GoalTargetSource(
                new GoalSelector(GoalSelector.Strategy.NEAREST, goal).withSwitchFrames(0));
        TargetSource.Target fromTable = table.update(wrong, new Translation2d(), 0.0,
                Alliance.RED);
        AimSolution tableSolution = AimLogic.calculate(
                wrong, new Translation2d(), 0.0, fromTable.fieldPosition, fromTable.map,
                ShootingConstants.AIM_CONFIG);
        double tableRelative = Math.toDegrees(AimLogic.wrapRadians(
                tableSolution.targetHeadingRadians - wrong.getHeading()));
        System.out.printf("   for contrast, the goal table from the same wrong pose"
                + " aims %+.1f deg off the nose%n", tableRelative);
        check("The goal table really is fooled by the same pose",
                Math.abs(tableRelative - Math.toDegrees(reference)) > 10.0,
                "the two modes agreed, so this comparison proves nothing");
    }

    // -----------------------------------------------------------------
    // Losing sight of the target
    // -----------------------------------------------------------------

    static void holding() {
        System.out.println("\n=== Losing the target from frame ===");
        final List<LLResultTypes.FiducialResult> view = new ArrayList<>();
        VisionTargetSource source = new VisionTargetSource(config(), () -> view);

        simTime = 100.0;
        view.clear();
        view.addAll(frame(tagAt(11, 60.0, 0.0)));
        TargetSource.Target measured = source.update(
                new Pose2d(0, 0, new Rotation2d(0)), new Translation2d(), 0.0,
                Alliance.RED);
        Translation2d where = measured.fieldPosition;

        // Camera loses it, and the robot drives on. The held target must stay where
        // it is IN THE FIELD, so the robot keeps aiming at the right place.
        view.clear();
        simTime += 0.1;
        TargetSource.Target held = source.update(
                new Pose2d(20, 5, Rotation2d.fromDegrees(10)), new Translation2d(), 0.0,
                Alliance.RED);
        check("A target lost from frame is held, not dropped", held != null,
                "dropped immediately: " + source.getRejectReason());
        check("The hold is marked as held, not fresh", !held.measuredThisLoop,
                "a stale measurement claimed to be fresh");
        check("The held target stays put in the field as the robot drives",
                held.fieldPosition.getDistance(where) < 1e-9,
                held.fieldPosition + " vs " + where);
        System.out.printf("   after %.2fs: %s%n", held.ageSeconds, held);

        simTime += 0.3;             // 0.4s total, still inside the 0.5s hold
        check("Still held just inside the window",
                source.update(new Pose2d(0, 0, new Rotation2d(0)), new Translation2d(),
                        0.0, Alliance.RED) != null,
                "dropped early");

        simTime += 0.2;             // 0.6s, past it
        check("Dropped once the hold expires",
                source.update(new Pose2d(0, 0, new Rotation2d(0)), new Translation2d(),
                        0.0, Alliance.RED) == null,
                "kept aiming at a measurement over half a second old");
        check("And it says why", source.getRejectReason() != null, "no reason given");
        System.out.printf("   after the hold: %s%n", source.describe());
    }

    // -----------------------------------------------------------------
    // Refusing, rather than producing a confident wrong number
    // -----------------------------------------------------------------

    static void refusals() {
        System.out.println("\n=== Refusing when the geometry cannot answer ===");
        Pose2d origin = new Pose2d(0, 0, new Rotation2d(0));
        final List<LLResultTypes.FiducialResult> view =
                frame(tagAt(11, 60.0, 0.0));

        // Heights left at their shipped zeros. This is the state every robot starts
        // in, and the mode must be inert rather than confidently wrong.
        VisionTargetSource unconfigured = new VisionTargetSource(
                new VisionTargetSource.Config(0.0, 0.0, Math.toRadians(18.0), -1.0,
                        MAP, 6.0, 0.5, Collections.emptySet(), 4),
                () -> view);
        check("Unmeasured camera/target heights produce no target",
                unconfigured.update(origin, new Translation2d(), 0.0, Alliance.RED) == null,
                "ranged off a height difference of zero");
        check("...and the reason names what to set",
                unconfigured.getRejectReason() != null
                        && unconfigured.getRejectReason().contains("height"),
                "" + unconfigured.getRejectReason());
        System.out.printf("   %s%n", unconfigured.getRejectReason());

        // Heights too close together: the tangent divides by almost nothing, so a
        // tenth of a degree of noise swings the answer by feet.
        VisionTargetSource tooClose = new VisionTargetSource(
                new VisionTargetSource.Config(30.0, 33.0, Math.toRadians(18.0), -1.0,
                        MAP, 6.0, 0.5, Collections.emptySet(), 4),
                () -> view);
        check("A height difference too small to range off is refused",
                tooClose.update(origin, new Translation2d(), 0.0, Alliance.RED) == null,
                "ranged off a 3 in height difference");

        // A camera pitch entered with the wrong sign: tilted DOWN at a target ABOVE
        // it. The elevation then has the wrong sign and the "distance" is negative.
        VisionTargetSource wrongPitch = new VisionTargetSource(
                new VisionTargetSource.Config(CAMERA_HEIGHT, TARGET_HEIGHT,
                        Math.toRadians(-40.0), -1.0, MAP, 6.0, 0.5,
                        Collections.emptySet(), 4),
                () -> view);
        check("A mis-signed camera pitch is refused, not ranged",
                wrongPitch.update(origin, new Translation2d(), 0.0, Alliance.RED) == null,
                "produced a target from an impossible elevation");
        check("...and the reason points at the pitch",
                wrongPitch.getRejectReason() != null
                        && wrongPitch.getRejectReason().contains("PITCH"),
                "" + wrongPitch.getRejectReason());
        System.out.printf("   %s%n", wrongPitch.getRejectReason());

        // A tag so nearly level with the camera that the range comes out absurd.
        final List<LLResultTypes.FiducialResult> almostLevel = frame(
                new LLResultTypes.FiducialResult(11, 0.0, -17.98, 1.0));
        VisionTargetSource farOut = new VisionTargetSource(config(), () -> almostLevel);
        check("An implausible range is refused",
                farOut.update(origin, new Translation2d(), 0.0, Alliance.RED) == null,
                "accepted a range off the end of the field");
        System.out.printf("   %s%n", farOut.getRejectReason());

        // An empty frame is not an error, just nothing to shoot at.
        VisionTargetSource blind = new VisionTargetSource(config(),
                Collections::emptyList);
        check("An empty frame means no target, and says so",
                blind.update(origin, new Translation2d(), 0.0, Alliance.RED) == null
                        && blind.getRejectReason() != null,
                "" + blind.getRejectReason());

        // Tags that are not goals must not be ranged off.
        final List<LLResultTypes.FiducialResult> wrongTag = frame(tagAt(99, 60.0, 0.0));
        VisionTargetSource filtered = new VisionTargetSource(config(), () -> wrongTag);
        check("A tag that is not in the target list is ignored",
                filtered.update(origin, new Translation2d(), 0.0, Alliance.RED) == null,
                "ranged off tag 99");
        System.out.printf("   %s%n", filtered.getRejectReason());

        check("This mode does not require a trusted pose",
                !new VisionTargetSource(config(), () -> view).requiresTrustedPose(),
                "gating it on the pose would refuse correct shots");
        check("The goal table does require one",
                new GoalTargetSource(new GoalSelector(GoalSelector.Strategy.NEAREST,
                        Goal.named("g", 70, 0).tags(11).map(MAP).build()))
                        .requiresTrustedPose(),
                "a mis-seeded robot would shoot at empty field");
    }

    // -----------------------------------------------------------------
    // A flickering detection must not swing the chassis
    // -----------------------------------------------------------------

    static void stickiness() {
        System.out.println("\n=== Which tag to track ===");
        final List<LLResultTypes.FiducialResult> view = new ArrayList<>();
        VisionTargetSource source = new VisionTargetSource(config(), () -> view);
        Pose2d origin = new Pose2d(0, 0, new Rotation2d(0));

        // Two goal tags in frame. 21 is nearer, so it has the larger area and wins
        // the first frame.
        view.clear();
        view.addAll(frame(tagAt(11, 90.0, Math.toRadians(20.0)),
                tagAt(21, 40.0, Math.toRadians(-10.0))));
        source.update(origin, new Translation2d(), 0.0, Alliance.RED);
        check("Picks the nearest of several tags in frame",
                source.getTrackedTagId() == 21, "tracked " + source.getTrackedTagId());

        // Now 11 becomes the larger. It must NOT take over on one frame: on a
        // turretless robot that is the whole chassis swinging between two aims.
        view.clear();
        view.addAll(frame(tagAt(11, 30.0, Math.toRadians(20.0)),
                tagAt(21, 80.0, Math.toRadians(-10.0))));
        source.update(origin, new Translation2d(), 0.0, Alliance.RED);
        check("One frame does not move the target",
                source.getTrackedTagId() == 21,
                "switched on a single frame - the robot would oscillate");

        for (int i = 0; i < 3; i++) {
            source.update(origin, new Translation2d(), 0.0, Alliance.RED);
        }
        check("Still held through the required streak",
                source.getTrackedTagId() == 21,
                "switched early at " + source.getTrackedTagId());
        source.update(origin, new Translation2d(), 0.0, Alliance.RED);
        check("Switches once the challenger has earned it",
                source.getTrackedTagId() == 11, "" + source.getTrackedTagId());

        // An intermittent challenger never accumulates a streak.
        VisionTargetSource flaky = new VisionTargetSource(config(), () -> view);
        view.clear();
        view.addAll(frame(tagAt(11, 30.0, 0.0)));
        flaky.update(origin, new Translation2d(), 0.0, Alliance.RED);
        boolean everSwitched = false;
        for (int i = 0; i < 40; i++) {
            view.clear();
            if (i % 2 == 0) {
                view.addAll(frame(tagAt(11, 90.0, 0.0), tagAt(21, 30.0, 0.0)));
            } else {
                view.addAll(frame(tagAt(11, 30.0, 0.0), tagAt(21, 90.0, 0.0)));
            }
            flaky.update(origin, new Translation2d(), 0.0, Alliance.RED);
            if (flaky.getTrackedTagId() != 11) everSwitched = true;
        }
        check("An alternating challenger never takes over", !everSwitched,
                "switched on a flickering detection");

        // When the tracked tag genuinely leaves frame there is nothing to be sticky
        // about: take what is on offer straight away.
        view.clear();
        view.addAll(frame(tagAt(21, 50.0, 0.0)));
        flaky.update(origin, new Translation2d(), 0.0, Alliance.RED);
        check("A tag that has actually gone is replaced at once",
                flaky.getTrackedTagId() == 21, "" + flaky.getTrackedTagId());

        // Frozen means a shot is in progress.
        VisionTargetSource frozen = new VisionTargetSource(config(), () -> view);
        view.clear();
        view.addAll(frame(tagAt(21, 50.0, 0.0), tagAt(11, 90.0, 0.0)));
        frozen.update(origin, new Translation2d(), 0.0, Alliance.RED);
        frozen.freeze(true);
        view.clear();
        view.addAll(frame(tagAt(11, 20.0, 0.0), tagAt(21, 95.0, 0.0)));
        for (int i = 0; i < 30; i++) {
            frozen.update(origin, new Translation2d(), 0.0, Alliance.RED);
        }
        check("Frozen, the target cannot change mid-shot",
                frozen.getTrackedTagId() == 21, "target changed while feeding");
        frozen.freeze(false);
        for (int i = 0; i < 10; i++) {
            frozen.update(origin, new Translation2d(), 0.0, Alliance.RED);
        }
        check("Unfreezing resumes tracking", frozen.getTrackedTagId() == 11,
                "" + frozen.getTrackedTagId());
    }

    // -----------------------------------------------------------------
    // Choosing between the two modes
    // -----------------------------------------------------------------

    static void switching() {
        System.out.println("\n=== Switching between the modes ===");
        final List<LLResultTypes.FiducialResult> view = new ArrayList<>();
        Goal goal = Goal.named("goal", 70, 0).tags(11).map(MAP).build();
        Pose2d origin = new Pose2d(0, 0, new Rotation2d(0));

        SwitchableTargetSource both = new SwitchableTargetSource(
                new GoalTargetSource(new GoalSelector(GoalSelector.Strategy.NEAREST, goal)
                        .withSwitchFrames(0)),
                new VisionTargetSource(config(), () -> view),
                SwitchableTargetSource.Mode.AUTO);

        // Camera can see it: vision answers, and the shot does not rest on the pose.
        view.clear();
        view.addAll(frame(tagAt(11, 55.0, 0.0)));
        TargetSource.Target target = both.update(origin, new Translation2d(), 0.0,
                Alliance.RED);
        System.out.printf("   with a tag in frame: %s%n", both.describe());
        check("AUTO uses the camera when it has a target",
                target != null && target.name.startsWith("tag"), "" + target);
        check("...and is not gated on the pose while it does",
                !both.requiresTrustedPose(),
                "gated a shot that does not depend on the pose");
        check("The camera's range is used, not the table's geometry",
                Math.abs(target.fieldPosition.getX() - 55.0) < 0.01,
                target.fieldPosition.toString());

        // Camera goes blind: the table answers, and the pose gate comes back.
        view.clear();
        // Past the hold, so the vision source has genuinely given up.
        simTime += 5.0;
        target = both.update(origin, new Translation2d(), 0.0, Alliance.RED);
        System.out.printf("   with nothing in frame: %s%n", both.describe());
        check("AUTO falls back to the goal table",
                target != null && target.name.equals("goal"), "" + target);
        check("...and the pose gate comes back with it",
                both.requiresTrustedPose(),
                "a table shot went unguarded, which is the mis-seed failure");

        // Explicit modes.
        both.setMode(SwitchableTargetSource.Mode.VISION_TARGET);
        check("VISION_TARGET refuses rather than falling back",
                both.update(origin, new Translation2d(), 0.0, Alliance.RED) == null,
                "fell back to the table in a camera-only mode");
        both.setMode(SwitchableTargetSource.Mode.GOAL_TABLE);
        view.clear();
        view.addAll(frame(tagAt(11, 55.0, 0.0)));
        target = both.update(origin, new Translation2d(), 0.0, Alliance.RED);
        check("GOAL_TABLE ignores the camera's range",
                target != null && target.name.equals("goal"), "" + target);
        check("Cycling steps through every mode",
                both.cycleMode() == SwitchableTargetSource.Mode.VISION_TARGET
                        && both.cycleMode() == SwitchableTargetSource.Mode.AUTO
                        && both.cycleMode() == SwitchableTargetSource.Mode.GOAL_TABLE,
                "" + both.getMode());

        // A switch mid-shot would move the whole chassis.
        both.freeze(true);
        SwitchableTargetSource.Mode before = both.getMode();
        both.setMode(SwitchableTargetSource.Mode.VISION_TARGET);
        check("A mode switch is refused while a shot is in progress",
                both.getMode() == before, "changed mode mid-shot");
        both.freeze(false);
        check("...and allowed again afterwards",
                both.setMode(SwitchableTargetSource.Mode.VISION_TARGET)
                        == SwitchableTargetSource.Mode.VISION_TARGET,
                "" + both.getMode());
    }

    // -----------------------------------------------------------------
    // The shipped configuration
    // -----------------------------------------------------------------

    static void shippedConstants() {
        System.out.println("\n=== The shipped configuration ===");
        VisionTargetSource.Config shipped = VisionTargetSource.configFromConstants();
        System.out.printf("   camera %.1f in, target %.1f in, elevation %.1f deg,"
                        + " tx sign %+.0f%n",
                shipped.cameraHeightInches, shipped.targetHeightInches,
                Math.toDegrees(shipped.cameraElevationRadians), shipped.txSign);
        System.out.printf("   %d target tag(s), hold %.2fs, switch frames %d%n",
                shipped.tagIds.size(), shipped.holdSeconds, shipped.switchFrames);

        // Ships unusable ON PURPOSE: the heights are the one thing a team must
        // measure, and a plausible-looking default would be a wrong answer nobody
        // checks. This asserts the shipped state is the inert one.
        check("Ships inert until the heights are measured", !shipped.isUsable(),
                "a default height difference would be a wrong answer nobody checks");
        check("The elevation follows CAMERA_PITCH_OFFSET_DEG, negated",
                Math.abs(Math.toDegrees(shipped.cameraElevationRadians)
                        + org.firstinspires.ftc.teamcode.subsystems.VisionConstants
                                .CAMERA_PITCH_OFFSET_DEG) < 1e-9,
                "a positive pitch tilts DOWN, so elevation is its negation");
        check("tx is negated, because Limelight is positive-right and this is CCW",
                shipped.txSign < 0, "" + shipped.txSign);
        check("Target tags come from the tag table",
                shipped.tagIds.equals(ShootingConstants.TAG_FIELD_POSES.keySet()),
                shipped.tagIds.toString());
        check("The hold is long enough to survive a blocked frame or two",
                shipped.holdSeconds >= 0.1 && shipped.holdSeconds <= 2.0,
                "" + shipped.holdSeconds);
        check("Switch frames are non-zero, so a flicker cannot move the chassis",
                shipped.switchFrames > 0, "0 invites oscillation");
    }
}
