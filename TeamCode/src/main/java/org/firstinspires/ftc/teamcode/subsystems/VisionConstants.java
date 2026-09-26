/*
 * Central tuning + configuration constants for the localization stack.
 *
 * Everything a team needs to adapt this template to their robot lives here:
 * hardware-map names, the Pinpoint mounting geometry, the Limelight pipeline,
 * and the standard deviations that tune the Kalman fusion.
 *
 * Distances are in inches and angles in radians unless noted otherwise.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.geometry.Transform3d;

public final class VisionConstants {
    private VisionConstants() {}

    // ---------------------------------------------------------------------
    // Master switch for vision. When false, Localization never initializes the
    // Limelight and runs on the Pinpoint alone (odometry-only). Use this to run
    // without a Limelight plugged in, or to A/B test fusion vs. dead reckoning.
    // Can also be toggled at runtime via Localization.setVisionEnabled(...).
    // ---------------------------------------------------------------------
    public static final boolean VISION_ENABLED = true;

    // ---------------------------------------------------------------------
    // Hardware-map device names. These must match your robot configuration.
    // ---------------------------------------------------------------------
    public static final String PINPOINT_NAME = "pinpoint";
    public static final String LIMELIGHT_NAME = "limelight";

    // ---------------------------------------------------------------------
    // goBILDA Pinpoint configuration.
    // ---------------------------------------------------------------------
    /**
     * Odometry pod offsets from the robot's tracking point.
     *
     * X offset: how far sideways the X (forward) pod is. Left of center is
     * positive, right is negative.
     * Y offset: how far forward the Y (strafe) pod is. Forward of center is
     * positive, backward is negative.
     *
     * The sample defaults below are for goBILDA's reference build; measure and
     * replace them for your robot.
     */
    public static final double PINPOINT_X_OFFSET = -3.125;
    public static final double PINPOINT_Y_OFFSET = 3.375;
    /**
     * The unit the two offsets above are written in. Change this if you measured
     * in millimetres -- the numbers and this constant must agree.
     */
    public static final DistanceUnit PINPOINT_OFFSET_UNIT = DistanceUnit.INCH;

    /** Pod type. Use goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD for goBILDA pods. */
    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /** Direction each pod counts. Flip these if a pod reads backwards. */
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // ---------------------------------------------------------------------
    // Limelight 3A configuration.
    // ---------------------------------------------------------------------
    /** AprilTag/localization pipeline index configured in the Limelight UI. */
    public static final int LIMELIGHT_PIPELINE = 0;
    /** How often (Hz) to poll the Limelight for new results. */
    public static final int LIMELIGHT_POLL_RATE_HZ = 100;

    // ---------------------------------------------------------------------
    // Camera mounting offset (the Limelight is rarely at the robot's center).
    //
    // There are two ways to account for the offset; pick ONE:
    //
    //   (A) RECOMMENDED — Enter the camera pose in the Limelight web UI
    //       (the camera/robot offset fields). Then the botpose this code reads
    //       is already the robot-center pose, and you should leave
    //       APPLY_CAMERA_OFFSET_IN_CODE = false.
    //
    //   (B) Enter ZERO offset in the Limelight UI and set the offset here in
    //       code instead. botpose is then the camera's field pose, and we
    //       convert it to the robot-center pose with ROBOT_TO_CAMERA below.
    //       Set APPLY_CAMERA_OFFSET_IN_CODE = true.
    //
    // Do NOT do both, or the offset is applied twice.
    //
    // Option (B) now handles the FULL 3D mount: forward/left/up position and
    // roll/pitch/yaw orientation. The Limelight's botpose is a 3D pose, so we
    // recover the robot-center pose with a 3D (SE(3)) transform and then project
    // to 2D for fusion. Option (A) is still the most accurate because MegaTag2
    // also uses the offset inside its own tag solve; (B) assumes botpose carries
    // the true 3D camera pose (best with a level robot / MegaTag1).
    // ---------------------------------------------------------------------
    public static final boolean APPLY_CAMERA_OFFSET_IN_CODE = false;

    // MEASURE THESE ON YOUR OWN ROBOT. They are only read when
    // APPLY_CAMERA_OFFSET_IN_CODE is true, and they ship as zero on purpose:
    // a wrong mount model corrupts every vision pose, which is worse than
    // having no mount model at all.

    /** Camera position relative to robot center (inches): +X fwd, +Y left, +Z up. */
    public static final double CAMERA_FORWARD_OFFSET_IN = 0.0;
    public static final double CAMERA_LEFT_OFFSET_IN = 0.0;
    public static final double CAMERA_UP_OFFSET_IN = 0.0;

    /**
     * Camera orientation relative to robot forward (degrees), right-handed
     * about +X fwd / +Y left / +Z up.
     *
     * MIND THE PITCH SIGN. Pitch rotates about +Y (left), so by the right-hand
     * rule a POSITIVE pitch tilts the camera DOWN and a NEGATIVE pitch tilts it
     * UP. A camera angled 15 degrees upward to see tags is -15.0, not +15.0.
     * Getting this backwards doubles the error instead of removing it.
     *
     * Yaw is CCW-positive: a camera rotated 15 degrees to the robot's left
     * is +15.0.
     */
    public static final double CAMERA_ROLL_OFFSET_DEG = 0.0;
    public static final double CAMERA_PITCH_OFFSET_DEG = 0.0;   // negative = tilted up
    public static final double CAMERA_YAW_OFFSET_DEG = 0.0;

    /**
     * Full 3D transform from the robot-center frame to the camera frame.
     * Composing the robot pose with this yields the camera's field pose; the
     * inverse converts a measured camera field pose back to the robot-center pose.
     */
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            CAMERA_FORWARD_OFFSET_IN, CAMERA_LEFT_OFFSET_IN, CAMERA_UP_OFFSET_IN,
            Math.toRadians(CAMERA_ROLL_OFFSET_DEG),
            Math.toRadians(CAMERA_PITCH_OFFSET_DEG),
            Math.toRadians(CAMERA_YAW_OFFSET_DEG));

    // ---------------------------------------------------------------------
    // Kalman fusion tuning (std devs: {x in, y in, heading rad}).
    // ---------------------------------------------------------------------
    /**
     * Trust in the Pinpoint odometry between vision updates. Smaller numbers =
     * trust odometry more (vision nudges the pose more slowly). The Pinpoint is
     * accurate, so these are small.
     */
    public static final double[] ODOMETRY_STD_DEVS = {0.5, 0.5, Math.toRadians(2.0)};

    /**
     * Fallback vision std devs, used before the first measurement. Per-frame
     * std devs are normally computed dynamically (see below).
     */
    public static final double[] DEFAULT_VISION_STD_DEVS = {2.0, 2.0, Math.toRadians(30.0)};

    /**
     * The unit {@code LLResult.getBotposeAvgDist()} reports in.
     *
     * THIS ONE IS WORTH CHECKING ON YOUR SDK VERSION. Unlike the botpose itself --
     * which arrives as a Position carrying its own unit, so the code converts it
     * safely no matter what -- the average tag distance is a bare double with no
     * unit attached. The Limelight reports `botpose_avgdist` in METRES natively
     * (its own documentation says so), and the FTC SDK most likely passes that
     * straight through, so METER is the default here.
     *
     * Getting it wrong is not subtle, but it is silent. Treating metres as inches
     * shrinks every distance by 39x, and since the std dev goes as distance
     * SQUARED that shrinks it ~1550x -- which drives the Kalman gain to 0.99 at
     * every range. Distance weighting disappears, close and far frames are trusted
     * equally, and a single noisy tag yanks the pose across the field.
     *
     * You do not have to take this on faith: {@link Localization#getLastAvgTagDistance()}
     * is printed on telemetry in inches. Stand a measured distance from a tag and
     * compare. If it reads about 39x too small, this should be INCH; about 39x too
     * large, METER. The subsystem also range-checks it and says so on telemetry.
     */
    public static final DistanceUnit BOTPOSE_AVG_DIST_UNIT = DistanceUnit.METER;

    /**
     * Range a believable average tag distance falls in, inches. An FTC field's
     * corner-to-corner span is about 204 inches, and a tag closer than a few
     * inches cannot produce a usable fix, so anything outside this points at
     * {@link #BOTPOSE_AVG_DIST_UNIT} being wrong. Outside it, the value is clamped
     * for the trust calculation -- so a misconfiguration degrades the weighting
     * instead of destroying it -- and flagged on telemetry.
     */
    public static final double MIN_PLAUSIBLE_TAG_DISTANCE_IN = 4.0;
    public static final double MAX_PLAUSIBLE_TAG_DISTANCE_IN = 250.0;

    /**
     * Base XY std dev coefficient used to scale per-frame vision trust:
     *
     *     xyStdDev = COEFFICIENT * avgTagDistance^2 / tagCount
     *
     * Farther tags and fewer tags give a larger std dev and so less trust.
     *
     * UNITS MATTER ENORMOUSLY HERE. AdvantageKit's published value is 0.02,
     * but its distances are in METRES. This template works in INCHES, and the
     * distance term is SQUARED, so the coefficient has to be converted rather
     * than copied across:
     *
     *     0.02 m per m^2  ->  0.02 / 39.37  ~=  0.0005 in per in^2
     *
     * Using 0.02 (or worse, 2.0) with inches produces std devs of hundreds or
     * thousands of inches, which drives the Kalman gain to nearly zero and
     * silently turns vision fusion off altogether -- the pose still looks
     * plausible because odometry is carrying it, so the failure is easy to miss.
     *
     * For a sanity check, this value should put a typical frame's std dev in
     * the range of a fraction of an inch up close to a handful of inches across
     * the field. Raise it to trust vision less, lower it to trust vision more.
     */
    public static final double VISION_XY_STD_DEV_COEFFICIENT = 0.0005;

    /**
     * Heading std dev (radians) for vision (MegaTag1 heading). Heading is fused
     * with the gyro but heavily biased toward it: the Kalman gain is
     *   q / (q + sqrt(q*r)),  q = ODOMETRY heading var, r = this^2.
     * With odometry heading std = 2 deg and this = 45 deg, the gain is ~0.04, so
     * the gyro dominates short-term and vision only slowly corrects drift.
     * Increase to trust the gyro even more; decrease to let vision pull harder.
     */
    public static final double VISION_HEADING_STD_DEV = Math.toRadians(45.0);

    // ---------------------------------------------------------------------
    // MegaTag1 vs MegaTag2, and the heading-trust gate.
    //
    // MegaTag1 (getBotpose) solves the robot pose from tag geometry alone. It
    // is independent of the gyro, which makes it the ONLY thing that can check
    // whether the gyro heading is right. Its weakness is pose ambiguity: a
    // single tag viewed near head-on has two mathematically valid solutions
    // that are mirror images of each other, and the solver can pick the wrong
    // one.
    //
    // MegaTag2 (getBotpose_MT2) feeds the yaw we push down via
    // updateRobotOrientation() into the solve, which removes the ambiguous
    // second solution entirely. It is far steadier, especially on one tag and
    // at distance. Its weakness is the mirror image of MegaTag1's: the heading
    // it reports is just our own yaw handed back, so it can never correct
    // heading drift -- and if our heading is wrong, MegaTag2 returns a
    // CONFIDENTLY wrong position.
    //
    // So we use both: heading always comes from MegaTag1, and position comes
    // from MegaTag2 only once MegaTag1 has agreed with our heading for a while.
    // Until then we fall back to MegaTag1 for position too, because a bad
    // heading poisons MegaTag2 but cannot poison MegaTag1.
    // ---------------------------------------------------------------------

    /**
     * Use MegaTag2 for X/Y once the heading is trusted. Set false to stay on
     * MegaTag1 everywhere (useful for A/B testing, or if your field map or
     * pipeline does not support MegaTag2).
     */
    public static final boolean PREFER_MEGATAG2 = true;

    /**
     * How many consecutive frames MegaTag1 must agree with our heading before
     * the heading is considered trustworthy and MegaTag2 takes over position.
     * At ~30 accepted frames/sec this is well under a second.
     */
    public static final int HEADING_TRUST_FRAMES = 10;

    /** MegaTag1-vs-estimate heading gap that still counts as agreement. */
    public static final double HEADING_TRUST_TOLERANCE = Math.toRadians(20.0);

    /**
     * MegaTag1-vs-estimate heading gap that revokes trust outright and drops
     * position back to MegaTag1. Set comfortably below 180 so a flipped seed
     * trips it immediately.
     */
    public static final double HEADING_DISTRUST_THRESHOLD = Math.toRadians(60.0);

    /**
     * How fresh (seconds) the last vision fix must be for
     * {@link Localization#seedFromVision()} to act on it. Stops a stale fix from
     * being snapped to long after the robot has driven away from it.
     */
    public static final double SEED_FROM_VISION_MAX_AGE_S = 0.5;

    /**
     * Heading disagreement that makes the pre-match check shout. A 180-degree
     * seed error (wrong alliance, robot placed backwards) lands far past this.
     */
    public static final double HEADING_SEED_WARN_THRESHOLD = Math.toRadians(45.0);

    // ---------------------------------------------------------------------
    // Velocity estimation.
    //
    // Shoot-on-the-move needs to know how fast the robot is travelling. Velocity
    // is differentiated from the ODOMETRY pose, not the fused pose: a vision
    // correction moves the fused pose in a step, and differentiating a step
    // produces a huge false velocity spike that would throw the shot. Odometry is
    // smooth and locally accurate, which is exactly what a derivative needs.
    // ---------------------------------------------------------------------

    /**
     * Low-pass weight for the new velocity sample each loop, 0..1. Lower is
     * smoother but lags more. Differentiating encoder counts is inherently
     * noisy, so some filtering is needed; 0.3 is a reasonable start.
     */
    public static final double VELOCITY_FILTER_ALPHA = 0.3;

    /**
     * Ignore velocity samples taken over a longer gap than this (seconds). A
     * stalled loop would otherwise produce a meaningless derivative.
     */
    public static final double MAX_VELOCITY_DT_SECONDS = 0.25;

    // ---------------------------------------------------------------------
    // Outlier rejection.
    // ---------------------------------------------------------------------
    /**
     * Reject a vision frame that lands more than this far (inches) from the
     * current estimate. This is the last line of defence against an ambiguous
     * single-tag solve teleporting the robot across the field.
     *
     * Only applied once the estimate has settled (see MIN_FRAMES_BEFORE_JUMP_
     * REJECT), because at startup a badly seeded estimate is the wrong one and
     * vision is right.
     */
    public static final double MAX_POSE_JUMP_IN = 36.0;

    /** Accepted frames required before jump rejection starts applying. */
    public static final int MIN_FRAMES_BEFORE_JUMP_REJECT = 10;

    /**
     * If this many frames in a row are rejected as jumps, accept the next one
     * anyway. Without this escape hatch a genuinely wrong estimate could
     * reject every correction forever and never recover.
     */
    public static final int JUMP_REJECT_LIMIT = 25;

    // ---------------------------------------------------------------------
    // Vision measurement rejection filters.
    // ---------------------------------------------------------------------
    /** Reject frames using fewer than this many tags. */
    public static final int MIN_TAG_COUNT = 1;
    /** Reject results older than this many milliseconds. */
    public static final long MAX_STALENESS_MS = 200;

    /**
     * Field bounds check. The FTC field is 144" x 144". Limelight botpose uses
     * the field map you upload; the FTC/FRC convention places the origin at the
     * field center, so valid coordinates span roughly [-72, 72] inches. A frame
     * landing well outside the field (plus a margin) is rejected as garbage.
     */
    public static final double FIELD_HALF_SIZE_IN = 72.0;
    public static final double FIELD_MARGIN_IN = 12.0;
}
