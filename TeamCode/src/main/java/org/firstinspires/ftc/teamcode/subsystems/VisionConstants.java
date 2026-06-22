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
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public final class VisionConstants {
    private VisionConstants() {}

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
    public static final double PINPOINT_X_OFFSET_MM = -84.0;
    public static final double PINPOINT_Y_OFFSET_MM = -168.0;
    public static final DistanceUnit PINPOINT_OFFSET_UNIT = DistanceUnit.MM;

    /** Pod type. Use goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD for goBILDA pods. */
    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /** Direction each pod counts. Flip these if a pod reads backwards. */
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
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
    //       (the camera/robot offset fields). Then getBotpose_MT2() is already
    //       the robot-center pose, and you should leave
    //       APPLY_CAMERA_OFFSET_IN_CODE = false.
    //
    //   (B) Enter ZERO offset in the Limelight UI and set the offset here in
    //       code instead. botpose is then the camera's field pose, and we
    //       convert it to the robot-center pose with ROBOT_TO_CAMERA below.
    //       Set APPLY_CAMERA_OFFSET_IN_CODE = true.
    //
    // Do NOT do both, or the offset is applied twice.
    // ---------------------------------------------------------------------
    public static final boolean APPLY_CAMERA_OFFSET_IN_CODE = false;

    /** Camera position relative to robot center: +X = forward, +Y = left (inches). */
    public static final double CAMERA_FORWARD_OFFSET_IN = 6.0;
    public static final double CAMERA_LEFT_OFFSET_IN = 0.0;
    /** Camera yaw relative to robot forward, CCW positive (degrees). */
    public static final double CAMERA_YAW_OFFSET_DEG = 0.0;

    /**
     * Transform from the robot-center frame to the camera frame. Composing the
     * robot pose with this yields the camera pose; the inverse converts a
     * measured camera field pose back to the robot-center pose.
     */
    public static final Transform2d ROBOT_TO_CAMERA = new Transform2d(
            new Translation2d(CAMERA_FORWARD_OFFSET_IN, CAMERA_LEFT_OFFSET_IN),
            Rotation2d.fromDegrees(CAMERA_YAW_OFFSET_DEG));

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
     * Base XY std dev (inches) used to scale per-frame vision trust. The
     * AdvantageKit-style scaling multiplies this by (avgTagDistance^2 /
     * tagCount): farther tags and fewer tags => larger std dev => less trust.
     */
    public static final double VISION_XY_STD_DEV_COEFFICIENT = 2.0;

    /**
     * Heading std dev (radians) for vision. We use MegaTag2, whose heading comes
     * from the gyro we feed in, so vision should NOT move heading. A very large
     * value drives the heading Kalman gain to ~0.
     */
    public static final double VISION_HEADING_STD_DEV = 9999.0;

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
