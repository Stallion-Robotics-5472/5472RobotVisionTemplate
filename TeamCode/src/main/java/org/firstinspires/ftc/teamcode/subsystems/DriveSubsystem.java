/*
 * The drivetrain as a command-system subsystem, with the heading-lock mode that
 * shoot-on-the-move needs.
 *
 * With no turret, aiming means rotating the whole robot -- but the driver still
 * wants to drive. So this splits the two: the driver keeps full translation
 * control while the aiming solution takes over heading. That is what
 * {@link #driveWithHeadingLock} does, and it is the heart of shooting on the move
 * on a turretless robot.
 *
 * The heading controller is a PID on heading error PLUS a feedforward term. The
 * feedforward is what makes it work: while the robot translates past the goal the
 * aim direction keeps sweeping, so a position-only controller permanently trails
 * it and the robot shoots behind the target. Feeding the known sweep rate forward
 * lets the PID handle only the leftover error.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.pathing.Alliance;
import org.firstinspires.ftc.teamcode.pathing.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pathing.PIDFController;
import org.firstinspires.ftc.teamcode.pathing.PathConstants;
import org.firstinspires.ftc.teamcode.shooting.AimLogic;

public class DriveSubsystem extends SubsystemBase {

    /**
     * Ceiling on the heading-lock turn command. Leaving headroom below 1.0 keeps
     * some of the power budget for translation, so locking onto a target does not
     * stop the driver moving.
     */
    private static final double MAX_HEADING_LOCK_TURN = 0.8;

    /**
     * Converts the feedforward (rad/s) into turn power. Set it to
     * 1 / (turn rate at full power, rad/s): if the robot spins about 6 rad/s flat
     * out, this is 1/6 = 0.167. Measure it with the Drivetrain Direction Check
     * OpMode, or tune by eye until a moving lock stops trailing.
     */
    private static final double TURN_POWER_PER_RAD_PER_SEC = 0.167;

    private final MecanumDrivetrain drivetrain;
    private final Localization localization;
    private final PIDFController headingController;

    private double lastTimeSeconds = Double.NaN;

    // Telemetry from the most recent heading-lock call.
    private double lastHeadingTargetRad = 0.0;
    private double lastHeadingErrorRad = 0.0;
    private double lastTurnCommand = 0.0;
    private boolean headingLockActive = false;

    public DriveSubsystem(HardwareMap hardwareMap) {
        this(new MecanumDrivetrain(hardwareMap), new Localization(hardwareMap));
    }

    public DriveSubsystem(MecanumDrivetrain drivetrain, Localization localization) {
        this.drivetrain = drivetrain;
        this.localization = localization;
        this.headingController = new PIDFController(
                PathConstants.HEADING_kP, PathConstants.HEADING_kI,
                PathConstants.HEADING_kD, PathConstants.HEADING_kF);
    }

    /** Runs the pose estimator once per loop, before any command executes. */
    @Override
    public void periodic() {
        localization.update();
    }

    // ---------------------------------------------------------------------
    // Driving
    // ---------------------------------------------------------------------

    /**
     * Ordinary field-centric drive. Inputs are field-frame powers in [-1, 1] and
     * a turn power, CCW positive.
     */
    public void driveFieldCentric(double fieldX, double fieldY, double turn) {
        headingLockActive = false;
        drivetrain.driveFieldCentric(fieldX, fieldY, turn, getHeading());
    }

    /**
     * Field-centric drive from the DRIVER's point of view, which differs by
     * alliance because the two drive teams stand at opposite ends.
     */
    public void driveDriverRelative(double driverForward, double driverLeft, double turn,
                                   Alliance alliance) {
        Translation2d fieldVector = new Translation2d(driverForward, driverLeft)
                .rotateBy(alliance.driverForward());
        driveFieldCentric(fieldVector.getX(), fieldVector.getY(), turn);
    }

    /**
     * Drives with the driver's translation but the heading commanded by the aiming
     * solution. This is the turretless shoot-on-the-move drive mode.
     *
     * @param fieldX field-frame translation power, [-1, 1].
     * @param fieldY field-frame translation power, [-1, 1].
     * @param targetHeadingRad field-relative heading to hold, radians.
     * @param feedforwardRadPerSec rate the target heading is itself sweeping. Pass
     *     0 to hold a fixed heading; pass the aiming solution's feedforward to
     *     track a moving one.
     */
    public void driveWithHeadingLock(double fieldX, double fieldY, double targetHeadingRad,
                                    double feedforwardRadPerSec) {
        double now = System.nanoTime() / 1.0e9;
        double dt = Double.isNaN(lastTimeSeconds) ? 0.0 : now - lastTimeSeconds;
        lastTimeSeconds = now;

        double error = AimLogic.wrapRadians(targetHeadingRad - getPose().getHeading());
        double pid = headingController.calculate(error, dt);
        double turn = PathConstants.HEADING_CORRECTION_SIGN * pid
                + feedforwardRadPerSec * TURN_POWER_PER_RAD_PER_SEC;
        turn = Math.max(-MAX_HEADING_LOCK_TURN, Math.min(MAX_HEADING_LOCK_TURN, turn));

        lastHeadingTargetRad = targetHeadingRad;
        lastHeadingErrorRad = error;
        lastTurnCommand = turn;
        headingLockActive = true;

        drivetrain.driveFieldCentric(fieldX, fieldY, turn, getHeading());
    }

    /** Resets the heading controller. Call when a heading-lock command starts. */
    public void resetHeadingController() {
        headingController.reset();
        lastTimeSeconds = Double.NaN;
    }

    public void stop() {
        headingLockActive = false;
        drivetrain.stop();
    }

    // ---------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------

    public Pose2d getPose() {
        return localization.getPose();
    }

    public Rotation2d getHeading() {
        return localization.getPose().getRotation();
    }

    /** Field-frame translational velocity, inches/sec. */
    public Translation2d getFieldVelocity() {
        return localization.getFieldVelocity();
    }

    /** Angular velocity, radians/sec CCW. */
    public double getAngularVelocity() {
        return localization.getAngularVelocity();
    }

    public Localization getLocalization() {
        return localization;
    }

    public MecanumDrivetrain getDrivetrain() {
        return drivetrain;
    }

    /** Signed heading error from the most recent heading-lock call, radians. */
    public double getHeadingErrorRadians() {
        return lastHeadingErrorRad;
    }

    public boolean isHeadingLockActive() {
        return headingLockActive;
    }

    public void addTelemetry(Telemetry telemetry) {
        localization.addTelemetry(telemetry);
        Translation2d v = getFieldVelocity();
        telemetry.addData("Velocity", "%.1f in/s  (x %.1f, y %.1f)  omega %.2f rad/s",
                v.getNorm(), v.getX(), v.getY(), getAngularVelocity());
        if (headingLockActive) {
            telemetry.addData("Heading lock", "target %.1f  err %.1f deg  turn %.2f",
                    Math.toDegrees(lastHeadingTargetRad),
                    Math.toDegrees(lastHeadingErrorRad), lastTurnCommand);
        }
    }
}
