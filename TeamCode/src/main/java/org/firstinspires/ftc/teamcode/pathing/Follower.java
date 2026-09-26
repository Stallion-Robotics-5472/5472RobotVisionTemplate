/*
 * Vector-based path follower that drives the robot along a PathChain using the
 * fused pose from the Kalman pose estimator (via the Localizer interface).
 * Original implementation for this template.
 *
 * Each cycle it builds a field-frame command from three contributions:
 *   - translational: a PIDF pull from the robot toward the closest point on the
 *     path (corrects positional/cross-track error),
 *   - centripetal: a nudge toward the inside of a curve, scaled by speed^2 and
 *     curvature, so the robot tracks curved paths without drifting wide,
 *   - drive: a PIDF push along the path tangent sized by the remaining length,
 *     which naturally decelerates the robot as it approaches the end.
 * Heading is controlled independently by a PIDF on the heading error toward the
 * path's target heading. The corrective (translational + centripetal) vector is
 * given priority over the drive vector within the unit power budget.
 */
package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.RobotClock;

public class Follower {
    private final Localizer localizer;
    private final Drivetrain drivetrain;

    private final PIDFController translationalPID;
    private final PIDFController drivePID;
    private final PIDFController headingPID;

    private boolean updatesLocalizer = true;
    private PathChain currentChain = null;
    private int pathIndex = 0;
    private double closestT = 0.0;
    private boolean busy = false;

    // Velocity estimate from pose finite-difference.
    private Pose2d lastPose = null;
    private double lastTimeSeconds = Double.NaN;
    private double speed = 0.0;
    private Translation2d estimatedFieldVelocity = new Translation2d();
    private double estimatedOmega = 0.0;

    // Diagnostics from the most recent update.
    private double lastCrossTrackError = 0.0;
    private double lastHeadingError = 0.0;
    private double lastRemainingLength = 0.0;
    private int markersFiredLastLoop = 0;

    public Follower(Localizer localizer, Drivetrain drivetrain) {
        this.localizer = localizer;
        this.drivetrain = drivetrain;
        translationalPID = new PIDFController(PathConstants.TRANSLATIONAL_kP,
                PathConstants.TRANSLATIONAL_kI, PathConstants.TRANSLATIONAL_kD,
                PathConstants.TRANSLATIONAL_kF);
        drivePID = new PIDFController(PathConstants.DRIVE_kP, PathConstants.DRIVE_kI,
                PathConstants.DRIVE_kD, PathConstants.DRIVE_kF);
        headingPID = new PIDFController(PathConstants.HEADING_kP, PathConstants.HEADING_kI,
                PathConstants.HEADING_kD, PathConstants.HEADING_kF);
    }

    /**
     * Whether this follower advances the pose estimate itself.
     *
     * True (the default) suits a plain OpMode that owns nothing else. Set it FALSE
     * when a subsystem already updates localization each loop -- DriveSubsystem
     * does, in periodic(). Updating twice in one loop feeds the estimator two
     * samples microseconds apart with no movement between them, which drags the
     * filtered velocity toward zero. That would quietly break shoot-on-the-move,
     * since the whole correction is proportional to velocity.
     */
    public void setUpdatesLocalizer(boolean updatesLocalizer) {
        this.updatesLocalizer = updatesLocalizer;
    }

    /** Begins following the given chain from its start. */
    public void followPath(PathChain chain) {
        this.currentChain = chain;
        this.pathIndex = 0;
        this.closestT = 0.0;
        this.busy = chain != null && !chain.isEmpty();
        translationalPID.reset();
        drivePID.reset();
        headingPID.reset();
        if (chain != null) {
            for (int i = 0; i < chain.size(); i++) {
                chain.get(i).resetHeading();
                chain.get(i).rearmMarkers();
            }
        }
        lastPose = null;
        lastTimeSeconds = Double.NaN;
    }

    public boolean isBusy() {
        return busy;
    }

    /** Convenience for OpModes: drives using the wall clock. */
    public boolean update() {
        return update(RobotClock.nowSeconds());
    }

    /**
     * Runs one control cycle at the given timestamp (seconds). Returns true while
     * still following. The timestamp overload keeps the controller deterministic
     * and testable.
     */
    public boolean update(double currentTimeSeconds) {
        if (updatesLocalizer) {
            localizer.update();
        }
        Pose2d pose = localizer.getPose();

        double dt = 0.0;
        if (!Double.isNaN(lastTimeSeconds)) {
            dt = currentTimeSeconds - lastTimeSeconds;
        }
        if (lastPose != null && dt > 1e-6) {
            double dx = pose.getX() - lastPose.getX();
            double dy = pose.getY() - lastPose.getY();
            speed = Math.hypot(dx, dy) / dt;
            estimatedFieldVelocity = new Translation2d(dx / dt, dy / dt);
            estimatedOmega = Path.shortestAngle(
                    pose.getHeading() - lastPose.getHeading()) / dt;
        }
        lastPose = pose;
        lastTimeSeconds = currentTimeSeconds;

        if (!busy || currentChain == null) {
            drivetrain.stop();
            return false;
        }

        Path path = currentChain.get(pathIndex);
        boolean isLastPath = pathIndex == currentChain.size() - 1;

        closestT = path.getClosestT(pose.getTranslation(), closestT);
        Translation2d closest = path.getPoint(closestT);
        Translation2d tangent = path.getUnitTangent(closestT);
        double curvature = path.getCurvature(closestT);
        HeadingSource.Target headingTarget = path.getHeadingTarget(
                closestT, pose, localizerFieldVelocity(), localizerOmega());
        double targetHeading = headingTarget.headingRadians;
        double remaining = path.getRemainingLength(closestT);

        // --- translational correction (toward closest point) ---
        Translation2d toPath = closest.minus(pose.getTranslation());
        double crossTrack = toPath.getNorm();
        Translation2d translationalVec = new Translation2d(0, 0);
        if (crossTrack > 1e-6) {
            double mag = translationalPID.calculate(crossTrack, dt);
            translationalVec = toPath.times(mag / crossTrack);
        }

        // --- centripetal correction (toward inside of curve) ---
        // Left normal of the tangent; sign of curvature points to the center.
        Translation2d leftNormal = new Translation2d(-tangent.getY(), tangent.getX());
        double centMag = PathConstants.CENTRIPETAL_SCALE * speed * speed * curvature;
        Translation2d centripetalVec = leftNormal.times(centMag);

        // --- drive along the path ---
        // Deceleration feedforward: target approach speed = √(2·decelRate·remaining).
        // This gives a physically-shaped slowdown that brings the robot to zero speed
        // at the path end. The drive PID output is taken as a floor (whichever is
        // larger wins), so the PID still helps when remaining is large.
        double targetApproachSpeed = Math.min(PathConstants.MAX_ROBOT_SPEED,
                Math.sqrt(2.0 * PathConstants.ZERO_POWER_DECEL_RATE * Math.max(0.0, remaining)));
        double driveFF = targetApproachSpeed / PathConstants.MAX_ROBOT_SPEED;
        double drivePIDOut = drivePID.calculate(remaining, dt);
        if (drivePIDOut < 0) drivePIDOut = 0;
        double driveMag = Math.max(driveFF, drivePIDOut);
        Translation2d driveVec = tangent.times(driveMag);

        // --- combine with corrective priority within the unit budget ---
        Translation2d corrective = clampNorm(translationalVec.plus(centripetalVec), 1.0);
        double headroom = Math.max(0.0, 1.0 - corrective.getNorm());
        Translation2d limitedDrive = clampNorm(driveVec, headroom);
        Translation2d fieldVec = clampNorm(corrective.plus(limitedDrive), 1.0);

        // --- heading control ---
        double headingError = Path.shortestAngle(targetHeading - pose.getHeading());
        // The feedforward is what lets a sweeping target (aiming at a goal while
        // translating past it) be tracked rather than trailed.
        double turn = clamp(PathConstants.HEADING_CORRECTION_SIGN
                        * headingPID.calculate(headingError, dt)
                        + headingTarget.omegaFeedforwardRadPerSec
                                * PathConstants.TURN_POWER_PER_RAD_PER_SEC,
                -1.0, 1.0);

        drivetrain.driveFieldCentric(fieldVec.getX(), fieldVec.getY(), turn, pose.getRotation());

        // Markers fire after the drive command is issued, so an action that sets a
        // setpoint takes effect on the next loop rather than fighting this one.
        markersFiredLastLoop = path.pollMarkers(closestT, remaining);

        lastCrossTrackError = crossTrack;
        lastHeadingError = headingError;
        lastRemainingLength = remaining;

        // --- advancement / completion ---
        if (!isLastPath) {
            if (remaining < PathConstants.ADVANCE_LENGTH_TOLERANCE || closestT >= PathConstants.ADVANCE_T) {
                pathIndex++;
                closestT = 0.0;
                drivePID.reset();
            }
        } else {
            boolean atEnd = crossTrack < PathConstants.END_TRANSLATION_TOLERANCE
                    && Math.abs(headingError) < PathConstants.END_HEADING_TOLERANCE
                    && remaining < PathConstants.END_TRANSLATION_TOLERANCE
                    && speed < PathConstants.END_VELOCITY_TOLERANCE;
            if (atEnd) {
                busy = false;
                drivetrain.stop();
            }
        }
        return busy;
    }

    public void stop() {
        busy = false;
        drivetrain.stop();
    }

    // ----- diagnostics -----
    public double getCrossTrackError() {
        return lastCrossTrackError;
    }

    public double getHeadingError() {
        return lastHeadingError;
    }

    public double getRemainingLength() {
        return lastRemainingLength;
    }

    public double getSpeed() {
        return speed;
    }

    public int getPathIndex() {
        return pathIndex;
    }

    public double getClosestT() {
        return closestT;
    }

    /** How many markers fired on the most recent update, for telemetry. */
    public int getMarkersFiredLastLoop() {
        return markersFiredLastLoop;
    }

    /**
     * Field velocity for a custom heading source. Taken from the Localizer when it
     * can supply one, else finite-differenced from the pose here.
     */
    private Translation2d localizerFieldVelocity() {
        if (localizer instanceof VelocityAware) {
            return ((VelocityAware) localizer).getFieldVelocity();
        }
        return estimatedFieldVelocity;
    }

    private double localizerOmega() {
        if (localizer instanceof VelocityAware) {
            return ((VelocityAware) localizer).getAngularVelocity();
        }
        return estimatedOmega;
    }

    /**
     * Implemented by a Localizer that already estimates velocity, so a heading
     * source gets the filtered value rather than a second, noisier derivative.
     */
    public interface VelocityAware {
        Translation2d getFieldVelocity();

        double getAngularVelocity();
    }

    // ----- helpers -----
    private static Translation2d clampNorm(Translation2d v, double maxNorm) {
        double norm = v.getNorm();
        if (norm > maxNorm && norm > 1e-9) {
            return v.times(maxNorm / norm);
        }
        return v;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
