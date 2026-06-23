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

public class Follower {
    private final Localizer localizer;
    private final Drivetrain drivetrain;

    private final PIDFController translationalPID;
    private final PIDFController drivePID;
    private final PIDFController headingPID;

    private PathChain currentChain = null;
    private int pathIndex = 0;
    private double closestT = 0.0;
    private boolean busy = false;

    // Velocity estimate from pose finite-difference.
    private Pose2d lastPose = null;
    private double lastTimeSeconds = Double.NaN;
    private double speed = 0.0;

    // Diagnostics from the most recent update.
    private double lastCrossTrackError = 0.0;
    private double lastHeadingError = 0.0;
    private double lastRemainingLength = 0.0;

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

    /** Begins following the given chain from its start. */
    public void followPath(PathChain chain) {
        this.currentChain = chain;
        this.pathIndex = 0;
        this.closestT = 0.0;
        this.busy = chain != null && !chain.isEmpty();
        translationalPID.reset();
        drivePID.reset();
        headingPID.reset();
        lastPose = null;
        lastTimeSeconds = Double.NaN;
    }

    public boolean isBusy() {
        return busy;
    }

    /** Convenience for OpModes: drives using the wall clock. */
    public boolean update() {
        return update(System.nanoTime() / 1.0e9);
    }

    /**
     * Runs one control cycle at the given timestamp (seconds). Returns true while
     * still following. The timestamp overload keeps the controller deterministic
     * and testable.
     */
    public boolean update(double currentTimeSeconds) {
        localizer.update();
        Pose2d pose = localizer.getPose();

        double dt = 0.0;
        if (!Double.isNaN(lastTimeSeconds)) {
            dt = currentTimeSeconds - lastTimeSeconds;
        }
        if (lastPose != null && dt > 1e-6) {
            double dx = pose.getX() - lastPose.getX();
            double dy = pose.getY() - lastPose.getY();
            speed = Math.hypot(dx, dy) / dt;
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
        double targetHeading = path.getHeading(closestT);
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

        // --- drive along the path (decelerates as remaining -> 0) ---
        double driveMag = drivePID.calculate(remaining, dt);
        if (driveMag < 0) {
            driveMag = 0;
        }
        Translation2d driveVec = tangent.times(driveMag);

        // --- combine with corrective priority within the unit budget ---
        Translation2d corrective = clampNorm(translationalVec.plus(centripetalVec), 1.0);
        double headroom = Math.max(0.0, 1.0 - corrective.getNorm());
        Translation2d limitedDrive = clampNorm(driveVec, headroom);
        Translation2d fieldVec = clampNorm(corrective.plus(limitedDrive), 1.0);

        // --- heading control ---
        double headingError = Path.shortestAngle(targetHeading - pose.getHeading());
        double turn = clamp(-1 * headingPID.calculate(headingError, dt), -1.0, 1.0);

        drivetrain.driveFieldCentric(fieldVec.getX(), fieldVec.getY(), turn, pose.getRotation());

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
