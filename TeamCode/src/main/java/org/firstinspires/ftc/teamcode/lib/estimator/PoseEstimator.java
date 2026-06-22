/*
 * Pose estimator ported from WPILib's PoseEstimator (BSD-3-Clause, FIRST) and
 * adapted for FTC.
 *
 * This is the same algorithm FRC teams use (and the one the AdvantageKit vision
 * template feeds): a single-frame Kalman gain blends absolute vision
 * measurements into a continuously-integrated odometry pose. The gain is
 * derived from the ratio of odometry "state" standard deviations to the
 * per-measurement vision standard deviations, so noisier vision is trusted
 * less. Delayed vision frames are handled by sampling the historical odometry
 * pose at the frame's capture time and replaying any newer odometry on top of
 * the correction.
 *
 * In WPILib the estimator integrates wheel encoder + gyro deltas itself. On
 * FTC the goBILDA Pinpoint already fuses two dead-wheel encoders with its on-
 * board IMU and reports an absolute pose, so here we feed that pose in directly
 * as the odometry source via {@link #updateWithTime(double, Pose2d)}.
 *
 * Units in this template are inches and radians: state/vision std devs are
 * {x (in), y (in), heading (rad)}.
 */
package org.firstinspires.ftc.teamcode.lib.estimator;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Twist2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeInterpolatableBuffer;

import java.util.NavigableMap;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.TreeMap;

public class PoseEstimator {
    /** How far back, in seconds, odometry history and vision updates are kept. */
    private static final double BUFFER_DURATION = 1.5;

    private final TimeInterpolatableBuffer m_odometryPoseBuffer =
            new TimeInterpolatableBuffer(BUFFER_DURATION);

    /** Timestamp -> vision correction, used to replay corrections over odometry. */
    private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();

    // q = odometry (state) variance for x, y, theta.
    private final double[] m_q = new double[3];
    // Diagonal of the Kalman gain matrix for the most recent vision std devs.
    private final double[] m_visionK = new double[3];

    private Pose2d m_odometryPose = new Pose2d();
    private Pose2d m_poseEstimate = new Pose2d();

    /**
     * @param stateStdDevs standard deviations of the odometry estimate
     *     {x (in), y (in), heading (rad)}. Smaller values trust odometry more.
     * @param visionMeasurementStdDevs default standard deviations of a vision
     *     measurement {x (in), y (in), heading (rad)}. May be overridden per
     *     measurement. Larger values trust vision less.
     */
    public PoseEstimator(double[] stateStdDevs, double[] visionMeasurementStdDevs) {
        for (int i = 0; i < 3; i++) {
            m_q[i] = stateStdDevs[i] * stateStdDevs[i];
        }
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    /**
     * Recomputes the Kalman gain for a given set of vision standard deviations.
     * Gain_i = q_i / (q_i + sqrt(q_i * r_i)), where r_i is the vision variance.
     * A very large std dev drives the gain to ~0 (ignore that axis); this is how
     * we tell the estimator to keep heading from the gyro when using MegaTag2.
     */
    public final void setVisionMeasurementStdDevs(double[] visionMeasurementStdDevs) {
        double[] r = new double[3];
        for (int i = 0; i < 3; i++) {
            r[i] = visionMeasurementStdDevs[i] * visionMeasurementStdDevs[i];
        }
        for (int row = 0; row < 3; row++) {
            if (m_q[row] == 0.0) {
                m_visionK[row] = 0.0;
            } else {
                m_visionK[row] = m_q[row] / (m_q[row] + Math.sqrt(m_q[row] * r[row]));
            }
        }
    }

    /** Hard-resets the estimate, odometry reference, and all history. */
    public void resetPose(Pose2d pose) {
        m_odometryPose = pose;
        m_poseEstimate = pose;
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
    }

    public Pose2d getEstimatedPosition() {
        return m_poseEstimate;
    }

    /**
     * Feeds the latest absolute odometry pose (from the Pinpoint) at the given
     * timestamp and returns the updated fused estimate.
     */
    public Pose2d updateWithTime(double currentTimeSeconds, Pose2d odometryPose) {
        m_odometryPose = odometryPose;
        m_odometryPoseBuffer.addSample(currentTimeSeconds, odometryPose);

        if (m_visionUpdates.isEmpty()) {
            m_poseEstimate = odometryPose;
        } else {
            VisionUpdate visionUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
            m_poseEstimate = visionUpdate.compensate(odometryPose);
        }
        return getEstimatedPosition();
    }

    /**
     * Returns the fused estimate as of an arbitrary (recent) timestamp, applying
     * the relevant vision correction to the interpolated odometry pose.
     */
    public Optional<Pose2d> sampleAt(double timestampSeconds) {
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        double oldest = m_odometryPoseBuffer.getInternalBuffer().firstKey();
        double newest = m_odometryPoseBuffer.getInternalBuffer().lastKey();
        double clamped = Math.max(oldest, Math.min(timestampSeconds, newest));

        if (m_visionUpdates.isEmpty() || clamped < m_visionUpdates.firstKey()) {
            return m_odometryPoseBuffer.getSample(clamped);
        }

        double floorTimestamp = m_visionUpdates.floorKey(clamped);
        VisionUpdate visionUpdate = m_visionUpdates.get(floorTimestamp);
        Optional<Pose2d> odometryEstimate = m_odometryPoseBuffer.getSample(clamped);
        return odometryEstimate.map(visionUpdate::compensate);
    }

    /**
     * Adds a vision measurement using the estimator's current vision std devs.
     *
     * @param visionRobotPose the field-relative robot pose the camera reports.
     * @param timestampSeconds the time the frame was captured, on the same clock
     *     as {@link #updateWithTime(double, Pose2d)} (i.e. latency-compensated).
     */
    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
        // Step 0: drop measurements older than our history window.
        try {
            if (m_odometryPoseBuffer.getInternalBuffer().lastKey() - BUFFER_DURATION
                    > timestampSeconds) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        // Step 1: expire vision updates that are no longer needed.
        cleanUpVisionUpdates();

        // Step 2: where did odometry think we were when the frame was captured?
        Optional<Pose2d> odometrySample = m_odometryPoseBuffer.getSample(timestampSeconds);
        if (!odometrySample.isPresent()) {
            return;
        }

        // Step 3: where did the fused estimate think we were at that instant?
        Optional<Pose2d> visionSample = sampleAt(timestampSeconds);
        if (!visionSample.isPresent()) {
            return;
        }

        // Step 4: error between the estimate and the camera, as a twist.
        Twist2d twist = visionSample.get().log(visionRobotPose);

        // Step 5: scale the correction by the Kalman gain (per-axis trust).
        Twist2d scaledTwist = new Twist2d(
                m_visionK[0] * twist.dx,
                m_visionK[1] * twist.dy,
                m_visionK[2] * twist.dtheta);

        // Step 6: record the corrected pose, anchored to the odometry at capture.
        VisionUpdate visionUpdate =
                new VisionUpdate(visionSample.get().exp(scaledTwist), odometrySample.get());
        m_visionUpdates.put(timestampSeconds, visionUpdate);

        // Step 7: discard any later corrections; they are stale now.
        m_visionUpdates.tailMap(timestampSeconds, false).clear();

        // Step 8: replay current odometry on top of this newest correction.
        m_poseEstimate = visionUpdate.compensate(m_odometryPose);
    }

    /** Convenience overload that sets the per-measurement std devs first. */
    public void addVisionMeasurement(
            Pose2d visionRobotPose, double timestampSeconds, double[] visionMeasurementStdDevs) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPose, timestampSeconds);
    }

    private void cleanUpVisionUpdates() {
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return;
        }
        double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();
        if (m_visionUpdates.isEmpty() || oldestOdometryTimestamp < m_visionUpdates.firstKey()) {
            return;
        }
        double newestNeeded = m_visionUpdates.floorKey(oldestOdometryTimestamp);
        m_visionUpdates.headMap(newestNeeded, false).clear();
    }

    /**
     * A recorded vision correction. {@link #compensate(Pose2d)} re-expresses the
     * correction onto whatever odometry pose is current by adding the odometry
     * delta accumulated since the frame was captured.
     */
    private static final class VisionUpdate {
        private final Pose2d visionPose;
        private final Pose2d odometryPose;

        private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        public Pose2d compensate(Pose2d pose) {
            Transform2d delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }
}
