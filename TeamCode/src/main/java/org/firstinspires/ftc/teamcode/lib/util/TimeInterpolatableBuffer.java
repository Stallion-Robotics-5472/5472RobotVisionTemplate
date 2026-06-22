/*
 * Time-interpolatable history buffer of poses, ported from WPILib
 * (BSD-3-Clause, FIRST).
 *
 * The pose estimator keeps a short rolling history of odometry poses keyed by
 * timestamp. When a (delayed) vision measurement arrives, we can look up where
 * odometry thought the robot was at the exact instant the camera frame was
 * captured, interpolating between samples as needed. This is how the FRC pose
 * estimator performs latency compensation.
 */
package org.firstinspires.ftc.teamcode.lib.util;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;

import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

public final class TimeInterpolatableBuffer {
    private final double m_historySize;
    private final TreeMap<Double, Pose2d> m_buffer = new TreeMap<>();

    /**
     * @param historySizeSeconds how long, in seconds, to retain samples. Must be
     *     comfortably longer than the worst-case vision latency.
     */
    public TimeInterpolatableBuffer(double historySizeSeconds) {
        m_historySize = historySizeSeconds;
    }

    /** Adds a sample, evicting anything older than the history window. */
    public void addSample(double timeSeconds, Pose2d sample) {
        cleanUp(timeSeconds);
        m_buffer.put(timeSeconds, sample);
    }

    private void cleanUp(double time) {
        while (!m_buffer.isEmpty()) {
            Map.Entry<Double, Pose2d> entry = m_buffer.firstEntry();
            if (time - entry.getKey() >= m_historySize) {
                m_buffer.remove(entry.getKey());
            } else {
                return;
            }
        }
    }

    public void clear() {
        m_buffer.clear();
    }

    /**
     * Returns the pose at the given timestamp, interpolating between the
     * surrounding samples. If the timestamp is outside the buffer it is clamped
     * to the nearest stored sample.
     */
    public Optional<Pose2d> getSample(double timeSeconds) {
        if (m_buffer.isEmpty()) {
            return Optional.empty();
        }

        Map.Entry<Double, Pose2d> bottom = m_buffer.floorEntry(timeSeconds);
        Map.Entry<Double, Pose2d> top = m_buffer.ceilingEntry(timeSeconds);

        if (bottom == null) {
            return Optional.of(top.getValue());
        }
        if (top == null) {
            return Optional.of(bottom.getValue());
        }
        if (bottom.getKey().doubleValue() == top.getKey().doubleValue()) {
            return Optional.of(bottom.getValue());
        }

        double t = (timeSeconds - bottom.getKey()) / (top.getKey() - bottom.getKey());
        return Optional.of(bottom.getValue().interpolate(top.getValue(), t));
    }

    /** Exposes the underlying ordered map for the estimator's bookkeeping. */
    public TreeMap<Double, Pose2d> getInternalBuffer() {
        return m_buffer;
    }
}
