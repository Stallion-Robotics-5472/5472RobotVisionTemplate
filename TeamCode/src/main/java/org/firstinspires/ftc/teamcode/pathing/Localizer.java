/*
 * Minimal pose source the follower depends on. The fused Localization
 * subsystem implements this, so the follower runs directly off the Kalman pose
 * estimator. Original implementation for this template.
 */
package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;

public interface Localizer {
    /** Advance the pose estimate by one cycle. */
    void update();

    /** Latest fused field pose (inches, radians). */
    Pose2d getPose();
}
