package qa;

import org.firstinspires.ftc.teamcode.lib.geometry.*;
import org.firstinspires.ftc.teamcode.pathing.*;

/**
 * Closed-loop simulation of the Follower against an ideal holonomic robot with a
 * CORRECTLY wired drivetrain, so only the controller logic is under test.
 */
public class FollowerSimTest {

    static final double MAX_LIN = PathConstants.MAX_ROBOT_SPEED; // in/s at full power
    static final double MAX_ANG = Math.toRadians(360);           // rad/s at full turn power
    static final double DT = 0.02;

    /** Ideal robot: integrates commanded field velocity + CCW-positive turn. */
    static class SimRobot implements Localizer, Drivetrain {
        Pose2d pose;
        double headingSign;   // +1 = correctly wired, -1 = inverted rotation
        SimRobot(Pose2d start, double headingSign) {
            this.pose = start; this.headingSign = headingSign;
        }
        public void update() {}
        public Pose2d getPose() { return pose; }
        public void driveFieldCentric(double fx, double fy, double turn, Rotation2d h) {
            double nx = pose.getX() + fx * MAX_LIN * DT;
            double ny = pose.getY() + fy * MAX_LIN * DT;
            double nh = pose.getHeading() + headingSign * turn * MAX_ANG * DT;
            pose = new Pose2d(nx, ny, new Rotation2d(nh));
        }
        public void stop() {}
    }

    static PathChain plan() {
        Path s = new Path(new BezierCurve(
                new Translation2d(0, 0), new Translation2d(20, 0),
                new Translation2d(10, 30), new Translation2d(30, 30)))
                .setLinearHeading(0, Math.toRadians(90));
        Path straight = new Path(new BezierCurve(
                new Translation2d(30, 30), new Translation2d(30, 50)))
                .setConstantHeading(Math.toRadians(90));
        return new PathChain(s, straight);
    }

    static void run(String label, double headingSign, double correctionSign) {
        SimRobot robot = new SimRobot(new Pose2d(0, 0, new Rotation2d(0)), headingSign);
        // Follower reads PathConstants.HEADING_CORRECTION_SIGN directly, so to test
        // the alternative we mirror the robot's rotation sense instead - equivalent.
        Follower f = new Follower(robot, robot);
        f.followPath(plan());
        double t = 0;
        int steps = 0;
        double maxHeadErrDeg = 0;
        while (f.isBusy() && steps < 3000) {
            f.update(t);
            t += DT; steps++;
            maxHeadErrDeg = Math.max(maxHeadErrDeg, Math.abs(Math.toDegrees(f.getHeadingError())));
        }
        Pose2d p = robot.pose;
        System.out.printf("%-42s finished=%-5s  t=%5.2fs  end=(%.1f, %.1f, %.0fdeg)  "
                        + "maxHeadErr=%.0fdeg%n",
                label, !f.isBusy(), t, p.getX(), p.getY(), p.getRotation().getDegrees(),
                maxHeadErrDeg);
    }

    public static void main(String[] args) {
        System.out.printf("PathConstants.HEADING_CORRECTION_SIGN = %+.1f%n%n",
                PathConstants.HEADING_CORRECTION_SIGN);
        System.out.println("Target: end at (30, 50) facing 90 deg.\n");
        // headingSign=+1 models a correct drivetrain; the shipped SIGN then fights it.
        run("shipped SIGN on a correct drivetrain", +1.0, PathConstants.HEADING_CORRECTION_SIGN);
        // headingSign=-1 inverts the robot's rotation, which is equivalent to
        // flipping HEADING_CORRECTION_SIGN. This is the failure mode to avoid:
        // it must NOT converge, which is what makes the constant worth testing.
        run("the WRONG sign (must not converge)", -1.0, PathConstants.HEADING_CORRECTION_SIGN);
    }
}
