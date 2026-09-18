package com.qualcomm.hardware.gobilda;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/** Stub Pinpoint. Holds a pose so tests can simulate odometry. */
public class GoBildaPinpointDriver {
    public enum GoBildaOdometryPods { goBILDA_SWINGARM_POD, goBILDA_4_BAR_POD }
    public enum EncoderDirection { FORWARD, REVERSED }

    public double x = 0, y = 0, headingRad = 0;

    public void setOffsets(double x, double y, DistanceUnit u) {}
    public void setEncoderResolution(GoBildaOdometryPods p) {}
    public void setEncoderDirections(EncoderDirection x, EncoderDirection y) {}
    public void resetPosAndIMU() {}
    public void update() {}
    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRad);
    }
    public void setPosition(Pose2D p) {
        x = p.getX(DistanceUnit.INCH);
        y = p.getY(DistanceUnit.INCH);
        headingRad = p.getHeading(AngleUnit.RADIANS);
    }
}
