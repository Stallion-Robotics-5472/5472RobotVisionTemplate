package qa;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.pathing.PathConstants;

/**
 * Replicates MecanumDrivetrain.driveFieldCentric exactly, applies the configured
 * motor Direction constants, then runs standard mecanum FORWARD kinematics to
 * recover what the chassis actually does. This turns a sign question into a
 * measurement.
 */
public class DriveTest {
    static int fails = 0;
    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-58s %s%s%n", name, ok ? "PASS" : "**FAIL**", ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    /** The physically-correct baseline for a standard mecanum build. */
    static final Direction BASE_FL = Direction.REVERSE, BASE_BL = Direction.REVERSE;
    static final Direction BASE_FR = Direction.FORWARD, BASE_BR = Direction.FORWARD;

    /** +1 if the configured direction matches the standard build, -1 if inverted. */
    static double rel(Direction cfg, Direction base) { return cfg == base ? 1.0 : -1.0; }

    /** chassis motion actually produced: {vx forward, vy left, omega CCW} */
    static double[] chassis(double fieldX, double fieldY, double turn, double headingRad) {
        Rotation2d h = new Rotation2d(headingRad);
        double cos = h.getCos(), sin = h.getSin();
        // --- verbatim from MecanumDrivetrain ---
        double forward = fieldX * cos + fieldY * sin;
        double right = fieldX * sin - fieldY * cos;
        double rotate = -turn;
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(br), Math.abs(bl)))));
        fl /= max; fr /= max; br /= max; bl /= max;
        // --- apply the configured motor directions ---
        fl *= rel(PathConstants.FRONT_LEFT_DIRECTION, BASE_FL);
        bl *= rel(PathConstants.BACK_LEFT_DIRECTION, BASE_BL);
        fr *= rel(PathConstants.FRONT_RIGHT_DIRECTION, BASE_FR);
        br *= rel(PathConstants.BACK_RIGHT_DIRECTION, BASE_BR);
        // --- standard mecanum forward kinematics ---
        return new double[]{
                (fl + fr + bl + br) / 4.0,     // vx  (forward)
                (-fl + fr + bl - br) / 4.0,    // vy  (left)
                (-fl + fr - bl + br) / 4.0     // omega (CCW)
        };
    }

    public static void main(String[] args) {
        System.out.println("Configured motor directions:");
        System.out.printf("   FL=%s  BL=%s  FR=%s  BR=%s%n",
                PathConstants.FRONT_LEFT_DIRECTION, PathConstants.BACK_LEFT_DIRECTION,
                PathConstants.FRONT_RIGHT_DIRECTION, PathConstants.BACK_RIGHT_DIRECTION);
        System.out.printf("   (standard mecanum build needs FL=%s BL=%s FR=%s BR=%s)%n%n",
                BASE_FL, BASE_BL, BASE_FR, BASE_BR);

        System.out.println("=== Commanded vs. actual chassis motion (heading 0) ===");
        double[] fwd = chassis(1, 0, 0, 0);
        System.out.printf("   command 'drive +X':  vx=%+.2f  vy=%+.2f  omega=%+.2f%n",
                fwd[0], fwd[1], fwd[2]);
        check("Commanding field +X drives the robot forward",
                fwd[0] > 0.5 && Math.abs(fwd[2]) < 0.1,
                "robot spins instead of translating");

        double[] left = chassis(0, 1, 0, 0);
        System.out.printf("   command 'drive +Y':  vx=%+.2f  vy=%+.2f  omega=%+.2f%n",
                left[0], left[1], left[2]);
        check("Commanding field +Y strafes the robot left",
                left[1] > 0.5 && Math.abs(left[2]) < 0.1, "wrong axis");

        double[] ccw = chassis(0, 0, 1, 0);
        System.out.printf("   command 'turn CCW':  vx=%+.2f  vy=%+.2f  omega=%+.2f%n",
                ccw[0], ccw[1], ccw[2]);
        check("Positive turnPower rotates CCW (as the interface documents)",
                ccw[2] > 0.5 && Math.abs(ccw[0]) < 0.1, "turn maps to translation or wrong sign");

        System.out.println("\n=== Field-centric correctness at heading 90 deg ===");
        double[] rot = chassis(1, 0, 0, Math.PI / 2);
        System.out.printf("   heading 90, command field +X: vx=%+.2f vy=%+.2f omega=%+.2f%n",
                rot[0], rot[1], rot[2]);
        check("Field +X at heading 90 becomes robot 'right' (vy<0)",
                rot[1] < -0.5, "field-centric rotation is wrong");

        System.out.println("\n=== Gamepad turn-stick polarity (TeleOp OpModes) ===");
        // The polarity the OpModes actually compile with is passed in as args[0]
        // (+1 if the source reads `gamepad1.right_stick_x`, -1 if `-gamepad1...`),
        // so this test tracks the real source instead of a copy of it.
        double stickPolarity = args.length > 0 ? Double.parseDouble(args[0]) : 1.0;
        double rightStickPushedRight = +1.0;
        double turn = stickPolarity * rightStickPushedRight;
        double[] g = chassis(0, 0, turn, 0);
        System.out.printf("   right stick pushed RIGHT (+1) -> omega=%+.2f (%s)%n",
                g[2], g[2] > 0 ? "CCW / robot turns LEFT" : "CW / robot turns RIGHT");
        check("Right stick pushed right turns the robot right",
                g[2] < 0, "stick and robot rotation are inverted");

        System.out.println("\n=== HEADING_CORRECTION_SIGN (Follower) ===");
        System.out.printf("   HEADING_CORRECTION_SIGN = %+.1f%n",
                PathConstants.HEADING_CORRECTION_SIGN);
        // Follower: headingError = target - current;  turn = SIGN * kP * error
        double headingError = Math.toRadians(30);   // target is 30 deg CCW of us
        double turnOut = PathConstants.HEADING_CORRECTION_SIGN
                * PathConstants.HEADING_kP * headingError;
        double[] hc = chassis(0, 0, turnOut, 0);
        System.out.printf("   heading error +30deg (need CCW) -> omega=%+.3f (%s)%n",
                hc[2], hc[2] > 0 ? "CCW - corrects" : "CW - diverges");
        check("Heading controller rotates toward the target", hc[2] > 0,
                "positive feedback: the robot turns AWAY and spins up");

        System.out.println(fails == 0 ? "\nAll drivetrain checks passed."
                : "\n" + fails + " FAILURE(S)");
    }
}
