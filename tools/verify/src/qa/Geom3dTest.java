package qa;

import org.firstinspires.ftc.teamcode.lib.geometry.*;

/** Verifies the SE(3) layer used for the Limelight camera-offset transform. */
public class Geom3dTest {
    static int fails = 0;
    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-58s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }
    static boolean near(double a, double b, double tol) { return Math.abs(a - b) <= tol; }

    public static void main(String[] args) {
        System.out.println("=== SE(3) / Rotation3d ===");

        // 1. Matrix build/extract round trip for a generic orientation.
        double roll = Math.toRadians(11), pitch = Math.toRadians(-23), yaw = Math.toRadians(67);
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        check("Rotation3d rpy round-trip (roll)", near(r.getRoll(), roll, 1e-9),
                "got " + Math.toDegrees(r.getRoll()));
        check("Rotation3d rpy round-trip (pitch)", near(r.getPitch(), pitch, 1e-9),
                "got " + Math.toDegrees(r.getPitch()));
        check("Rotation3d rpy round-trip (yaw)", near(r.getYaw(), yaw, 1e-9),
                "got " + Math.toDegrees(r.getYaw()));

        // 2. Orthonormality: R * R^T == I.
        Rotation3d ident = r.times(r.inverse());
        boolean orth = near(ident.getRoll(), 0, 1e-12) && near(ident.getPitch(), 0, 1e-12)
                && near(ident.getYaw(), 0, 1e-12);
        check("Rotation3d R * R^-1 == I", orth, ident.toString());

        // 3. PITCH SIGN CONVENTION: where does the camera boresight (+X) point
        //    for a POSITIVE pitch value? VisionConstants claims "+ = tilted up".
        Rotation3d pitchOnly = new Rotation3d(0, Math.toRadians(15), 0);
        Translation3d boresight = pitchOnly.rotate(new Translation3d(1, 0, 0));
        System.out.printf("   pitch=+15deg maps robot +X (forward) to (%.3f, %.3f, %.3f)%n",
                boresight.getX(), boresight.getY(), boresight.getZ());
        // Right-hand rule about +Y (left): positive pitch tilts the boresight DOWN.
        // This is the WPILib/FTC convention; VisionConstants must document it that
        // way, because a camera angled UP to see tags is a NEGATIVE pitch value.
        check("POSITIVE pitch tilts boresight DOWN (-Z)", boresight.getZ() < 0,
                "boresight z = " + String.format("%.3f", boresight.getZ()));
        try {
            String vc = new String(java.nio.file.Files.readAllBytes(java.nio.file.Paths.get(
                    System.getProperty("repo.root", ".")
                            + "/TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
                            + "/subsystems/VisionConstants.java")));
            check("VisionConstants documents the pitch sign correctly",
                    vc.contains("POSITIVE pitch tilts the camera DOWN")
                            && !vc.contains("+ = tilted up"),
                    "the pitch-sign comment is missing or still inverted");
        } catch (Exception e) {
            check("VisionConstants readable", false, e.toString());
        }

        // 4. Camera-offset recovery: this is exactly what Localization does.
        //    Given a true robot pose, synthesize what the camera would report,
        //    then run the code's inverse transform and demand the robot pose back.
        Transform3d robotToCamera = new Transform3d(
                -0.98, 6.057, 4.597,
                Math.toRadians(0), Math.toRadians(15), Math.toRadians(-15));
        Pose3d trueRobot = new Pose3d(24.0, -13.5, 0.0, 0, 0, Math.toRadians(42));
        Pose3d cameraField = trueRobot.transformBy(robotToCamera);          // what LL sees
        Pose3d recovered = cameraField.transformBy(robotToCamera.inverse()); // what code does
        boolean ok = near(recovered.getX(), trueRobot.getX(), 1e-9)
                && near(recovered.getY(), trueRobot.getY(), 1e-9)
                && near(recovered.getZ(), trueRobot.getZ(), 1e-9)
                && near(recovered.getYaw(), trueRobot.getYaw(), 1e-9);
        check("Camera field pose -> robot pose round-trip", ok,
                "recovered " + recovered + " vs true " + trueRobot);

        // 5. Magnitude sanity: how far does the offset actually move the pose?
        System.out.printf("   camera sits %.2f in from robot centre; "
                        + "botpose XY differs by %.2f in%n",
                Math.hypot(-0.98, 6.057),
                Math.hypot(cameraField.getX() - trueRobot.getX(),
                        cameraField.getY() - trueRobot.getY()));

        // 6. Transform3d.inverse() is a true inverse.
        Transform3d t = new Transform3d(3, -4, 5,
                Math.toRadians(10), Math.toRadians(20), Math.toRadians(30));
        Pose3d p = new Pose3d(1, 2, 3, Math.toRadians(5), Math.toRadians(-7), Math.toRadians(100));
        Pose3d back = p.transformBy(t).transformBy(t.inverse());
        check("Transform3d inverse round-trip",
                near(back.getX(), p.getX(), 1e-9) && near(back.getY(), p.getY(), 1e-9)
                        && near(back.getZ(), p.getZ(), 1e-9)
                        && near(back.getYaw(), p.getYaw(), 1e-9)
                        && near(back.getPitch(), p.getPitch(), 1e-9)
                        && near(back.getRoll(), p.getRoll(), 1e-9),
                back + " vs " + p);

        // 7. Gimbal-lock guard doesn't throw / NaN at pitch = 90.
        Rotation3d lock = new Rotation3d(0, Math.PI / 2, Math.toRadians(30));
        check("Gimbal-lock guard returns finite angles",
                !Double.isNaN(lock.getYaw()) && !Double.isNaN(lock.getRoll())
                        && !Double.isNaN(lock.getPitch()), lock.toString());

        System.out.println(fails == 0 ? "\nAll 3D geometry checks passed."
                : "\n" + fails + " FAILURE(S)");
    }
}
