package qa;

import org.firstinspires.ftc.teamcode.lib.util.InterpolatingDoubleTreeMap;
import org.firstinspires.ftc.teamcode.shooting.ShooterMap;
import org.firstinspires.ftc.teamcode.shooting.ShooterSetpoint;
import org.firstinspires.ftc.teamcode.shooting.ShootingConstants;

/** Checks the shot table interpolates, clamps, and never extrapolates. */
public class ShooterMapTest {
    static int fails = 0;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%-56s %s%s%n", name, ok ? "PASS" : "**FAIL**",
                ok ? "" : "   " + detail);
        if (!ok) fails++;
    }

    static boolean near(double a, double b, double tol) { return Math.abs(a - b) <= tol; }

    public static void main(String[] args) {
        System.out.println("=== InterpolatingDoubleTreeMap ===");
        InterpolatingDoubleTreeMap m = new InterpolatingDoubleTreeMap();
        check("Empty map returns 0", m.get(5.0) == 0.0, "" + m.get(5.0));
        m.put(10.0, 100.0);
        m.put(20.0, 200.0);
        m.put(40.0, 300.0);
        check("Exact key returns its value", m.get(20.0) == 200.0, "" + m.get(20.0));
        check("Midpoint interpolates linearly", near(m.get(15.0), 150.0, 1e-9),
                "" + m.get(15.0));
        check("Interpolates on an uneven span", near(m.get(30.0), 250.0, 1e-9),
                "" + m.get(30.0));
        check("Below the range clamps, does not extrapolate", m.get(0.0) == 100.0,
                "" + m.get(0.0));
        check("Above the range clamps, does not extrapolate", m.get(1000.0) == 300.0,
                "" + m.get(1000.0));
        check("covers() reports the measured range",
                m.covers(15.0) && !m.covers(5.0) && !m.covers(50.0), "wrong");

        System.out.println("\n=== ShooterMap ===");
        ShooterMap map = ShooterMap.builder()
                .add(24, 2000, 20.0, 0.30)
                .add(48, 3000, 30.0, 0.50)
                .build();

        ShooterSetpoint mid = map.setpointAt(36);
        System.out.printf("   36 in -> %s%n", mid);
        check("All three columns interpolate together",
                near(mid.flywheelRpm, 2500, 1e-9) && near(mid.hoodDegrees, 25.0, 1e-9)
                        && near(mid.timeOfFlightSeconds, 0.40, 1e-9), mid.toString());
        check("Below range clamps to the nearest row",
                near(map.rpmAt(0), 2000, 1e-9), "" + map.rpmAt(0));
        check("Above range clamps to the nearest row",
                near(map.rpmAt(500), 3000, 1e-9), "" + map.rpmAt(500));
        check("covers() reports the measured range",
                map.covers(36) && !map.covers(10) && !map.covers(100), "wrong");
        check("minDistance / maxDistance", map.minDistance() == 24 && map.maxDistance() == 48,
                map.minDistance() + ".." + map.maxDistance());

        // A row is added whole, so the three curves cannot drift apart the way
        // three separately-keyed maps can.
        check("Every row carries all three columns", map.size() == 2, "" + map.size());
        boolean allPresent = true;
        for (java.util.Map.Entry<Double, ShooterSetpoint> e : map.rows().entrySet()) {
            if (e.getValue().timeOfFlightSeconds <= 0) allPresent = false;
        }
        check("No row can exist without a flight time", allPresent, "a row has tof <= 0");

        // Guard rails.
        boolean empty = false;
        try { ShooterMap.builder().build(); } catch (IllegalStateException e) { empty = true; }
        check("An empty map is rejected", empty, "accepted");
        boolean negTof = false;
        try { ShooterMap.builder().add(10, 100, 10, -1); }
        catch (IllegalArgumentException e) { negTof = true; }
        check("A negative flight time is rejected", negTof, "accepted");

        // Trim.
        ShooterSetpoint trimmed = map.setpointAt(24).withTrim(150, -2.0);
        check("Trim adjusts rpm and hood but not flight time",
                near(trimmed.flywheelRpm, 2150, 1e-9)
                        && near(trimmed.hoodDegrees, 18.0, 1e-9)
                        && near(trimmed.timeOfFlightSeconds, 0.30, 1e-9),
                trimmed.toString());

        System.out.println("\n=== The shipped table in ShootingConstants ===");
        ShooterMap shipped = ShootingConstants.SHOT_MAP;
        System.out.print("   " + shipped.toString().replace("\n", "\n   "));
        boolean monotonicTof = true;
        boolean monotonicRpm = true;
        double prevTof = -1, prevRpm = -1;
        for (java.util.Map.Entry<Double, ShooterSetpoint> e : shipped.rows().entrySet()) {
            if (e.getValue().timeOfFlightSeconds <= prevTof) monotonicTof = false;
            if (e.getValue().flywheelRpm <= prevRpm) monotonicRpm = false;
            prevTof = e.getValue().timeOfFlightSeconds;
            prevRpm = e.getValue().flywheelRpm;
        }
        // Not laws of physics, but a table that dips is nearly always a typo.
        check("Flight time rises with distance", monotonicTof, "a row dips");
        check("Flywheel speed rises with distance", monotonicRpm, "a row dips");
        check("Every shipped row has a non-zero flight time", prevTof > 0,
                "a zero flight time silently disables the moving-shot correction");
        check("The shot range sits inside the table's data",
                shipped.covers(ShootingConstants.MIN_SHOT_DISTANCE_IN)
                        && shipped.covers(ShootingConstants.MAX_SHOT_DISTANCE_IN),
                String.format("range %.0f..%.0f vs table %.0f..%.0f",
                        ShootingConstants.MIN_SHOT_DISTANCE_IN,
                        ShootingConstants.MAX_SHOT_DISTANCE_IN,
                        shipped.minDistance(), shipped.maxDistance()));

        System.out.println(fails == 0 ? "\nAll shooter-map checks passed."
                : "\n" + fails + " FAILURE(S)");
        if (fails != 0) System.exit(1);
    }
}
