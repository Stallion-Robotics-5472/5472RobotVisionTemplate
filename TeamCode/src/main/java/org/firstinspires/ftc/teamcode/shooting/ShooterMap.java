/*
 * The shot table: distance to the goal -> what the shooter should do.
 *
 * You measure a handful of real shots at known distances and record the
 * flywheel speed, hood angle and flight time that worked. The map interpolates
 * between them and clamps outside them (never extrapolates -- a shooter curve
 * guessed past your data is confident nonsense).
 *
 *   ShooterMap map = ShooterMap.builder()
 *           //     distance(in)  rpm   hood(deg)  tof(s)
 *           .add(         24,   2200,     22.0,    0.32)
 *           .add(         48,   2700,     30.0,    0.48)
 *           .add(         72,   3200,     35.0,    0.64)
 *           .build();
 *
 * Rows are added whole, on purpose. The FRC version of this kept three separate
 * maps -- rpm, hood, flight time -- keyed by distance, and nothing stopped one
 * of them gaining a data point the others lacked; the maps would silently
 * interpolate from different distance keys and the shot would drift. Here a row
 * is one call, so the three curves cannot come apart.
 *
 * DISTANCES ARE IN INCHES. This template works in inches throughout, unlike the
 * metre-based FRC original. Mixing the two puts every shot wildly off.
 */
package org.firstinspires.ftc.teamcode.shooting;

import java.util.Collections;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public final class ShooterMap {

    private final TreeMap<Double, ShooterSetpoint> rows;

    private ShooterMap(TreeMap<Double, ShooterSetpoint> rows) {
        this.rows = rows;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static final class Builder {
        private final TreeMap<Double, ShooterSetpoint> rows = new TreeMap<>();

        /**
         * Adds one measured shot.
         *
         * @param distanceInches distance from the shooter to the goal.
         * @param flywheelRpm flywheel speed that made the shot.
         * @param hoodDegrees hood angle that made the shot.
         * @param timeOfFlightSeconds how long the piece was in the air. Time it
         *     from video if you can; a rough value still beats leaving it zero,
         *     because zero disables the whole moving-shot correction.
         */
        public Builder add(double distanceInches, double flywheelRpm, double hoodDegrees,
                           double timeOfFlightSeconds) {
            if (distanceInches < 0) {
                throw new IllegalArgumentException("distance cannot be negative");
            }
            if (timeOfFlightSeconds < 0) {
                throw new IllegalArgumentException("time of flight cannot be negative");
            }
            rows.put(distanceInches,
                    new ShooterSetpoint(flywheelRpm, hoodDegrees, timeOfFlightSeconds));
            return this;
        }

        public ShooterMap build() {
            if (rows.isEmpty()) {
                throw new IllegalStateException(
                        "A ShooterMap needs at least one row; with none there is nothing to aim with");
            }
            return new ShooterMap(new TreeMap<>(rows));
        }
    }

    /**
     * The setpoint for a shot at {@code distanceInches}, interpolated between
     * the two nearest measured rows, or clamped to the nearest row outside the
     * measured range.
     */
    public ShooterSetpoint setpointAt(double distanceInches) {
        Map.Entry<Double, ShooterSetpoint> below = rows.floorEntry(distanceInches);
        Map.Entry<Double, ShooterSetpoint> above = rows.ceilingEntry(distanceInches);

        if (below == null) {
            return above.getValue();
        }
        if (above == null) {
            return below.getValue();
        }
        double span = above.getKey() - below.getKey();
        if (span == 0.0) {
            return below.getValue();
        }
        double t = (distanceInches - below.getKey()) / span;
        return below.getValue().interpolate(above.getValue(), t);
    }

    public double rpmAt(double distanceInches) {
        return setpointAt(distanceInches).flywheelRpm;
    }

    public double hoodAt(double distanceInches) {
        return setpointAt(distanceInches).hoodDegrees;
    }

    /**
     * Flight time for a shot at this distance. This is the input the moving-shot
     * correction in {@link AimLogic} runs on.
     */
    public double timeOfFlightAt(double distanceInches) {
        return setpointAt(distanceInches).timeOfFlightSeconds;
    }

    /** Closest distance you measured. */
    public double minDistance() {
        return rows.firstKey();
    }

    /** Farthest distance you measured. */
    public double maxDistance() {
        return rows.lastKey();
    }

    /**
     * True when the distance falls inside the measured range. Outside it the map
     * clamps, so the shot is a guess -- worth gating a shoot command on.
     */
    public boolean covers(double distanceInches) {
        return distanceInches >= minDistance() && distanceInches <= maxDistance();
    }

    public int size() {
        return rows.size();
    }

    public NavigableMap<Double, ShooterSetpoint> rows() {
        return Collections.unmodifiableNavigableMap(rows);
    }

    /** The whole table, for telemetry while tuning. */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("ShooterMap:\n");
        for (Map.Entry<Double, ShooterSetpoint> e : rows.entrySet()) {
            sb.append(String.format("  %6.1f in -> %s%n", e.getKey(), e.getValue()));
        }
        return sb.toString();
    }
}
