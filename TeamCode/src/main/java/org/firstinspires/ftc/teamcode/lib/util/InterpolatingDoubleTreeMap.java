/*
 * A lookup table that interpolates between the points you give it. The FTC
 * counterpart of WPILib's InterpolatingDoubleTreeMap.
 *
 * You measure a handful of real data points -- "at 40 inches the flywheel needs
 * 3000 rpm" -- and this fills in everything between them linearly. Outside the
 * measured range it clamps to the nearest endpoint rather than extrapolating,
 * because extrapolating a shooter curve past your data produces confident
 * nonsense.
 */
package org.firstinspires.ftc.teamcode.lib.util;

import java.util.Collections;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class InterpolatingDoubleTreeMap {
    private final TreeMap<Double, Double> map = new TreeMap<>();

    public void put(double key, double value) {
        map.put(key, value);
    }

    public void clear() {
        map.clear();
    }

    public int size() {
        return map.size();
    }

    public boolean isEmpty() {
        return map.isEmpty();
    }

    /**
     * The interpolated value at {@code key}.
     *
     * Returns 0 if the table is empty, the nearest endpoint's value if the key
     * is outside the measured range, and a linear blend otherwise.
     */
    public double get(double key) {
        if (map.isEmpty()) {
            return 0.0;
        }

        Double exact = map.get(key);
        if (exact != null) {
            return exact;
        }

        Map.Entry<Double, Double> below = map.floorEntry(key);
        Map.Entry<Double, Double> above = map.ceilingEntry(key);

        // Outside the measured range: clamp, never extrapolate.
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
        double t = (key - below.getKey()) / span;
        return below.getValue() + t * (above.getValue() - below.getValue());
    }

    /** Smallest measured key, or NaN if empty. */
    public double minKey() {
        return map.isEmpty() ? Double.NaN : map.firstKey();
    }

    /** Largest measured key, or NaN if empty. */
    public double maxKey() {
        return map.isEmpty() ? Double.NaN : map.lastKey();
    }

    /** True if the key falls inside the measured range (no clamping needed). */
    public boolean covers(double key) {
        return !map.isEmpty() && key >= map.firstKey() && key <= map.lastKey();
    }

    public NavigableMap<Double, Double> entries() {
        return Collections.unmodifiableNavigableMap(map);
    }
}
