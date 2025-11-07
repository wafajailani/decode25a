package org.firstinspires.ftc.teamcode.utils;

import java.util.*;

public class ShooterSpeedLookup {

    // Lookup table: distance → [shooter1Speed, shooter2Speed]
    private static final TreeMap<Double, Double> lookupTable = new TreeMap<>();
    static final double subtract = 200;

    static {
        // TO-DO: Determine distances and speeds
        lookupTable.put(30.0, 1800.0 - subtract);
        lookupTable.put(40.0, 1780.0 - subtract);
        lookupTable.put(50.0, 1740.0 - subtract);
        lookupTable.put(60.0, 1740.0 - subtract);
        lookupTable.put(70.0, 1720.0 - subtract);
        lookupTable.put(80.0, 1790.0 - subtract);
        lookupTable.put(90.0, 1830.0 - subtract);
        lookupTable.put(100.0, 1890.0);
        lookupTable.put(110.0, 1800.0);
        lookupTable.put(120.0, 1800.0);
        lookupTable.put(130.0, 1900.0);
        lookupTable.put(140.0, 2400.0); //2300 max rpm but PID weird
    }

    /**
     * Returns a list [shooter1Speed, shooter2Speed] for a given distance.
     * Performs linear interpolation if the distance is not in the table.
     */
    public static Double getShooterSpeed(double distance) {
        // Exact match
        if (lookupTable.containsKey(distance)) {
            return lookupTable.get(distance);
        }

        // Find nearest lower and higher entries
        Map.Entry<Double, Double> lower = lookupTable.floorEntry(distance);
        Map.Entry<Double, Double> higher = lookupTable.ceilingEntry(distance);

        // Handle values outside known range (clamp to nearest)
        if (lower == null) return higher.getValue();
        if (higher == null) return lower.getValue();

        // Interpolate linearly
        double x0 = lower.getKey();
        double x1 = higher.getKey();
        double t = (distance - x0) / (x1 - x0);

        double s1 = lower.getValue() + t * (higher.getValue() - lower.getValue());

        return s1;
    }

    /**
     * Adds or updates an entry in the lookup table.
     */
    public static void addLookupValue(double distance, double shooterSpeed) {
        lookupTable.put(distance, shooterSpeed);
    }
}

