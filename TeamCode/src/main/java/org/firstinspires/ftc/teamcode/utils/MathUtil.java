package org.firstinspires.ftc.teamcode.utils;

import java.util.function.DoubleSupplier;

public final class MathUtil {
    public static double average(double[] nums) {
        double a = 0;
        for (double num : nums) {
            a += num;
        }
        return a/nums.length;
    }

    public static double average(DoubleSupplier... nums) {
        double a = 0;
        for (DoubleSupplier num : nums) {
            a += num.getAsDouble();
        }
        return a/nums.length;
    }

    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static boolean withinTolerance(double value, double target, double tolerance) {
        return withinTolerance(value-target, tolerance);
    }

    public static boolean withinTolerance(double error, double tolerance) {
        return Math.abs(error) <= tolerance;
    }

    public static double wrap(double value, double min, double max) {
        double range = max - min;
        while (value < min) value += range;
        while (value >= max) value -= range;
        return value;
    }

}
