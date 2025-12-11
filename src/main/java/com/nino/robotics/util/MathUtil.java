package com.nino.robotics.util;

/**
 * Utility class providing common mathematical operations.
 * <p>
 * This class contains static helper methods for mathematical calculations
 * used throughout the simulation.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 */
public final class MathUtil {

    /**
     * Private constructor to prevent instantiation of this utility class.
     */
    private MathUtil() {}

    /**
     * Clamps a value between a minimum and maximum value.
     * <p>
     * If the value is less than min, returns min.
     * If the value is greater than max, returns max.
     * Otherwise, returns the value unchanged.
     * </p>
     *
     * @param value The value to clamp
     * @param min   The minimum allowed value
     * @param max   The maximum allowed value
     * @return The clamped value, guaranteed to be in range [min, max]
     */
    public static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
}
