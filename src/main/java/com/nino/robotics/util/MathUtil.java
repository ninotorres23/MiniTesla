package com.nino.robotics.util;

public final class MathUtil {

    /**
     * Private constructor to prevent instantiation of this utility class.
     */
    private MathUtil() {}

    /**
     * Clamps a value between a minimum and maximum value.
     *
     * @param value The value to clamp.
     * @param min   The minimum value.
     * @param max   The maximum value.
     * @return The clamped value.
     */
    public static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
}
