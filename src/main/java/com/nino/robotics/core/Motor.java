package com.nino.robotics.core;

import com.nino.robotics.util.MathUtil;

/**
 * Represents a single motor in the differential drive system.
 * <p>
 * Each motor has a speed value ranging from -1.0 (full reverse) to 1.0 (full forward).
 * The actual velocity is calculated by multiplying this value by {@link com.nino.robotics.util.Config#MAX_SPEED}.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see RobotCar
 */
public class Motor {
    /** Current speed setting (-1.0 to 1.0) */
    private float speed;

    /**
     * Creates a motor with initial speed of 0 (stopped).
     */
    public Motor() {
        this.speed = 0.0f;
    }

    /**
     * Gets the current motor speed.
     * 
     * @return Speed value from -1.0 (full reverse) to 1.0 (full forward)
     */
    public float getSpeed() {
        return speed;
    }

    /**
     * Sets the motor speed.
     * <p>
     * Values are automatically clamped to the range [-1.0, 1.0].
     * </p>
     * 
     * @param speed Desired speed (-1.0 to 1.0)
     */
    public void setSpeed(float speed) {
        this.speed = MathUtil.clamp(speed, -1.0f, 1.0f);
    }
}
