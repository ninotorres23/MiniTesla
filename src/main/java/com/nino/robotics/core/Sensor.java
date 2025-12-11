package com.nino.robotics.core;

import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.simulation.WorldMap;

/**
 * Abstract base class for all robot sensors.
 * <p>
 * Sensors are mounted at a fixed position relative to the robot's center.
 * Each sensor type implements its own detection logic in the {@link #update} method.
 * </p>
 * 
 * <h2>Coordinate System</h2>
 * <p>
 * Sensor positions are specified in local (robot-relative) coordinates:
 * <ul>
 *   <li><b>X-axis</b>: Forward direction (positive = front of robot)</li>
 *   <li><b>Y-axis</b>: Left direction (positive = left side of robot)</li>
 * </ul>
 * </p>
 * 
 * <h2>Subclasses</h2>
 * <ul>
 *   <li>{@link LineSensor} - Detects if the sensor is over a line</li>
 *   <li>{@link UltrasonicSensor} - Measures distance to obstacles</li>
 * </ul>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see LineSensor
 * @see UltrasonicSensor
 */
public abstract class Sensor {
    /** Current sensor reading value */
    protected float value;
    /** Position of sensor relative to robot center (local coordinates) */
    protected Vector2 relativePosition;

    /**
     * Creates a sensor at the specified offset from the robot center.
     * 
     * @param offsetX Forward offset (positive = front)
     * @param offsetY Lateral offset (positive = left)
     */
    public Sensor(float offsetX, float offsetY) {
        this.relativePosition = new Vector2(offsetX, offsetY);
        this.value = 0;
    }

    /**
     * Updates the sensor's reading based on the robot's position and the world state.
     * <p>
     * Called once per simulation frame. Implementations should update the
     * {@link #value} field with the current sensor reading.
     * </p>
     * 
     * @param car      The robot car this sensor is attached to
     * @param worldMap The world map containing lines and obstacles
     * @param delta    Time since the last frame (seconds)
     */
    public abstract void update(RobotCar car, WorldMap worldMap, float delta);

    /**
     * Returns the most recent sensor reading.
     * 
     * @return The sensor value (interpretation depends on sensor type)
     */
    public float readValue() {
        return value;
    }

    /**
     * Gets the sensor's position relative to the robot center.
     * 
     * @return Position in local coordinates
     */
    public Vector2 getRelativePosition() {
        return relativePosition;
    }
}
