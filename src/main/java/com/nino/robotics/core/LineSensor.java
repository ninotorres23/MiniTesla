package com.nino.robotics.core;

import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.simulation.WorldMap;

/**
 * A binary line-detection sensor that detects if it is positioned over a line.
 * <p>
 * This sensor simulates an IR reflectance sensor commonly used in line-following robots.
 * It returns a binary value:
 * <ul>
 *   <li><b>1.0</b> - Sensor is over the line</li>
 *   <li><b>0.0</b> - Sensor is not over the line</li>
 * </ul>
 * </p>
 * 
 * <h2>Usage</h2>
 * <p>
 * Three line sensors are typically mounted across the front of the robot
 * (left, middle, right) to enable line following and corner detection.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see Sensor
 * @see RobotCar
 */
public class LineSensor extends Sensor {

    /**
     * Creates a line sensor at the specified offset from the robot center.
     * 
     * @param offsetX Forward offset (positive = front)
     * @param offsetY Lateral offset (positive = left)
     */
    public LineSensor(float offsetX, float offsetY) {
        super(offsetX, offsetY);
    }

    /**
     * {@inheritDoc}
     * <p>
     * Checks if the sensor's world position is over the line path.
     * Sets value to 1.0 if on line, 0.0 otherwise.
     * </p>
     */
    @Override
    public void update(RobotCar car, WorldMap worldMap, float delta) {
        Vector2 worldPosition = car.localToWorld(relativePosition);

        if (worldMap.isOnLine(worldPosition.x, worldPosition.y)) {
            this.value = 1.0f;
        } else {
            this.value = 0.0f;
        }
    }
}
