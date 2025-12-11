package com.nino.robotics.core;

import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.simulation.Obstacle;
import com.nino.robotics.simulation.WorldMap;
import com.nino.robotics.util.Config;

/**
 * An ultrasonic distance sensor that measures distance to obstacles.
 * <p>
 * This sensor simulates an HC-SR04 ultrasonic sensor by casting multiple rays
 * in a cone pattern and returning the distance to the nearest obstacle.
 * </p>
 * 
 * <h2>Detection Method</h2>
 * <p>
 * The sensor casts 3 rays (center, left edge, right edge of cone) and returns
 * the minimum distance found. This simulates the cone-shaped detection pattern
 * of real ultrasonic sensors.
 * </p>
 * 
 * <h2>Return Values</h2>
 * <ul>
 *   <li><b>0 to MAX_DISTANCE</b>: Distance to nearest obstacle in meters</li>
 *   <li><b>MAX_DISTANCE</b>: No obstacle detected within range</li>
 * </ul>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see Sensor
 * @see Config#ULTRASONIC_MAX_DISTANCE
 * @see Config#ULTRASONIC_CONE_ANGLE
 */
public class UltrasonicSensor extends Sensor {

    /**
     * Creates an ultrasonic sensor at the specified offset from the robot center.
     * <p>
     * Initial value is set to {@link Config#ULTRASONIC_MAX_DISTANCE} (no obstacle).
     * </p>
     * 
     * @param offsetX Forward offset (positive = front)
     * @param offsetY Lateral offset (positive = left)
     */
    public UltrasonicSensor(float offsetX, float offsetY) {
        super(offsetX, offsetY);
        this.value = Config.ULTRASONIC_MAX_DISTANCE;
    }

    /**
     * {@inheritDoc}
     * <p>
     * Casts 3 rays in a cone pattern (center, left, right) and finds the
     * closest intersection with any obstacle. The minimum distance is stored
     * as the sensor value.
     * </p>
     */
    @Override
    public void update(RobotCar car, WorldMap worldMap, float delta) {
        Vector2 sensorPos = car.localToWorld(relativePosition);
        float carAngleRad = (float) Math.toRadians(car.getAngle());
        float halfConeRad = (float) Math.toRadians(Config.ULTRASONIC_CONE_ANGLE / 2);

        float minDistance = Config.ULTRASONIC_MAX_DISTANCE;

        // Cast 3 rays: Center, Left, Right to simulate a cone
        float[] angles = {0, -halfConeRad, halfConeRad};

        for (float angleOffset : angles) {
            float rayAngle = carAngleRad + angleOffset;
            
            Vector2 rayEnd = new Vector2(
                (float) (sensorPos.x + Config.ULTRASONIC_MAX_DISTANCE * Math.cos(rayAngle)),
                (float) (sensorPos.y + Config.ULTRASONIC_MAX_DISTANCE * Math.sin(rayAngle))
            );

            for (Obstacle obstacle : worldMap.getObstacles()) {
                Vector2 intersection = obstacle.getIntersection(sensorPos, rayEnd);
                if (intersection != null) {
                    float distance = sensorPos.dst(intersection);
                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                }
            }
        }
        
        this.value = minDistance;
    }
}
