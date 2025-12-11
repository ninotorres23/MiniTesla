package com.nino.robotics.ai;

import com.badlogic.gdx.math.Rectangle;
import com.nino.robotics.core.RobotCar;
import com.nino.robotics.simulation.WorldMap;
import com.nino.robotics.util.Config;

/**
 * Provides obstacle detection and collision prediction for the robot.
 * <p>
 * This system uses the ultrasonic sensor and world map to determine if
 * the robot is about to collide with an obstacle. It can be used by
 * controllers to implement safety behaviors.
 * </p>
 * 
 * <h2>Detection Methods</h2>
 * <ul>
 *   <li>{@link #isCollisionImminent(RobotCar)} - Uses ultrasonic sensor</li>
 *   <li>{@link #isMoveSafe(RobotCar, float, float, float)} - Simulates future position</li>
 * </ul>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see com.nino.robotics.core.UltrasonicSensor
 * @see WorldMap
 */
public class ObstacleAvoidanceSystem {

    /** Reference to the world map for collision checking */
    private final WorldMap worldMap;

    /**
     * Creates an obstacle avoidance system for the given world.
     * 
     * @param worldMap The world map containing obstacles
     */
    public ObstacleAvoidanceSystem(WorldMap worldMap) {
        this.worldMap = worldMap;
    }

    /**
     * Checks if an obstacle is within the stop distance using the ultrasonic sensor.
     * 
     * @param car The robot car to check
     * @return true if an obstacle is within {@link Config#ULTRASONIC_STOP_DISTANCE}
     */
    public boolean isCollisionImminent(RobotCar car) {
        return car.getUltrasonicSensor().readValue() < Config.ULTRASONIC_STOP_DISTANCE;
    }

    /**
     * Simulates a movement and checks if it would result in a collision.
     * <p>
     * This method predicts the robot's future position based on the proposed
     * motor speeds and checks if that position would overlap with any obstacle.
     * </p>
     * 
     * @param car                The robot car
     * @param proposedLeftSpeed  Proposed left motor speed (-1.0 to 1.0)
     * @param proposedRightSpeed Proposed right motor speed (-1.0 to 1.0)
     * @param delta              Time step for the simulation (seconds)
     * @return true if the move is safe (no collision), false otherwise
     */
    public boolean isMoveSafe(RobotCar car, float proposedLeftSpeed, float proposedRightSpeed, float delta) {
        // Simulate one step ahead
        float leftSpeed = proposedLeftSpeed * Config.MAX_SPEED;
        float rightSpeed = proposedRightSpeed * Config.MAX_SPEED;

        float linearVelocity = (leftSpeed + rightSpeed) / 2.0f;
        float angularVelocity = (float) Math.toDegrees((rightSpeed - leftSpeed) / Config.CAR_WIDTH);

        float currentAngle = car.getAngle();
        float angleRad = (float) Math.toRadians(currentAngle);

        float newX = car.getX() + linearVelocity * (float) Math.cos(angleRad) * delta;
        float newY = car.getY() + linearVelocity * (float) Math.sin(angleRad) * delta;

        Rectangle futureBounds = car.getBounds();
        futureBounds.setPosition(newX - futureBounds.width / 2, newY - futureBounds.height / 2);

        return !worldMap.isColliding(futureBounds);
    }
}
