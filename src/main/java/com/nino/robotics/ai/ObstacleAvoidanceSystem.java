package com.nino.robotics.ai;

import com.badlogic.gdx.math.Rectangle;
import com.nino.robotics.core.RobotCar;
import com.nino.robotics.simulation.WorldMap;
import com.nino.robotics.util.Config;

public class ObstacleAvoidanceSystem {

    private final WorldMap worldMap;

    public ObstacleAvoidanceSystem(WorldMap worldMap) {
        this.worldMap = worldMap;
    }

    /**
     * Checks if moving forward would result in a collision.
     * @param car The robot car.
     * @return True if a collision is imminent, false otherwise.
     */
    public boolean isCollisionImminent(RobotCar car) {
        return car.getUltrasonicSensor().readValue() < Config.ULTRASONIC_STOP_DISTANCE;
    }

    /**
     * Checks if a proposed movement is safe.
     * @param car The robot car.
     * @param proposedLeftSpeed The proposed left motor speed.
     * @param proposedRightSpeed The proposed right motor speed.
     * @param delta The time step for the simulation.
     * @return True if the move is safe, false otherwise.
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
