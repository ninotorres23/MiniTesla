package com.nino.robotics.core;

import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.simulation.WorldMap;

public class LineSensor extends Sensor {

    public LineSensor(float offsetX, float offsetY) {
        super(offsetX, offsetY);
    }

    @Override
    public void update(RobotCar car, WorldMap worldMap, float delta) {
        // Get the sensor's world position
        Vector2 worldPosition = car.localToWorld(relativePosition);

        // Check if this position is on the line
        if (worldMap.isOnLine(worldPosition.x, worldPosition.y)) {
            this.value = 1.0f; // On the line
        } else {
            this.value = 0.0f; // Off the line
        }
    }
}
