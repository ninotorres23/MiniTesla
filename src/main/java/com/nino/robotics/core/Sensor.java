package com.nino.robotics.core;

import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.simulation.WorldMap;

public abstract class Sensor {
    protected float value;
    protected Vector2 relativePosition; // Position relative to the car's center

    public Sensor(float offsetX, float offsetY) {
        this.relativePosition = new Vector2(offsetX, offsetY);
        this.value = 0;
    }

    /**
     * Updates the sensor's state based on the car's position and the world.
     * @param car The robot car instance.
     * @param worldMap The map of the world.
     * @param delta The time since the last frame.
     */
    public abstract void update(RobotCar car, WorldMap worldMap, float delta);

    /**
     * Returns the last read value of the sensor.
     * @return The sensor value.
     */
    public float readValue() {
        return value;
    }

    public Vector2 getRelativePosition() {
        return relativePosition;
    }
}
