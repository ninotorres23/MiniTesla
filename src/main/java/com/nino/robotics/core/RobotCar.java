package com.nino.robotics.core;

import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.simulation.WorldMap;
import com.nino.robotics.util.Config;

import java.util.ArrayList;
import java.util.List;

public class RobotCar {
    private float x, y, angle;
    private final Motor leftMotor, rightMotor;
    private final List<Sensor> sensors;

    // Direct references for convenience
    private final LineSensor leftLineSensor, midLineSensor, rightLineSensor;
    private final UltrasonicSensor ultrasonicSensor;

    public RobotCar(float startX, float startY) {
        this.x = startX;
        this.y = startY;
        this.angle = 90.0f; // Start facing up

        this.leftMotor = new Motor();
        this.rightMotor = new Motor();

        this.sensors = new ArrayList<>();
        this.leftLineSensor = new LineSensor(-Config.LINE_SENSOR_SIDE_OFFSET, Config.LINE_SENSOR_FORWARD_OFFSET);
        this.midLineSensor = new LineSensor(0, Config.LINE_SENSOR_FORWARD_OFFSET);
        this.rightLineSensor = new LineSensor(Config.LINE_SENSOR_SIDE_OFFSET, Config.LINE_SENSOR_FORWARD_OFFSET);
        this.ultrasonicSensor = new UltrasonicSensor(0, Config.ULTRASONIC_SENSOR_FORWARD_OFFSET);

        sensors.add(leftLineSensor);
        sensors.add(midLineSensor);
        sensors.add(rightLineSensor);
        sensors.add(ultrasonicSensor);
    }

    public void update(float delta, WorldMap worldMap) {
        // Differential drive physics
        float leftSpeed = leftMotor.getSpeed() * Config.MAX_SPEED;
        float rightSpeed = rightMotor.getSpeed() * Config.MAX_SPEED;

        float linearVelocity = (leftSpeed + rightSpeed) / 2.0f;
        float angularVelocity = (float) Math.toDegrees((rightSpeed - leftSpeed) / Config.CAR_WIDTH);

        float newAngle = angle + angularVelocity * delta;
        float angleRad = (float) Math.toRadians(angle);

        float newX = x + linearVelocity * (float) Math.cos(angleRad) * delta;
        float newY = y + linearVelocity * (float) Math.sin(angleRad) * delta;

        // Basic collision detection
        Rectangle bounds = getBounds();
        bounds.setPosition(newX - bounds.width / 2, newY - bounds.height / 2);
        if (!worldMap.isColliding(bounds)) {
            this.x = newX;
            this.y = newY;
            this.angle = newAngle;
        }

        // Update all sensors
        for (Sensor sensor : sensors) {
            sensor.update(this, worldMap, delta);
        }
    }

    public void setMotorSpeeds(float left, float right) {
        leftMotor.setSpeed(left);
        rightMotor.setSpeed(right);
    }

    public Vector2 localToWorld(Vector2 localPoint) {
        float angleRad = (float) Math.toRadians(angle);
        float cosA = (float) Math.cos(angleRad);
        float sinA = (float) Math.sin(angleRad);

        float worldX = x + localPoint.x * cosA - localPoint.y * sinA;
        float worldY = y + localPoint.x * sinA + localPoint.y * cosA;

        return new Vector2(worldX, worldY);
    }

    public Rectangle getBounds() {
        return new Rectangle(x - Config.CAR_WIDTH / 2, y - Config.CAR_HEIGHT / 2, Config.CAR_WIDTH, Config.CAR_HEIGHT);
    }

    // Getters
    public float getX() { return x; }
    public float getY() { return y; }
    public float getAngle() { return angle; }
    public void setAngle(float angle) { this.angle = angle; }
    public List<Sensor> getSensors() { return sensors; }
    public LineSensor getLeftLineSensor() { return leftLineSensor; }
    public LineSensor getMidLineSensor() { return midLineSensor; }
    public LineSensor getRightLineSensor() { return rightLineSensor; }
    public UltrasonicSensor getUltrasonicSensor() { return ultrasonicSensor; }
}
