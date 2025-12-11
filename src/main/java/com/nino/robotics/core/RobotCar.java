package com.nino.robotics.core;

import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.simulation.WorldMap;
import com.nino.robotics.util.Config;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a differential-drive robot car with sensors.
 * <p>
 * The robot uses a differential drive system with two independently controlled motors
 * (left and right). Movement is calculated using differential drive kinematics where:
 * <ul>
 *   <li>Linear velocity = average of left and right wheel speeds</li>
 *   <li>Angular velocity = difference between wheel speeds / wheel base width</li>
 * </ul>
 * </p>
 * 
 * <h2>Coordinate System</h2>
 * <ul>
 *   <li><b>Position</b>: (x, y) in world coordinates (meters)</li>
 *   <li><b>Angle</b>: Heading in degrees, where 0° = East (positive X), 90° = North (positive Y)</li>
 *   <li><b>Local Coordinates</b>: X = forward, Y = left relative to robot heading</li>
 * </ul>
 * 
 * <h2>Sensors</h2>
 * <ul>
 *   <li>Three {@link LineSensor}s mounted across the front bumper (left, middle, right)</li>
 *   <li>One {@link UltrasonicSensor} mounted at front center for obstacle detection</li>
 * </ul>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see Motor
 * @see Sensor
 * @see LineSensor
 * @see UltrasonicSensor
 */
public class RobotCar {
    /** Current X position in world coordinates (meters) */
    private float x;
    /** Current Y position in world coordinates (meters) */
    private float y;
    /** Current heading angle in degrees (0 = East, 90 = North) */
    private float angle;
    
    private final Motor leftMotor, rightMotor;
    private final List<Sensor> sensors;

    private final LineSensor leftLineSensor, midLineSensor, rightLineSensor;
    private final UltrasonicSensor ultrasonicSensor;

    /**
     * Creates a new robot car at the specified starting position.
     * <p>
     * The robot is initialized facing North (90°) with all motors stopped.
     * Sensors are automatically configured based on {@link Config} offsets.
     * </p>
     * 
     * @param startX Initial X position in world coordinates (meters)
     * @param startY Initial Y position in world coordinates (meters)
     */
    public RobotCar(float startX, float startY) {
        this.x = startX;
        this.y = startY;
        this.angle = 90.0f; // Start facing North (up)

        this.leftMotor = new Motor();
        this.rightMotor = new Motor();

        this.sensors = new ArrayList<>();
        // Fix: Place line sensors across the front (X-axis) spread along Y-axis
        // Left is at Y=+Side, Right is at Y=-Side
        this.leftLineSensor = new LineSensor(Config.LINE_SENSOR_FORWARD_OFFSET, Config.LINE_SENSOR_SIDE_OFFSET);
        this.midLineSensor = new LineSensor(Config.LINE_SENSOR_FORWARD_OFFSET, 0);
        this.rightLineSensor = new LineSensor(Config.LINE_SENSOR_FORWARD_OFFSET, -Config.LINE_SENSOR_SIDE_OFFSET);
        
        // Fix: Place ultrasonic sensor on X-axis (forward motion axis) instead of Y-axis
        // This puts it at the front center of the car relative to its movement
        this.ultrasonicSensor = new UltrasonicSensor(Config.ULTRASONIC_SENSOR_FORWARD_OFFSET, 0);

        sensors.add(leftLineSensor);
        sensors.add(midLineSensor);
        sensors.add(rightLineSensor);
        sensors.add(ultrasonicSensor);
    }

    /**
     * Updates the robot's position, orientation, and sensors.
     * <p>
     * This method performs the following steps:
     * <ol>
     *   <li>Calculate linear and angular velocities from motor speeds</li>
     *   <li>Compute new position using differential drive kinematics</li>
     *   <li>Check for collisions with obstacles</li>
     *   <li>Update all sensor readings</li>
     * </ol>
     * </p>
     * 
     * @param delta    Time elapsed since last update (seconds)
     * @param worldMap The world map for collision detection and sensor updates
     */
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

    /**
     * Sets the speed of both motors.
     * 
     * @param left  Left motor speed (-1.0 to 1.0)
     * @param right Right motor speed (-1.0 to 1.0)
     */
    public void setMotorSpeeds(float left, float right) {
        leftMotor.setSpeed(left);
        rightMotor.setSpeed(right);
    }

    /**
     * Transforms a point from local (robot-relative) coordinates to world coordinates.
     * <p>
     * In local coordinates:
     * <ul>
     *   <li>X-axis points forward (in the direction of robot heading)</li>
     *   <li>Y-axis points left (perpendicular to heading)</li>
     * </ul>
     * </p>
     * 
     * @param localPoint Point in local coordinates
     * @return The same point transformed to world coordinates
     */
    public Vector2 localToWorld(Vector2 localPoint) {
        float angleRad = (float) Math.toRadians(angle);
        float cosA = (float) Math.cos(angleRad);
        float sinA = (float) Math.sin(angleRad);

        float worldX = x + localPoint.x * cosA - localPoint.y * sinA;
        float worldY = y + localPoint.x * sinA + localPoint.y * cosA;

        return new Vector2(worldX, worldY);
    }

    /**
     * Gets the axis-aligned bounding box of the robot.
     * 
     * @return Rectangle representing the robot's bounds in world coordinates
     */
    public Rectangle getBounds() {
        return new Rectangle(x - Config.CAR_WIDTH / 2, y - Config.CAR_HEIGHT / 2, Config.CAR_WIDTH, Config.CAR_HEIGHT);
    }

    // ==================== Getters and Setters ====================
    
    /** @return Current X position in world coordinates (meters) */
    public float getX() { return x; }
    
    /** @return Current Y position in world coordinates (meters) */
    public float getY() { return y; }
    
    /** @return Current heading angle in degrees */
    public float getAngle() { return angle; }
    
    /** @param angle New heading angle in degrees */
    public void setAngle(float angle) { this.angle = angle; }
    
    /** @return List of all sensors attached to the robot */
    public List<Sensor> getSensors() { return sensors; }
    
    /** @return The left line sensor */
    public LineSensor getLeftLineSensor() { return leftLineSensor; }
    
    /** @return The middle line sensor */
    public LineSensor getMidLineSensor() { return midLineSensor; }
    
    /** @return The right line sensor */
    public LineSensor getRightLineSensor() { return rightLineSensor; }
    
    /** @return The ultrasonic distance sensor */
    public UltrasonicSensor getUltrasonicSensor() { return ultrasonicSensor; }
}
