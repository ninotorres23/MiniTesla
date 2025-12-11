package com.nino.robotics.util;

import com.badlogic.gdx.Input;

/**
 * Global configuration constants for the robot simulation.
 * <p>
 * This class contains all tunable parameters for the simulation including:
 * <ul>
 *   <li>Window and world dimensions</li>
 *   <li>Robot physical properties</li>
 *   <li>Sensor configurations</li>
 *   <li>Keyboard mappings</li>
 * </ul>
 * </p>
 * 
 * <h2>Units</h2>
 * <ul>
 *   <li>Distances are in <b>meters</b></li>
 *   <li>Angles are in <b>degrees</b></li>
 *   <li>Speeds are in <b>meters per second</b></li>
 * </ul>
 * 
 * @author Nino Torres
 * @version 1.0
 */
public class Config {

    // ==================== Window Configuration ====================
    /** Window width in pixels */
    public static final int WINDOW_WIDTH = 1280;
    /** Window height in pixels */
    public static final int WINDOW_HEIGHT = 720;

    // ==================== World Configuration ====================
    /** World width in meters */
    public static final float WORLD_WIDTH = 12.8f;
    /** World height in meters */
    public static final float WORLD_HEIGHT = 7.2f;

    // ==================== Robot Physical Properties ====================
    /** Robot width (wheel base) in meters */
    public static final float CAR_WIDTH = 0.3f;
    /** Robot length in meters */
    public static final float CAR_HEIGHT = 0.4f;
    /** Maximum linear speed in meters per second */
    public static final float MAX_SPEED = 2.0f;
    /** Maximum angular speed in degrees per second */
    public static final float MAX_ANGULAR_SPEED = 180.0f;

    // ==================== Line Sensor Configuration ====================
    /** Forward offset of line sensors from robot center (meters) */
    public static final float LINE_SENSOR_FORWARD_OFFSET = 0.15f;
    /** Lateral offset of left/right line sensors from center (meters) */
    public static final float LINE_SENSOR_SIDE_OFFSET = 0.1f;

    // ==================== Ultrasonic Sensor Configuration ====================
    /** Forward offset of ultrasonic sensor from robot center (meters) */
    public static final float ULTRASONIC_SENSOR_FORWARD_OFFSET = 0.2f;
    /** Maximum detection range of ultrasonic sensor (meters) */
    public static final float ULTRASONIC_MAX_DISTANCE = 4.0f;
    /** Detection cone angle of ultrasonic sensor (degrees) */
    public static final float ULTRASONIC_CONE_ANGLE = 30.0f;
    /** Distance at which to trigger collision avoidance (meters) */
    public static final float ULTRASONIC_STOP_DISTANCE = 0.4f;

    // ==================== Line Path Configuration ====================
    /** Thickness of the line path for detection (meters) */
    public static final float LINE_THICKNESS = 0.1f;

    // ==================== Keyboard Mappings ====================
    /** Key to drive forward */
    public static final int KEY_FORWARD = Input.Keys.W;
    /** Key to drive backward */
    public static final int KEY_BACKWARD = Input.Keys.S;
    /** Key to turn left */
    public static final int KEY_LEFT = Input.Keys.A;
    /** Key to turn right */
    public static final int KEY_RIGHT = Input.Keys.D;
    /** Key to switch to autonomous mode */
    public static final int KEY_MODE_AUTO = Input.Keys.NUM_1;
    /** Key to switch to manual mode */
    public static final int KEY_MODE_MANUAL = Input.Keys.NUM_2;
}
