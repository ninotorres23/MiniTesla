package com.nino.robotics.util;

import com.badlogic.gdx.Input;

public class Config {

    // Window dimensions
    public static final int WINDOW_WIDTH = 1280;
    public static final int WINDOW_HEIGHT = 720;

    // World size
    public static final float WORLD_WIDTH = 12.8f; // in meters
    public static final float WORLD_HEIGHT = 7.2f; // in meters

    // Robot Car dimensions
    public static final float CAR_WIDTH = 0.3f; // meters
    public static final float CAR_HEIGHT = 0.4f; // meters
    public static final float MAX_SPEED = 2.0f; // meters per second
    public static final float MAX_ANGULAR_SPEED = 180.0f; // degrees per second

    // Sensor offsets (from car center)
    public static final float LINE_SENSOR_FORWARD_OFFSET = 0.15f;
    public static final float LINE_SENSOR_SIDE_OFFSET = 0.1f;
    public static final float ULTRASONIC_SENSOR_FORWARD_OFFSET = 0.2f;

    // Ultrasonic Sensor properties
    public static final float ULTRASONIC_MAX_DISTANCE = 4.0f; // meters (max detection range)
    public static final float ULTRASONIC_CONE_ANGLE = 30.0f; // degrees
    public static final float ULTRASONIC_STOP_DISTANCE = 0.4f; // meters (~40 pixels) - stop safely before contact

    // Line properties
    public static final float LINE_THICKNESS = 0.1f; // meters

    // Keyboard mappings
    public static final int KEY_FORWARD = Input.Keys.W;
    public static final int KEY_BACKWARD = Input.Keys.S;
    public static final int KEY_LEFT = Input.Keys.A;
    public static final int KEY_RIGHT = Input.Keys.D;

    public static final int KEY_MODE_AUTO = Input.Keys.NUM_1;
    public static final int KEY_MODE_MANUAL = Input.Keys.NUM_2;
}
