package com.nino.robotics.ai;

import com.nino.robotics.core.RobotCar;
import com.nino.robotics.core.RobotController;
import com.nino.robotics.util.Config;

/**
 * Simple IR-style line follower using weighted position and proportional control.
 * - Proportional steering when line is visible
 * - Heading snap at 90° corners
 * - Stops when line is lost (no search implemented)
 */
public class AutonomousNavigator implements RobotController {

    private enum State {
        LINE_FOLLOWING,
        OBSTACLE_AVOIDANCE
    }

    private enum LineState {
        FOLLOWING,      // Actively following the line
        CORNER_LEFT,    // Detected left corner, snapping heading
        CORNER_RIGHT,   // Detected right corner, snapping heading
        STOPPED         // Line lost, stopped
    }

    private State currentState = State.LINE_FOLLOWING;
    private LineState lineState = LineState.FOLLOWING;

    // ===== TUNABLE CONSTANTS =====
    private static final float BASE_SPEED = 0.45f;           // Base forward speed
    private static final float Kp = 0.40f;                   // Proportional steering gain
    private static final float LINE_THRESHOLD = 0.30f;       // Sensor threshold (0-1)
    private static final int CORNER_DETECT_TICKS = 3;        // Debounce for corner detection
    private static final int POST_CORNER_HOLD_TICKS = 60;    // Hold straight after corner
    private static final int CORNER_REACQUIRE_MAX = 50;      // Max ticks to reacquire after corner snap
    private static final float CORNER_FORWARD_SPEED = 0.18f; // Speed during corner reacquisition

    // ===== STATE VARIABLES =====
    private float lastError = 0.0f;                          // Last known line position error
    private int cornerHoldCounter = 0;                       // Debounce counter for corner detection
    private int postCornerHold = 0;                          // Ticks to hold straight after corner
    private boolean cornerSnapped = false;                   // Whether heading has been snapped
    private int cornerReacquireTicks = 0;                    // Ticks spent trying to reacquire after corner

    @Override
    public void updateControl(RobotCar car, float delta) {
        float ultrasonicDist = car.getUltrasonicSensor().readValue();

        if (ultrasonicDist < Config.ULTRASONIC_STOP_DISTANCE) {
            currentState = State.OBSTACLE_AVOIDANCE;
        } else {
            currentState = State.LINE_FOLLOWING;
        }

        switch (currentState) {
            case LINE_FOLLOWING:
                followLine(car);
                break;
            case OBSTACLE_AVOIDANCE:
                avoidObstacle(car);
                break;
        }
    }

    private void followLine(RobotCar car) {
        // Read sensor values (0.0 = no line, 1.0 = on line)
        float left = car.getLeftLineSensor().readValue();
        float mid = car.getMidLineSensor().readValue();
        float right = car.getRightLineSensor().readValue();

        // Debug output
        System.out.println(String.format("State:%s | L:%.2f M:%.2f R:%.2f", 
                lineState, left, mid, right));

        switch (lineState) {
            case FOLLOWING:
                handleFollowing(car, left, mid, right);
                break;
            case CORNER_LEFT:
                handleCornerLeft(car, left, mid, right);
                break;
            case CORNER_RIGHT:
                handleCornerRight(car, left, mid, right);
                break;
            case STOPPED:
                car.setMotorSpeeds(0, 0);
                System.out.println("STOPPED: Line lost");
                break;
        }
    }

    private void handleFollowing(RobotCar car, float left, float mid, float right) {
        // Post-corner hold: drive straight briefly after completing a corner
        if (postCornerHold > 0) {
            postCornerHold--;
            car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
            System.out.println("Following: post-corner hold (" + postCornerHold + ")");
            return;
        }

        // Calculate weighted line position
        float sum = left + mid + right;

        if (sum < LINE_THRESHOLD) {
            // No line detected -> stop
            lineState = LineState.STOPPED;
            car.setMotorSpeeds(0, 0);
            System.out.println("Following: line lost -> STOPPED");
            return;
        }

        // Check for corner: only one side sensor sees line, mid does not
        boolean leftOnly = (left > LINE_THRESHOLD && mid <= LINE_THRESHOLD && right <= LINE_THRESHOLD);
        boolean rightOnly = (right > LINE_THRESHOLD && mid <= LINE_THRESHOLD && left <= LINE_THRESHOLD);

        if (leftOnly || rightOnly) {
            cornerHoldCounter++;
            if (cornerHoldCounter >= CORNER_DETECT_TICKS) {
                // Corner detected! Transition to corner state
                if (leftOnly) {
                    lineState = LineState.CORNER_RIGHT;  // Left sensor sees line -> turn RIGHT
                    System.out.println("Following: corner detected -> CORNER_RIGHT");
                } else {
                    lineState = LineState.CORNER_LEFT;   // Right sensor sees line -> turn LEFT
                    System.out.println("Following: corner detected -> CORNER_LEFT");
                }
                cornerHoldCounter = 0;
                cornerSnapped = false;
                cornerReacquireTicks = 0;
                return;
            }
        } else {
            cornerHoldCounter = 0;  // Reset debounce if not a clean corner signal
        }

        // Normal proportional control
        // Position: -1.0 (left) to +1.0 (right)
        float position = (left * -1.0f) + (mid * 0.0f) + (right * 1.0f);
        float error = position / sum;
        lastError = error;

        // Proportional steering: error > 0 means line is to the right
        float correction = error * Kp;
        float leftSpeed = BASE_SPEED - correction;
        float rightSpeed = BASE_SPEED + correction;

        car.setMotorSpeeds(leftSpeed, rightSpeed);
        System.out.println(String.format("Following: error=%.2f | L:%.2f R:%.2f", 
                error, leftSpeed, rightSpeed));
    }

    private void handleCornerLeft(RobotCar car, float left, float mid, float right) {
        // Snap heading 90° left (counterclockwise)
        if (!cornerSnapped) {
            float newAngle = car.getAngle() + 90f;
            car.setAngle(newAngle);
            cornerSnapped = true;
            cornerReacquireTicks = 0;
            System.out.println("CORNER_LEFT: snapped heading +90°");
        }

        cornerReacquireTicks++;

        // Check if mid sensor has reacquired the line
        if (mid > LINE_THRESHOLD) {
            lineState = LineState.FOLLOWING;
            postCornerHold = POST_CORNER_HOLD_TICKS;
            cornerHoldCounter = 0;  // Reset to prevent immediate re-detection
            System.out.println("CORNER_LEFT: mid reacquired -> FOLLOWING");
            return;
        }

        // Timeout: if we can't reacquire, stop
        if (cornerReacquireTicks >= CORNER_REACQUIRE_MAX) {
            lineState = LineState.STOPPED;
            car.setMotorSpeeds(0, 0);
            System.out.println("CORNER_LEFT: reacquire timeout -> STOPPED");
            return;
        }

        // Drive straight forward to bring mid sensor over the new line segment
        car.setMotorSpeeds(CORNER_FORWARD_SPEED, CORNER_FORWARD_SPEED);
        System.out.println("CORNER_LEFT: driving forward to reacquire (" + cornerReacquireTicks + ")");
    }

    private void handleCornerRight(RobotCar car, float left, float mid, float right) {
        // Snap heading 90° right (clockwise)
        if (!cornerSnapped) {
            float newAngle = car.getAngle() - 90f;
            car.setAngle(newAngle);
            cornerSnapped = true;
            cornerReacquireTicks = 0;
            System.out.println("CORNER_RIGHT: snapped heading -90°");
        }

        cornerReacquireTicks++;

        // Check if mid sensor has reacquired the line
        if (mid > LINE_THRESHOLD) {
            lineState = LineState.FOLLOWING;
            postCornerHold = POST_CORNER_HOLD_TICKS;
            cornerHoldCounter = 0;  // Reset to prevent immediate re-detection
            System.out.println("CORNER_RIGHT: mid reacquired -> FOLLOWING");
            return;
        }

        // Timeout: if we can't reacquire, stop
        if (cornerReacquireTicks >= CORNER_REACQUIRE_MAX) {
            lineState = LineState.STOPPED;
            car.setMotorSpeeds(0, 0);
            System.out.println("CORNER_RIGHT: reacquire timeout -> STOPPED");
            return;
        }

        // Drive straight forward to bring mid sensor over the new line segment
        car.setMotorSpeeds(CORNER_FORWARD_SPEED, CORNER_FORWARD_SPEED);
        System.out.println("CORNER_RIGHT: driving forward to reacquire (" + cornerReacquireTicks + ")");
    }

    private void avoidObstacle(RobotCar car) {
        // Simple obstacle avoidance: stop
        car.setMotorSpeeds(0, 0);
        System.out.println("OBSTACLE_AVOIDANCE: stopped");
    }
}
