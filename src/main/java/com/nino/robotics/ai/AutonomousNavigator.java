package com.nino.robotics.ai;

import com.nino.robotics.core.RobotCar;
import com.nino.robotics.core.RobotController;
import com.nino.robotics.util.Config;

public class AutonomousNavigator implements RobotController {

    private enum State {
        LINE_FOLLOWING,
        OBSTACLE_AVOIDANCE
    }

    private State currentState = State.LINE_FOLLOWING;
    
    // PD Controller Configuration
    private static final float BASE_SPEED = 0.3f;
    private static final float KP = 0.8f;  // Proportional gain
    private static final float KD = 0.2f;  // Derivative gain
    private static final float LINE_THRESHOLD = 0.3f;  // Sensor detection threshold
    
    // Corner detection
    private static final float CORNER_DETECT_TIME = 0.08f;  // Time (seconds) to confirm corner
    private static final float POST_CORNER_HOLD_TIME = 0.6f; // Time to hold straight after corner
    private static final float CORNER_FORWARD_SPEED = 0.15f; // Speed during corner reacquisition
    private static final float CORNER_REACQUIRE_TIMEOUT = 2.0f; // Max time to reacquire before giving up
    private static final float END_OF_LINE_TIME = 0.3f; // Time with no line before stopping (end of track)
    
    private float lastError = 0;
    private float noLineTimer = 0;  // Time with no line detected
    private boolean reachedEnd = false;  // True when robot has stopped at end of line
    private float cornerDetectTimer = 0;      // Accumulates time when corner pattern detected
    private int cornerDetectSide = 0;         // Which side triggered: -1=left, +1=right, 0=none
    private float postCornerHoldTimer = 0;    // Time remaining to hold straight after corner
    private boolean inCornerReacquire = false; // True when snapped and driving forward to reacquire
    private int cornerDirection = 0;          // -1 = left turn, +1 = right turn, 0 = none
    private float cornerReacquireTimer = 0;   // Time spent trying to reacquire

    @Override
    public void updateControl(RobotCar car, float delta) {
        // Ultrasonic disabled in autonomous mode - only line following
        followLine(car, delta);
    }

    private void followLine(RobotCar car, float delta) {
        // Read sensor values (0-1, where 1 means on the line)
        float left = car.getLeftLineSensor().readValue();
        float mid = car.getMidLineSensor().readValue();
        float right = car.getRightLineSensor().readValue();
        
        // Debug output
        System.out.println(String.format("L:%.2f M:%.2f R:%.2f | Corner:%s Hold:%.2f", 
            left, mid, right, inCornerReacquire ? (cornerDirection < 0 ? "LEFT" : "RIGHT") : "NO", postCornerHoldTimer));
        
        // === END OF LINE CHECK ===
        // If robot has reached the end, stay stopped
        if (reachedEnd) {
            car.setMotorSpeeds(0, 0);
            return;
        }
        
        // Check if all sensors see no line (potential end of track)
        float total = left + mid + right;
        if (total < 0.1f && !inCornerReacquire) {
            noLineTimer += delta;
            if (noLineTimer >= END_OF_LINE_TIME) {
                reachedEnd = true;
                car.setMotorSpeeds(0, 0);
                System.out.println("END OF LINE: Robot stopped at end of track");
                return;
            }
        } else {
            noLineTimer = 0;  // Reset if any sensor sees line
        }
        
        // === CORNER REACQUISITION MODE ===
        // After snapping heading, drive forward until mid sensor finds the line
        if (inCornerReacquire) {
            cornerReacquireTimer += delta;
            
            if (mid > LINE_THRESHOLD) {
                // Mid sensor found the line - exit corner mode
                inCornerReacquire = false;
                postCornerHoldTimer = POST_CORNER_HOLD_TIME;
                cornerDirection = 0;
                cornerReacquireTimer = 0;
                lastError = 0;  // Reset PD state
                car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
                System.out.println("CORNER: mid reacquired -> resuming PD control");
                return;
            }
            
            // Timeout - stop if we can't find the line
            if (cornerReacquireTimer > CORNER_REACQUIRE_TIMEOUT) {
                inCornerReacquire = false;
                cornerReacquireTimer = 0;
                car.setMotorSpeeds(0, 0);
                System.out.println("CORNER: TIMEOUT - could not reacquire line, stopping");
                return;
            }
            
            // Keep driving forward slowly to find the line
            car.setMotorSpeeds(CORNER_FORWARD_SPEED, CORNER_FORWARD_SPEED);
            System.out.println(String.format("CORNER: driving forward to reacquire... (%.2fs)", cornerReacquireTimer));
            return;
        }
        
        // === POST-CORNER HOLD ===
        // After completing a corner, hold straight briefly to stabilize
        if (postCornerHoldTimer > 0) {
            postCornerHoldTimer -= delta;
            car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
            System.out.println("POST-CORNER: holding straight");
            return;
        }
        
        // === CORNER DETECTION ===
        // Corner pattern: only one side sensor sees the line, mid does not
        boolean leftOnly = (left > LINE_THRESHOLD && mid <= LINE_THRESHOLD && right <= LINE_THRESHOLD);
        boolean rightOnly = (right > LINE_THRESHOLD && mid <= LINE_THRESHOLD && left <= LINE_THRESHOLD);
        
        if (leftOnly || rightOnly) {
            int currentSide = leftOnly ? -1 : 1;
            
            // If the detecting side changed, reset the timer (robot is oscillating)
            if (cornerDetectSide != currentSide) {
                cornerDetectTimer = 0;
                cornerDetectSide = currentSide;
            }
            
            cornerDetectTimer += delta;
            System.out.println(String.format("CORNER DETECT: side=%s timer=%.3f angle=%.1f", 
                currentSide < 0 ? "LEFT" : "RIGHT", cornerDetectTimer, car.getAngle()));
            
            if (cornerDetectTimer >= CORNER_DETECT_TIME) {
                // Corner confirmed! Snap heading 90 degrees based on which side was CONSISTENTLY detected
                if (cornerDetectSide < 0) {
                    // Left sensor sees line -> new path is to our left -> turn LEFT
                    float newAngle = car.getAngle() + 90f;
                    car.setAngle(newAngle);
                    cornerDirection = -1;
                    System.out.println("CORNER: LEFT confirmed, turning LEFT, snapped +90° to " + newAngle);
                } else {
                    // Right sensor sees line -> new path is to our right -> turn RIGHT  
                    float newAngle = car.getAngle() - 90f;
                    car.setAngle(newAngle);
                    cornerDirection = 1;
                    System.out.println("CORNER: RIGHT confirmed, turning RIGHT, snapped -90° to " + newAngle);
                }
                inCornerReacquire = true;
                cornerDetectTimer = 0;
                cornerDetectSide = 0;
                cornerReacquireTimer = 0;
                return;
            }
        } else {
            cornerDetectTimer = 0;
            cornerDetectSide = 0;
        }
        
        // === NORMAL PD CONTROL ===
        float error = 0;
        float PDtotal = left + mid + right;
        
        if (PDtotal > 0.1f) {
            // Calculate weighted position error
            error = (-left + right) / PDtotal;
        } else {
            // Line lost - use last error direction
            error = (lastError > 0) ? 1.0f : -1.0f;
        }
        
        // PD control
        float derivative = error - lastError;
        lastError = error;
        
        float turn = KP * error + KD * derivative;
        float leftSpeed = BASE_SPEED - turn;
        float rightSpeed = BASE_SPEED + turn;
        
        // Clamp speeds
        leftSpeed = Math.max(-1, Math.min(1, leftSpeed));
        rightSpeed = Math.max(-1, Math.min(1, rightSpeed));
        
        car.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    private void avoidObstacle(RobotCar car) {
        // Simple obstacle avoidance - stop
        car.setMotorSpeeds(0, 0);
        // More advanced: could turn left or right
    }
}
