package com.nino.robotics.ai;

import com.nino.robotics.core.RobotCar;
import com.nino.robotics.core.RobotController;
import com.nino.robotics.util.Config;

/**
 * Autonomous line-following controller for the robot car.
 * <p>
 * This controller implements an IR-style line following algorithm using three line sensors
 * (left, middle, right) mounted across the front of the robot. It uses a hybrid approach:
 * <ul>
 *   <li><b>PD Control</b>: For smooth steering along straight segments</li>
 *   <li><b>Heading Snap</b>: For 90-degree corner turns</li>
 * </ul>
 * </p>
 * 
 * <h2>Algorithm Overview</h2>
 * <ol>
 *   <li><b>Corner Detection</b>: When only a side sensor detects the line (mid sensor off),
 *       a corner is detected. After confirmation time, the robot snaps its heading ±90°.</li>
 *   <li><b>Corner Reacquisition</b>: After snapping, the robot drives forward until any
 *       sensor reacquires the line.</li>
 *   <li><b>PD Steering</b>: During normal operation, a dual-zone PD controller provides
 *       smooth corrections. Fine control when centered, coarse control when drifting.</li>
 *   <li><b>End Detection</b>: When all sensors lose the line for a sustained period,
 *       the robot stops (end of track).</li>
 * </ol>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see RobotController
 * @see RobotCar
 */
public class AutonomousNavigator implements RobotController {

    /**
     * Internal state machine for the autonomous navigator.
     */
    private enum State {
        /** Normal line following mode using PD control */
        LINE_FOLLOWING,
        /** Obstacle detected - currently stops (reserved for future enhancement) */
        OBSTACLE_AVOIDANCE
    }

    private State currentState = State.LINE_FOLLOWING;
    
    // ==================== PD Controller Configuration ====================
    /** Base forward speed in meters per second */
    private static final float BASE_SPEED = 0.3f;
    /** Proportional gain for steering correction (lower = smoother) */
    private static final float KP = 0.6f;
    /** Derivative gain for damping oscillations */
    private static final float KD = 0.6f;
    /** Minimum sensor value to consider "on the line" */
    private static final float LINE_THRESHOLD = 0.3f;
    
    // ==================== Corner Detection Configuration ====================
    /** Time in seconds a corner pattern must persist before triggering a turn */
    private static final float CORNER_DETECT_TIME = 0.08f;
    /** Time to hold straight after completing a corner turn */
    private static final float POST_CORNER_HOLD_TIME = 0.5f;
    /** Speed during corner reacquisition phase */
    private static final float CORNER_FORWARD_SPEED = 0.3f;
    /** Maximum time to search for line after a corner before giving up */
    private static final float CORNER_REACQUIRE_TIMEOUT = 2.0f;
    /** Time with no line detected before stopping (end of track) */
    private static final float END_OF_LINE_TIME = 0.3f;
    
    // ==================== State Variables ====================
    private float lastError = 0;
    private float noLineTimer = 0;
    private boolean reachedEnd = false;
    private float cornerDetectTimer = 0;
    private int cornerDetectSide = 0;
    private float postCornerHoldTimer = 0;
    private boolean inCornerReacquire = false;
    private int cornerDirection = 0;
    private float cornerReacquireTimer = 0;

    /**
     * {@inheritDoc}
     * <p>
     * In autonomous mode, this method delegates entirely to the line-following algorithm.
     * Ultrasonic obstacle detection is disabled in this mode.
     * </p>
     */
    @Override
    public void updateControl(RobotCar car, float delta) {
        followLine(car, delta);
    }

    /**
     * Main line-following algorithm.
     * <p>
     * Implements a state machine that handles:
     * <ul>
     *   <li>End-of-line detection and stopping</li>
     *   <li>Corner reacquisition after a 90° snap turn</li>
     *   <li>Post-corner stabilization hold</li>
     *   <li>Corner detection and heading snap</li>
     *   <li>Normal PD-based steering</li>
     * </ul>
     * </p>
     * 
     * @param car   The robot car to control
     * @param delta Time elapsed since last update (seconds)
     */
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
        // After snapping heading, drive forward until ANY sensor finds the line
        if (inCornerReacquire) {
            cornerReacquireTimer += delta;
            
            // Check if ANY sensor sees the line
            if (mid > LINE_THRESHOLD || left > LINE_THRESHOLD || right > LINE_THRESHOLD) {
                // Sensor found the line - exit corner mode
                inCornerReacquire = false;
                postCornerHoldTimer = POST_CORNER_HOLD_TIME;
                cornerDirection = 0;
                cornerReacquireTimer = 0;
                lastError = 0;  // Reset PD state
                car.setMotorSpeeds(BASE_SPEED, BASE_SPEED);
                System.out.println("CORNER: line reacquired -> resuming PD control");
                return;
            }
            
            // Timeout - give up and try to resume normal control
            if (cornerReacquireTimer > CORNER_REACQUIRE_TIMEOUT) {
                inCornerReacquire = false;
                cornerReacquireTimer = 0;
                System.out.println("CORNER: TIMEOUT - resuming normal control");
                // Don't return, let it fall through to normal control
            } else {
                // Keep driving forward to find the line
                car.setMotorSpeeds(CORNER_FORWARD_SPEED, CORNER_FORWARD_SPEED);
                System.out.println(String.format("CORNER: driving forward to reacquire... (%.2fs)", cornerReacquireTimer));
                return;
            }
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
        
        // === SMOOTH STEERING LOGIC ===
        float error = 0;
        
        if (mid > LINE_THRESHOLD) {
            // FINE CONTROL ZONE: Middle sensor is safe. Be gentle.
            if (left > LINE_THRESHOLD) {
                // Slightly left of center -> Gentle nudge LEFT (Positive error)
                error = 0.3f; 
            } else if (right > LINE_THRESHOLD) {
                // Slightly right of center -> Gentle nudge RIGHT (Negative error)
                error = -0.3f;
            } else {
                // Perfectly centered -> Drive straight
                error = 0.0f;
            }
        } else {
            // COARSE CONTROL ZONE: Middle sensor lost. We are drifting!
            if (left > LINE_THRESHOLD) {
                // Hard turn LEFT (Positive error)
                error = 1.0f;
            } else if (right > LINE_THRESHOLD) {
                // Hard turn RIGHT (Negative error)
                error = -1.0f;
            } else {
                // All sensors lost - use last known direction
                error = (lastError > 0) ? 1.0f : -1.0f;
            }
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
