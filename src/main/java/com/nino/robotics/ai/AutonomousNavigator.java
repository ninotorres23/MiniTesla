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
    private float lastError = 0;

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
        // Read sensor values (0-1, where 1 means on the line)
        float left = car.getLeftLineSensor().readValue();
        float mid = car.getMidLineSensor().readValue();
        float right = car.getRightLineSensor().readValue();
        
        // Calculate position error (-1.0 to 1.0, where 0 is centered)
        float error = 0;
        float total = left + mid + right;
        
        if (total > 0.1f) {  // If we see any line
            error = (-left + right) / total;  // Range from -1 to 1
        } else {
            // If we lose the line, use the last error to keep turning
            error = (lastError > 0) ? 1.0f : -1.0f;
        }
        
        // Calculate derivative (change in error)
        float derivative = error - lastError;
        lastError = error;
        
        // Calculate motor speeds using PD control
        float turn = KP * error + KD * derivative;
        float leftSpeed = BASE_SPEED - turn;
        float rightSpeed = BASE_SPEED + turn;
        
        // Limit speeds to valid range
        leftSpeed = Math.max(-1, Math.min(1, leftSpeed));
        rightSpeed = Math.max(-1, Math.min(1, rightSpeed));
        
        // Debug output
        System.out.println(String.format("L:%.2f M:%.2f R:%.2f | Err:%.2f | L:%.2f R:%.2f", 
            left, mid, right, error, leftSpeed, rightSpeed));
            
        // Set motor speeds
        car.setMotorSpeeds(leftSpeed, rightSpeed);
        
        // Small delay to prevent the method from being called too frequently
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void avoidObstacle(RobotCar car) {
        // Simple obstacle avoidance - stop
        car.setMotorSpeeds(0, 0);
        // More advanced: could turn left or right
    }
}
