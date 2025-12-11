package com.nino.robotics.input;

import com.badlogic.gdx.Gdx;
import com.nino.robotics.ai.ObstacleAvoidanceSystem;
import com.nino.robotics.core.RobotCar;
import com.nino.robotics.core.RobotController;
import com.nino.robotics.util.Config;

public class ManualController implements RobotController {

    private ObstacleAvoidanceSystem avoidanceSystem;
    
    // Ultrasonic obstacle detection state
    private boolean inRecovery = false;   // True when actively reversing away from obstacle
    private float reverseTimer = 0;
    
    // Constants for bounce-back behavior
    private static final float REVERSE_DURATION = 0.6f;     // Increased reverse time
    private static final float REVERSE_SPEED = 0.5f;        // Increased reverse speed
    private static final float RECOVERY_THRESHOLD = 0.35f;  // Must back up to this distance to clear flag (hysteresis)

    // The avoidance system will be injected later.
    public ManualController() {}

    public void setAvoidanceSystem(ObstacleAvoidanceSystem avoidanceSystem) {
        this.avoidanceSystem = avoidanceSystem;
    }

    @Override
    public void updateControl(RobotCar car, float delta) {
        float ultrasonicDist = car.getUltrasonicSensor().readValue();
        
        // 1. Check for new collision
        if (!inRecovery && ultrasonicDist < Config.ULTRASONIC_STOP_DISTANCE) {
            inRecovery = true;
            reverseTimer = REVERSE_DURATION;
            System.out.println(String.format("ULTRASONIC: Collision imminent (%.2fm) -> BOUNCE BACK", ultrasonicDist));
        }
        
        // 2. Handle active recovery (reverse maneuver)
        if (inRecovery) {
            if (reverseTimer > 0) {
                // Forced reverse phase
                reverseTimer -= delta;
                car.setMotorSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
                return;
            } else {
                // Timer finished, check if we're clear
                if (ultrasonicDist > RECOVERY_THRESHOLD) {
                    // We backed up enough! Clear recovery flag
                    inRecovery = false;
                    System.out.println("ULTRASONIC: Path cleared, resuming manual control");
                } else {
                    // Still too close! Stay stopped or keep reversing slowly
                    car.setMotorSpeeds(0, 0);
                    // Only exit if user backs up manually? Or just stay stuck?
                    // Let's allow manual backup (S key) but block forward
                }
            }
        }
        
        // 3. Normal manual control (with forward block if too close)
        float leftSpeed = 0;
        float rightSpeed = 0;

        boolean forward = Gdx.input.isKeyPressed(Config.KEY_FORWARD);
        boolean backward = Gdx.input.isKeyPressed(Config.KEY_BACKWARD);
        boolean left = Gdx.input.isKeyPressed(Config.KEY_LEFT);
        boolean right = Gdx.input.isKeyPressed(Config.KEY_RIGHT);
        
        // If we are still in recovery mode (timer done but still close), BLOCK forward movement
        if (inRecovery && forward) {
            System.out.println("ULTRASONIC: Forward blocked! Too close to obstacle.");
            forward = false; // Ignore forward input
        }

        if (forward) {
            leftSpeed = 1.0f;
            rightSpeed = 1.0f;
        } else if (backward) {
            leftSpeed = -1.0f;
            rightSpeed = -1.0f;
            // Manual reversing clears the recovery flag if we back up enough
            if (inRecovery && ultrasonicDist > RECOVERY_THRESHOLD) {
                inRecovery = false;
            }
        }

        if (left) {
            leftSpeed -= 0.5f;
            rightSpeed += 0.5f;
        } else if (right) {
            leftSpeed += 0.5f;
            rightSpeed -= 0.5f;
        }

        car.setMotorSpeeds(leftSpeed, rightSpeed);
    }
}
