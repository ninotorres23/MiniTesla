package com.nino.robotics.input;

import com.badlogic.gdx.Gdx;
import com.nino.robotics.ai.ObstacleAvoidanceSystem;
import com.nino.robotics.core.RobotCar;
import com.nino.robotics.core.RobotController;
import com.nino.robotics.util.Config;

public class ManualController implements RobotController {

    private ObstacleAvoidanceSystem avoidanceSystem;

    // The avoidance system will be injected later.
    public ManualController() {}

    public void setAvoidanceSystem(ObstacleAvoidanceSystem avoidanceSystem) {
        this.avoidanceSystem = avoidanceSystem;
    }

    @Override
    public void updateControl(RobotCar car, float delta) {
        float leftSpeed = 0;
        float rightSpeed = 0;

        boolean forward = Gdx.input.isKeyPressed(Config.KEY_FORWARD);
        boolean backward = Gdx.input.isKeyPressed(Config.KEY_BACKWARD);
        boolean left = Gdx.input.isKeyPressed(Config.KEY_LEFT);
        boolean right = Gdx.input.isKeyPressed(Config.KEY_RIGHT);

        if (forward) {
            leftSpeed = 1.0f;
            rightSpeed = 1.0f;
        } else if (backward) {
            leftSpeed = -1.0f;
            rightSpeed = -1.0f;
        }

        if (left) {
            leftSpeed -= 0.5f;
            rightSpeed += 0.5f;
        } else if (right) {
            leftSpeed += 0.5f;
            rightSpeed -= 0.5f;
        }

        // Safety check: prevent driving into obstacles
        if (avoidanceSystem != null && forward && avoidanceSystem.isCollisionImminent(car)) {
            car.setMotorSpeeds(0, 0);
        } else {
            car.setMotorSpeeds(leftSpeed, rightSpeed);
        }
    }
}
