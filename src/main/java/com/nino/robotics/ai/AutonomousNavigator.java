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
        boolean leftOnLine = car.getLeftLineSensor().readValue() > 0.5f;
        boolean midOnLine = car.getMidLineSensor().readValue() > 0.5f;
        boolean rightOnLine = car.getRightLineSensor().readValue() > 0.5f;

        float baseSpeed = 0.5f;
        float turnSpeed = 0.4f;

        if (midOnLine && !leftOnLine && !rightOnLine) {
            // Go straight
            car.setMotorSpeeds(baseSpeed, baseSpeed);
        } else if (leftOnLine) {
            // Turn left
            car.setMotorSpeeds(turnSpeed * 0.5f, turnSpeed);
        } else if (rightOnLine) {
            // Turn right
            car.setMotorSpeeds(turnSpeed, turnSpeed * 0.5f);
        } else {
            // Lost the line, search for it (e.g., slow rotation)
            car.setMotorSpeeds(0.3f, -0.3f);
        }
    }

    private void avoidObstacle(RobotCar car) {
        // Simple avoidance: stop
        car.setMotorSpeeds(0, 0);
        // More advanced: could turn left or right
    }
}
