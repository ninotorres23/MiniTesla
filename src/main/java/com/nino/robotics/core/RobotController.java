package com.nino.robotics.core;

public interface RobotController {
    /**
     * Updates the robot car's motor speeds based on the control logic.
     * @param car The robot car to control.
     * @param delta The time elapsed since the last update.
     */
    void updateControl(RobotCar car, float delta);
}
