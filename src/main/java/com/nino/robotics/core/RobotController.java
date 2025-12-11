package com.nino.robotics.core;

/**
 * Interface for robot control strategies.
 * <p>
 * Implementations of this interface define how the robot car should behave.
 * The simulation calls {@link #updateControl(RobotCar, float)} each frame,
 * allowing the controller to read sensors and set motor speeds.
 * </p>
 * 
 * <h2>Implementations</h2>
 * <ul>
 *   <li>{@link com.nino.robotics.ai.AutonomousNavigator} - Autonomous line following</li>
 *   <li>{@link com.nino.robotics.input.ManualController} - Keyboard-based manual control</li>
 * </ul>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see RobotCar
 */
public interface RobotController {
    /**
     * Updates the robot car's motor speeds based on the control logic.
     * <p>
     * This method is called once per simulation frame. Implementations should:
     * <ol>
     *   <li>Read sensor values from the car</li>
     *   <li>Apply control logic (PD control, user input, etc.)</li>
     *   <li>Set motor speeds via {@link RobotCar#setMotorSpeeds(float, float)}</li>
     * </ol>
     * </p>
     * 
     * @param car   The robot car to control
     * @param delta Time elapsed since the last update (seconds)
     */
    void updateControl(RobotCar car, float delta);
}
