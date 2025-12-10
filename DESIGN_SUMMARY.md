# Design Summary: MiniTesla Robot Car Simulation

## 1. Architecture Overview

The project is designed using a classic Model-View-Controller (MVC) architectural pattern, adapted for a game development context with LibGDX. The structure is organized into several distinct packages, each with a clear responsibility:

-   **`com.nino.robotics.core`**: Represents the **Model**. It contains the core logic of the robot car, including its physics, motors, and sensors. These classes are independent of how the robot is rendered or controlled.
-   **`com.nino.robotics.simulation`**: Represents the **View**. This package is responsible for all rendering and visual representation. It uses LibGDX to draw the world, the robot, and other elements. It also manages the game loop and screen.
-   **`com.nino.robotics.ai`** and **`com.nino.robotics.input`**: Represent the **Controller**. These packages handle the logic for controlling the robot. The `ai` package provides autonomous behavior, while the `input` package handles manual user control.
-   **`com.nino.robotics.util`**: A utility package for shared constants and helper functions, promoting code reuse and maintainability.

This separation of concerns makes the codebase modular, easier to understand, and extensible.

## 2. Object-Oriented Principles

The project was designed to explicitly demonstrate key OOP principles:

### Inheritance

Inheritance is used to create specialized versions of base classes, promoting code reuse and establishing common interfaces.

-   **`Sensor` -> `LineSensor`, `UltrasonicSensor`**: The abstract `Sensor` class defines a common contract with an `update()` method. `LineSensor` and `UltrasonicSensor` provide specific implementations for detecting the line and obstacles, respectively. This allows the `RobotCar` to manage all sensors through a single `List<Sensor>`.
-   **`RobotController` (Interface) -> `AutonomousNavigator`, `ManualController`**: The `RobotController` interface defines the `updateControl()` method. This allows the `SimulationScreen` to seamlessly switch between different control schemes (autonomous vs. manual) by simply swapping the concrete implementation.

### Polymorphism

Polymorphism is leveraged to treat objects of different classes in a uniform way.

-   The `RobotCar` holds a `List<Sensor>`, which can contain both `LineSensor` and `UltrasonicSensor` objects. During the update loop, the car iterates through this list and calls the `update()` method on each sensor, regardless of its specific type.
-   The `SimulationScreen` holds a single `RobotController` reference (`activeController`). This reference can point to either an `AutonomousNavigator` or a `ManualController` object. The main render loop calls `activeController.updateControl()`, and the appropriate behavior is executed depending on the object currently assigned to the reference.

### Encapsulation

Encapsulation is used to hide the internal state of objects and expose functionality through public methods.

-   Fields in classes like `RobotCar`, `Motor`, and `Sensor` are kept `private`. Access to these fields is controlled through getter methods (e.g., `getSpeed()`, `readValue()`) or higher-level methods that modify the state (e.g., `setMotorSpeeds()`).
-   The `Config.java` class encapsulates all simulation constants, preventing "magic numbers" from being scattered throughout the code and providing a single point for configuration changes.

## 3. LibGDX Integration

LibGDX is used for all aspects of rendering, input, and application lifecycle management.

-   **`SimulationGame`**: The main entry point, extending `com.badlogic.gdx.Game` to manage different screens.
-   **`SimulationScreen`**: Implements the `Screen` interface to handle the main game loop (`render()` method).
-   **`ShapeRenderer`**: Used for all drawing. It is a simple and efficient way to render basic geometric shapes (rectangles, circles, lines), which is sufficient for this simulation.
-   **`OrthographicCamera`**: Used to define the views. Two separate cameras are used: one for the top-down view and one for the side view, demonstrating how different viewports can be managed.
-   **Input Handling**: The `Gdx.input` module is used in `ManualController` to read keyboard state for driving the car and in `SimulationScreen` to switch between modes.

## 4. Autonomous and Manual Logic

### Autonomous Mode (`AutonomousNavigator`)

The autonomous logic is based on a simple state machine.

1.  **Line Following**: The primary state. It uses the three line sensors to determine the car's position relative to the line and adjust motor speeds to stay on track.
    -   If only the middle sensor is on the line, the car moves straight.
    -   If the left or right sensor is on the line, the car turns accordingly.
    -   If no sensors are on the line, it performs a slow rotation to find it again.
2.  **Obstacle Avoidance**: If the ultrasonic sensor detects an obstacle within a predefined threshold (`ULTRASONIC_STOP_DISTANCE`), the system switches to this state. The current implementation is simple: it stops the car to prevent a collision.

### Manual Mode (`ManualController`)

This mode translates keyboard input directly into motor speeds to give the user full control over the car's movement.

## 5. Safety System (`ObstacleAvoidanceSystem`)

The safety system is a critical component that prevents collisions.

-   In **Manual Mode**, the `ManualController` consults the `ObstacleAvoidanceSystem` before applying forward movement. If the ultrasonic sensor reports an imminent collision, the controller overrides the user's input and stops the car.
-   In **Autonomous Mode**, the `AutonomousNavigator` uses the ultrasonic sensor reading to switch to its obstacle avoidance behavior, which also stops the car.

This demonstrates how a shared safety module can enforce constraints on different control systems.

## 6. File and Folder Structure

```
MiniTesla/
├── pom.xml
├── src/
│   └── main/
│       └── java/
│           └── com/
│               └── nino/
│                   └── robotics/
│                       ├── ai/
│                       │   ├── AutonomousNavigator.java
│                       │   └── ObstacleAvoidanceSystem.java
│                       ├── core/
│                       │   ├── LineSensor.java
│                       │   ├── Motor.java
│                       │   ├── RobotCar.java
│                       │   ├── RobotController.java
│                       │   ├── Sensor.java
│                       │   └── UltrasonicSensor.java
│                       ├── input/
│                       │   └── ManualController.java
│                       ├── simulation/
│                       │   ├── LinePath.java
│                       │   ├── Obstacle.java
│                       │   ├── SideCamera.java
│                       │   ├── SimulationGame.java
│                       │   ├── SimulationScreen.java
│                       │   ├── TopDownCamera.java
│                       │   └── WorldMap.java
│                       ├── util/
│                       │   ├── Config.java
│                       │   └── MathUtil.java
│                       └── DesktopLauncher.java
├── README.md
└── DESIGN_SUMMARY.md
```
