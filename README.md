# MiniTesla - 2D Robot Car Simulation

This project is a 2D top-down simulation of a Freenove-style Raspberry Pi smart car, developed in Java using the LibGDX framework. It is intended as a final project for an introductory Object-Oriented Programming course.

## Features

- **Autonomous Mode**: The robot car follows a black line and uses an ultrasonic sensor to avoid obstacles.
- **Manual Mode**: The user can control the car with the keyboard (WASD keys).
- **Safety System**: The car prevents movements that would result in a collision with an obstacle.
- **Multiple Camera Views**: A top-down view of the entire simulation and a simplified side-scrolling view.
- **Simple Graphics**: All visuals are rendered using basic shapes for clarity.

## How to Build and Run

This project is built using Apache Maven. You will need to have Maven and a Java Development Kit (JDK) version 11 or higher installed.

### Building the Project

1.  Open a terminal or use the Maven tool window in your IDE.
2.  Navigate to the project's root directory.
3.  Run the following Maven command to build the executable JAR:

    ```sh
    mvn clean package
    ```

    This will create a file named `MiniTesla-1.0-SNAPSHOT.jar` in the `target` directory.

### Running the Project

After building the project, you can run the simulation from your terminal with the following command:

```sh
java -jar target/MiniTesla-1.0-SNAPSHOT.jar
```

This will start the application and open the simulation window.

## Controls

-   **`1`**: Switch to **Autonomous Mode**.
-   **`2`**: Switch to **Manual Mode**.
-   **`W`**: Move forward (Manual Mode).
-   **`S`**: Move backward (Manual Mode).
-   **`A`**: Turn left (Manual Mode).
-   **`D`**: Turn right (Manual Mode).