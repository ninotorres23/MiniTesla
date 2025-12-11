package com.nino.robotics.simulation;

import com.badlogic.gdx.math.Rectangle;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents the simulation world containing the line path and obstacles.
 * <p>
 * The world map defines:
 * <ul>
 *   <li>A {@link LinePath} that the robot follows in autonomous mode</li>
 *   <li>A collection of {@link Obstacle}s that block movement</li>
 * </ul>
 * </p>
 * 
 * <h2>Default Map</h2>
 * <p>
 * The default map contains an S-shaped path with 4 corners and 2 obstacles.
 * The path coordinates are in meters.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see LinePath
 * @see Obstacle
 */
public class WorldMap {
    /** The line path for autonomous navigation */
    private final LinePath linePath;
    /** List of obstacles in the world */
    private final List<Obstacle> obstacles;

    /**
     * Creates a new world map with the default S-shaped path and obstacles.
     */
    public WorldMap() {
        this.linePath = new LinePath();
        this.obstacles = new ArrayList<>();
        createDefaultMap();
    }

    /**
     * Initializes the default map layout.
     * <p>
     * Creates an S-shaped path with the following waypoints:
     * (1,1) → (1,6) → (6,6) → (6,1) → (11,1) → (11,6)
     * </p>
     */
    private void createDefaultMap() {
        // S-shaped path with 4 corners
        linePath.addPoint(1.0f, 1.0f);   // Start
        linePath.addPoint(1.0f, 6.0f);   // First corner (turn right)
        linePath.addPoint(6.0f, 6.0f);   // Second corner (turn right)
        linePath.addPoint(6.0f, 1.0f);   // Third corner (turn left)
        linePath.addPoint(11.0f, 1.0f);  // Fourth corner (turn left)
        linePath.addPoint(11.0f, 6.0f);  // End

        // Obstacles for manual mode testing
        obstacles.add(new Obstacle(3.0f, 4.0f, 1.0f, 1.0f));
        obstacles.add(new Obstacle(8.0f, 2.0f, 1.5f, 1.0f));
    }

    /**
     * Checks if a point is on the line path.
     * 
     * @param x X coordinate in world space
     * @param y Y coordinate in world space
     * @return true if the point is on the line, false otherwise
     */
    public boolean isOnLine(float x, float y) {
        return linePath.isOnLine(x, y);
    }

    /**
     * Checks if a rectangle collides with any obstacle.
     * 
     * @param other The rectangle to check
     * @return true if collision detected, false otherwise
     */
    public boolean isColliding(Rectangle other) {
        for (Obstacle obstacle : obstacles) {
            if (obstacle.isColliding(other)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the line path for rendering and navigation.
     * 
     * @return The line path
     */
    public LinePath getLinePath() {
        return linePath;
    }

    /**
     * Gets all obstacles in the world.
     * 
     * @return List of obstacles
     */
    public List<Obstacle> getObstacles() {
        return obstacles;
    }
}
