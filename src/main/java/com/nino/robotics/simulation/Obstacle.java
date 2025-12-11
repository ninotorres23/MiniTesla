package com.nino.robotics.simulation;

import com.badlogic.gdx.math.Intersector;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;

/**
 * Represents a rectangular obstacle in the simulation world.
 * <p>
 * Obstacles block robot movement and can be detected by the ultrasonic sensor.
 * They are rendered as red rectangles in the simulation view.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see WorldMap
 * @see com.nino.robotics.core.UltrasonicSensor
 */
public class Obstacle {
    /** Axis-aligned bounding box of the obstacle */
    private final Rectangle bounds;

    /**
     * Creates an obstacle at the specified position and size.
     * 
     * @param x      X coordinate of bottom-left corner (meters)
     * @param y      Y coordinate of bottom-left corner (meters)
     * @param width  Width of the obstacle (meters)
     * @param height Height of the obstacle (meters)
     */
    public Obstacle(float x, float y, float width, float height) {
        this.bounds = new Rectangle(x, y, width, height);
    }

    /**
     * Gets the bounding rectangle of this obstacle.
     * 
     * @return The obstacle bounds
     */
    public Rectangle getBounds() {
        return bounds;
    }

    /**
     * Checks if this obstacle overlaps with another rectangle.
     * 
     * @param other The rectangle to check against
     * @return true if overlapping, false otherwise
     */
    public boolean isColliding(Rectangle other) {
        return bounds.overlaps(other);
    }

    /**
     * Finds the closest intersection point between a line segment and this obstacle.
     * <p>
     * This method checks all 4 sides of the rectangle and returns the intersection
     * point that is closest to the start of the line segment. This is used by the
     * ultrasonic sensor to determine distance to obstacles.
     * </p>
     * 
     * @param start Start point of the line segment
     * @param end   End point of the line segment
     * @return The closest intersection point, or null if no intersection
     */
    public Vector2 getIntersection(Vector2 start, Vector2 end) {
        Vector2 p1 = new Vector2(bounds.x, bounds.y);
        Vector2 p2 = new Vector2(bounds.x + bounds.width, bounds.y);
        Vector2 p3 = new Vector2(bounds.x + bounds.width, bounds.y + bounds.height);
        Vector2 p4 = new Vector2(bounds.x, bounds.y + bounds.height);

        Vector2 intersection = new Vector2();
        Vector2 closestIntersection = null;
        float minDst = Float.MAX_VALUE;

        // Check all 4 sides and find the closest intersection
        if (Intersector.intersectSegments(start, end, p1, p2, intersection)) {
            float dst = start.dst2(intersection);
            if (dst < minDst) {
                minDst = dst;
                closestIntersection = new Vector2(intersection);
            }
        }
        if (Intersector.intersectSegments(start, end, p2, p3, intersection)) {
            float dst = start.dst2(intersection);
            if (dst < minDst) {
                minDst = dst;
                closestIntersection = new Vector2(intersection);
            }
        }
        if (Intersector.intersectSegments(start, end, p3, p4, intersection)) {
            float dst = start.dst2(intersection);
            if (dst < minDst) {
                minDst = dst;
                closestIntersection = new Vector2(intersection);
            }
        }
        if (Intersector.intersectSegments(start, end, p4, p1, intersection)) {
            float dst = start.dst2(intersection);
            if (dst < minDst) {
                minDst = dst;
                closestIntersection = new Vector2(intersection);
            }
        }

        return closestIntersection;
    }
}
