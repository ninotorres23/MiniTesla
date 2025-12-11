package com.nino.robotics.simulation;

import com.badlogic.gdx.math.Intersector;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;

public class Obstacle {
    private final Rectangle bounds;

    public Obstacle(float x, float y, float width, float height) {
        this.bounds = new Rectangle(x, y, width, height);
    }

    public Rectangle getBounds() {
        return bounds;
    }

    public boolean isColliding(Rectangle other) {
        return bounds.overlaps(other);
    }

    /**
     * Checks if a line segment intersects with the obstacle.
     * @param start Start point of the line segment.
     * @param end End point of the line segment.
     * @return The closest intersection point, or null if no intersection.
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
