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
     * @return The intersection point, or null if no intersection.
     */
    public Vector2 getIntersection(Vector2 start, Vector2 end) {
        Vector2 intersection = new Vector2();
        // The Intersector class has multiple methods for intersection. We'll find one that works.
        // Let's check intersection with each of the 4 segments of the rectangle.
        Vector2 p1 = new Vector2(bounds.x, bounds.y);
        Vector2 p2 = new Vector2(bounds.x + bounds.width, bounds.y);
        Vector2 p3 = new Vector2(bounds.x + bounds.width, bounds.y + bounds.height);
        Vector2 p4 = new Vector2(bounds.x, bounds.y + bounds.height);

        if (Intersector.intersectSegments(start, end, p1, p2, intersection)) return intersection;
        if (Intersector.intersectSegments(start, end, p2, p3, intersection)) return intersection;
        if (Intersector.intersectSegments(start, end, p3, p4, intersection)) return intersection;
        if (Intersector.intersectSegments(start, end, p4, p1, intersection)) return intersection;

        return null;
    }
}
