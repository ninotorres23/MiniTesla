package com.nino.robotics.simulation;

import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.util.Config;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a path made of connected line segments for the robot to follow.
 * <p>
 * The path is defined by a series of waypoints. Line segments connect consecutive
 * waypoints. The {@link #isOnLine(float, float)} method checks if a point is within
 * {@link Config#LINE_THICKNESS} of any segment.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see WorldMap
 * @see Config#LINE_THICKNESS
 */
public class LinePath {
    /** Ordered list of waypoints defining the path */
    private final List<Vector2> points;

    /**
     * Creates an empty line path.
     */
    public LinePath() {
        this.points = new ArrayList<>();
    }

    /**
     * Adds a waypoint to the end of the path.
     * 
     * @param x X coordinate of the waypoint (meters)
     * @param y Y coordinate of the waypoint (meters)
     */
    public void addPoint(float x, float y) {
        points.add(new Vector2(x, y));
    }

    /**
     * Gets all waypoints in the path.
     * 
     * @return List of waypoint positions
     */
    public List<Vector2> getPoints() {
        return points;
    }

    /**
     * Checks if a given world coordinate is on the line path.
     * <p>
     * A point is considered "on the line" if it is within half the line thickness
     * of any segment in the path.
     * </p>
     * 
     * @param x The x-coordinate to check
     * @param y The y-coordinate to check
     * @return true if the point is on the line, false otherwise
     */
    public boolean isOnLine(float x, float y) {
        if (points.size() < 2) {
            return false;
        }

        Vector2 checkPoint = new Vector2(x, y);

        for (int i = 0; i < points.size() - 1; i++) {
            Vector2 start = points.get(i);
            Vector2 end = points.get(i + 1);

            float dist = distToSegment(checkPoint, start, end);
            if (dist <= Config.LINE_THICKNESS / 2.0f) {
                return true;
            }
        }
        return false;
    }

    /**
     * Calculates the shortest distance from a point to a line segment.
     * 
     * @param p The point to measure from
     * @param v Start of the line segment
     * @param w End of the line segment
     * @return The shortest distance from point p to segment vw
     */
    private float distToSegment(Vector2 p, Vector2 v, Vector2 w) {
        float l2 = v.dst2(w);
        if (l2 == 0.0) return p.dst(v);
        float t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;
        t = Math.max(0, Math.min(1, t));
        Vector2 projection = new Vector2(v.x + t * (w.x - v.x), v.y + t * (w.y - v.y));
        return p.dst(projection);
    }
}
