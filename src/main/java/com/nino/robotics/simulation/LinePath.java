package com.nino.robotics.simulation;

import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.util.Config;

import java.util.ArrayList;
import java.util.List;

public class LinePath {
    private final List<Vector2> points;

    public LinePath() {
        this.points = new ArrayList<>();
    }

    public void addPoint(float x, float y) {
        points.add(new Vector2(x, y));
    }

    public List<Vector2> getPoints() {
        return points;
    }

    /**
     * Checks if a given world coordinate is on the line path.
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     * @return True if the point is on the line, false otherwise.
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

    // Helper method to calculate the distance from a point to a line segment.
    private float distToSegment(Vector2 p, Vector2 v, Vector2 w) {
        float l2 = v.dst2(w);
        if (l2 == 0.0) return p.dst(v);
        float t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;
        t = Math.max(0, Math.min(1, t));
        Vector2 projection = new Vector2(v.x + t * (w.x - v.x), v.y + t * (w.y - v.y));
        return p.dst(projection);
    }
}
