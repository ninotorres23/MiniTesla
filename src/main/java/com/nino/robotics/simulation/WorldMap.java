package com.nino.robotics.simulation;

import com.badlogic.gdx.math.Rectangle;

import java.util.ArrayList;
import java.util.List;

public class WorldMap {
    private final LinePath linePath;
    private final List<Obstacle> obstacles;

    public WorldMap() {
        this.linePath = new LinePath();
        this.obstacles = new ArrayList<>();
        createDefaultMap();
    }

    private void createDefaultMap() {
        // A simple S-shaped path
        linePath.addPoint(1.0f, 1.0f);
        linePath.addPoint(1.0f, 6.0f);
        linePath.addPoint(6.0f, 6.0f);
        linePath.addPoint(6.0f, 1.0f);
        linePath.addPoint(11.0f, 1.0f);
        linePath.addPoint(11.0f, 6.0f);

        // Some obstacles
        obstacles.add(new Obstacle(3.0f, 4.0f, 1.0f, 1.0f));
        obstacles.add(new Obstacle(8.0f, 2.0f, 1.5f, 1.0f));
    }

    public boolean isOnLine(float x, float y) {
        return linePath.isOnLine(x, y);
    }

    public boolean isColliding(Rectangle other) {
        for (Obstacle obstacle : obstacles) {
            if (obstacle.isColliding(other)) {
                return true;
            }
        }
        return false;
    }

    public LinePath getLinePath() {
        return linePath;
    }

    public List<Obstacle> getObstacles() {
        return obstacles;
    }
}
