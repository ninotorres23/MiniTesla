package com.nino.robotics.core;

import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.simulation.Obstacle;
import com.nino.robotics.simulation.WorldMap;
import com.nino.robotics.util.Config;

public class UltrasonicSensor extends Sensor {

    public UltrasonicSensor(float offsetX, float offsetY) {
        super(offsetX, offsetY);
        this.value = Config.ULTRASONIC_MAX_DISTANCE;
    }

    @Override
    public void update(RobotCar car, WorldMap worldMap, float delta) {
        Vector2 sensorPos = car.localToWorld(relativePosition);
        float carAngleRad = (float) Math.toRadians(car.getAngle());
        float halfConeRad = (float) Math.toRadians(Config.ULTRASONIC_CONE_ANGLE / 2);

        float minDistance = Config.ULTRASONIC_MAX_DISTANCE;

        // Cast 3 rays: Center, Left (-15°), Right (+15°) to simulate a cone
        float[] angles = {0, -halfConeRad, halfConeRad};

        for (float angleOffset : angles) {
            float rayAngle = carAngleRad + angleOffset;
            
            Vector2 rayEnd = new Vector2(
                (float) (sensorPos.x + Config.ULTRASONIC_MAX_DISTANCE * Math.cos(rayAngle)),
                (float) (sensorPos.y + Config.ULTRASONIC_MAX_DISTANCE * Math.sin(rayAngle))
            );

            // Check for intersections with all obstacles
            for (Obstacle obstacle : worldMap.getObstacles()) {
                Vector2 intersection = obstacle.getIntersection(sensorPos, rayEnd);
                if (intersection != null) {
                    float distance = sensorPos.dst(intersection);
                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                }
            }
        }
        
        this.value = minDistance;
    }
}
