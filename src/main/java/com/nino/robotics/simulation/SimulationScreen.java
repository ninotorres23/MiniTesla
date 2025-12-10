package com.nino.robotics.simulation;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.ai.temp1;
import com.nino.robotics.ai.ObstacleAvoidanceSystem;
import com.nino.robotics.core.RobotCar;
import com.nino.robotics.core.RobotController;
import com.nino.robotics.core.Sensor;
import com.nino.robotics.input.ManualController;
import com.nino.robotics.util.Config;

public class SimulationScreen implements Screen {

    private final ShapeRenderer shapeRenderer;
    private final TopDownCamera topDownCamera;
    private final SideCamera sideCamera;

    private final WorldMap worldMap;
    private final RobotCar robotCar;

    private RobotController activeController;
    private final temp1 temp1;
    private final ManualController manualController;

    public SimulationScreen() {
        shapeRenderer = new ShapeRenderer();
        topDownCamera = new TopDownCamera();
        sideCamera = new SideCamera();

        worldMap = new WorldMap();
        // Start the car so the MID line sensor sits directly over the first vertical line at x=1.0.
        // With initial angle 90° and LINE_SENSOR_FORWARD_OFFSET = 0.15, the mid sensor world X is x - 0.15.
        // Therefore, place the car at x = 1.15 so the mid sensor is at x = 1.0.
        robotCar = new RobotCar(1.15f, 1.5f);

        ObstacleAvoidanceSystem obstacleAvoidanceSystem = new ObstacleAvoidanceSystem(worldMap);
        temp1 = new temp1();
        manualController = new ManualController();
        manualController.setAvoidanceSystem(obstacleAvoidanceSystem);
        activeController = temp1; // Start in autonomous mode
    }

    @Override
    public void show() { }

    @Override
    public void render(float delta) {
        handleInput();

        // Update logic
        activeController.updateControl(robotCar, delta);
        robotCar.update(delta, worldMap);

        // Clear screen
        Gdx.gl.glClearColor(0.1f, 0.1f, 0.1f, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        // Render top-down view
        renderTopDownView();

        // Render side view
        renderSideView();
    }

    private void handleInput() {
        if (Gdx.input.isKeyJustPressed(Config.KEY_MODE_AUTO)) {
            activeController = temp1;
        }
        if (Gdx.input.isKeyJustPressed(Config.KEY_MODE_MANUAL)) {
            activeController = manualController;
        }
    }

    private void renderTopDownView() {
        topDownCamera.getCamera().update();
        shapeRenderer.setProjectionMatrix(topDownCamera.getCamera().combined);

        Gdx.gl.glViewport(0, Gdx.graphics.getHeight() / 4, Gdx.graphics.getWidth(), Gdx.graphics.getHeight() * 3 / 4);

        shapeRenderer.begin(ShapeRenderer.ShapeType.Filled);
        // Draw background
        shapeRenderer.setColor(Color.DARK_GRAY);
        shapeRenderer.rect(0, 0, Config.WORLD_WIDTH, Config.WORLD_HEIGHT);

        // Draw line path
        shapeRenderer.setColor(Color.BLACK);
        for (int i = 0; i < worldMap.getLinePath().getPoints().size() - 1; i++) {
            Vector2 start = worldMap.getLinePath().getPoints().get(i);
            Vector2 end = worldMap.getLinePath().getPoints().get(i + 1);
            shapeRenderer.rectLine(start, end, Config.LINE_THICKNESS);
        }

        // Draw obstacles
        shapeRenderer.setColor(Color.FIREBRICK);
        for (Obstacle obstacle : worldMap.getObstacles()) {
            Rectangle bounds = obstacle.getBounds();
            shapeRenderer.rect(bounds.x, bounds.y, bounds.width, bounds.height);
        }

        // Draw robot
        drawRobot();

        shapeRenderer.end();
    }

    private void drawRobot() {
        shapeRenderer.setColor(Color.ROYAL);
        Rectangle carBounds = robotCar.getBounds();
        shapeRenderer.rect(carBounds.x, carBounds.y, Config.CAR_WIDTH / 2, Config.CAR_HEIGHT / 2, Config.CAR_WIDTH, Config.CAR_HEIGHT, 1, 1, robotCar.getAngle());

        // Draw sensors
        shapeRenderer.setColor(Color.YELLOW);
        for (Sensor sensor : robotCar.getSensors()) {
            Vector2 sensorPos = robotCar.localToWorld(sensor.getRelativePosition());
            shapeRenderer.circle(sensorPos.x, sensorPos.y, 0.02f);
        }

        // Draw ultrasonic ray
        shapeRenderer.setColor(Color.CYAN);
        Vector2 ultrasonicSensorPos = robotCar.localToWorld(robotCar.getUltrasonicSensor().getRelativePosition());
        float dist = robotCar.getUltrasonicSensor().readValue();
        float angleRad = (float) Math.toRadians(robotCar.getAngle());
        shapeRenderer.line(ultrasonicSensorPos.x, ultrasonicSensorPos.y, ultrasonicSensorPos.x + dist * (float)Math.cos(angleRad), ultrasonicSensorPos.y + dist * (float)Math.sin(angleRad));
    }

    private void renderSideView() {
        sideCamera.getViewport().apply();
        sideCamera.getCamera().update();
        shapeRenderer.setProjectionMatrix(sideCamera.getCamera().combined);

        shapeRenderer.begin(ShapeRenderer.ShapeType.Filled);
        // Draw ground
        shapeRenderer.setColor(Color.BROWN);
        shapeRenderer.rect(0, 0, Config.WORLD_WIDTH, 0.1f);

        // Draw a simple representation of the car
        shapeRenderer.setColor(Color.LIGHT_GRAY);
        shapeRenderer.rect(robotCar.getX() - Config.CAR_WIDTH / 2, 0.1f, Config.CAR_WIDTH, 0.2f);

        // Draw obstacles in side view
        shapeRenderer.setColor(Color.FIREBRICK);
        for (Obstacle obstacle : worldMap.getObstacles()) {
            if (Math.abs(obstacle.getBounds().y - robotCar.getY()) < 1.0f) { // Only draw close obstacles
                shapeRenderer.rect(obstacle.getBounds().x, 0.1f, obstacle.getBounds().width, 0.5f);
            }
        }
        shapeRenderer.end();
    }

    @Override
    public void resize(int width, int height) {
        topDownCamera.resize(width, height * 3 / 4);
        sideCamera.resize(width, height);
    }

    @Override
    public void pause() { }

    @Override
    public void resume() { }

    @Override
    public void hide() { }

    @Override
    public void dispose() {
        shapeRenderer.dispose();
    }
}
