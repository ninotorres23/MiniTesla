package com.nino.robotics.simulation;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;
import com.nino.robotics.ai.AutonomousNavigator;
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
    private final AutonomousNavigator autonomousNavigator;
    private final ManualController manualController;

    public SimulationScreen() {
        shapeRenderer = new ShapeRenderer();
        topDownCamera = new TopDownCamera();
        sideCamera = new SideCamera();

        worldMap = new WorldMap();
        robotCar = new RobotCar(1.0f, 1.2f); // Start centered on line (X=1.0) and near beginning (Y=1.2)

        ObstacleAvoidanceSystem obstacleAvoidanceSystem = new ObstacleAvoidanceSystem(worldMap);
        autonomousNavigator = new AutonomousNavigator();
        manualController = new ManualController();
        manualController.setAvoidanceSystem(obstacleAvoidanceSystem);
        activeController = autonomousNavigator; // Start in autonomous mode
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
            activeController = autonomousNavigator;
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

        // Draw ultrasonic cone (only in manual mode, with transparency)
        if (activeController == manualController) {
            // Enable blending for transparency
            Gdx.gl.glEnable(GL20.GL_BLEND);
            Gdx.gl.glBlendFunc(GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);
            
            Vector2 ultrasonicSensorPos = robotCar.localToWorld(robotCar.getUltrasonicSensor().getRelativePosition());
            float dist = robotCar.getUltrasonicSensor().readValue();
            float angleRad = (float) Math.toRadians(robotCar.getAngle());
            float halfConeRad = (float) Math.toRadians(Config.ULTRASONIC_CONE_ANGLE / 2);
            
            // Draw cone as a triangle with transparency
            shapeRenderer.setColor(new Color(0, 1, 1, 0.3f));  // Transparent cyan
            float leftAngle = angleRad + halfConeRad;
            float rightAngle = angleRad - halfConeRad;
            float endLeftX = ultrasonicSensorPos.x + dist * (float)Math.cos(leftAngle);
            float endLeftY = ultrasonicSensorPos.y + dist * (float)Math.sin(leftAngle);
            float endRightX = ultrasonicSensorPos.x + dist * (float)Math.cos(rightAngle);
            float endRightY = ultrasonicSensorPos.y + dist * (float)Math.sin(rightAngle);
            shapeRenderer.triangle(ultrasonicSensorPos.x, ultrasonicSensorPos.y, endLeftX, endLeftY, endRightX, endRightY);
            
            // Draw center ray line (more visible)
            shapeRenderer.setColor(new Color(0, 1, 1, 0.7f));  // Semi-transparent cyan
            float endX = ultrasonicSensorPos.x + dist * (float)Math.cos(angleRad);
            float endY = ultrasonicSensorPos.y + dist * (float)Math.sin(angleRad);
            shapeRenderer.rectLine(ultrasonicSensorPos.x, ultrasonicSensorPos.y, endX, endY, 0.02f);
        }
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
        
        // Draw ultrasonic beam in side view (only in manual mode)
        if (activeController == manualController) {
            Gdx.gl.glEnable(GL20.GL_BLEND);
            Gdx.gl.glBlendFunc(GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);
            
            float dist = robotCar.getUltrasonicSensor().readValue();
            float carX = robotCar.getX();
            float angleRad = (float) Math.toRadians(robotCar.getAngle());
            
            // Calculate beam end position in X (side view shows X axis)
            float beamEndX = carX + dist * (float)Math.cos(angleRad);
            
            // Draw beam as a transparent rectangle
            shapeRenderer.setColor(new Color(0, 1, 1, 0.4f));  // Transparent cyan
            float beamStartX = Math.min(carX, beamEndX);
            float beamWidth = Math.abs(beamEndX - carX);
            if (beamWidth > 0.01f) {  // Only draw if there's meaningful width
                shapeRenderer.rect(beamStartX, 0.15f, beamWidth, 0.1f);
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
