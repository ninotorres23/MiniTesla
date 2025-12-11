package com.nino.robotics.simulation;

import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import com.nino.robotics.util.Config;

/**
 * Camera for the top-down (bird's eye) view of the simulation.
 * <p>
 * This camera provides an orthographic view looking straight down at the
 * simulation world. It uses a {@link FitViewport} to maintain aspect ratio
 * when the window is resized.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see SimulationScreen
 * @see SideCamera
 */
public class TopDownCamera {
    private final OrthographicCamera camera;
    private final Viewport viewport;

    /**
     * Creates a top-down camera centered on the world.
     */
    public TopDownCamera() {
        camera = new OrthographicCamera();
        viewport = new FitViewport(Config.WORLD_WIDTH, Config.WORLD_HEIGHT, camera);
        camera.position.set(Config.WORLD_WIDTH / 2, Config.WORLD_HEIGHT / 2, 0);
        camera.update();
    }

    /**
     * Updates the viewport when the window is resized.
     * 
     * @param width  New window width in pixels
     * @param height New window height in pixels
     */
    public void resize(int width, int height) {
        viewport.update(width, height);
    }

    /**
     * Gets the orthographic camera for rendering.
     * 
     * @return The camera instance
     */
    public OrthographicCamera getCamera() {
        return camera;
    }
}
