package com.nino.robotics.simulation;

import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import com.nino.robotics.util.Config;

/**
 * Camera for the side profile view of the simulation.
 * <p>
 * This camera provides a simplified side view of the robot and obstacles,
 * displayed in the bottom quarter of the screen. It shows a 2D profile
 * as if viewing the simulation from the side.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see SimulationScreen
 * @see TopDownCamera
 */
public class SideCamera {
    private final OrthographicCamera camera;
    private final Viewport viewport;

    /**
     * Creates a side camera with a compressed vertical view.
     */
    public SideCamera() {
        camera = new OrthographicCamera();
        float sideViewWorldHeight = Config.WORLD_HEIGHT / 4.0f;
        viewport = new FitViewport(Config.WORLD_WIDTH, sideViewWorldHeight, camera);
        camera.position.set(Config.WORLD_WIDTH / 2, sideViewWorldHeight / 2, 0);
        camera.update();
    }

    /**
     * Updates the viewport when the window is resized.
     * <p>
     * The side camera occupies the bottom quarter of the screen.
     * </p>
     * 
     * @param width  New window width in pixels
     * @param height New window height in pixels
     */
    public void resize(int width, int height) {
        viewport.update(width, height / 4);
        viewport.setScreenPosition(0, 0);
    }

    /**
     * Gets the orthographic camera for rendering.
     * 
     * @return The camera instance
     */
    public OrthographicCamera getCamera() {
        return camera;
    }

    /**
     * Gets the viewport for this camera.
     * 
     * @return The viewport instance
     */
    public Viewport getViewport() {
        return viewport;
    }
}
