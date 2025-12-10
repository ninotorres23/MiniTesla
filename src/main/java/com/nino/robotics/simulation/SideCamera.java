package com.nino.robotics.simulation;

import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import com.nino.robotics.util.Config;

public class SideCamera {
    private final OrthographicCamera camera;
    private final Viewport viewport;

    public SideCamera() {
        camera = new OrthographicCamera();
        // A different world height for the side view to make it look like a separate panel
        float sideViewWorldHeight = Config.WORLD_HEIGHT / 4.0f;
        viewport = new FitViewport(Config.WORLD_WIDTH, sideViewWorldHeight, camera);
        camera.position.set(Config.WORLD_WIDTH / 2, sideViewWorldHeight / 2, 0);
        camera.update();
    }

    public void resize(int width, int height) {
        // The side camera will occupy a portion of the screen, e.g., the bottom quarter
        viewport.update(width, height / 4);
        viewport.setScreenPosition(0, 0);
    }

    public OrthographicCamera getCamera() {
        return camera;
    }

    public Viewport getViewport() {
        return viewport;
    }
}
