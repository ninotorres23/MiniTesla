package com.nino.robotics.simulation;

import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import com.nino.robotics.util.Config;

public class TopDownCamera {
    private final OrthographicCamera camera;
    private final Viewport viewport;

    public TopDownCamera() {
        camera = new OrthographicCamera();
        viewport = new FitViewport(Config.WORLD_WIDTH, Config.WORLD_HEIGHT, camera);
        camera.position.set(Config.WORLD_WIDTH / 2, Config.WORLD_HEIGHT / 2, 0);
        camera.update();
    }

    public void resize(int width, int height) {
        viewport.update(width, height);
    }

    public OrthographicCamera getCamera() {
        return camera;
    }
}
