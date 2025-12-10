package com.nino.robotics.simulation;

import com.badlogic.gdx.Game;

public class SimulationGame extends Game {
    @Override
    public void create() {
        setScreen(new SimulationScreen());
    }
}
