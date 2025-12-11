package com.nino.robotics.simulation;

import com.badlogic.gdx.Game;

/**
 * Main game class for the Robot Car Simulation.
 * <p>
 * This class extends LibGDX's {@link Game} class and serves as the
 * application adapter. It initializes and displays the {@link SimulationScreen}.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see SimulationScreen
 */
public class SimulationGame extends Game {
    /**
     * Called when the application is created.
     * Initializes and displays the main simulation screen.
     */
    @Override
    public void create() {
        setScreen(new SimulationScreen());
    }
}
