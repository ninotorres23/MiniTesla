package com.nino.robotics;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.nino.robotics.simulation.SimulationGame;
import com.nino.robotics.util.Config;

/**
 * Desktop application launcher for the Robot Car Simulation.
 * <p>
 * This is the main entry point for running the simulation on desktop platforms.
 * It configures the LibGDX application window and starts the simulation.
 * </p>
 * 
 * @author Nino Torres
 * @version 1.0
 * @see SimulationGame
 */
public class DesktopLauncher {
	/**
	 * Main entry point for the application.
	 * 
	 * @param arg Command line arguments (not used)
	 */
	public static void main (String[] arg) {
		Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
		config.setForegroundFPS(60);
		config.setTitle("Robot Car Simulation");
		config.setWindowedMode(Config.WINDOW_WIDTH, Config.WINDOW_HEIGHT);
		new Lwjgl3Application(new SimulationGame(), config);
	}
}
