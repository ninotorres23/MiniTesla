package com.nino.robotics;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.nino.robotics.simulation.SimulationGame;
import com.nino.robotics.util.Config;

public class DesktopLauncher {
	public static void main (String[] arg) {
		Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
		config.setForegroundFPS(60);
		config.setTitle("Robot Car Simulation");
		config.setWindowedMode(Config.WINDOW_WIDTH, Config.WINDOW_HEIGHT);
		new Lwjgl3Application(new SimulationGame(), config);
	}
}
