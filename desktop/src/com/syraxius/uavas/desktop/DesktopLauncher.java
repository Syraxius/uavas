package com.syraxius.uavas.desktop;

import com.badlogic.gdx.backends.lwjgl.LwjglApplication;
import com.badlogic.gdx.backends.lwjgl.LwjglApplicationConfiguration;
import com.syraxius.uavas.main.UavasMain;
import com.syraxius.uavas.util.Constants;

public class DesktopLauncher {
	public static void main (String[] arg) {
		LwjglApplicationConfiguration config = new LwjglApplicationConfiguration();
		config.height = (int) Constants.VIEWPORT_GUI_HEIGHT;
		config.width = (int) Constants.VIEWPORT_GUI_WIDTH;
		new LwjglApplication(new UavasMain(), config);
	}
}
