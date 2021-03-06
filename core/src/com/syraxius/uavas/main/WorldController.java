package com.syraxius.uavas.main;

import com.badlogic.gdx.Application.ApplicationType;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input.Keys;
import com.badlogic.gdx.InputAdapter;
import com.syraxius.uavas.objects.Quadcopter;
import com.syraxius.uavas.util.CameraHelper;
import com.syraxius.uavas.util.Constants;

public class WorldController extends InputAdapter {
	private static final String TAG = WorldController.class.getName();
	public CameraHelper cameraHelper;
	public Level level;

	public WorldController() {
		init();
	}

	private void init() {
		Gdx.input.setInputProcessor(this);
		cameraHelper = new CameraHelper();
		level = new Level(Constants.LEVEL_TYPE);
	}

	public void update(float deltaTime) {
		handleInput(deltaTime);
		cameraHelper.update(deltaTime);
		level.update(deltaTime);
	}

	private void handleInput(float deltaTime) {
		if (Gdx.app.getType() != ApplicationType.Desktop)
			return;

		float moveSpeed = 5 * deltaTime;

		if (Gdx.input.isKeyPressed(Keys.W)) {
		}
		if (Gdx.input.isKeyPressed(Keys.S)) {
		}
		if (Gdx.input.isKeyPressed(Keys.A)) {
		}
		if (Gdx.input.isKeyPressed(Keys.D)) {
		}
		if (Gdx.input.isKeyPressed(Keys.F)) {
		}
		if (Gdx.input.isKeyPressed(Keys.ESCAPE)) {
			Gdx.app.exit();
		}
	}

	@Override
	public boolean keyUp(int keycode) {
		if (keycode == Keys.R) {
			if (Quadcopter.locationBroadcast != null) {
				Quadcopter.locationBroadcast.clear();
			}

			if (Quadcopter.directCommunication != null) {
				Quadcopter.directCommunication.clear();
			}
			
			Quadcopter.globalWaypointCount = 0;
			Quadcopter.firstWaypointTime = 0;
			
			init();
		} else if (keycode == Keys.SPACE) {
		}
		return false;
	}
}
