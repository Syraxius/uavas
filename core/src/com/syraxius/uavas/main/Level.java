package com.syraxius.uavas.main;

import java.util.ArrayList;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.syraxius.uavas.objects.Quadcopter;
import com.syraxius.uavas.objects.Quadcopter.States;
import com.syraxius.uavas.util.QuadcopterHelper;

public class Level {
	public static final String TAG = Level.class.getName();

	public Array<Quadcopter> quadcopters;

	public Level() {
		init();
	}

	private void init() {
		quadcopters = new Array<Quadcopter>();
		int numQuadcopters = 3;

		// Generate Circular Case
		// float degreesPerUav = 360.0f / numQuadcopters;
		// int circleRadius = 100;
		// for (int i = 0; i < numQuadcopters; i++) {
		// double x1 = circleRadius * Math.cos(degreesPerUav * i / 180 * Math.PI);
		// double y1 = circleRadius * Math.sin(degreesPerUav * i / 180 * Math.PI);
		// double x2 = circleRadius * Math.cos((degreesPerUav * i + 180) / 180 * Math.PI);
		// double y2 = circleRadius * Math.sin((degreesPerUav * i + 180) / 180 * Math.PI);
		// quadcopters.add(new Quadcopter(i, new Vector3((float) x1, (float) y1, 3.0f), new Vector3((float) x2, (float) y2, 3.0f)));
		// }

		// QuadcopterHelper.generateToFro(30, 100);
		// QuadcopterHelper.generateRandom(10, 1000);
		// QuadcopterHelper.generateCentral(1000);

		for (int i = 0; i < numQuadcopters; i++) {
			int spawnRadius = 5;

			Vector3 starting = new Vector3();
			starting.x = (float) (-spawnRadius + Math.random() * (2 * spawnRadius));
			starting.y = (float) (-spawnRadius + Math.random() * (2 * spawnRadius));
			starting.z = 3;

			ArrayList<Vector3> waypoints = QuadcopterHelper.generateRandomCorners(10, 1000);

			Quadcopter quadcopter = new Quadcopter(i, starting, waypoints);

			// Color will be between 0.6 to 0.7
			float a = 0.6f;
			float b = 0.3f;
			float red = a + (float) ((1 - a) * Math.random()) - b;
			float green = a + (float) ((1 - a) * Math.random()) - b;
			float blue = a + (float) ((1 - a) * Math.random()) - b;

			quadcopter.setColor(red, green, blue);

			quadcopters.add(quadcopter);
		}
	}

	public void render(SpriteBatch batch) {
		for (Quadcopter q : quadcopters) {
			q.render(batch);
		}
	}

	public void shapeRender(ShapeRenderer shapeRenderer, Camera camera) {
		for (Quadcopter q : quadcopters) {
			q.shapeRender(shapeRenderer, camera);
		}
	}

	float minSeparation = Float.MAX_VALUE;

	public void update(float deltaTime) {
		for (Quadcopter q : quadcopters) {
			q.controllerUpdate(deltaTime);
		}

		for (Quadcopter q : quadcopters) {
			q.update(deltaTime);
		}

		for (int i = 0; i < quadcopters.size; i++) {
			for (int j = 0; j < quadcopters.size; j++) {
				boolean differentQuadcopters = i != j;
				boolean firstArmed = quadcopters.get(i).state == States.ARMED;
				boolean secondArmed = quadcopters.get(j).state == States.ARMED;
				boolean bothArmed = firstArmed && secondArmed;
				float separation = quadcopters.get(i).position.dst(quadcopters.get(j).position);
				boolean collided = separation < 0.6f;

				if (differentQuadcopters && bothArmed && collided) {
					quadcopters.get(i).state = States.COLLIDED;
					quadcopters.get(j).state = States.COLLIDED;

					System.out.printf("COLLISION! UAV %d and UAV %d.\r\n", i, j);
				}
				if (differentQuadcopters && minSeparation > separation) {
					minSeparation = separation;
					// System.out.printf("Minimum separation is %f\r\n", minSeparation);
				}
			}
		}
	}
}
