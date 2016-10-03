package com.syraxius.uavas.main;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector3;
import com.syraxius.uavas.objects.Quadcopter;
import com.syraxius.uavas.objects.Quadcopter.States;
import com.syraxius.uavas.util.Constants;
import com.syraxius.uavas.util.QuadcopterHelper;

public class Level {
	public static final String TAG = Level.class.getName();

	public ArrayList<Quadcopter> quadcopters;

	public Level(int type) {
		init(type);
	}

	private void init(int type) {
		quadcopters = new ArrayList<Quadcopter>();
		int numQuadcopters = Constants.NUM_QUADCOPTERS;

		// Generate Circular Case
		if (type == 1) {
			float degreesPerUav = 360.0f / numQuadcopters;
			int circleRadius = Constants.SPAWN_RADIUS;
			for (int i = 0; i < numQuadcopters; i++) {
				double x1 = circleRadius * Math.cos(degreesPerUav * i / 180 * Math.PI);
				double y1 = circleRadius * Math.sin(degreesPerUav * i / 180 * Math.PI);
				double x2 = circleRadius * Math.cos((degreesPerUav * i + 180) / 180 * Math.PI);
				double y2 = circleRadius * Math.sin((degreesPerUav * i + 180) / 180 * Math.PI);

				Quadcopter quadcopter = new Quadcopter(i, new Vector3((float) x1, (float) y1, 3.0f), new Vector3((float) x2, (float) y2, 3.0f));

				// Color will be between 0.6 to 0.7
				float a = 0.6f;
				float b = 0.3f;
				float red = a + (float) ((1 - a) * Math.random()) - b;
				float green = a + (float) ((1 - a) * Math.random()) - b;
				float blue = a + (float) ((1 - a) * Math.random()) - b;

				quadcopter.setColor(red, green, blue);

				quadcopters.add(quadcopter);
			}
		} else {
			for (int i = 0; i < numQuadcopters; i++) {
				int spawnRadius = Constants.SPAWN_RADIUS;

				Vector3 starting = new Vector3();
				starting.x = (float) (-spawnRadius + Math.random() * (2 * spawnRadius));
				starting.y = (float) (-spawnRadius + Math.random() * (2 * spawnRadius));
				starting.z = 3;
				ArrayList<Vector3> waypoints;
				switch (type) {
				case 2:
					waypoints = QuadcopterHelper.generateRandomCorners(10, 100);
					break;
				case 3:
					waypoints = QuadcopterHelper.generateToFro(30, 100);
					break;
				case 4:
					waypoints = QuadcopterHelper.generateRandom(10, 100);
					break;
				case 5:
				default:
					waypoints = QuadcopterHelper.generateCentral(100);
					break;
				}

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
		List<Quadcopter> list = Collections.synchronizedList(quadcopters);

		list.parallelStream().forEach((quadcopter) -> {
			quadcopter.controllerUpdate(deltaTime);
		});
		
		list.parallelStream().forEach((quadcopter) -> {
			quadcopter.update(deltaTime);
		});

		list.parallelStream().forEach((quadcopterA) -> {
			list.parallelStream().forEach((quadcopterB) -> {
				boolean differentQuadcopters = quadcopterA.id != quadcopterB.id;
				boolean firstArmed = quadcopterA.state == States.ARMED;
				boolean secondArmed = quadcopterB.state == States.ARMED;
				boolean bothArmed = firstArmed && secondArmed;
				float separation = quadcopterA.position.dst(quadcopterB.position);
				boolean collided = separation < 0.6f;

				if (differentQuadcopters && bothArmed && collided) {
					quadcopterA.state = States.COLLIDED;
					quadcopterB.state = States.COLLIDED;

					System.out.printf("COLLISION! UAV %d and UAV %d.\r\n", quadcopterA.id, quadcopterB.id);
				}
				if (differentQuadcopters && minSeparation > separation) {
					minSeparation = separation;
					// System.out.printf("Minimum separation is %f\r\n", minSeparation);
				}
			});
		});
	}
}
