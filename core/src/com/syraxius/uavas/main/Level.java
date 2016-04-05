package com.syraxius.uavas.main;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.utils.Array;
import com.syraxius.uavas.objects.Quadcopter;
import com.syraxius.uavas.objects.Quadcopter.States;

public class Level {
	public static final String TAG = Level.class.getName();

	public Array<Quadcopter> quadcopters;

	public Level() {
		init();
	}

	private void init() {
		quadcopters = new Array<Quadcopter>();
		quadcopters.add(new Quadcopter(0));
		quadcopters.add(new Quadcopter(1));
		quadcopters.add(new Quadcopter(2));
		quadcopters.add(new Quadcopter(3));
		quadcopters.add(new Quadcopter(4));
		quadcopters.add(new Quadcopter(5));
		quadcopters.add(new Quadcopter(6));
		quadcopters.add(new Quadcopter(7));
		quadcopters.add(new Quadcopter(8));
		quadcopters.add(new Quadcopter(9));
		quadcopters.add(new Quadcopter(10));
		quadcopters.add(new Quadcopter(11));
		quadcopters.add(new Quadcopter(12));
		quadcopters.add(new Quadcopter(13));
		quadcopters.add(new Quadcopter(14));
		quadcopters.add(new Quadcopter(15));
		quadcopters.add(new Quadcopter(16));
		quadcopters.add(new Quadcopter(17));
		quadcopters.add(new Quadcopter(18));
		quadcopters.add(new Quadcopter(19));
		quadcopters.add(new Quadcopter(20));// */
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
					//System.out.printf("Minimum separation is %f\r\n", minSeparation);
				}
			}
		}
	}
}
