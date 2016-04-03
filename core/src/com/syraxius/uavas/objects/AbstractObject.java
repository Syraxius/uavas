package com.syraxius.uavas.objects;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;

public abstract class AbstractObject {
	public Vector3 position;
	public Vector2 dimension;
	public Vector2 origin;
	public Vector2 scale;
	public Vector3 velocity;
	public Vector3 acceleration;
	public float rotation;
	public float terminalVelocity;
	public float accelerationLimit;

	public AbstractObject() {
		position = new Vector3();
		dimension = new Vector2(1, 1);
		origin = new Vector2();
		scale = new Vector2(1, 1);
		velocity = new Vector3();
		acceleration = new Vector3();
		rotation = 0;
		terminalVelocity = 1f;
		accelerationLimit = 0.1f;
	}

	public void update(float deltaTime) {
		acceleration.clamp(0, accelerationLimit);
		velocity.x += acceleration.x * deltaTime;
		velocity.y += acceleration.y * deltaTime;
		velocity.z += acceleration.z * deltaTime;
		velocity.clamp(0, terminalVelocity);
		position.x += velocity.x * deltaTime;
		position.y += velocity.y * deltaTime;
		position.z += velocity.z * deltaTime;
	}

	public abstract void render(SpriteBatch batch);

	public void shapeRender(ShapeRenderer shapeRenderer, Camera camera) {
	}
}