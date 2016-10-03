package com.syraxius.uavas.objects;

import java.util.ArrayList;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.syraxius.uavas.main.Assets;
import com.syraxius.uavas.util.Constants;
import com.syraxius.uavas.util.QuadcopterHelper;

public class Quadcopter extends AbstractObject {
	private static final float TIME_SCALE = 1;
	private static final float TIME_CAP = 0.05f;
	private static final float PRIORITY_HEIGHT = 17f;
	private static final float BROADCAST_PERIOD = 0.1f;
	private static final int NUM_QUADCOPTERS = Constants.NUM_QUADCOPTERS;

	private TextureRegion quadcopter;
	private float red;
	private float green;
	private float blue;

	public static ArrayList<BroadcastMessage> locationBroadcast = new ArrayList<BroadcastMessage>();
	public static ArrayList<Quadcopter> directCommunication = new ArrayList<Quadcopter>(NUM_QUADCOPTERS);

	public static int globalWaypointCount = 0;
	public static long firstWaypointTime = 0;

	public enum States {
		ARMED, COLLIDED
	};

	public enum ControllerState {
		NORMAL, AVOIDING, CRITICAL, POST_AVOIDANCE
	};

	public States state;
	public ControllerState cState;

	public int id;
	public int priority;
	public int effectivePriority;
	public float broadcastTimer;
	public float postAvoidanceTimer;

	public ArrayList<Vector3> waypoints;
	public Vector3 targetWaypoint;
	public Vector3 guidedWaypoint;

	public Vector3 wind;
	public float windAzimuth;
	public float windPolar;

	public double broadcastInterval;

	public float dmin;
	public boolean avoiding;
	public boolean predictivelyAvoiding;
	public boolean[] checkedFlag = new boolean[NUM_QUADCOPTERS];

	Vector3 lastPosition;

	double v;
	double a;
	double ta;
	double tb;
	double r;
	double originalDeltaTime;
	double transformedDeltaTime;

	public Quadcopter(int id, Vector3 starting, ArrayList<Vector3> waypoints) {
		this.position = starting;
		this.waypoints = waypoints;
		init(id);
	}

	public Quadcopter(int id, Vector3 starting, Vector3 ending) {
		this.position = starting;
		this.waypoints = new ArrayList<Vector3>();
		this.waypoints.add(ending);
		init(id);
	}

	private void init(int id) {
		quadcopter = Assets.instance.greypuck.greypuck;
		red = 1.0f;
		green = 1.0f;
		blue = 1.0f;

		float diameter = Math.max(Constants.VIEWPORT_HEIGHT * 0.02f, 0.6f);

		if (Constants.VIEW_2D_ONLY) {
			dimension.set(diameter, diameter);
			origin.set(diameter / 2, diameter / 2);
		} else {
			dimension.set(diameter, diameter / 2);
			origin.set(diameter / 2, diameter / 4);
		}

		targetWaypoint = new Vector3();
		guidedWaypoint = new Vector3();

		locationBroadcast.add(id, new BroadcastMessage(id, 0, new Vector3(position)));
		directCommunication.add(id, this);

		state = States.ARMED;
		cState = ControllerState.NORMAL;

		this.id = id;
		priority = 0;
		effectivePriority = 0;
		broadcastTimer = (float) (Math.random() * 1000);
		postAvoidanceTimer = 0;

		wind = new Vector3();
		wind.x = 1;

		lastPosition = new Vector3(position);

		broadcastInterval = BROADCAST_PERIOD;
		terminalVelocity = 5f;
		accelerationLimit = 10f;

		v = terminalVelocity;
		a = accelerationLimit;
		ta = 0;
		tb = broadcastInterval;
		r = 0.3;

		targetWaypoint = waypoints.remove(0);
	}

	public void setColor(float red, float green, float blue) {
		this.red = red;
		this.green = green;
		this.blue = blue;
	}

	@Override
	public void update(float deltaTime) {
		deltaTime *= TIME_SCALE;
		if (deltaTime > TIME_CAP) {
			deltaTime = TIME_CAP;
		}

		BroadcastMessage bM = locationBroadcast.get(id);
		bM.time += deltaTime;

		if (state == States.ARMED) {
			super.update(deltaTime);
		}
	}

	public void controllerUpdate(float deltaTime) {
		originalDeltaTime = (originalDeltaTime + deltaTime) / 2;

		deltaTime *= TIME_SCALE;
		if (deltaTime > TIME_CAP) {
			deltaTime = TIME_CAP;
		}

		transformedDeltaTime = (transformedDeltaTime + deltaTime) / 2;

		ta = Math.max(deltaTime, ta);
		tb = broadcastInterval;

		// Basic Distance Set
		double d4ds = 0.5 * v * v / a + ta * ta * a + 4 * ta * v;
		double d4dsdec = 2 * ta * v;
		double d1 = 2 * v * v / a + 2 * r;
		double d2 = d1 + tb * v;
		double d3 = d2 + ta * v;
		double d4 = d3 + d4ds + (locationBroadcast.size() - 2) * d4dsdec;

		// Extended Distance Sets
		double d1multi = v * v / a;
		double bda = tb * v;
		double d3extended = d3 + d1multi;
		double d4partial = d3 + d1multi + d4ds + 3 * d4dsdec;

		dmin = (float) (d3extended);

		if (state == States.ARMED) {
			taskWaypoint();

			Vector3 outputVector = taskGeneratorBasedControl(deltaTime);
			guidedWaypoint.set(outputVector);

			simulatePidController(deltaTime);
			taskLocationBroadcast(deltaTime);
		}
	}

	float kp = 1f;
	float ki = 0f;
	float kd = 1.8f;
	Vector3 previousError = new Vector3(0, 0, 0);
	Vector3 integralError = new Vector3(0, 0, 0);

	private void simulatePidController(float deltaTime) {
		Vector3 error = new Vector3(guidedWaypoint).sub(position);

		Vector3 proportionalError = new Vector3(error);
		integralError = integralError.add((new Vector3(error)).scl(deltaTime));
		Vector3 differentialError = (new Vector3(error)).sub(previousError).scl(1 / deltaTime);

		Vector3 p = new Vector3(proportionalError).scl(kp);
		Vector3 i = new Vector3(integralError).scl(ki);
		Vector3 d = new Vector3(differentialError).scl(kd);

		Vector3 output = new Vector3(0, 0, 0);
		output.add(p).add(i).add(d);

		acceleration = new Vector3(output);

		previousError = error;
	}

	private void simulateWind(float windChangeRate, float maximumAmplitude) {
		windAzimuth += 1 - Math.random() * windChangeRate;
		windPolar += 1 - Math.random() * windChangeRate;
		wind.setFromSpherical(windAzimuth, windPolar);
		wind.setLength((float) Math.random() * maximumAmplitude);
		velocity.add(wind);
	}

	private void taskWaypoint() {
		if (position.dst(targetWaypoint) < 1f) {
			outputStatistics();
			if (waypoints.size() > 0) {
				priority++;
				targetWaypoint = waypoints.remove(0);
			} else {
				state = States.COLLIDED;
				BroadcastMessage broadcastMessage = locationBroadcast.get(id);
				broadcastMessage.disabled = true;
			}
		}
	}

	private void outputStatistics() {
		if (firstWaypointTime == 0) {
			firstWaypointTime = System.currentTimeMillis();
		}
		globalWaypointCount++;
		long nowWaypointTime = System.currentTimeMillis();
		float timeConversionRatio = (float) (transformedDeltaTime / originalDeltaTime);
		float averageVisitationRate = (float) (nowWaypointTime - firstWaypointTime) / globalWaypointCount * timeConversionRatio;
		System.out.println("UAV " + id + ": Waypoints left " + waypoints.size());
		System.out.printf("ta = %f, tb = %f, waypoint %d. Average time is %f\r\n", ta, tb, globalWaypointCount, averageVisitationRate);
	}

	private Vector3 taskGeneratorBasedControl(float deltaTime) {
		Vector3 accumulatorVector = new Vector3(0, 0, 0);

		avoiding = false;
		predictivelyAvoiding = false;

		clearCheckedFlag();
		BroadcastMessage highestUav = getHighestPriorityLinkedUav(id);

		boolean isCritical = highestUav.id != id;
		boolean isUavInTheWay = false;

		int nearestUavInTheWay = getNearestUavInTheWay();
		if (nearestUavInTheWay != -1) {
			Vector3 thisBroadcastedPosition = locationBroadcast.get(this.id).position;
			Vector3 otherBroadcastedPosition = locationBroadcast.get(nearestUavInTheWay).position;
			float otherDistanceToWaypoint = otherBroadcastedPosition.dst(targetWaypoint);
			float thisDistanceToWaypoint = thisBroadcastedPosition.dst(targetWaypoint);
			isUavInTheWay = otherDistanceToWaypoint < thisDistanceToWaypoint;
		}

		switch (cState) {
		case NORMAL:

			if (isCritical) {
				accumulatorVector.add(generatorCollisionAvoidance(highestUav));
			} else if (isUavInTheWay) {
				accumulatorVector.add(QuadcopterHelper.predictive2d(this.position, locationBroadcast.get(nearestUavInTheWay).position, targetWaypoint, 2 * dmin, 90.0f).scl(1000));
			} else {
				accumulatorVector.add(generatorHorizontalWaypoint());
				accumulatorVector.add(generatorElevation());
			}
			break;

		case POST_AVOIDANCE:
			break;
		}

		Vector3 outputVector = new Vector3(accumulatorVector).add(this.position);

		return outputVector;
	}

	private void taskLocationBroadcast(float deltaTime) {
		broadcastTimer += deltaTime;

		BroadcastMessage bM = locationBroadcast.get(id);

		if (broadcastTimer >= broadcastInterval) {
			broadcastTimer %= broadcastInterval;

			effectivePriority = priority;

			bM.priority = effectivePriority;
			bM.position.set(position);
			bM.time = 0;
		}
	}

	private Vector3 generatorCollisionAvoidance(BroadcastMessage highestUav) {
		Vector3 generatorVector = new Vector3(0, 0, 0);

		float thisBroadcastedDistanceToHighestUav = locationBroadcast.get(id).position.dst(highestUav.position);

		for (int i = 0; i < locationBroadcast.size(); i++) {
			BroadcastMessage other = locationBroadcast.get(i);

			boolean isOtherUavDisabled = other.disabled;

			if (isOtherUavDisabled) {
				continue;
			}

			boolean isSameUav = this.id == other.id;

			if (isSameUav) {
				continue;
			}

			float absoluteDistance = this.position.dst(other.position);
			float xyDistance = Vector2.dst(this.position.x, this.position.y, other.position.x, other.position.y);
			float zDistance = this.position.z - other.position.z;

			float otherDistanceToHighestUav = other.position.dst(highestUav.position);

			float deltaTb = other.time;
			float deltaDistance = (float) (deltaTb * v);
			float worstAbsoluteDistance = Math.max(absoluteDistance - deltaDistance, 0);
			float timeScaleDistance = (1 - worstAbsoluteDistance / dmin) * 1000;

			boolean isInCollisionSphere = absoluteDistance <= dmin;
			//boolean isInCollisionSphere = (absoluteDistance - deltaDistance) <= dmin;
			boolean isCloserThanThisUav = otherDistanceToHighestUav <= thisBroadcastedDistanceToHighestUav;

			if (isInCollisionSphere && isCloserThanThisUav) {
				avoiding = true;
				generatorVector.add(QuadcopterHelper.repulsion2d(this.position, other.position, timeScaleDistance));
			}
		}

		return generatorVector;
	}

	private Vector3 generatorHorizontalWaypoint() {
		return new Vector3(targetWaypoint.x - this.position.x, targetWaypoint.y - this.position.y, 0);
	}

	private Vector3 generatorElevation() {
		Vector3 elevationVector = new Vector3(0, 0, targetWaypoint.z + getNoHigherPriority() * PRIORITY_HEIGHT - this.position.z);
		return elevationVector;
	}

	private void clearCheckedFlag() {
		for (int i = 0; i < NUM_QUADCOPTERS; i++) {
			checkedFlag[i] = false;
		}
	}

	public BroadcastMessage getHighestPriorityLinkedUav(int id) {
		checkedFlag[id] = true;
		BroadcastMessage thisUav = locationBroadcast.get(id);
		BroadcastMessage thisHighestUav = thisUav;

		for (int i = 0; i < locationBroadcast.size(); i++) {
			BroadcastMessage otherUav = locationBroadcast.get(i);

			boolean isOtherUavDisabled = otherUav.disabled;

			if (isOtherUavDisabled) {
				continue;
			}

			boolean isSameUav = thisUav.id == otherUav.id;

			if (isSameUav) {
				continue;
			}

			boolean isProcessedBefore = checkedFlag[i] == true;

			if (isProcessedBefore) {
				continue;
			}

			float absoluteDistance = thisUav.position.dst(otherUav.position);
			float xyDistance = Vector2.dst(thisUav.position.x, thisUav.position.y, otherUav.position.x, otherUav.position.y);
			float zDistance = thisUav.position.z - otherUav.position.z;

			boolean isInCollisionSphere = absoluteDistance <= dmin;
			boolean isPossiblyInCollisionSphere = absoluteDistance - (tb * v) <= dmin;

			if (isPossiblyInCollisionSphere) {
				BroadcastMessage otherHighestUav = getHighestPriorityLinkedUav(otherUav.id);

				boolean isLowerPriority = thisHighestUav.priority > otherHighestUav.priority;
				boolean isEqualPriority = thisHighestUav.priority == otherHighestUav.priority;
				boolean isLowerId = thisHighestUav.id < otherHighestUav.id;

				if (isLowerPriority) {
					thisHighestUav = otherHighestUav;
				}

				if (isEqualPriority && isLowerId) {
					thisHighestUav = otherHighestUav;
				}
			}
		}

		return thisHighestUav;
	}

	private int getNearestUavInTheWay() {
		float tmin = Float.MAX_VALUE;
		int idmin = -1;

		for (int i = 0; i < locationBroadcast.size(); i++) {
			BroadcastMessage other = locationBroadcast.get(i);

			if (this.id == other.id) {
				continue;
			}

			if (other.disabled) {
				continue;
			}

			Vector3 ownshipDirection = new Vector3(targetWaypoint).sub(this.position);
			float t = QuadcopterHelper.calculateCollisionDistance(this.position, ownshipDirection, other.position, dmin);

			if (t < 0) {
				continue;
			}

			if (t > tmin) {
				continue;
			}

			tmin = t;
			idmin = other.id;

			predictivelyAvoiding = true;
		}

		return idmin;
	}

	public int getNoHigherPriority() {
		int higherPriority = 0;

		for (int i = 0; i < locationBroadcast.size(); i++) {
			BroadcastMessage other = locationBroadcast.get(i);
			boolean isLowerPriority = effectivePriority > other.priority;
			boolean isEqualPriority = effectivePriority == other.priority;
			boolean isLowerId = this.id < other.id;

			if (isLowerPriority) {
				higherPriority++;
			}

			if (isEqualPriority && isLowerId) {
				higherPriority++;
			}
		}
		return higherPriority / 1000;
	}

	@Override
	public void render(SpriteBatch batch) {
		Vector3 printedWaypoint;
		printedWaypoint = targetWaypoint;

		if (Constants.VIEW_2D_ONLY) {
			if (Constants.DISPLAY_OWNSHIP) {
				batch.setColor(red, green, blue, 1.0f);
				batch.draw(quadcopter.getTexture(), position.x - origin.x, position.y - origin.y, origin.x, origin.y, dimension.x, dimension.y, scale.x, scale.y, rotation, quadcopter.getRegionX(), quadcopter.getRegionY(), quadcopter.getRegionWidth(), quadcopter.getRegionHeight(), true, false);
			}

			if (Constants.DISPLAY_TARGET) {
				batch.setColor(red, green, blue, 0.2f);
				batch.draw(quadcopter.getTexture(), printedWaypoint.x - origin.x, printedWaypoint.y - origin.y, origin.x, origin.y, dimension.x, dimension.y, scale.x, scale.y, rotation, quadcopter.getRegionX(), quadcopter.getRegionY(), quadcopter.getRegionWidth(), quadcopter.getRegionHeight(), true, false);
			}
		} else {
			if (Constants.DISPLAY_OWNSHIP_SHADOW) {
				if (state == States.ARMED) {
					batch.setColor(1.0f, 1.0f, 1.0f, 0.5f);
				} else {
					batch.setColor(1.0f, 0, 0, 0.5f);
				}
				batch.draw(quadcopter.getTexture(), position.x - origin.x, (position.y - origin.y) / 2.0f, origin.x, origin.y, dimension.x, dimension.y, scale.x, scale.y, rotation, quadcopter.getRegionX(), quadcopter.getRegionY(), quadcopter.getRegionWidth(), quadcopter.getRegionHeight(), true, false);
			}

			if (Constants.DISPLAY_OWNSHIP) {
				if (state == States.ARMED) {
					batch.setColor(1.0f, 1.0f, 1.0f, 1.0f);
				} else {
					batch.setColor(1.0f, 0, 0, 1.0f);
				}
				batch.draw(quadcopter.getTexture(), position.x - origin.x, (position.y - origin.y) / 2.0f + position.z, origin.x, origin.y, dimension.x, dimension.y, scale.x, scale.y, rotation, quadcopter.getRegionX(), quadcopter.getRegionY(), quadcopter.getRegionWidth(), quadcopter.getRegionHeight(), true, false);
			}

			if (Constants.DISPLAY_TARGET_SHADOW) {
				batch.setColor(1.0f, 1.0f, 1.0f, 0.1f);
				batch.draw(quadcopter.getTexture(), printedWaypoint.x - origin.x, (printedWaypoint.y - origin.y) / 2.0f, origin.x, origin.y, dimension.x, dimension.y, scale.x, scale.y, rotation, quadcopter.getRegionX(), quadcopter.getRegionY(), quadcopter.getRegionWidth(), quadcopter.getRegionHeight(), true, false);
			}

			if (Constants.DISPLAY_TARGET) {
				batch.setColor(1.0f, 1.0f, 1.0f, 0.2f);
				batch.draw(quadcopter.getTexture(), printedWaypoint.x - origin.x, (printedWaypoint.y - origin.y) / 2.0f + printedWaypoint.z, origin.x, origin.y, dimension.x, dimension.y, scale.x, scale.y, rotation, quadcopter.getRegionX(), quadcopter.getRegionY(), quadcopter.getRegionWidth(), quadcopter.getRegionHeight(), true, false);
			}
		}

		batch.setColor(1.0f, 1.0f, 1.0f, 1.0f);
	}

	@Override
	public void shapeRender(ShapeRenderer shapeRenderer, Camera camera) {
		Vector3 printedWaypoint;
		printedWaypoint = guidedWaypoint;

		if (Constants.VIEW_2D_ONLY) {
			if (Constants.DISPLAY_TARGET_LINE) {
				float x1 = position.x;
				float y1 = position.y;
				float x2 = printedWaypoint.x;
				float y2 = printedWaypoint.y;

				if (avoiding) {
					shapeRenderer.setColor(1, 0, 0, 1);
				} else {
					shapeRenderer.setColor(0, 1, 0, 1);
				}
				shapeRenderer.line(x1, y1, x2, y2);
			}

			if (Constants.DISPLAY_BROADCAST_DANGER_AREA) {
				BroadcastMessage bM = locationBroadcast.get(id);
				// float dTb = (System.currentTimeMillis() - bM.time) / 1000f;
				// float bda = (float) (dTb * v);
				float bda = dmin;
				shapeRenderer.setColor(1, 0, 0, 0.1f);
				shapeRenderer.circle(bM.position.x, bM.position.y, bda, 40);
			}
		} else {
			if (Constants.DISPLAY_OWNSHIP_HEIGHT_LINE) {
				float x1 = position.x;
				float y1 = (position.y - origin.y) / 2.0f + position.z;
				float x2 = position.x;
				float y2 = (position.y - origin.y) / 2.0f;

				shapeRenderer.setColor(0, 0, 0, 0.1f);
				shapeRenderer.line(x1, y1, x2, y2);
			}

			if (Constants.DISPLAY_TARGET_HEIGHT_LINE) {
				float x1 = printedWaypoint.x;
				float y1 = (printedWaypoint.y - origin.y) / 2.0f + printedWaypoint.z;
				float x2 = printedWaypoint.x;
				float y2 = (printedWaypoint.y - origin.y) / 2.0f;

				shapeRenderer.setColor(0, 0, 1, 0.1f);
				shapeRenderer.line(x1, y1, x2, y2);
			}

			if (Constants.DISPLAY_TARGET_LINE) {
				float x1 = position.x;
				float y1 = (position.y - origin.y) / 2.0f + position.z;
				float x2 = printedWaypoint.x;
				float y2 = (printedWaypoint.y - origin.y) / 2.0f + printedWaypoint.z;

				if (avoiding) {
					shapeRenderer.setColor(1, 0, 0, 1);
				} else {
					shapeRenderer.setColor(0, 1, 0, 1);
				}
				shapeRenderer.line(x1, y1, x2, y2);
			}

			if (Constants.DISPLAY_BROADCAST_DANGER_AREA) {
				BroadcastMessage bM = locationBroadcast.get(id);
				// float dTb = (System.currentTimeMillis() - bM.time) / 1000f;
				// float bda = (float) (dTb * v);
				float bda = dmin;
				shapeRenderer.setColor(1, 0, 0, 0.1f);
				shapeRenderer.circle(bM.position.x, bM.position.y / 2 + bM.position.z, bda, 40);
			}
		}
	}
}

class BroadcastMessage {
	int id;
	int priority;
	Vector3 position;
	float time;
	boolean disabled;

	public BroadcastMessage(int id, int priority, Vector3 location) {
		this.id = id;
		this.priority = priority;
		this.position = location;
		time = 0;
		disabled = false;
	}
}