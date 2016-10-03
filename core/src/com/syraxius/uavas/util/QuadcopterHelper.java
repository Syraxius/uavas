package com.syraxius.uavas.util;

import java.util.ArrayList;

import com.badlogic.gdx.math.Vector3;

public class QuadcopterHelper {
	public static Vector3 repulsion3d(Vector3 thisPosition, Vector3 otherPosition, float vectorWeight) {
		Vector3 repulsionVector = new Vector3(thisPosition).sub(otherPosition);
		repulsionVector.setLength(vectorWeight);
		return repulsionVector;
	}

	public static Vector3 repulsion2d(Vector3 thisPosition, Vector3 otherPosition, float vectorWeight) {
		Vector3 repulsionVector = new Vector3(thisPosition).sub(otherPosition);
		repulsionVector.z = 0;
		repulsionVector.setLength(vectorWeight);
		return repulsionVector;
	}

	public static Vector3 rotation3d(Vector3 thisPosition, Vector3 otherPosition, float vectorWeight) {
		Vector3 rotationVector = new Vector3(otherPosition).sub(thisPosition);
		rotationVector.setLength(vectorWeight);
		rotationVector.rotate(90, 0, 0, 1);
		return rotationVector;
	}

	public static Vector3 rotation2d(Vector3 thisPosition, Vector3 otherPosition, float vectorWeight, float angle) {
		Vector3 rotationVector = new Vector3(otherPosition).sub(thisPosition);
		rotationVector.z = 0;
		rotationVector.setLength(vectorWeight);
		rotationVector.rotate(angle, 0, 0, 1);
		return rotationVector;
	}
	
	public static Vector3 predictive2d(Vector3 thisPosition, Vector3 otherPosition, Vector3 targetPosition, float vectorWeight, float angle) {
		Vector3 a = new Vector3(otherPosition).sub(thisPosition).rotate(90, 0, 0, 1);
		Vector3 b = new Vector3(targetPosition).sub(thisPosition);
		float dot = a.dot(b);
		float finalAngle;
		if (dot > 0) {
			finalAngle = angle;
		} else {
			finalAngle = -angle;
		}
		
		Vector3 predictiveVector = new Vector3(otherPosition).sub(thisPosition);
		predictiveVector.z = 0;
		predictiveVector.add(rotation2d(thisPosition, otherPosition, vectorWeight, finalAngle));
		return predictiveVector;
	}

	public static ArrayList<Vector3> generateRandomCorners(float separation, int count) {
		ArrayList<Vector3> waypoints = new ArrayList<Vector3>();

		for (int i = 0; i < count; i++) {
			float x = (float) (Math.random() > 0.5 ? (-separation / 2) : (separation / 2));
			float y = (float) (Math.random() > 0.5 ? (-separation / 2) : (separation / 2));
			float z = (float) (3);
			waypoints.add(new Vector3(x, y, z));
		}

		return waypoints;
	}

	public static ArrayList<Vector3> generateToFro(float separation, int count) {
		ArrayList<Vector3> waypoints = new ArrayList<Vector3>();

		for (int i = 0; i < count; i++) {
			float y = 0;
			float z = (float) (3);
			float x;

			x = (float) (-separation / 2);
			waypoints.add(new Vector3(x, y, z));
			x = (float) (separation / 2);
			waypoints.add(new Vector3(x, y, z));
		}

		return waypoints;
	}

	public static ArrayList<Vector3> generateRandom(float separation, int count) {
		ArrayList<Vector3> waypoints = new ArrayList<Vector3>();

		for (int i = 0; i < count; i++) {
			float x = (float) (-separation / 2 + Math.random() * separation);
			float y = (float) (-separation / 2 + Math.random() * separation);
			float z = (float) (3);
			waypoints.add(new Vector3(x, y, z));
		}

		return waypoints;
	}

	public static ArrayList<Vector3> generateCentral(int count) {
		ArrayList<Vector3> waypoints = new ArrayList<Vector3>();

		for (int i = 0; i < count; i++) {
			waypoints.add(new Vector3(0, 0, 3));
		}

		return waypoints;
	}

	public static float calculateCollisionTime(Vector3 ownship, Vector3 ownshipVelocity, Vector3 intruder, float dmin) {
		Vector3 o = new Vector3(ownship);
		Vector3 l = new Vector3(ownshipVelocity).scl(5);
		Vector3 c = new Vector3(intruder);

		float r = dmin;

		Vector3 omc = new Vector3(o).sub(c);

		float a1 = l.dot(l);
		float b1 = 2 * l.dot(omc);
		float c1 = omc.dot(omc) - r * r;

		float b2m4ac = b1 * b1 - 4 * a1 * c1;

		if (b2m4ac < 0) {
			return -1;
		} else {
			double t = (-b1 - Math.sqrt(b2m4ac)) / (2 * a1);

			return (float) t;
		}
	}

	public static Vector3 calculatePredictiveRepulsion(Vector3 ownship, Vector3 ownshipVelocity, Vector3 intruder, float collisionTime) {
		Vector3 o = new Vector3(ownship);
		Vector3 l = new Vector3(ownshipVelocity);
		Vector3 c = new Vector3(intruder);
		float t = collisionTime;

		Vector3 lt = new Vector3(l).scl(t);
		Vector3 oplt = new Vector3(o).add(lt);

		Vector3 repulsionDirection = new Vector3(oplt).sub(c).setLength(1);

		return repulsionDirection;
	}

	public static Vector3 calculateCollisionPoint(Vector3 ownship, Vector3 ownshipDirection, Vector3 intruder, float dmin) {
		Vector3 o = new Vector3(ownship);
		Vector3 l = new Vector3(ownshipDirection);
		Vector3 c = new Vector3(intruder);

		float r = dmin;

		Vector3 omc = new Vector3(o).sub(c);

		float a1 = l.dot(l);
		float b1 = 2 * l.dot(omc);
		float c1 = omc.dot(omc) - r * r;

		float b2m4ac = b1 * b1 - 4 * a1 * c1;

		if (b2m4ac < 0) {
			return null;
		} else {
			double t = (-b1 - Math.sqrt(b2m4ac)) / (2 * a1);

			return new Vector3(l).scl((float) t);
		}
	}
	
	public static float calculateCollisionDistance(Vector3 ownship, Vector3 ownshipDirection, Vector3 intruder, float dmin) {
		Vector3 o = new Vector3(ownship);
		Vector3 l = new Vector3(ownshipDirection).setLength(1);
		Vector3 c = new Vector3(intruder);

		float r = dmin;

		Vector3 omc = new Vector3(o).sub(c);

		float a1 = l.dot(l);
		float b1 = 2 * l.dot(omc);
		float c1 = omc.dot(omc) - r * r;

		float b2m4ac = b1 * b1 - 4 * a1 * c1;

		if (b2m4ac < 0) {
			return Float.MAX_VALUE;
		} else {
			float t1 = (float) ((-b1 - Math.sqrt(b2m4ac)) / (2 * a1));
			//float t2 = (float) ((-b1 - Math.sqrt(b2m4ac)) / (2 * a1));
			
			return t1;
		}
	}
}
