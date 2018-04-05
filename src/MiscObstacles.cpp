#ifndef MISC_OBSTACLES_H
#define MISC_OBSTACLES_H

#include "btBulletDynamicsCommon.h"
#include "Obstacle.h"


// Solid sphere
class SphereObstacle : public Obstacle {
	const btVector3 centre;
	const btScalar radius;

public:
	SphereObstacle(const btVector3 &c, btScalar r) :
		centre(c), radius(r) {}

	bool missed(const btVector3 &pos, btVector3 &target) const {
		if (centre.distance(pos) > radius) {
			target = (centre - pos).cross(btVector3(0, 1, 0)).normalized() * (radius * 2);
			return true;
		}
		else {
			return false;
		}
	}

	bool inPath(const btVector3 &pos, const btVector3 &vel, btScalar boundingRadius, btScalar &distance, btVector3 &avoidPoint) const {
		
		for (btVector3 futurePosition = pos ; futurePosition.x <= 110 && futurePosition.z <= 110 && futurePosition.x >= 0 && futurePosition.z >= 0; futurePosition += vel) {
			if (futurePosition.distance(centre) < radius + boundingRadius) {
				avoidPoint = ((futurePosition.x + centre.x) / 2, (futurePosition.y + centre.y) / 2, (futurePosition.y + centre.y) / 2);
				distance = avoidPoint.distance(pos);
				return true;
			}
		}

		return false;
	}
};

// Vertical column of infinite height
class ColumnObstacle : public Obstacle {
	const btVector3 centre;
	const btScalar radius;

public:
	ColumnObstacle(const btVector3 &c, btScalar r) :
		centre(c), radius(r) {}

	bool missed(const btVector3 &pos, btVector3 &target) const {
		if (centre.distance(pos) > radius) {
			target = (centre - pos).cross(btVector3(0, 1, 0)).normalized() * (radius * 2);
			return true;
		}
		else {
			return false;
		}
	}

	bool inPath(const btVector3 &pos, const btVector3 &vel,	btScalar boundingRadius,	btScalar &distance, btVector3 &avoidPoint) const {
		btVector3 futurePosition;
		float z;

		for (float x = pos.x, z = pos.z; x <= 110 && z <= 110 && x >= 0 && z >= 0; x += vel.x, z += vel.z) {
			futurePosition.setValue(x, 0, z);
			if (futurePosition.distance(centre) < radius + boundingRadius) {
				avoidPoint = ((futurePosition.x + centre.x) / 2, pos.y, (futurePosition.y + centre.y) / 2);
				distance = avoidPoint.distance(pos);
				return true;
			}
		}

		return false;
	}
};

#endif
