
#include "btBulletDynamicsCommon.h"
#include "Obstacle.h"
#include "MiscObstacles.h"


bool SphereObstacle::missed(const btVector3 &pos, btVector3 &target) const {
	if (centre.distance(pos) > radius) {
		target = (centre - pos).cross(btVector3(0, 1, 0)).normalized() * (radius * 2);
		return true;
	}
	else {
		return false;
	}
}

bool SphereObstacle::inPath(const btVector3 &pos, const btVector3 &vel, btScalar boundingRadius, btScalar &distance, btVector3 &avoidPoint) const {
	
		
	for (btVector3 futurePosition = pos; futurePosition.getX() <= 110.0f && futurePosition.getZ() <= 110.0f && futurePosition.getX() >= 0 && futurePosition.getZ() >= 0; futurePosition += vel) {
		if (futurePosition.distance(centre) < radius + boundingRadius) {
			avoidPoint = btVector3((futurePosition.getX() + centre.getX()) / 2, (futurePosition.getY() + centre.getY()) / 2, (futurePosition.getZ() + centre.getZ()) / 2);
			distance = avoidPoint.distance(pos);
			return true;
		}
	}

	return false;
}


bool ColumnObstacle::missed(const btVector3 &pos, btVector3 &target) const {
	if (centre.distance(pos) > radius) {
		target = (centre - pos).cross(btVector3(0, 1, 0)).normalized() * (radius * 2);
		return true;
	}
	else {
		return false;
	}
}

bool ColumnObstacle::inPath(const btVector3 &pos, const btVector3 &vel,	btScalar boundingRadius,	btScalar &distance, btVector3 &avoidPoint) const {
	btVector3 futurePosition;
	float z = 0.0f;

	for (float x = pos.getX(), z = pos.getZ(); x <= 110 && z <= 110 && x >= 0 && z >= 0; x += vel.getX(), z += vel.getZ()) {
		futurePosition.setValue(x, 0, z);
		if (futurePosition.distance(centre) < radius + boundingRadius) {
			avoidPoint = btVector3((futurePosition.getX() + centre.getX()) / 2, pos.getY(), (futurePosition.getZ() + centre.getZ()) / 2);
			distance = avoidPoint.distance(pos);
			return true;
		}
	}

	return false;
}
