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

	virtual bool missed(const btVector3 &pos, btVector3 &target) const;

	virtual bool inPath(const btVector3 &pos, const btVector3 &vel,
		btScalar boundingRadius,
		btScalar &distance, btVector3 &avoidPoint) const;
};

// Vertical column of infinite height
class ColumnObstacle : public Obstacle {
	const btVector3 centre;
	const btScalar radius;

public:
	ColumnObstacle(const btVector3 &c, btScalar r) :
		centre(c), radius(r) {}

	virtual bool missed(const btVector3 &pos, btVector3 &target) const;

	virtual bool inPath(const btVector3 &pos, const btVector3 &vel,
		btScalar boundingRadius,
		btScalar &distance, btVector3 &avoidPoint) const;
};

#endif
