#ifndef BOID_H
#define BOID_H

#include "btBulletDynamicsCommon.h"

#include "Obstacle.h"

#include <vector>

// A boid is a single agent in the flock
class Boid {
	// Body of the boid (deletion managed by the Demo class)
	btRigidBody* body;

	// radius of a bounding sphere of the shape
	btScalar radius;

	// mass of each boid
	btScalar mass;

	// Unit vector for the direction in which the boid is heading
	btVector3 heading() const;

	// Can the boid see the point?
	bool canSee(const btVector3 &pos) const;

	// Forces on the boid
	btVector3 physicalForce() const;
	btVector3 flockingForce(const std::vector<Boid>& boids) const;
	btVector3 avoidanceForce(const std::vector<Obstacle *>& obstacles) const;

public:
	static btCollisionShape *shape;

	Boid(btRigidBody* b);

	// Apply steering forces (called on each iteration of the physics
	// engine) including physical forces of flight, flocking behaviour
	// and avoiding obstacles.
	void steer(const std::vector<Boid>& boids,
		const std::vector<Obstacle *>& obstacles) const;
};

#endif // BOID_H
