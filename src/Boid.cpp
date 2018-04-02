#include "Boid.h"
#include <limits>

	// Unit vector for the direction in which the boid is heading
	btVector3 Boid::heading() const {
		return this->body->getOrientation().getAxis().normalized();
	}

	// Can the boid see the point?
	bool Boid::canSee(const btVector3 &pos) const{
		// See if the point is in the boid's arc of vision; if so check if it's within arc radius
		if (pos.angle(this->body->getCenterOfMassPosition()) > 135) {
			return false;
		}
		else {
			return (this->body->getCenterOfMassPosition().distance(pos) < this->radius * 2);
		}
	}


	// Forces on the boid
	
	btVector3 Boid::physicalForce() const{
		// Thrust force that moves the boid forward
		return 3.5 * this->heading();
	}

	// Steering forces that adjust the boid with the surrounding flock
	btVector3 Boid::flockingForce(const std::vector<Boid>& boids) const{
		btVector3 avgHeading, avgPosNeigh;
		bool leader = true;

		for (auto const &boid : boids) {
			if (this->canSee(boid.body->getCenterOfMassPosition())) {
				leader = false;
			}
			avgHeading += boid.heading();
			avgPosNeigh += boid.body->getCenterOfMassPosition();
		}

		avgHeading = avgHeading / (btScalar) boids.size();
		avgPosNeigh = avgPosNeigh / (btScalar) boids.size();

		return !leader ? avgHeading + avgPosNeigh : btVector3(0,0,0);

	}

	// Force that allows the boid to avoid obstacles by steering
	btVector3 Boid::avoidanceForce(const std::vector<Obstacle *>& obstacles) const{
		btScalar distObstacle = 0, minObstacle = (btScalar) std::numeric_limits<int>::max();
		btVector3 avoidPoint, targetPoint, avoidingForce;
		Obstacle * nearestObstacle = nullptr;

		for (auto const &obstacle : obstacles) {
			if (obstacle->inPath(this->body->getCenterOfMassPosition(),this->body->getLinearVelocity(),this->radius,distObstacle,avoidPoint)){
				if (distObstacle < minObstacle) {
					minObstacle = distObstacle;
					nearestObstacle = obstacle;
					targetPoint = avoidPoint;
				}
			}
		}
		if (nearestObstacle != nullptr) {
			avoidingForce = avoidPoint.cross(this->body->getCenterOfMassPosition());
			return avoidingForce.normalized();
		}else	{
			return btVector3(0, 0, 0);
		}

	}

	Boid::Boid(btRigidBody* b){}

	// Apply steering forces (called on each iteration of the physics
	// engine) including physical forces of flight, flocking behaviour
	// and avoiding obstacles.
	void Boid::steer(const std::vector<Boid>& boids, const std::vector<Obstacle *>& obstacles) const{

		this->body->applyCentralForce(this->avoidanceForce(obstacles) + this->flockingForce(boids) + this->physicalForce());

	}
