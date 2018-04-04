

#include "MyFlockingDemo.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"


Flock::Flock(const Flock &flock) {
	this->boids = flock.boids;
	this->obstacles = flock.obstacles;
}

Flock & Flock::operator=(const Flock &flock) {}

Flock::Flock() {}

// Add a boid with the given body.
// (deletion of the body is handled by the Demo class)
void Flock::addBoid(btRigidBody* b) {
	Boid boid(b);
	boids.push_back(boid);
}

// Add an obstacle for boids to avoid.
void Flock::addObstacle(Obstacle* o) {
	this->obstacles.push_back(o);
}

// Apply steering forces to each boid in the flock.
void Flock::steer() const {}

void MyFlockingDemo::addSphereObstacle(btSphereShape *shape, const btVector3 &pos) {}

void MyFlockingDemo::addColumnObstacle(btCylinderShape *shape, const btVector3 &pos){}

MyFlockingDemo::~MyFlockingDemo() {}

void MyFlockingDemo::initPhysics() {}

