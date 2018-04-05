#ifndef MY_FLOCKING_DEMO_H
#define MY_FLOCKING_DEMO_H

#include "btBulletDynamicsCommon.h"
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#include <vector>

#include "Obstacle.h"
#include "MiscObstacles.h"
#include "Boid.h"

class Flock {
	std::vector<Boid> boids;
	std::vector<Obstacle *> obstacles;

	// declare dummies as private to forbid copying
	Flock(const Flock &flock);
	Flock & operator=(const Flock &flock);

public:
	Flock();

	// Add a boid with the given body.
	// (deletion of the body is handled by the Demo class)
	void addBoid(btRigidBody* b);

	// Add an obstacle for boids to avoid.
	void addObstacle(Obstacle* o);

	// Apply steering forces to each boid in the flock.
	void steer() const;
};

class MyFlockingDemo : public PlatformDemoApplication
{

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	enum
	{
		USE_CCD = 1,
		USE_NO_CCD
	};
	int 	m_ccdMode;

	void addSphereObstacle(btSphereShape *shape, const btVector3 &pos);
	void addColumnObstacle(btCylinderShape *shape, const btVector3 &pos);

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

public:
	Flock flock;

	MyFlockingDemo();

	virtual ~MyFlockingDemo()
	{
		exitPhysics();
	}

	void initPhysics();

	void exitPhysics();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	virtual void	clientResetScene();

	void displayText();

	virtual void displayCallback();

	virtual void clientMoveAndDisplay();

	static DemoApplication* Create()
	{
		MyFlockingDemo* demo = new MyFlockingDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	

};

#endif //MY_FLOCKING_DEMO_H
