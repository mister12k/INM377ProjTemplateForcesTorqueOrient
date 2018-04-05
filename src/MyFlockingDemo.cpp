

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

Flock & Flock::operator=(const Flock &flock) {
	this->boids = flock.boids;
	this->obstacles = flock.obstacles;
	return *this;
}

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
void Flock::steer() const {
	for (auto boid : this->boids) {
		boid.steer(this->boids,this->obstacles);
	}
}

void MyFlockingDemo::addSphereObstacle(btSphereShape *shape, const btVector3 &pos) {
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(pos);
	btScalar mass(0.);
	btVector3 localInertia(0, 0, 0);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setFriction(0.5);
	m_dynamicsWorld->addRigidBody(body);

	flock.addObstacle(new SphereObstacle(pos, shape->getRadius()));
}

void MyFlockingDemo::addColumnObstacle(btCylinderShape *shape, const btVector3 &pos){
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(pos);
	btScalar mass(0.);
	btVector3 localInertia(0, 0, 0);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setFriction(0.5);
	m_dynamicsWorld->addRigidBody(body);

	flock.addObstacle(new ColumnObstacle(pos, shape->getRadius()));
}

MyFlockingDemo::MyFlockingDemo()
	:m_ccdMode(USE_CCD)
{
	setDebugMode(btIDebugDraw::DBG_DrawText + btIDebugDraw::DBG_NoHelpText);
	setCameraDistance(btScalar(40.));
}

void MyTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	static_cast<MyFlockingDemo *>(world->getWorldUserInfo())->flock.steer();
}

void MyFlockingDemo::initPhysics() {

	setTexturing(true);
	setShadows(false);

	setCameraDistance(50.f);

	// init world
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	btDiscreteDynamicsWorld* wp = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);
	//	wp->getSolverInfo().m_numIterations = 20; // default is 10
	m_dynamicsWorld = wp;
	m_dynamicsWorld->setInternalTickCallback(MyTickCallback, static_cast<void *>(this), true);


	///create a few basic rigid bodies
	btBoxShape* box = new btBoxShape(btVector3(btScalar(110.), btScalar(1.), btScalar(110.)));
	//	box->initializePolyhedralFeatures();
	btCollisionShape* groundShape = box;


	m_collisionShapes.push_back(groundShape);
	m_collisionShapes.push_back(new btBoxShape(btVector3(1, 1, 1)));

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0));

	// Creation of the ground
	{
		btScalar mass(0.);

		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setFriction(0.5);

		m_dynamicsWorld->addRigidBody(body);
	}

	// Creation of the flock
	{
		btConvexHullShape * bShape = new btConvexHullShape();
		bShape->addPoint(btVector3(10, 0, 0));
		bShape->addPoint(btVector3(0, 5, 0));
		bShape->addPoint(btVector3(0, 0, 5));
		bShape->addPoint(btVector3(0, 0, -5));

		m_collisionShapes.push_back(bShape);
		btTransform btrans;
		btrans.setIdentity();
		btVector3 bpos(20, 0, 0);
		btrans.setOrigin(bpos);
		btScalar bmass(1.0f);
		btVector3 bLocalInertia;
		bShape->calculateLocalInertia(bmass, bLocalInertia);
		btRigidBody* b = localCreateRigidBody(bmass, btrans, bShape);
		b->setAnisotropicFriction(bShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
		b->setFriction(0.5);
		b->activate(true);
		flock.addBoid(b);

		/*

		btConvexHullShape * bShape = new btConvexHullShape();
		bShape->addPoint(btVector3(10, 0, 0));
		bShape->addPoint(btVector3(0, 5, 0));
		bShape->addPoint(btVector3(0, 0, 5));
		bShape->addPoint(btVector3(0, 0, -5));

		m_collisionShapes.push_back(bShape);
		btTransform btrans;
		btrans.setIdentity();
		//		btCollisionShape* bshape = m_collisionShapes[3];
		btVector3 bpos(20, 0, 0);
		btrans.setOrigin(bpos);
		btScalar bmass(1.0f);
		btVector3 bLocalInertia;
		bShape->calculateLocalInertia(bmass, bLocalInertia);
		boid = localCreateRigidBody(bmass, btrans, bShape);
		boid->setAnisotropicFriction(bShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
		boid->setFriction(0.5);
		//		boid->setLinearVelocity(btVector3(1, 0, 0));
		boid->activate(true);
		*/
	}
}

void MyFlockingDemo::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j<m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

}

void MyFlockingDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void MyFlockingDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	displayText();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->debugDrawWorld();
	}
#if 0
	for (int i = 0; i<debugContacts.size(); i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i], debugNormals[i], 0, 0, btVector3(1, 0, 0));
	}
#endif

	glFlush();
	swapBuffers();
}

void MyFlockingDemo::displayText()
{
	int lineWidth = 440;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if ((getDebugMode() & btIDebugDraw::DBG_DrawText) != 0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];

		glRasterPos3f(xStart, yStart, 0);
		switch (m_ccdMode)
		{
		case USE_CCD:
		{
			sprintf_s(buf, "Predictive contacts and motion clamping");
			break;
		}
		case USE_NO_CCD:
		{
			sprintf_s(buf, "CCD handling disabled");
			break;
		}
		default:
		{
			sprintf_s(buf, "unknown CCD setting");
		};
		};

		GLDebugDrawString(xStart, 20, buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf_s(buf, "Press 'p' to change CCD mode");
		yStart += 20;
		GLDebugDrawString(xStart, yStart, buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf_s(buf, "Press '.' or right mouse to shoot bullets");
		yStart += 20;
		GLDebugDrawString(xStart, yStart, buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf_s(buf, "space to restart, h(elp), t(ext), w(ire)");
		yStart += 20;
		GLDebugDrawString(xStart, yStart, buf);

		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}

}

void MyFlockingDemo::keyboardCallback(unsigned char key, int x, int y)
{
	if (key == 'p')
	{
		switch (m_ccdMode)
		{
		case USE_CCD:
		{
			m_ccdMode = USE_NO_CCD;
			break;
		}
		case USE_NO_CCD:
		default:
		{
			m_ccdMode = USE_CCD;
		}
		};
		clientResetScene();
	}
	else
	{
		DemoApplication::keyboardCallback(key, x, y);
	}
}

void MyFlockingDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	//float ms = getDeltaTimeMicroseconds();

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1. / 60., 0);//ms / 1000000.f);
													 //optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	renderme();

	//	displayText();
#if 0
	for (int i = 0; i<debugContacts.size(); i++)
	{
		getDynamicsWorld()->getDebugDrawer()->drawContactPoint(debugContacts[i], debugNormals[i], 0, 0, btVector3(1, 0, 0));
	}
#endif

	glFlush();

	swapBuffers();

}

