/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "Breakout.h"
#include <math.h>
#include <vector>
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#define GAME_AREA_WIDTH 27.0f
#define GAME_AREA_HEIGHT 48.0f

#define BRICKS_PER_COL 8
#define BRICKS_PER_ROW 9

#define PADDLE_WIDTH 4.0f

static bool left_button_down = false;

static bool right_button_down = false;

/**
 * An example of a simple breakout game.
 */
struct BreakoutExample : public CommonRigidBodyBase
{
	BreakoutExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper), m_gameArea(GAME_AREA_WIDTH,GAME_AREA_HEIGHT,0),m_border(NULL),m_ball(NULL),m_paddle(NULL),m_desiredBallVelocity(0), m_hitBricks(0)
	{
	}
	virtual ~BreakoutExample(){
		delete m_border;
		delete m_ball;
		delete m_paddle;

		for(int i = 0; i < m_bricks.size(); ++i){
		   delete m_bricks[i];
		}
	}
	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
	virtual void renderScene();
	virtual bool keyboardCallback(int key, int state);
	void resetCamera()
	{
		float dist = m_gameArea.y()/2.0f / 1.191753593f;// tanf(100.0/2);
		float pitch = 0;
		float yaw = 0;
		float targetPos[3]={m_gameArea.x()/2, m_gameArea.y()/2, -dist};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

	virtual void resetBricks();

	virtual void movePaddle(float move);

	Border* m_border; // game area border

	Ball* m_ball; // game ball

	Paddle* m_paddle; // player paddle

	float m_desiredBallVelocity; // the ball velocity is kept constant to this velocity

	btVector3 m_gameArea; // size of the game area

	btVector3 m_lastBallDirection; // the ball direction should stay the same when keeping the velocity constant

	std::vector<Brick*> m_bricks; // game bricks

	int m_hitBricks; // the bricks that are already out of the area
};

void calmGameObject(GameObject* go) { // function to stop an object from moving
	go->m_body->clearForces(); // clear all applied forces
	btVector3 zeroVector(0, 0, 0);
	go->m_body->setLinearVelocity(zeroVector); // zero the linear velocity
	go->m_body->setAngularVelocity(zeroVector); // zero the angular velocity
}

void removeBrick(Brick* go) {

	//TODO: Fix this with correct graphical removal
	btVector3 oldPosition = go->m_position;
	btVector3 removePosition = btVector3(oldPosition.x() - GAME_AREA_WIDTH *1.1f, oldPosition.y(), oldPosition.z()); // we move the block by this position

	calmGameObject(go); // clear all forces, torques and velocities
	go->setPosition(removePosition);
}

bool processBallBrickCollisions(GameObject* a, GameObject* b){
	if (a && a->m_tag == GameObject::BALL && b && b->m_tag == GameObject::BRICK) { 	// if a collision between a ball and a brick happens
		removeBrick((Brick*)b);
		return true;
	}
	return false;
}

void BreakoutExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,0)); // we do not need gravity
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	{
		m_border = new Border();
	    m_border->setPosition(btVector3(btScalar(m_gameArea.x()/2.0), btScalar(m_gameArea.y()/2.0), btScalar(0)));
	    m_dynamicsWorld->addRigidBody(m_border->m_body);

		m_paddle = new Paddle();
	    m_paddle->setPosition(btVector3(btScalar(m_gameArea.x()/2.0), btScalar(m_gameArea.y() * 0.05), btScalar(0)));
	    m_dynamicsWorld->addRigidBody(m_paddle->m_body);

		m_ball = new Ball();
	    m_ball->setPosition(btVector3(btScalar(m_gameArea.x()/2.0), btScalar(m_gameArea.y() * 0.1), btScalar(0)));
	    m_ball->m_body->setActivationState(DISABLE_DEACTIVATION);
	    m_dynamicsWorld->addRigidBody(m_ball->m_body); //Adding ball to world
		m_ball->m_body->setLinearVelocity(btVector3(0,0,0));
		m_lastBallDirection = btVector3(0,0,0);
	    m_desiredBallVelocity = 20;

		// Generate array of bricks
		for (int j = 0; j < BRICKS_PER_COL; ++j) {
			for (int i = 0; i < BRICKS_PER_ROW; ++i) {
				Brick* brick = new Brick();

				m_dynamicsWorld->addRigidBody(brick->m_body);

				m_bricks.push_back(brick);
			}
		}

		resetBricks();

	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BreakoutExample::stepSimulation(float deltaTime){
	CommonRigidBodyBase::stepSimulation(deltaTime);

	// if object leaves area below
	if (m_ball->m_body->getWorldTransform().getOrigin().y() < 0)
	{
		m_ball->setPosition(btVector3(btScalar(m_paddle->m_body->getWorldTransform().getOrigin().x()), btScalar(m_gameArea.y() * 0.1), btScalar(0)));
		calmGameObject(m_ball);
		m_lastBallDirection = btVector3(0, 0, 0);
		return;
	 }

	int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds(); // for all manifolds
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

		int numContacts = contactManifold->getNumContacts(); // if there are contacts
		if (numContacts > 0) {
			// cast the bodies to collision objects
			const btCollisionObject* obA = contactManifold->getBody0();
			const btCollisionObject* obB = contactManifold->getBody1();

			// get the game objects from the collision objects
			GameObject* goA = (GameObject*) obA->getUserPointer();
			GameObject* goB = (GameObject*) obB->getUserPointer();

			m_hitBricks += processBallBrickCollisions(goA, goB); // if the object A is the ball and B is the brick

			m_hitBricks += processBallBrickCollisions(goB, goA); // if the object b is the ball and A is the brick
		}
	}

	btVector3 newVelocity(0, 0, 0);
	btVector3 currentBallDirection(0, 0, 0);

	if (!m_ball->m_body->getLinearVelocity().isZero()) { // if ball speed is not zero
		currentBallDirection = m_ball->m_body->getLinearVelocity().normalized(); // get normalized velocity as ball direction
	} else if (!m_lastBallDirection.isZero()) {
		currentBallDirection = -m_lastBallDirection.normalized();
	}

	newVelocity = currentBallDirection * m_desiredBallVelocity; // current direction scaled by desired velocity
	m_lastBallDirection = m_ball->m_body->getLinearVelocity(); // keep last ball direction
	m_ball->m_body->setLinearVelocity(newVelocity); // set new velocity


	if(m_bricks.size() == m_hitBricks) { // if all bricks are hit, reset the bricks
		resetBricks();
	}

	if(left_button_down){
		movePaddle(0.5f);
	}

	if(right_button_down){
		movePaddle(-0.5f);
	}

}

void BreakoutExample::resetBricks(){

	// reset array of bricks
	for (int j = 0; j < BRICKS_PER_COL; ++j) {
		for (int i = 0; i < BRICKS_PER_ROW; ++i) {
			float margin = m_gameArea.x() * 0.1;
			float startY = m_gameArea.y() * 0.5;

			Brick* brick = m_bricks.at(BRICKS_PER_ROW* j + i);

			calmGameObject(brick); // clear all forces, torques and velocities

			brick->setPosition(
				btVector3(btScalar(margin + (margin * i)),
					btScalar(startY + (margin * j)), btScalar(0)));

		}
	}

	m_hitBricks = 0;
}

bool BreakoutExample::keyboardCallback(int key, int state) {
//	b3Printf("Key pressed: %d in state %d ",key,state);

	//key 1, key 2, key 3
	switch (key) {
	case 32 /*ASCII for space */:{
		if(m_ball->m_body->getLinearVelocity().isZero()){
			b3Printf("Shooting ball!");
			m_ball->m_body->setLinearVelocity(btVector3((rand() % 50)-25,rand() % 15,0));
		}
		return true;
	}
	case 110 /*ASCII for n*/: {
		left_button_down = state;

		return true;
	}
	case 109 /*ASCII for m*/: {
		right_button_down = state;
		return true;
	}
	}

	return false;
}

void BreakoutExample::movePaddle(float i){
	btVector3 oldPosition = m_paddle->m_position;
	if(oldPosition.x() + i <= (GAME_AREA_WIDTH - PADDLE_WIDTH/2.0f) && oldPosition.x() + i >= (PADDLE_WIDTH/2.0f))
	{
		oldPosition += btVector3(i,0,0);
	}
	m_paddle->setPosition(oldPosition);

	if(m_ball->m_body->getLinearVelocity().isZero()){
		m_ball->setPosition(btVector3(oldPosition.x(), btScalar(m_gameArea.y() * 0.1), btScalar(0)));
	}
}


void BreakoutExample::renderScene()
{
	CommonRigidBodyBase::renderScene();	
}

CommonExampleInterface*    ET_BreakoutCreateFunc(CommonExampleOptions& options)
{
	return new BreakoutExample(options.m_guiHelper);
}

/**
 * Create a collision shape from triangle data.
 * @param vertices
 * @param vertexCount
 * @param isConvex
 */
void GameObject::createShapeWithVertices(const Vertex* vertices, unsigned int vertexCount, bool isConvex)
{
    if (isConvex) // if object is convex, we can use convex hull
    {
        m_shape = new btConvexHullShape();
        for (int i = 0; i < vertexCount; i++)
        {
            Vertex v = vertices[i];
            btVector3 btv = btVector3(v.position.p[0], v.position.p[1], v.position.p[2]);
            ((btConvexHullShape*)m_shape)->addPoint(btv);
        }
    }
    else // if object is concave, we use an ordinary triangle mesh
    {
        btTriangleMesh* mesh = new btTriangleMesh();
        for (int i=0; i < vertexCount; i += 3)
        {
        	// get next 3 vertices as btVectors
            Vertex v1 = vertices[i];    btVector3 bv1 = btVector3(v1.position.p[0], v1.position.p[1], v1.position.p[2]);
            Vertex v2 = vertices[i+1];	btVector3 bv2 = btVector3(v2.position.p[0], v2.position.p[1], v2.position.p[2]);
            Vertex v3 = vertices[i+2];  btVector3 bv3 = btVector3(v3.position.p[0], v3.position.p[1], v3.position.p[2]);

            // create a triangle from vertices
            mesh->addTriangle(bv1, bv2, bv3);
        }

		m_shape = new btBvhTriangleMeshShape(mesh, true); // create a shape from the mesh
    }
}

/**
 * Create a rigid body with the defined mass and the game object's collision shape.
 * @param mass the mass of the game rigid body.
 */
void GameObject::createBodyWithMass(float mass)
{
	btAssert((!m_shape || m_shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	bool isDynamic = (mass != 0.f); //rigidbody is dynamic if and only if mass is non zero, otherwise static

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		m_shape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	#define USE_MOTIONSTATE 1
	#ifdef USE_MOTIONSTATE
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(m_orientation, m_position));

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, motionState, m_shape, localInertia);

		m_body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

	#else
		m_body = new btRigidBody(mass, 0, shape, localInertia);
		m_body->setWorldTransform(btTransform(m_orientation, m_position));
	#endif

    m_body->setRestitution(1.0f); // set object to reflect the total impulse
    m_body->setFriction(0.0f); // set no object friction

    m_body->setLinearFactor(btVector3(1, 1, 0)); // the body is not allowed to move in the z axis
    m_body->setAngularFactor(btVector3(0, 0, 0)); // the body is not allowed to rotate in any axis

    m_body->setUserPointer(this);
}

void GameObject::setPosition(btVector3 position){
	m_position = position;
	m_body->getWorldTransform().setOrigin(position);
}

void GameObject::setOrientation(btQuaternion orientation){
	m_orientation = orientation;
	m_body->getWorldTransform().setRotation(orientation);
}

/**
 * Vertices to describe the breakout area border
 *
 * ||#######||
 * ||       ||
 * ||       ||
 * ||       ||
 * ||       ||
 *
 */
Vertex Border::m_border_vertices[132] = {
	 Vertex(vec3(-13.0, 24.0,-0.5)),
	 Vertex(vec3(-14.0, 24.0,-0.5)),
	 Vertex(vec3(-14.0, 25.0,-0.5)),
	 Vertex(vec3(-13.0, 24.0,-0.5)),
	 Vertex(vec3(-14.0, 25.0,-0.5)),
	 Vertex(vec3(-13.0, 25.0,-0.5)),
	 Vertex(vec3(-13.0, 24.0, 0.5)),
	 Vertex(vec3(-13.0, 25.0, 0.5)),
	 Vertex(vec3(-14.0, 25.0, 0.5)),
	 Vertex(vec3(-13.0, 24.0, 0.5)),
	 Vertex(vec3(-14.0, 25.0, 0.5)),
	 Vertex(vec3(-14.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0, 24.0,-0.5)),
	 Vertex(vec3( 15.0, 24.0,-0.5)),
	 Vertex(vec3( 15.0,-24.0,-0.5)),
	 Vertex(vec3( 14.0, 24.0,-0.5)),
	 Vertex(vec3( 15.0,-24.0,-0.5)),
	 Vertex(vec3( 14.0,-24.0,-0.5)),
	 Vertex(vec3(-14.0, 24.0,-0.5)),
	 Vertex(vec3(-14.0, 24.0, 0.5)),
	 Vertex(vec3(-14.0, 25.0, 0.5)),
	 Vertex(vec3(-14.0, 24.0,-0.5)),
	 Vertex(vec3(-14.0, 25.0, 0.5)),
	 Vertex(vec3(-14.0, 25.0,-0.5)),
	 Vertex(vec3( 14.0, 25.0,-0.5)),
	 Vertex(vec3( 14.0, 25.0, 0.5)),
	 Vertex(vec3( 15.0, 25.0, 0.5)),
	 Vertex(vec3( 14.0, 25.0,-0.5)),
	 Vertex(vec3( 15.0, 25.0, 0.5)),
	 Vertex(vec3( 15.0, 25.0,-0.5)),
	 Vertex(vec3( 15.0, 24.0, 0.5)),
	 Vertex(vec3( 15.0, 24.0,-0.5)),
	 Vertex(vec3( 15.0, 25.0,-0.5)),
	 Vertex(vec3( 15.0, 24.0, 0.5)),
	 Vertex(vec3( 15.0, 25.0,-0.5)),
	 Vertex(vec3( 15.0, 25.0, 0.5)),
	 Vertex(vec3( 14.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0, 25.0,-0.5)),
	 Vertex(vec3( 14.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0, 25.0,-0.5)),
	 Vertex(vec3( 14.0, 25.0,-0.5)),
	 Vertex(vec3( 14.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0, 25.0, 0.5)),
	 Vertex(vec3(-13.0, 25.0, 0.5)),
	 Vertex(vec3( 14.0, 24.0, 0.5)),
	 Vertex(vec3(-13.0, 25.0, 0.5)),
	 Vertex(vec3(-13.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0, 24.0,-0.5)),
	 Vertex(vec3( 14.0, 24.0, 0.5)),
	 Vertex(vec3(-13.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0, 24.0, 0.5)),
	 Vertex(vec3(-13.0, 24.0,-0.5)),
	 Vertex(vec3(-14.0, 25.0,-0.5)),
	 Vertex(vec3(-14.0, 25.0, 0.5)),
	 Vertex(vec3(-13.0, 25.0, 0.5)),
	 Vertex(vec3(-14.0, 25.0,-0.5)),
	 Vertex(vec3(-13.0, 25.0, 0.5)),
	 Vertex(vec3(-13.0, 25.0,-0.5)),
	 Vertex(vec3(-13.0, 25.0,-0.5)),
	 Vertex(vec3(-13.0, 25.0, 0.5)),
	 Vertex(vec3( 14.0, 25.0, 0.5)),
	 Vertex(vec3(-13.0, 25.0,-0.5)),
	 Vertex(vec3( 14.0, 25.0, 0.5)),
	 Vertex(vec3( 14.0, 25.0,-0.5)),
	 Vertex(vec3( 15.0, 24.0,-0.5)),
	 Vertex(vec3( 14.0, 24.0,-0.5)),
	 Vertex(vec3( 14.0, 25.0,-0.5)),
	 Vertex(vec3( 15.0, 24.0,-0.5)),
	 Vertex(vec3( 14.0, 25.0,-0.5)),
	 Vertex(vec3( 15.0, 25.0,-0.5)),
	 Vertex(vec3( 15.0, 24.0, 0.5)),
	 Vertex(vec3( 15.0, 25.0, 0.5)),
	 Vertex(vec3( 14.0, 25.0, 0.5)),
	 Vertex(vec3( 15.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0, 25.0, 0.5)),
	 Vertex(vec3( 14.0, 24.0, 0.5)),
	 Vertex(vec3( 15.0, 24.0,-0.5)),
	 Vertex(vec3( 15.0, 24.0, 0.5)),
	 Vertex(vec3( 15.0,-24.0, 0.5)),
	 Vertex(vec3( 15.0, 24.0,-0.5)),
	 Vertex(vec3( 15.0,-24.0, 0.5)),
	 Vertex(vec3( 15.0,-24.0,-0.5)),
	 Vertex(vec3(-13.0,-24.0,-0.5)),
	 Vertex(vec3(-13.0,-24.0, 0.5)),
	 Vertex(vec3(-14.0,-24.0, 0.5)),
	 Vertex(vec3(-13.0,-24.0,-0.5)),
	 Vertex(vec3(-14.0,-24.0, 0.5)),
	 Vertex(vec3(-14.0,-24.0,-0.5)),
	 Vertex(vec3( 15.0,-24.0,-0.5)),
	 Vertex(vec3( 15.0,-24.0, 0.5)),
	 Vertex(vec3( 14.0,-24.0, 0.5)),
	 Vertex(vec3( 15.0,-24.0,-0.5)),
	 Vertex(vec3( 14.0,-24.0, 0.5)),
	 Vertex(vec3( 14.0,-24.0,-0.5)),
	 Vertex(vec3(-13.0, 24.0, 0.5)),
	 Vertex(vec3(-14.0, 24.0, 0.5)),
	 Vertex(vec3(-14.0,-24.0, 0.5)),
	 Vertex(vec3(-13.0, 24.0, 0.5)),
	 Vertex(vec3(-14.0,-24.0, 0.5)),
	 Vertex(vec3(-13.0,-24.0, 0.5)),
	 Vertex(vec3(-13.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0, 24.0, 0.5)),
	 Vertex(vec3(-13.0,-24.0, 0.5)),
	 Vertex(vec3(-13.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0,-24.0, 0.5)),
	 Vertex(vec3(-13.0,-24.0,-0.5)),
	 Vertex(vec3(-14.0, 24.0, 0.5)),
	 Vertex(vec3(-14.0, 24.0,-0.5)),
	 Vertex(vec3(-14.0,-24.0,-0.5)),
	 Vertex(vec3(-14.0, 24.0, 0.5)),
	 Vertex(vec3(-14.0,-24.0,-0.5)),
	 Vertex(vec3(-14.0,-24.0, 0.5)),
	 Vertex(vec3( 15.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0,-24.0, 0.5)),
	 Vertex(vec3( 15.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0,-24.0, 0.5)),
	 Vertex(vec3( 15.0,-24.0, 0.5)),
	 Vertex(vec3(-14.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0,-24.0,-0.5)),
	 Vertex(vec3(-14.0, 24.0,-0.5)),
	 Vertex(vec3(-13.0,-24.0,-0.5)),
	 Vertex(vec3(-14.0,-24.0,-0.5)),
	 Vertex(vec3( 14.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0, 24.0,-0.5)),
	 Vertex(vec3( 14.0,-24.0,-0.5)),
	 Vertex(vec3( 14.0, 24.0, 0.5)),
	 Vertex(vec3( 14.0,-24.0,-0.5)),
	 Vertex(vec3( 14.0,-24.0, 0.5)),
	};

Border::Border() {
	m_tag = BORDER;
	createShapeWithVertices(m_border_vertices, 132, false); // create shape from vertex data

	createBodyWithMass(btScalar(0.0f)); // create a kinematic object by creating one with mass 0
}

Border::~Border() {

}

Paddle::Paddle() {
	m_tag = PADDLE;
	m_shape = new btBoxShape(btVector3(PADDLE_WIDTH,1,1)); // define paddle shape

	createBodyWithMass(btScalar(0.0f)); // create kinematic object for paddle

}

Paddle::~Paddle() {

}

Ball::Ball() {
	m_tag = BALL;
	m_shape = new btSphereShape(btScalar(1)); // define sphere shape

	btScalar sphereMass(1.f);

	createBodyWithMass(sphereMass); // sphere with mass
}

Ball::~Ball() {

}

Brick::Brick() {
	m_tag = BRICK;
	m_shape = new btBoxShape(btVector3(1.0f,0.5f,0.5f)); // define brick shape

	btScalar brickMass(1.f);

	createBodyWithMass(brickMass); // create brick with mass
}

Brick::~Brick() {

}




