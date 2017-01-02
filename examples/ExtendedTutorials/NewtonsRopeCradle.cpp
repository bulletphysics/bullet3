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

#include "NewtonsRopeCradle.h"

#include <vector> // TODO: Should I use another data structure?
#include <iterator>

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

static btScalar gPendulaQty = 5; // Number of pendula in newton's cradle
//TODO: This would actually be an Integer, but the Slider does not like integers, so I floor it when changed

static btScalar gDisplacedPendula = 1; // number of displaced pendula
//TODO: This is an int as well

static btScalar gPendulaRestitution = 1; // pendula restition when hitting against each other

static btScalar gSphereRadius = 1; // pendula radius

static btScalar gInitialPendulumWidth = 4; // default pendula width

static btScalar gInitialPendulumHeight = 8; // default pendula height

static btScalar gRopeResolution = 1; // default rope resolution (number of links as in a chain)

static btScalar gDisplacementForce = 30; // default force to displace the pendula

static btScalar gForceScalar = 0; // default force scalar to apply a displacement

struct NewtonsRopeCradleExample : public CommonRigidBodyBase { 
	NewtonsRopeCradleExample(struct GUIHelperInterface* helper) :
		CommonRigidBodyBase(helper) {
	}
	virtual ~NewtonsRopeCradleExample(){}
	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
	virtual void renderScene();
	virtual void applyPendulumForce(btScalar pendulumForce);
	void createEmptyDynamicsWorld()
	{
		m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
        m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

		m_broadphase = new btDbvtBroadphase();

		m_solver = new btSequentialImpulseConstraintSolver;

		m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
		m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

		softBodyWorldInfo.m_broadphase = m_broadphase;
		softBodyWorldInfo.m_dispatcher = m_dispatcher;
		softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
		softBodyWorldInfo.m_sparsesdf.Initialize();
	}

	virtual void createRopePendulum(btSphereShape* colShape,
		const btVector3& position, const btQuaternion& pendulumOrientation, btScalar width, btScalar height, btScalar mass);
	virtual void changePendulaRestitution(btScalar restitution);
	virtual void connectWithRope(btRigidBody* body1, btRigidBody* body2);
	virtual bool keyboardCallback(int key, int state);

	virtual btSoftRigidDynamicsWorld*	getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}
	void resetCamera()
	{
		float dist = 41;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

	std::vector<btSliderConstraint*> constraints;
	std::vector<btRigidBody*> pendula;
	
	btSoftBodyWorldInfo softBodyWorldInfo;

};

static NewtonsRopeCradleExample* nex = NULL;

void onRopePendulaRestitutionChanged(float pendulaRestitution, void*);

void applyRForceWithForceScalar(float forceScalar);

void NewtonsRopeCradleExample::initPhysics()
{

	{ // create a slider to change the number of pendula
		SliderParams slider("Number of Pendula", &gPendulaQty);
		slider.m_minVal = 1;
		slider.m_maxVal = 50;
        slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the number of displaced pendula
		SliderParams slider("Number of Displaced Pendula", &gDisplacedPendula);
		slider.m_minVal = 0;
		slider.m_maxVal = 49;
        slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the pendula restitution
		SliderParams slider("Pendula Restitution", &gPendulaRestitution);
		slider.m_minVal = 0;
		slider.m_maxVal = 1;
		slider.m_clampToNotches = false;
		slider.m_callback = onRopePendulaRestitutionChanged;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the rope resolution
		SliderParams slider("Rope Resolution", &gRopeResolution);
		slider.m_minVal = 1;
		slider.m_maxVal = 20;
        slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the pendulum width
		SliderParams slider("Pendulum Width", &gInitialPendulumWidth);
		slider.m_minVal = 0;
		slider.m_maxVal = 40;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the pendulum height
		SliderParams slider("Pendulum Height", &gInitialPendulumHeight);
		slider.m_minVal = 0;
		slider.m_maxVal = 40;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{ // create a slider to change the force to displace the lowest pendulum
		SliderParams slider("Displacement force", &gDisplacementForce);
		slider.m_minVal = 0.1;
		slider.m_maxVal = 200;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}
	
	{ // create a slider to apply the force by slider
		SliderParams slider("Apply displacement force", &gForceScalar);
		slider.m_minVal = -1;
		slider.m_maxVal = 1;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	// create a debug drawer
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(
			btIDebugDraw::DBG_DrawWireframe
				+ btIDebugDraw::DBG_DrawContactPoints
				+ btIDebugDraw::DBG_DrawConstraints
				+ btIDebugDraw::DBG_DrawConstraintLimits);

	{ // create the pendula starting at the indicated position below and where each pendulum has the following mass
		btScalar pendulumMass(1.0f);

		btVector3 position(0.0f,15.0f,0.0f); // initial left-most pendulum position
		btQuaternion orientation(0,0,0,1); // orientation of the pendula

		// Re-using the same collision is better for memory usage and performance
		btSphereShape* pendulumShape = new btSphereShape(gSphereRadius);
		m_collisionShapes.push_back(pendulumShape);

		for (int i = 0; i < floor(gPendulaQty); i++) {

			// create pendulum
			createRopePendulum(pendulumShape, position, orientation,gInitialPendulumWidth,
				gInitialPendulumHeight, pendulumMass);

			// displace the pendula 1.05 sphere size, so that they all nearly touch (small spacings in between)
			position.setX(position.x()-2.1f * gSphereRadius);
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void NewtonsRopeCradleExample::connectWithRope(btRigidBody* body1, btRigidBody* body2)
{
	btSoftBody*	softBodyRope0 = btSoftBodyHelpers::CreateRope(softBodyWorldInfo,body1->getWorldTransform().getOrigin(),body2->getWorldTransform().getOrigin(),gRopeResolution,0);
	softBodyRope0->setTotalMass(0.1f);

	softBodyRope0->appendAnchor(0,body1);
	softBodyRope0->appendAnchor(softBodyRope0->m_nodes.size()-1,body2);

	softBodyRope0->m_cfg.piterations = 5;
	softBodyRope0->m_cfg.kDP = 0.005f;
	softBodyRope0->m_cfg.kSHR = 1;
	softBodyRope0->m_cfg.kCHR = 1;
	softBodyRope0->m_cfg.kKHR = 1;

	getSoftDynamicsWorld()->addSoftBody(softBodyRope0);
}

void NewtonsRopeCradleExample::stepSimulation(float deltaTime) {

	applyRForceWithForceScalar(gForceScalar); // apply force defined by apply force slider

	if (m_dynamicsWorld) {
		m_dynamicsWorld->stepSimulation(deltaTime);
	}
}

void NewtonsRopeCradleExample::createRopePendulum(btSphereShape* colShape,
	const btVector3& position, const btQuaternion& pendulumOrientation, btScalar width, btScalar height, btScalar mass) {

	// The pendulum looks like this (names when built):
	// O   O  topSphere1 topSphere2
	//  \ /
	//   O   bottomSphere

	//create a dynamic pendulum
	btTransform startTransform;
	startTransform.setIdentity();

	// calculate sphere positions
	btVector3 topSphere1RelPosition(0,0,width);
	btVector3 topSphere2RelPosition(0,0,-width);
	btVector3 bottomSphereRelPosition(0,-height,0);


	// position the top sphere above ground with appropriate orientation
	startTransform.setOrigin(btVector3(0,0,0)); // no translation intitially
	startTransform.setRotation(pendulumOrientation); // pendulum rotation
	startTransform.setOrigin(startTransform * topSphere1RelPosition); // rotate this position
	startTransform.setOrigin(position + startTransform.getOrigin()); // add non-rotated position to the relative position
	btRigidBody* topSphere1 = createRigidBody(0, startTransform, colShape); // make top sphere static

	// position the top sphere above ground with appropriate orientation
	startTransform.setOrigin(btVector3(0,0,0)); // no translation intitially
	startTransform.setRotation(pendulumOrientation); // pendulum rotation
	startTransform.setOrigin(startTransform * topSphere2RelPosition); // rotate this position
	startTransform.setOrigin(position + startTransform.getOrigin()); // add non-rotated position to the relative position
	btRigidBody* topSphere2 = createRigidBody(0, startTransform, colShape); // make top sphere static

	// position the bottom sphere below the top sphere
	startTransform.setOrigin(btVector3(0,0,0)); // no translation intitially
	startTransform.setRotation(pendulumOrientation); // pendulum rotation
	startTransform.setOrigin(startTransform * bottomSphereRelPosition); // rotate this position
	startTransform.setOrigin(position + startTransform.getOrigin()); // add non-rotated position to the relative position
	btRigidBody* bottomSphere = createRigidBody(mass, startTransform, colShape);
	bottomSphere->setFriction(0); // we do not need friction here
	pendula.push_back(bottomSphere);

	// disable the deactivation when objects do not move anymore
	topSphere1->setActivationState(DISABLE_DEACTIVATION);
	topSphere2->setActivationState(DISABLE_DEACTIVATION);
	bottomSphere->setActivationState(DISABLE_DEACTIVATION);

	bottomSphere->setRestitution(gPendulaRestitution); // set pendula restitution

	// add ropes between spheres
	connectWithRope(topSphere1, bottomSphere);
	connectWithRope(topSphere2, bottomSphere);
}

void NewtonsRopeCradleExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
	btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

		for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
		{
			btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
			//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
			}
		}
}

void NewtonsRopeCradleExample::changePendulaRestitution(btScalar restitution) {
	for (std::vector<btRigidBody*>::iterator rit = pendula.begin();
		rit != pendula.end(); rit++) {
		btAssert((*rit) && "Null constraint");

		(*rit)->setRestitution(restitution);
	}
}

bool NewtonsRopeCradleExample::keyboardCallback(int key, int state) {
	//b3Printf("Key pressed: %d in state %d \n",key,state);

	// key 3
	switch (key) {
	case '3' /*ASCII for 3*/: {
		applyPendulumForce(gDisplacementForce);
		return true;
	}
	}

	return false;
}

void NewtonsRopeCradleExample::applyPendulumForce(btScalar pendulumForce){
	if(pendulumForce != 0){
		b3Printf("Apply %f to pendulum",pendulumForce);
		for (int i = 0; i < gDisplacedPendula; i++) {
			if (gDisplacedPendula >= 0 && gDisplacedPendula <= gPendulaQty)
				pendula[i]->applyCentralForce(btVector3(pendulumForce, 0, 0));
		}
	}
}

// GUI parameter modifiers

void onRopePendulaRestitutionChanged(float pendulaRestitution, void*) {
	if (nex){
		nex->changePendulaRestitution(pendulaRestitution);
	}
}

void applyRForceWithForceScalar(float forceScalar) {
	if(nex){
		btScalar appliedForce = forceScalar * gDisplacementForce;

		if(fabs(gForceScalar) < 0.2f)
			gForceScalar = 0;

		nex->applyPendulumForce(appliedForce);
	}
}

CommonExampleInterface* ET_NewtonsRopeCradleCreateFunc(
	CommonExampleOptions& options) {
	nex = new NewtonsRopeCradleExample(options.m_guiHelper);
	return nex;
}
