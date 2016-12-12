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



#include "InclinedPlane.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

static btScalar gTilt = 20.0f/180.0f*SIMD_PI; // tilt the ramp 20 degrees

static btScalar gRampFriction = 1; // set ramp friction to 1

static btScalar gRampRestitution = 0; // set ramp restitution to 0 (no restitution)

static btScalar gBoxFriction = 1; // set box friction to 1

static btScalar gBoxRestitution = 0; // set box restitution to 0

static btScalar gSphereFriction = 1; // set sphere friction to 1

static btScalar gSphereRollingFriction = 1; // set sphere rolling friction to 1

static btScalar gSphereRestitution = 0; // set sphere restitution to 0

// handles for changes
static btRigidBody* ramp = NULL;
static btRigidBody* gBox = NULL;
static btRigidBody* gSphere = NULL;

struct InclinedPlaneExample : public CommonRigidBodyBase
{
	InclinedPlaneExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~InclinedPlaneExample(){}
	virtual void initPhysics();
	virtual void resetScene();
	virtual void renderScene();
	virtual void stepSimulation(float deltaTime);
	virtual bool keyboardCallback(int key, int state);
	void resetCamera()
	{
		float dist = 41;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}



};

void onBoxFrictionChanged(float friction, void* userPtr);

void onBoxRestitutionChanged(float restitution, void* userPtr);

void onSphereFrictionChanged(float friction, void* userPtr);

void onSphereRestitutionChanged(float restitution, void* userPtr);

void onRampInclinationChanged(float inclination, void* userPtr);

void onRampFrictionChanged(float friction, void* userPtr);

void onRampRestitutionChanged(float restitution, void* userPtr);

void InclinedPlaneExample::initPhysics()
{

	{ // create slider to change the ramp tilt
    SliderParams slider("Ramp Tilt",&gTilt);
    slider.m_minVal=0;
    slider.m_maxVal=SIMD_PI/2.0f;
    slider.m_clampToNotches = false;
    slider.m_callback = onRampInclinationChanged;
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

    { // create slider to change the ramp friction
    SliderParams slider("Ramp Friction",&gRampFriction);
    slider.m_minVal=0;
    slider.m_maxVal=10;
    slider.m_clampToNotches = false;
    slider.m_callback = onRampFrictionChanged;
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

    { // create slider to change the ramp restitution
    SliderParams slider("Ramp Restitution",&gRampRestitution);
    slider.m_minVal=0;
    slider.m_maxVal=1;
    slider.m_clampToNotches = false;
    slider.m_callback = onRampRestitutionChanged;
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

    { // create slider to change the box friction
    SliderParams slider("Box Friction",&gBoxFriction);
    slider.m_minVal=0;
    slider.m_maxVal=10;
    slider.m_clampToNotches = false;
    slider.m_callback = onBoxFrictionChanged;
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

    { // create slider to change the box restitution
    SliderParams slider("Box Restitution",&gBoxRestitution);
    slider.m_minVal=0;
    slider.m_maxVal=1;
    slider.m_clampToNotches = false;
    slider.m_callback = onBoxRestitutionChanged;
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

    { // create slider to change the sphere friction
    SliderParams slider("Sphere Friction",&gSphereFriction);
    slider.m_minVal=0;
    slider.m_maxVal=10;
    slider.m_clampToNotches = false;
    slider.m_callback = onSphereFrictionChanged;
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

    { // create slider to change the sphere rolling friction
    SliderParams slider("Sphere Rolling Friction",&gSphereRollingFriction);
    slider.m_minVal=0;
    slider.m_maxVal=10;
    slider.m_clampToNotches = false;
    slider.m_callback = onSphereRestitutionChanged;
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

    { // create slider to change the sphere restitution
    SliderParams slider("Sphere Restitution",&gSphereRestitution);
    slider.m_minVal=0;
    slider.m_maxVal=1;
    slider.m_clampToNotches = false;
    m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

	m_guiHelper->setUpAxis(1); // set Y axis as up axis

	createEmptyDynamicsWorld();
	
	// create debug drawer
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);


	{ // create a static ground
		btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-50,0));

		btScalar mass(0.);
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
	}

	{ //create a static inclined plane
		btBoxShape* inclinedPlaneShape = createBoxShape(btVector3(btScalar(20.),btScalar(1.),btScalar(10.)));
		m_collisionShapes.push_back(inclinedPlaneShape);

		btTransform startTransform;
		startTransform.setIdentity();

		// position the inclined plane above ground
		startTransform.setOrigin(btVector3(
								 btScalar(0),
								 btScalar(15),
								 btScalar(0)));

		btQuaternion incline;
		incline.setRotation(btVector3(0,0,1),gTilt);
		startTransform.setRotation(incline);

		btScalar mass(0.);
		ramp = createRigidBody(mass,startTransform,inclinedPlaneShape);
		ramp->setFriction(gRampFriction);
		ramp->setRestitution(gRampRestitution);
	}


	{ //create a cube above the inclined plane
        btBoxShape* boxShape = createBoxShape(btVector3(1,1,1));
		 
		m_collisionShapes.push_back(boxShape);

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar boxMass(1.f);

		startTransform.setOrigin(
			btVector3(btScalar(0), btScalar(20), btScalar(2)));

		gBox = createRigidBody(boxMass, startTransform, boxShape);
		gBox->forceActivationState(DISABLE_DEACTIVATION); // to prevent the box on the ramp from disabling
		gBox->setFriction(gBoxFriction);
		gBox->setRestitution(gBoxRestitution);
	}

	{ //create a sphere above the inclined plane
        btSphereShape* sphereShape = new btSphereShape(btScalar(1));

		m_collisionShapes.push_back(sphereShape);

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar sphereMass(1.f);

		startTransform.setOrigin(
			btVector3(btScalar(0), btScalar(20), btScalar(4)));

		gSphere = createRigidBody(sphereMass, startTransform, sphereShape);
		gSphere->forceActivationState(DISABLE_DEACTIVATION); // to prevent the sphere on the ramp from disabling
		gSphere->setFriction(gSphereFriction);
		gSphere->setRestitution(gSphereRestitution);
		gSphere->setRollingFriction(gSphereRollingFriction);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void InclinedPlaneExample::resetScene() {
	{ //reset a cube above the inclined plane

		btTransform startTransform;
		startTransform.setIdentity();

		startTransform.setOrigin(
			btVector3(btScalar(0), btScalar(20), btScalar(2)));

		gBox->setWorldTransform(startTransform);
		btVector3 zero(0, 0, 0);
		gBox->setAngularVelocity(zero);
		gBox->setLinearVelocity(zero);
		gBox->clearForces();
	}

	{ //reset a sphere above the inclined plane
		btTransform startTransform;
		startTransform.setIdentity();

		startTransform.setOrigin(
			btVector3(btScalar(0), btScalar(20), btScalar(4)));

		gSphere->setWorldTransform(startTransform);
		btVector3 zero(0, 0, 0);
		gSphere->setAngularVelocity(zero);
		gSphere->setLinearVelocity(zero);
		gSphere->clearForces();
	}
}

void InclinedPlaneExample::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(deltaTime);
	}

}


void InclinedPlaneExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

bool InclinedPlaneExample::keyboardCallback(int key, int state) {
//	b3Printf("Key pressed: %d in state %d \n",key,state);

	switch (key) {
	case 32 /*ASCII for space*/: {
		resetScene();
		break;
	}
	}

	return false;
}


// GUI parameter modifiers
void onBoxFrictionChanged(float friction, void*){
	if(gBox){
		gBox->setFriction(friction);
//		b3Printf("Friction of box changed to %f",friction );
	}
}

void onBoxRestitutionChanged(float restitution, void*){
	if(gBox){
		gBox->setRestitution(restitution);
		//b3Printf("Restitution of box changed to %f",restitution);
	}
}

void onSphereFrictionChanged(float friction, void*){
	if(gSphere){
		gSphere->setFriction(friction);
		//b3Printf("Friction of sphere changed to %f",friction );
	}
}

void onSphereRestitutionChanged(float restitution, void*){
	if(gSphere){
		gSphere->setRestitution(restitution);
		//b3Printf("Restitution of sphere changed to %f",restitution);
	}
}

void onRampInclinationChanged(float inclination, void*){
	if(ramp){
		btTransform startTransform;
		startTransform.setIdentity();

		// position the inclined plane above ground
		startTransform.setOrigin(
			btVector3(btScalar(0), btScalar(15), btScalar(0)));

		btQuaternion incline;
		incline.setRotation(btVector3(0,0,1),gTilt);
		startTransform.setRotation(incline);
		ramp->setWorldTransform(startTransform);
		//b3Printf("Inclination of ramp changed to %f",inclination );
	}
}

void onRampFrictionChanged(float friction, void*){
	if(ramp){
		ramp->setFriction(friction);
		//b3Printf("Friction of ramp changed to %f \n",friction );
	}
}

void onRampRestitutionChanged(float restitution, void*){
	if(ramp){
		ramp->setRestitution(restitution);
		//b3Printf("Restitution of ramp changed to %f \n",restitution);
	}
}


CommonExampleInterface*    ET_InclinedPlaneCreateFunc(CommonExampleOptions& options)
{
	return new InclinedPlaneExample(options.m_guiHelper);
}
