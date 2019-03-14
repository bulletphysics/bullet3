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

#include "NewtonsCradle.h"

#include <cmath>
#include <iterator>
#include <vector>  // TODO: Should I use another data structure?

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

static btScalar gPendulaQty = 5;  // Number of pendula in newton's cradle
//TODO: This would actually be an Integer, but the Slider does not like integers, so I floor it when changed

static btScalar gDisplacedPendula = 1;  // number of displaced pendula
//TODO: This is an int as well

static btScalar gPendulaRestitution = 1;  // pendula restitution when hitting against each other

static btScalar gSphereRadius = 1;  // pendula radius

static btScalar gCurrentPendulumLength = 8;  // current pendula length

static btScalar gInitialPendulumLength = 8;  // default pendula length

static btScalar gDisplacementForce = 30;  // default force to displace the pendula

static btScalar gForceScalar = 0;  // default force scalar to apply a displacement

struct NewtonsCradleExample : public CommonRigidBodyBase
{
	NewtonsCradleExample(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper)
	{
	}
	virtual ~NewtonsCradleExample()
	{
	}
	virtual void initPhysics();
	virtual void renderScene();
	virtual void createPendulum(btSphereShape* colShape, const btVector3& position, btScalar length, btScalar mass);
	virtual void changePendulaLength(btScalar length);
	virtual void changePendulaRestitution(btScalar restitution);
	virtual void stepSimulation(float deltaTime);
	virtual bool keyboardCallback(int key, int state);
	virtual void applyPendulumForce(btScalar pendulumForce);
	void resetCamera()
	{
		float dist = 41;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.46, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1],
								 targetPos[2]);
	}

	std::vector<btSliderConstraint*> constraints;  // keep a handle to the slider constraints
	std::vector<btRigidBody*> pendula;             // keep a handle to the pendula
};

static NewtonsCradleExample* nex = NULL;

void onPendulaLengthChanged(float pendulaLength, void* userPtr);  // Change the pendula length

void onPendulaRestitutionChanged(float pendulaRestitution, void* userPtr);  // change the pendula restitution

void applyForceWithForceScalar(float forceScalar);

void NewtonsCradleExample::initPhysics()
{
	{  // create a slider to change the number of pendula
		SliderParams slider("Number of Pendula", &gPendulaQty);
		slider.m_minVal = 1;
		slider.m_maxVal = 50;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{  // create a slider to change the number of displaced pendula
		SliderParams slider("Number of Displaced Pendula", &gDisplacedPendula);
		slider.m_minVal = 0;
		slider.m_maxVal = 49;
		slider.m_clampToIntegers = true;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{  // create a slider to change the pendula restitution
		SliderParams slider("Pendula Restitution", &gPendulaRestitution);
		slider.m_minVal = 0;
		slider.m_maxVal = 1;
		slider.m_clampToNotches = false;
		slider.m_callback = onPendulaRestitutionChanged;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{  // create a slider to change the pendulum length
		SliderParams slider("Pendula Length", &gCurrentPendulumLength);
		slider.m_minVal = 0;
		slider.m_maxVal = 49;
		slider.m_clampToNotches = false;
		slider.m_callback = onPendulaLengthChanged;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{  // create a slider to change the force to displace the lowest pendulum
		SliderParams slider("Displacement force", &gDisplacementForce);
		slider.m_minVal = 0.1;
		slider.m_maxVal = 200;
		slider.m_clampToNotches = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{  // create a slider to apply the force by slider
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
			btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawConstraintLimits);

	{  // create the pendula starting at the indicated position below and where each pendulum has the following mass
		btScalar pendulumMass(1.f);

		btVector3 position(0.0f, 15.0f, 0.0f);  // initial left-most pendulum position
		btQuaternion orientation(0, 0, 0, 1);   // orientation of the pendula

		// Re-using the same collision is better for memory usage and performance
		btSphereShape* pendulumShape = new btSphereShape(gSphereRadius);
		m_collisionShapes.push_back(pendulumShape);

		for (int i = 0; i < std::floor(gPendulaQty); i++)
		{
			// create pendulum
			createPendulum(pendulumShape, position, gInitialPendulumLength, pendulumMass);

			// displace the pendula 1.05 sphere size, so that they all nearly touch (small spacings in between
			position.setX(position.x() - 2.1f * gSphereRadius);
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void NewtonsCradleExample::stepSimulation(float deltaTime)
{
	applyForceWithForceScalar(gForceScalar);  // apply force defined by apply force slider

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(deltaTime);
	}
}

void NewtonsCradleExample::createPendulum(btSphereShape* colShape, const btVector3& position, btScalar length, btScalar mass)
{
	// The pendulum looks like this (names when built):
	// O   topSphere
	// |
	// O   bottomSphere

	//create a dynamic pendulum
	btTransform startTransform;
	startTransform.setIdentity();

	// position the top sphere above ground with a moving x position
	startTransform.setOrigin(position);
	startTransform.setRotation(btQuaternion(0, 0, 0, 1));  // zero rotation
	btRigidBody* topSphere = createRigidBody(mass, startTransform, colShape);

	// position the bottom sphere below the top sphere
	startTransform.setOrigin(
		btVector3(position.x(), btScalar(position.y() - length),
				  position.z()));

	startTransform.setRotation(btQuaternion(0, 0, 0, 1));  // zero rotation
	btRigidBody* bottomSphere = createRigidBody(mass, startTransform, colShape);
	bottomSphere->setFriction(0);  // we do not need friction here
	pendula.push_back(bottomSphere);

	// disable the deactivation when objects do not move anymore
	topSphere->setActivationState(DISABLE_DEACTIVATION);
	bottomSphere->setActivationState(DISABLE_DEACTIVATION);

	bottomSphere->setRestitution(gPendulaRestitution);  // set pendula restitution

	//make the top sphere position "fixed" to the world by attaching with a point to point constraint
	// The pivot is defined in the reference frame of topSphere, so the attachment is exactly at the center of the topSphere
	btVector3 constraintPivot(btVector3(0.0f, 0.0f, 0.0f));
	btPoint2PointConstraint* p2pconst = new btPoint2PointConstraint(*topSphere,
																	constraintPivot);

	p2pconst->setDbgDrawSize(btScalar(5.f));  // set the size of the debug drawing

	// add the constraint to the world
	m_dynamicsWorld->addConstraint(p2pconst, true);

	//create constraint between spheres
	// this is represented by the constraint pivot in the local frames of reference of both constrained spheres
	// furthermore we need to rotate the constraint appropriately to orient it correctly in space
	btTransform constraintPivotInTopSphereRF, constraintPivotInBottomSphereRF;

	constraintPivotInTopSphereRF.setIdentity();
	constraintPivotInBottomSphereRF.setIdentity();

	// the slider constraint is x aligned per default, but we want it to be y aligned, therefore we rotate it
	btQuaternion qt;
	qt.setEuler(0, 0, -SIMD_HALF_PI);
	constraintPivotInTopSphereRF.setRotation(qt);     //we use Y like up Axis
	constraintPivotInBottomSphereRF.setRotation(qt);  //we use Y like up Axis

	//Obtain the position of topSphere in local reference frame of bottomSphere (the pivot is therefore in the center of topSphere)
	btVector3 topSphereInBottomSphereRF =
		(bottomSphere->getWorldTransform().inverse()(
			topSphere->getWorldTransform().getOrigin()));
	constraintPivotInBottomSphereRF.setOrigin(topSphereInBottomSphereRF);

	btSliderConstraint* sliderConst = new btSliderConstraint(*topSphere,
															 *bottomSphere, constraintPivotInTopSphereRF, constraintPivotInBottomSphereRF, true);

	sliderConst->setDbgDrawSize(btScalar(5.f));  // set the size of the debug drawing

	// set limits
	// the initial setup of the constraint defines the origins of the limit dimensions,
	// therefore we set both limits directly to the current position of the topSphere
	sliderConst->setLowerLinLimit(btScalar(0));
	sliderConst->setUpperLinLimit(btScalar(0));
	sliderConst->setLowerAngLimit(btScalar(0));
	sliderConst->setUpperAngLimit(btScalar(0));
	constraints.push_back(sliderConst);

	// add the constraint to the world
	m_dynamicsWorld->addConstraint(sliderConst, true);
}

void NewtonsCradleExample::changePendulaLength(btScalar length)
{
	btScalar lowerLimit = -gInitialPendulumLength;
	for (std::vector<btSliderConstraint*>::iterator sit = constraints.begin();
		 sit != constraints.end(); sit++)
	{
		btAssert((*sit) && "Null constraint");

		//if the pendulum is being shortened beyond it's own length, we don't let the lower sphere to go past the upper one
		if (lowerLimit <= length)
		{
			(*sit)->setLowerLinLimit(length + lowerLimit);
			(*sit)->setUpperLinLimit(length + lowerLimit);
		}
	}
}

void NewtonsCradleExample::changePendulaRestitution(btScalar restitution)
{
	for (std::vector<btRigidBody*>::iterator rit = pendula.begin();
		 rit != pendula.end(); rit++)
	{
		btAssert((*rit) && "Null constraint");

		(*rit)->setRestitution(restitution);
	}
}

void NewtonsCradleExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

bool NewtonsCradleExample::keyboardCallback(int key, int state)
{
	//b3Printf("Key pressed: %d in state %d \n",key,state);

	//key 1, key 2, key 3
	switch (key)
	{
		case '1' /*ASCII for 1*/:
		{
			//assumption: Sphere are aligned in Z axis
			btScalar newLimit = btScalar(gCurrentPendulumLength + 0.1);

			changePendulaLength(newLimit);
			gCurrentPendulumLength = newLimit;

			b3Printf("Increase pendulum length to %f", gCurrentPendulumLength);
			return true;
		}
		case '2' /*ASCII for 2*/:
		{
			//assumption: Sphere are aligned in Z axis
			btScalar newLimit = btScalar(gCurrentPendulumLength - 0.1);

			//is being shortened beyond it's own length, we don't let the lower sphere to go over the upper one
			if (0 <= newLimit)
			{
				changePendulaLength(newLimit);
				gCurrentPendulumLength = newLimit;
			}

			b3Printf("Decrease pendulum length to %f", gCurrentPendulumLength);
			return true;
		}
		case '3' /*ASCII for 3*/:
		{
			applyPendulumForce(gDisplacementForce);
			return true;
		}
	}

	return false;
}

void NewtonsCradleExample::applyPendulumForce(btScalar pendulumForce)
{
	if (pendulumForce != 0)
	{
		b3Printf("Apply %f to pendulum", pendulumForce);
		for (int i = 0; i < gDisplacedPendula; i++)
		{
			if (gDisplacedPendula >= 0 && gDisplacedPendula <= gPendulaQty)
				pendula[i]->applyCentralForce(btVector3(pendulumForce, 0, 0));
		}
	}
}

// GUI parameter modifiers

void onPendulaLengthChanged(float pendulaLength, void*)
{
	if (nex)
	{
		nex->changePendulaLength(pendulaLength);
		//b3Printf("Pendula length changed to %f \n",sliderValue );
	}
}

void onPendulaRestitutionChanged(float pendulaRestitution, void*)
{
	if (nex)
	{
		nex->changePendulaRestitution(pendulaRestitution);
	}
}

void applyForceWithForceScalar(float forceScalar)
{
	if (nex)
	{
		btScalar appliedForce = forceScalar * gDisplacementForce;

		if (fabs(gForceScalar) < 0.2f)
			gForceScalar = 0;

		nex->applyPendulumForce(appliedForce);
	}
}

CommonExampleInterface* ET_NewtonsCradleCreateFunc(
	CommonExampleOptions& options)
{
	nex = new NewtonsCradleExample(options.m_guiHelper);
	return nex;
}
