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

#include "MultiPendulum.h"

#include <vector>  // TODO: Should I use another data structure?
#include <iterator>

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

static btScalar gPendulaQty = 2;  //TODO: This would actually be an Integer, but the Slider does not like integers, so I floor it when changed

static btScalar gDisplacedPendula = 1;  //TODO: This is an int as well

static btScalar gPendulaRestitution = 1;  // Default pendulum restitution is 1 to restore all force

static btScalar gSphereRadius = 1;  // The sphere radius

static btScalar gCurrentPendulumLength = 8;

static btScalar gInitialPendulumLength = 8;  // Default pendulum length (distance between two spheres)

static btScalar gDisplacementForce = 30;  // The default force with which we move the pendulum

static btScalar gForceScalar = 0;  // default force scalar to apply a displacement

struct MultiPendulumExample : public CommonRigidBodyBase
{
	MultiPendulumExample(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper)
	{
	}

	virtual ~MultiPendulumExample()
	{
	}

	virtual void initPhysics();                                                                                                                 // build a multi pendulum
	virtual void renderScene();                                                                                                                 // render the scene to screen
	virtual void createMultiPendulum(btSphereShape* colShape, btScalar pendulaQty, const btVector3& position, btScalar length, btScalar mass);  // create a multi pendulum at the indicated x and y position, the specified number of pendula formed into a chain, each with indicated length and mass
	virtual void changePendulaLength(btScalar length);                                                                                          // change the pendulum length
	virtual void changePendulaRestitution(btScalar restitution);                                                                                // change the pendula restitution
	virtual void stepSimulation(float deltaTime);                                                                                               // step the simulation
	virtual bool keyboardCallback(int key, int state);                                                                                          // handle keyboard callbacks
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

static MultiPendulumExample* mex = NULL;  // Handle to the example to access it via functions. Do not use this in your simulation!

void onMultiPendulaLengthChanged(float pendulaLength, void*);  // Change the pendula length

void onMultiPendulaRestitutionChanged(float pendulaRestitution, void*);  // change the pendula restitution

void applyMForceWithForceScalar(float forceScalar);

void MultiPendulumExample::initPhysics()
{  // Setup your physics scene

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
		slider.m_callback = onMultiPendulaRestitutionChanged;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(
			slider);
	}

	{  // create a slider to change the pendulum length
		SliderParams slider("Pendula Length", &gCurrentPendulumLength);
		slider.m_minVal = 0;
		slider.m_maxVal = 49;
		slider.m_clampToNotches = false;
		slider.m_callback = onMultiPendulaLengthChanged;
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

	{  // create the multipendulum starting at the indicated position below and where each pendulum has the following mass
		btScalar pendulumMass(1.f);

		btVector3 position(0.0f, 15.0f, 0.0f);  // initial top-most pendulum position

		// Re-using the same collision is better for memory usage and performance
		btSphereShape* pendulumShape = new btSphereShape(gSphereRadius);
		m_collisionShapes.push_back(pendulumShape);

		// create multi-pendulum
		createMultiPendulum(pendulumShape, floor(gPendulaQty), position,
							gInitialPendulumLength, pendulumMass);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void MultiPendulumExample::stepSimulation(float deltaTime)
{
	applyMForceWithForceScalar(gForceScalar);  // apply force defined by apply force slider

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(deltaTime);
	}
}

void MultiPendulumExample::createMultiPendulum(btSphereShape* colShape,
											   btScalar pendulaQty, const btVector3& position,
											   btScalar length, btScalar mass)
{
	// The multi-pendulum looks like this (names when built):
	//..........0......./.......1...../.......2......./..etc...:pendulum build iterations
	// O   parentSphere
	// |
	// O   childSphere  / parentSphere
	// |
	// O   ............./ childSphere / parentSphere
	// |
	// O   .........................../ childSphere
	// etc.

	//create the top element of the pendulum
	btTransform startTransform;
	startTransform.setIdentity();

	// position the top sphere
	startTransform.setOrigin(position);

	startTransform.setRotation(btQuaternion(0, 0, 0, 1));  // zero rotation

	btRigidBody* topSphere = createRigidBody(mass, startTransform, colShape);

	// disable the deactivation when object does not move anymore
	topSphere->setActivationState(DISABLE_DEACTIVATION);

	//make top sphere position "fixed" in the world by attaching it to a the world with a point to point constraint
	// The pivot is defined in the reference frame of topSphere, so the attachment should be exactly at the center of topSphere
	btVector3 constraintPivot(0.0f, 0.0f, 0.0f);
	btPoint2PointConstraint* p2pconst = new btPoint2PointConstraint(
		*topSphere, constraintPivot);

	p2pconst->setDbgDrawSize(btScalar(5.f));  // set the size of the debug drawing

	// add the constraint to the world
	m_dynamicsWorld->addConstraint(p2pconst, true);

	btRigidBody* parentSphere = topSphere;  // set the top sphere as the parent sphere for the next sphere to be created

	for (int i = 0; i < pendulaQty; i++)
	{  // produce the number of pendula

		// create joint element to make the pendulum rotate it

		// position the joint sphere at the same position as the top sphere
		startTransform.setOrigin(position - btVector3(0, length * (i), 0));

		startTransform.setRotation(btQuaternion(0, 0, 0, 1));  // zero rotation

		btRigidBody* jointSphere = createRigidBody(mass, startTransform,
												   colShape);
		jointSphere->setFriction(0);  // we do not need friction here

		// disable the deactivation when object does not move anymore
		jointSphere->setActivationState(DISABLE_DEACTIVATION);

		//create constraint between parentSphere and jointSphere
		// this is represented by the constraint pivot in the local frames of reference of both constrained spheres
		btTransform constraintPivotInParentSphereRF, constraintPivotInJointSphereRF;

		constraintPivotInParentSphereRF.setIdentity();
		constraintPivotInJointSphereRF.setIdentity();

		// the orientation of a point-to-point constraint does not matter, as is has no rotational limits

		//Obtain the position of parentSphere in local reference frame of the jointSphere (the pivot is therefore in the center of parentSphere)
		btVector3 parentSphereInJointSphereRF =
			(jointSphere->getWorldTransform().inverse()(
				parentSphere->getWorldTransform().getOrigin()));
		constraintPivotInJointSphereRF.setOrigin(parentSphereInJointSphereRF);

		btPoint2PointConstraint* p2pconst = new btPoint2PointConstraint(
			*parentSphere, *jointSphere, constraintPivotInParentSphereRF.getOrigin(), constraintPivotInJointSphereRF.getOrigin());

		p2pconst->setDbgDrawSize(btScalar(5.f));  // set the size of the debug drawing

		// add the constraint to the world
		m_dynamicsWorld->addConstraint(p2pconst, true);

		// create a slider constraint to change the length of the pendula while it swings

		startTransform.setIdentity();  // reset start transform

		// position the child sphere below the joint sphere
		startTransform.setOrigin(position - btVector3(0, length * (i + 1), 0));

		startTransform.setRotation(btQuaternion(0, 0, 0, 1));  // zero rotation

		btRigidBody* childSphere = createRigidBody(mass, startTransform,
												   colShape);
		childSphere->setFriction(0);  // we do not need friction here
		pendula.push_back(childSphere);

		// disable the deactivation when object does not move anymore
		childSphere->setActivationState(DISABLE_DEACTIVATION);

		//create slider constraint between jointSphere and childSphere
		// this is represented by the constraint pivot in the local frames of reference of both constrained spheres
		// furthermore we need to rotate the constraint appropriately to orient it correctly in space
		btTransform constraintPivotInChildSphereRF;

		constraintPivotInJointSphereRF.setIdentity();
		constraintPivotInChildSphereRF.setIdentity();

		// the orientation of a point-to-point constraint does not matter, as is has no rotational limits

		//Obtain the position of jointSphere in local reference frame of the childSphere (the pivot is therefore in the center of jointSphere)
		btVector3 jointSphereInChildSphereRF =
			(childSphere->getWorldTransform().inverse()(
				jointSphere->getWorldTransform().getOrigin()));
		constraintPivotInChildSphereRF.setOrigin(jointSphereInChildSphereRF);

		// the slider constraint is x aligned per default, but we want it to be y aligned, therefore we rotate it
		btQuaternion qt;
		qt.setEuler(0, 0, -SIMD_HALF_PI);
		constraintPivotInJointSphereRF.setRotation(qt);  //we use Y like up Axis
		constraintPivotInChildSphereRF.setRotation(qt);  //we use Y like up Axis

		btSliderConstraint* sliderConst = new btSliderConstraint(*jointSphere,
																 *childSphere, constraintPivotInJointSphereRF, constraintPivotInChildSphereRF, true);

		sliderConst->setDbgDrawSize(btScalar(5.f));  // set the size of the debug drawing

		// set limits
		// the initial setup of the constraint defines the origins of the limit dimensions,
		// therefore we set both limits directly to the current position of the parentSphere
		sliderConst->setLowerLinLimit(btScalar(0));
		sliderConst->setUpperLinLimit(btScalar(0));
		sliderConst->setLowerAngLimit(btScalar(0));
		sliderConst->setUpperAngLimit(btScalar(0));
		constraints.push_back(sliderConst);

		// add the constraint to the world
		m_dynamicsWorld->addConstraint(sliderConst, true);
		parentSphere = childSphere;
	}
}

void MultiPendulumExample::changePendulaLength(btScalar length)
{
	btScalar lowerLimit = -gInitialPendulumLength;
	for (std::vector<btSliderConstraint*>::iterator sit = constraints.begin();
		 sit != constraints.end(); sit++)
	{
		btAssert((*sit) && "Null constraint");

		// if the pendulum is being shortened beyond it's own length, we don't let the lower sphere to go past the upper one
		if (lowerLimit <= length)
		{
			(*sit)->setLowerLinLimit(length + lowerLimit);
			(*sit)->setUpperLinLimit(length + lowerLimit);
		}
	}
}

void MultiPendulumExample::changePendulaRestitution(btScalar restitution)
{
	for (std::vector<btRigidBody*>::iterator rit = pendula.begin();
		 rit != pendula.end(); rit++)
	{
		btAssert((*rit) && "Null constraint");

		(*rit)->setRestitution(restitution);
	}
}

void MultiPendulumExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

bool MultiPendulumExample::keyboardCallback(int key, int state)
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

void MultiPendulumExample::applyPendulumForce(btScalar pendulumForce)
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

void onMultiPendulaLengthChanged(float pendulaLength, void*)
{  // Change the pendula length
	if (mex)
	{
		mex->changePendulaLength(pendulaLength);
	}
	//b3Printf("Pendula length changed to %f \n",sliderValue );
}

void onMultiPendulaRestitutionChanged(float pendulaRestitution, void*)
{  // change the pendula restitution
	if (mex)
	{
		mex->changePendulaRestitution(pendulaRestitution);
	}
}

void applyMForceWithForceScalar(float forceScalar)
{
	if (mex)
	{
		btScalar appliedForce = forceScalar * gDisplacementForce;

		if (fabs(gForceScalar) < 0.2f)
			gForceScalar = 0;

		mex->applyPendulumForce(appliedForce);
	}
}

CommonExampleInterface* ET_MultiPendulumCreateFunc(
	CommonExampleOptions& options)
{
	mex = new MultiPendulumExample(options.m_guiHelper);
	return mex;
}
