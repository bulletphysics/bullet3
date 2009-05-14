/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef FORKLIFT_DEMO_H
#define FORKLIFT_DEMO_H

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;

#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

#include "GlutDemoApplication.h"

///VehicleDemo shows how to setup and use the built-in raycast vehicle
class ForkLiftDemo : public GlutDemoApplication
{
	public:

	btRigidBody* m_carChassis;

//----------------------------
	btRigidBody* m_liftBody;
	btVector3	m_liftStartPos;
	btHingeConstraint* m_liftHinge;

	btRigidBody* m_forkBody;
	btVector3	m_forkStartPos;
	btSliderConstraint* m_forkSlider;

	btRigidBody* m_loadBody;
	btVector3	m_loadStartPos;

	void lockLiftHinge(void);
	void lockForkSlider(void);

	bool m_useDefaultCamera;
//----------------------------


	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;

	
	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster;
	btRaycastVehicle*	m_vehicle;
	btCollisionShape*	m_wheelShape;

	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;


	ForkLiftDemo();

	virtual ~ForkLiftDemo();

	virtual void clientMoveAndDisplay();

	virtual void	clientResetScene();

	virtual void displayCallback();
	
	///a very basic camera following the vehicle
	virtual void updateCamera();

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	void renderme();

	void initPhysics();
	void termPhysics();

	static DemoApplication* Create()
	{
		ForkLiftDemo* demo = new ForkLiftDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
};

#endif // FORKLIFT_DEMO_H


