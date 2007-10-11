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
#ifndef TEST_CONCAVE_DEMO_H
#define TEST_CONCAVE_DEMO_H

#include "DemoApplication.h"




struct btCollisionAlgorithmCreateFunc;

///StressTestDemo shows usage of static concave triangle meshes
///It also shows per-triangle material (friction/restitution) through CustomMaterialCombinerCallback
class StressTestDemo : public DemoApplication
{

public:
	StressTestDemo()
		: m_collisionConfiguration(NULL),
		  m_dispatcher(NULL),
		  m_broadphase(NULL),
		  m_constraintSolver(NULL),
		  m_steps_done(0),
		  m_spheresphere_collisionCreateFunc(0),
		  m_spherebox_collisionCreateFunc(0)
	{
	}

	virtual ~StressTestDemo()
	{
		delete m_spheresphere_collisionCreateFunc;
		delete m_spherebox_collisionCreateFunc;

		delete m_collisionConfiguration;
		delete m_dispatcher;
		delete m_broadphase;
		delete m_constraintSolver;

		delete m_dynamicsWorld;
	}

	void	initGImpactCollision();
	void	initPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void clientResetScene();

	virtual void keyboardCallback(unsigned char key, int x, int y);



public: ///data
	unsigned int			m_steps_done;


	btVector3				kinTorusTran;
	btQuaternion			kinTorusRot;
	btRigidBody				*kinematicTorus;


	btCollisionAlgorithmCreateFunc*  m_spheresphere_collisionCreateFunc;
	btCollisionAlgorithmCreateFunc*  m_spherebox_collisionCreateFunc;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btCollisionDispatcher*			 m_dispatcher;
	btBroadphaseInterface*			 m_broadphase;
	btConstraintSolver*				 m_constraintSolver;
};


#define SPHERES 1

#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3


#define ARRAY_SIZE_X 10
#define ARRAY_SIZE_Y 10
#define ARRAY_SIZE_Z 10

#endif //CONCAVE_DEMO_H

