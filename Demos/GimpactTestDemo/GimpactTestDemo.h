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


#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

class btTriangleIndexVertexArray;
class btDefaultCollisionConfiguration;

//#define BULLET_TRIANGLE_COLLISION 1
#define BULLET_GIMPACT 1
//#define BULLET_GIMPACT_CONVEX_DECOMPOSITION 1

#define TEST_GIMPACT_TORUS

#ifdef BULLET_GIMPACT

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
	#ifdef BULLET_GIMPACT_CONVEX_DECOMPOSITION
	#include "../Extras/GIMPACTUtils/btGImpactConvexDecompositionShape.h"
	#endif


#else

#include "BulletCollision/Gimpact/btConcaveConcaveCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGIMPACTMeshShape.h"

#endif



class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;

///GimpactConcaveDemo shows usage of static concave triangle meshes
///It also shows per-triangle material (friction/restitution) through CustomMaterialCombinerCallback
class GimpactConcaveDemo : public PlatformDemoApplication
{

public:
	GimpactConcaveDemo()
		:	m_steps_done(0), 
			m_trimeshShape(NULL),
		  m_trimeshShape2(NULL),
		  m_indexVertexArrays(NULL),
		  m_indexVertexArrays2(NULL),
	

	kinematicTorus(NULL),
	
	
	m_gimpactCollisionCreateFunc(NULL),
	m_collisionConfiguration(NULL),

	m_dispatcher(NULL),

	m_broadphase(NULL),	 
		 m_constraintSolver(NULL)
	{
	}

	virtual ~GimpactConcaveDemo();


	void	initGImpactCollision();
	void	initPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void clientResetScene();

	virtual void renderme();
	virtual void keyboardCallback(unsigned char key, int x, int y);

	///Demo functions
	void	shootTrimesh(const btVector3& destination);

public: ///data
	unsigned int			m_steps_done;

#ifdef BULLET_GIMPACT
	btCollisionShape			*m_trimeshShape;
	btCollisionShape			*m_trimeshShape2;
#else

	btGIMPACTMeshData * m_trimeshShape;
	btGIMPACTMeshData * m_trimeshShape2;

	btCollisionShape * createTorusShape();
	btCollisionShape * createBunnyShape();

#endif

		//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btTriangleIndexVertexArray  *m_indexVertexArrays;
	btTriangleIndexVertexArray  *m_indexVertexArrays2;

	btVector3				kinTorusTran;
	btQuaternion			kinTorusRot;
	btRigidBody				*kinematicTorus;


	btCollisionAlgorithmCreateFunc*  m_gimpactCollisionCreateFunc;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btCollisionDispatcher*			 m_dispatcher;
	btBroadphaseInterface*			 m_broadphase;
	btConstraintSolver*				 m_constraintSolver;

	static DemoApplication* Create()
	{
		GimpactConcaveDemo* demo = new GimpactConcaveDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
};

#endif //CONCAVE_DEMO_H

