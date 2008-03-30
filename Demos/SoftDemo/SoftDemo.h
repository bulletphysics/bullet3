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

///btSoftBody implementation by Nathanael Presson

#ifndef SOFT_DEMO_H
#define SOFT_DEMO_H

#include "DemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/SoftBody/btSoftBody.h"
#include "BulletDynamics/SoftBody/btSparseSDF.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;


///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class SoftDemo : public DemoApplication
{
public:
struct	SoftBodyImpl : btSoftBody::ISoftBody
	{
	void		Attach(btSoftBody*);
	void		Detach(btSoftBody*);
	void		StartCollide(const btVector3&,const btVector3&);
	bool		CheckContactPrecise(const btVector3&,
									btSoftBody::ISoftBody::sCti&);
	bool		CheckContact(	const btVector3&,
								btSoftBody::ISoftBody::sCti&);
	void		EndCollide();
	void		EvaluateMedium(	const btVector3&,
								btSoftBody::ISoftBody::sMedium&);
	SoftDemo*	pdemo;
	btScalar	air_density;
	btScalar	water_density;
	btScalar	water_offset;
	btVector3	water_normal;	
	}									m_softbodyimpl;
	btAlignedObjectArray<btSoftBody*>	m_softbodies;
	btSparseSdf<3>						m_sparsesdf;
	bool								m_autocam;
	

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	class	Win32ThreadSupport*		m_threadSupportCollision;
	class	Win32ThreadSupport*		m_threadSupportSolver;
#endif
#endif

	btConstraintSolver*	m_solver;

	btCollisionAlgorithmCreateFunc*	m_boxBoxCF;

	btDefaultCollisionConfiguration* m_collisionConfiguration;


	public:

	void	initPhysics();

	void	exitPhysics();

	virtual ~SoftDemo()
	{
		exitPhysics();
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	
	void createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos );
	
	static DemoApplication* Create()
	{
		SoftDemo* demo = new SoftDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
	//
	void	clientResetScene();
	void	renderme();
	void	keyboardCallback(unsigned char key, int x, int y);

};

#endif //CCD_PHYSICS_DEMO_H

