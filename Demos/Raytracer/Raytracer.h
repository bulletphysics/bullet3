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
#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "GlutDemoApplication.h"

class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btAxisSweep3;
class btCollisionWorld;

///Raytracer shows the inner working of the ray casting, using ray tracing rendering into a texture.
class Raytracer : public GlutDemoApplication
{

	btDefaultCollisionConfiguration*	m_collisionConfiguration;
	btCollisionDispatcher*	m_dispatcher;
	btAxisSweep3*	m_overlappingPairCache;
	btCollisionWorld*	m_collisionWorld;
	bool	m_initialized;
	
	public:

	void	initPhysics();

	virtual ~Raytracer();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	///worldRaytest performs a ray versus all objects in a collision world, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool	worldRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint);

	///singleObjectRaytest performs a ray versus one collision shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool	singleObjectRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint);
	
	///lowlevelRaytest performs a ray versus convex shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool	lowlevelRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint);

	static DemoApplication* Create()
	{
		Raytracer* demo = new Raytracer();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}	
};

#endif //RAYTRACER_H


