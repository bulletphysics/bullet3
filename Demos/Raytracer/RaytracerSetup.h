#ifndef RAYTRACER_SETUP_H
#define RAYTRACER_SETUP_H


#include "Bullet3AppSupport/CommonRigidBodySetup.h"

struct RaytracerPhysicsSetup : public CommonPhysicsSetup
{
	
	struct RaytracerInternalData* m_internalData;

	RaytracerPhysicsSetup();

	virtual ~RaytracerPhysicsSetup();

	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

	virtual void exitPhysics();

	virtual void stepSimulation(float deltaTime);

    virtual void    debugDraw(int debugDrawFlags);

	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual void removePickingConstraint();

	virtual void syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge);

		///worldRaytest performs a ray versus all objects in a collision world, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool	worldRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint);

	///singleObjectRaytest performs a ray versus one collision shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool	singleObjectRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint);
	
	///lowlevelRaytest performs a ray versus convex shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool	lowlevelRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint);


};

#endif //RAYTRACER_SETUP_H
