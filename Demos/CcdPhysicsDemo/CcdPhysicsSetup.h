
#ifndef CCD_PHYSICS_SETUP_H
#define CCD_PHYSICS_SETUP_H


#include "../CommonRigidBodySetup.h"

struct CcdPhysicsSetup : public CommonRigidBodySetup
{
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

};

struct KinematicObjectSetup : public CommonRigidBodySetup
{
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

	virtual void stepSimulation(float deltaTime);

};


#endif //CCD_PHYSICS_SETUP_H
