#ifndef CONSTAINT_PHYSICS_SETUP_H
#define CONSTAINT_PHYSICS_SETUP_H

#include "Bullet3AppSupport/CommonRigidBodySetup.h"

struct ConstraintPhysicsSetup : public CommonRigidBodySetup
{
	ConstraintPhysicsSetup();
	virtual ~ConstraintPhysicsSetup();
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

	virtual void stepSimulation(float deltaTime);

};

#endif //CONSTAINT_PHYSICS_SETUP_H
