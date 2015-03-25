#ifndef GENERIC_6DOF_SPRING2_CONSTRAINT_DEMO_H
#define GENERIC_6DOF_SPRING2_CONSTRAINT_DEMO_H

#include "Bullet3AppSupport/CommonRigidBodySetup.h"

struct Dof6Spring2Setup : public CommonRigidBodySetup
{
	struct Dof6Spring2SetupInternalData* m_data;

	Dof6Spring2Setup();
	virtual ~Dof6Spring2Setup();
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

	virtual void stepSimulation(float deltaTime);

	void animate();
};

#endif //GENERIC_6DOF_SPRING2_CONSTRAINT_DEMO_H
