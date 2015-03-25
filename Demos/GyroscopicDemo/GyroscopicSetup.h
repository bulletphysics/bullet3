
#ifndef GYROSCOPIC_SETUP_H
#define GYROSCOPIC_SETUP_H

#include "Bullet3AppSupport/CommonRigidBodySetup.h"

struct GyroscopicSetup : public CommonRigidBodySetup
{
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

    virtual void syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge);

};


#endif //GYROSCOPIC_SETUP_H
