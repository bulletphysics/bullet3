#ifndef TEST_JOINT_TORQUE_SETUP_H
#define TEST_JOINT_TORQUE_SETUP_H

#include "Bullet3AppSupport/CommonMultiBodySetup.h"

struct TestJointTorqueSetup : public CommonMultiBodySetup
{
    btMultiBody* m_multiBody;

public:

    TestJointTorqueSetup();
    virtual ~TestJointTorqueSetup();

    virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

    virtual void stepSimulation(float deltaTime);

};
#endif //TEST_JOINT_TORQUE_SETUP_H

