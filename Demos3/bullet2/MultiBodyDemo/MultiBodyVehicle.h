#ifndef TEST_MULTIBODY_VEHICLE_SETUP_H
#define TEST_MULTIBODY_VEHICLE_SETUP_H

#include "Bullet3AppSupport/CommonMultiBodySetup.h"

struct MultiBodyVehicleSetup : public CommonMultiBodySetup
{
    btMultiBody* m_multiBody;

public:

    MultiBodyVehicleSetup();
    virtual ~MultiBodyVehicleSetup();

    virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

    virtual void stepSimulation(float deltaTime);
    
    class btMultiBody* createMultiBodyVehicle();


};
#endif //TEST_MULTIBODY_VEHICLE_SETUP_H

