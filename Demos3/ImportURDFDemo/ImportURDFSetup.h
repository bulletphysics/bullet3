#ifndef IMPORT_URDF_SETUP_H
#define IMPORT_URDF_SETUP_H


#include "../../Demos/CommonRigidBodySetup.h"

class ImportUrdfDemo : public CommonRigidBodySetup
{
public:
    ImportUrdfDemo();
    virtual ~ImportUrdfDemo();
    
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
};

#endif //IMPORT_URDF_SETUP_H
