#ifndef IMPORT_URDF_SETUP_H
#define IMPORT_URDF_SETUP_H


#include "../../Demos/CommonMultiBodySetup.h"

class ImportUrdfDemo : public CommonMultiBodySetup
{
public:
    ImportUrdfDemo();
    virtual ~ImportUrdfDemo();

	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
};

#endif //IMPORT_URDF_SETUP_H
