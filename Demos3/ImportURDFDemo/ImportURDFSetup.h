#ifndef IMPORT_URDF_SETUP_H
#define IMPORT_URDF_SETUP_H


#include "../../Demos/CommonMultiBodySetup.h"

class ImportUrdfDemo : public CommonMultiBodySetup
{
    char m_fileName[1024];
    
public:
    ImportUrdfDemo();
    virtual ~ImportUrdfDemo();

	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
	virtual void stepSimulation(float deltaTime);
    
    void setFileName(const char* urdfFileName);
};

#endif //IMPORT_URDF_SETUP_H
