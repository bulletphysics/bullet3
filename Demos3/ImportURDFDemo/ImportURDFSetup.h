#ifndef IMPORT_URDF_SETUP_H
#define IMPORT_URDF_SETUP_H


#include "Bullet3AppSupport/CommonMultiBodySetup.h"

class ImportUrdfSetup : public CommonMultiBodySetup
{
    char m_fileName[1024];
    
public:
    ImportUrdfSetup();
    virtual ~ImportUrdfSetup();

	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
	virtual void stepSimulation(float deltaTime);
    
    void setFileName(const char* urdfFileName);
};

#endif //IMPORT_URDF_SETUP_H
