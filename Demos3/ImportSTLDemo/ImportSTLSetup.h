#ifndef IMPORT_STL_SETUP_H
#define IMPORT_STL_SETUP_H


#include "Bullet3AppSupport/CommonRigidBodySetup.h"

class ImportSTLSetup : public CommonRigidBodySetup
{
	struct CommonGraphicsApp* m_app;
public:
    ImportSTLSetup(CommonGraphicsApp* app);
    virtual ~ImportSTLSetup();
    
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
};

#endif //IMPORT_OBJ_SETUP_H
