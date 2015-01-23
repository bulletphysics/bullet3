#ifndef IMPORT_OBJ_SETUP_H
#define IMPORT_OBJ_SETUP_H


#include "Bullet3AppSupport/CommonRigidBodySetup.h"

class ImportObjSetup : public CommonRigidBodySetup
{
	struct CommonGraphicsApp* m_app;
public:
    ImportObjSetup(CommonGraphicsApp* app);
    virtual ~ImportObjSetup();
    
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
};

#endif //IMPORT_OBJ_SETUP_H
