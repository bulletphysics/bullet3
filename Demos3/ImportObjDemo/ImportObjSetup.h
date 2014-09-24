#ifndef IMPORT_OBJ_SETUP_H
#define IMPORT_OBJ_SETUP_H


#include "Bullet3AppSupport/CommonRigidBodySetup.h"

class ImportObjDemo : public CommonRigidBodySetup
{
	struct CommonGraphicsApp* m_app;
public:
    ImportObjDemo(CommonGraphicsApp* app);
    virtual ~ImportObjDemo();
    
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
};

#endif //IMPORT_OBJ_SETUP_H
