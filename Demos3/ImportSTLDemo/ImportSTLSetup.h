#ifndef IMPORT_STL_SETUP_H
#define IMPORT_STL_SETUP_H


#include "Bullet3AppSupport/CommonRigidBodySetup.h"

class ImportSTLDemo : public CommonRigidBodySetup
{
	struct CommonGraphicsApp* m_app;
public:
    ImportSTLDemo(CommonGraphicsApp* app);
    virtual ~ImportSTLDemo();
    
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
};

#endif //IMPORT_OBJ_SETUP_H
