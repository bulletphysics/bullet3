#ifndef IMPORT_OBJ_SETUP_H
#define IMPORT_OBJ_SETUP_H


#include "../../Demos/CommonRigidBodySetup.h"

class ImportObjDemo : public CommonRigidBodySetup
{
	struct SimpleOpenGL3App* m_app;
public:
    ImportObjDemo(SimpleOpenGL3App* app);
    virtual ~ImportObjDemo();
    
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
};

#endif //IMPORT_OBJ_SETUP_H
