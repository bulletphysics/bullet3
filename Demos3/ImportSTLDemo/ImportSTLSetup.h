#ifndef IMPORT_STL_SETUP_H
#define IMPORT_STL_SETUP_H


#include "../../Demos/CommonRigidBodySetup.h"

class ImportSTLDemo : public CommonRigidBodySetup
{
	struct SimpleOpenGL3App* m_app;
public:
    ImportSTLDemo(SimpleOpenGL3App* app);
    virtual ~ImportSTLDemo();
    
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
};

#endif //IMPORT_OBJ_SETUP_H
