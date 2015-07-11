#ifndef OPENGL_GUI_HELPER_H
#define OPENGL_GUI_HELPER_H
#include "../CommonInterfaces/CommonGUIHelperInterface.h"

class btCollisionShape;
class btTransform;
#include "LinearMath/btAlignedObjectArray.h"

struct OpenGLGuiHelper : public GUIHelperInterface
{
	struct OpenGLGuiHelperInternalData* m_data;

	OpenGLGuiHelper(struct CommonGraphicsApp* glApp, bool useOpenGL2);

	virtual ~OpenGLGuiHelper();

	virtual struct CommonRenderInterface* getRenderInterface();

	virtual void createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color);

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* body, const btVector3& color);

	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices);

	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling);

	

	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape);

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld);

	
	virtual void render(const btDiscreteDynamicsWorld* rbWorld);

	virtual void createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld);

	virtual struct Common2dCanvasInterface*	get2dCanvasInterface();

	virtual CommonParameterInterface* getParameterInterface();

	virtual struct CommonGraphicsApp* getAppInterface();

	virtual void setUpAxis(int axis);
	
	virtual void resetCamera(float camDist, float pitch, float yaw, float camPosX,float camPosY, float camPosZ);

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) ;
    
    virtual void drawText3D( const char* txt, float posX, float posY, float posZ, float size);

	void renderInternalGl2(int  pass, const btDiscreteDynamicsWorld* dynamicsWorld);
};

#endif //OPENGL_GUI_HELPER_H

