#ifndef GUI_HELPER_INTERFACE_H
#define GUI_HELPER_INTERFACE_H


class btRigidBody;
class btVector3;
class btCollisionObject;
class btDiscreteDynamicsWorld;
class btCollisionShape;



///The Bullet 2 GraphicsPhysicsBridge let's the graphics engine create graphics representation and synchronize
struct GUIHelperInterface
{
	virtual ~GUIHelperInterface() {}

	virtual void createRigidBodyGraphicsObject(btRigidBody* body,const btVector3& color)
	{
	}

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj,const btVector3& color)
	{
	}
	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
	}
	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
	{
	}
	virtual void createPhysicsDebugDrawer( btDiscreteDynamicsWorld* rbWorld)
	{
	}

	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices) { return -1; }//, int primitiveType = B3_GL_TRIANGLES, int textureIndex = -1);

	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling) { return -1;}

	virtual struct Common2dCanvasInterface* get2dCanvasInterface()
	{
		return 0;
	}
	
	virtual struct CommonParameterInterface* getParameterInterface()
	{
		return 0;
	}

	virtual struct CommonRenderInterface* getRenderInterface()
	{
		return 0;
	}
	
	virtual struct CommonGraphicsApp* getAppInterface()
	{
		return 0;
	}


	virtual void setUpAxis(int axis)
	{
	}

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) 
	{
	}
    
    virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size)
    {
        
    }
};

#endif //GUI_HELPER_INTERFACE_H

