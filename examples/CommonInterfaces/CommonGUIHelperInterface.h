#ifndef GUI_HELPER_INTERFACE_H
#define GUI_HELPER_INTERFACE_H


class btRigidBody;
class btVector3;
class btCollisionObject;
class btDiscreteDynamicsWorld;
class btCollisionShape;
struct Common2dCanvasInterface;
struct CommonParameterInterface;
struct CommonRenderInterface;
struct CommonGraphicsApp;

///The Bullet 2 GraphicsPhysicsBridge let's the graphics engine create graphics representation and synchronize
struct GUIHelperInterface
{
	virtual ~GUIHelperInterface() {}

	virtual void createRigidBodyGraphicsObject(btRigidBody* body,const btVector3& color) = 0;

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj,const btVector3& color) = 0;

	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)=0;

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)=0;

	virtual void render(const btDiscreteDynamicsWorld* rbWorld)=0;

	virtual void createPhysicsDebugDrawer( btDiscreteDynamicsWorld* rbWorld)=0;

	virtual int	registerTexture(const unsigned char* texels, int width, int height)=0;
	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType, int textureId) = 0;
	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling) =0;
    virtual void removeAllGraphicsInstances()=0;
	
	virtual Common2dCanvasInterface* get2dCanvasInterface()=0;
	
	virtual CommonParameterInterface* getParameterInterface()=0;

	virtual CommonRenderInterface* getRenderInterface()=0;

	virtual CommonGraphicsApp* getAppInterface()=0;

	virtual void setUpAxis(int axis)=0;

	virtual void resetCamera(float camDist, float pitch, float yaw, float camPosX,float camPosY, float camPosZ)=0;
	
	virtual void copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16], 
                                  unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, 
                                  float* depthBuffer, int depthBufferSizeInPixels, 
                                  int startPixelIndex, int destinationWidth, int destinationHeight, int* numPixelsCopied)
  {
      copyCameraImageData(viewMatrix,projectionMatrix,pixelsRGBA,rgbaBufferSizeInPixels,
                           depthBuffer,depthBufferSizeInPixels,
                           0,0,
                           startPixelIndex,destinationWidth,
                           destinationHeight,numPixelsCopied);
  }

    virtual void copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16], 
                                  unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, 
                                  float* depthBuffer, int depthBufferSizeInPixels, 
                                  int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels,
                                  int startPixelIndex, int destinationWidth, int destinationHeight, int* numPixelsCopied)=0;

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) =0;
	
	virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size)=0;


	
	virtual int		addUserDebugText3D( const char* txt, const double posisionXYZ[3], const double	textColorRGB[3], double size, double lifeTime){return -1;};
	virtual int		addUserDebugLine(const double	debugLineFromXYZ[3], const double	debugLineToXYZ[3], const double	debugLineColorRGB[3], double lineWidth, double lifeTime ){return -1;};
	virtual int		addUserDebugParameter(const char* txt, double	rangeMin, double	rangeMax, double startValue){return -1;};
	virtual int		readUserDebugParameter(int itemUniqueId, double* value) { return 0;}

	virtual void	removeUserDebugItem( int debugItemUniqueId){};
	virtual void	removeAllUserDebugItems( ){};

};


///the DummyGUIHelper does nothing, so we can test the examples without GUI/graphics (in 'console mode')
struct DummyGUIHelper : public GUIHelperInterface
{
	DummyGUIHelper() {}
	virtual ~DummyGUIHelper() {}

	virtual void createRigidBodyGraphicsObject(btRigidBody* body,const btVector3& color){}

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj,const btVector3& color) {}

	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape){}

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld){}

	virtual void render(const btDiscreteDynamicsWorld* rbWorld) {}

	virtual void createPhysicsDebugDrawer( btDiscreteDynamicsWorld* rbWorld){}

	virtual int	registerTexture(const unsigned char* texels, int width, int height){return -1;}
	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType, int textureId){return -1;}
	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling) {return -1;}
    virtual void removeAllGraphicsInstances(){}
	
	virtual Common2dCanvasInterface* get2dCanvasInterface()
	{
		return 0;
	}
	
	virtual CommonParameterInterface* getParameterInterface()
	{
		return 0;
	}

	virtual CommonRenderInterface* getRenderInterface()
	{
		return 0;
	}
	
	virtual CommonGraphicsApp* getAppInterface()
	{
		return 0;
	}


	virtual void setUpAxis(int axis)
	{
	}
	virtual void resetCamera(float camDist, float pitch, float yaw, float camPosX,float camPosY, float camPosZ)
	{
	}

	virtual void copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16], 
                                  unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, 
                                  float* depthBuffer, int depthBufferSizeInPixels,
                                  int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels,
                                  int startPixelIndex, int width, int height, int* numPixelsCopied)
	
	{
        if (numPixelsCopied)
            *numPixelsCopied = 0;
	}

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) 
	{
	}
    
	virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size)
	{
	}

	virtual int		addUserDebugText3D( const char* txt, const double positionXYZ[3], const double	textColorRGB[3], double size, double lifeTime)
	{
		return -1;
	}
	virtual int		addUserDebugLine(const double	debugLineFromXYZ[3], const double	debugLineToXYZ[3], const double	debugLineColorRGB[3], double lineWidth, double lifeTime )
	{
		return -1;
	}
	virtual void	removeUserDebugItem( int debugItemUniqueId)
	{
	}
	virtual void	removeAllUserDebugItems( )
	{
	}

};

#endif //GUI_HELPER_INTERFACE_H

