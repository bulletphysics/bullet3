#ifndef REMOTE_HELPER_TCP_H
#define REMOTE_HELPER_TCP_H

#include "../CommonInterfaces/CommonGUIHelperInterface.h"

///a RemoteGUIHelper will connect to an existing graphics server over TCP
struct RemoteGUIHelperTCP : public GUIHelperInterface
{
	struct RemoteGUIHelperTCPInternalData* m_data;

	RemoteGUIHelperTCP(const char* hostName, int port);

	virtual ~RemoteGUIHelperTCP();

	virtual void setVisualizerFlag(int flag, int enable);

	virtual void createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color);

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* body, const btVector3& color);

	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape);

	virtual bool getCameraInfo(int* width, int* height, float viewMatrix[16], float projectionMatrix[16], float camUp[3], float camForward[3], float hor[3], float vert[3], float* yaw, float* pitch, float* camDist, float camTarget[3]) const;

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld);
	virtual void syncPhysicsToGraphics2(const class btDiscreteDynamicsWorld* rbWorld);
	virtual void syncPhysicsToGraphics2(const GUISyncPosition* positions, int numPositions);

	virtual void render(const btDiscreteDynamicsWorld* rbWorld);

	virtual void createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld);

	virtual int registerTexture(const unsigned char* texels, int width, int height);
	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType, int textureId);
	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling);
	virtual void removeAllGraphicsInstances();
	virtual void removeGraphicsInstance(int graphicsUid);
	virtual void changeRGBAColor(int instanceUid, const double rgbaColor[4]);

	virtual Common2dCanvasInterface* get2dCanvasInterface();

	virtual CommonParameterInterface* getParameterInterface();

	virtual CommonRenderInterface* getRenderInterface();

	virtual CommonGraphicsApp* getAppInterface();

	virtual void setUpAxis(int axis);
	
	virtual void resetCamera(float camDist, float yaw, float pitch, float camPosX, float camPosY, float camPosZ);
	
	virtual void copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16],
									 unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
									 float* depthBuffer, int depthBufferSizeInPixels,
									 int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels,
									 int startPixelIndex, int width, int height, int* numPixelsCopied);

	virtual void setProjectiveTextureMatrices(const float viewMatrix[16], const float projectionMatrix[16]);
	
	virtual void setProjectiveTexture(bool useProjectiveTexture);

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld);

	virtual void drawText3D(const char* txt, float posX, float posZY, float posZ, float size);

	virtual void drawText3D(const char* txt, float position[3], float orientation[4], float color[4], float size, int optionFlag);

	virtual int addUserDebugLine(const double debugLineFromXYZ[3], const double debugLineToXYZ[3], const double debugLineColorRGB[3], double lineWidth, double lifeTime, int trackingVisualShapeIndex, int replaceItemUid);
	virtual int addUserDebugPoints(const double debugPointPositionXYZ[3], const double debugPointColorRGB[3], double pointSize, double lifeTime, int trackingVisualShapeIndex, int replaceItemUid, int debugPointNum);
	virtual void removeUserDebugItem(int debugItemUniqueId);
	virtual void removeAllUserDebugItems();

	int uploadData(const unsigned char* data, int sizeInBytes, int slot);
	virtual bool isRemoteVisualizer() { return true; }
};

#endif  //REMOTE_HELPER_TCP_H
