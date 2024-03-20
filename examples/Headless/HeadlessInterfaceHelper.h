/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2020 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/// @author Ian Purvis <ian@purvisresearch.com>

#ifndef BT_HEADLESS_INTERFACE_HELPER_H
#define BT_HEADLESS_INTERFACE_HELPER_H
#include "CommonInterfaces/CommonGUIHelperInterface.h"

struct HeadlessInterfaceHelper : public GUIHelperInterface
{
private:
	CommonRenderInterface* m_renderer;
	CommonParameterInterface* m_parameters;

public:
	HeadlessInterfaceHelper();
	~HeadlessInterfaceHelper();

	void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld);
	void copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16], unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, float* depthBuffer, int depthBufferSizeInPixels, int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels, int startPixelIndex, int width, int height, int* numPixelsCopied);
	void createCollisionObjectGraphicsObject(btCollisionObject* obj, const btVector3& color);
	void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape);
	void createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld);
	void createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color);
	Common2dCanvasInterface* get2dCanvasInterface();
	CommonGraphicsApp* getAppInterface();
	CommonParameterInterface* getParameterInterface();
	CommonRenderInterface* getRenderInterface();
	int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling);
	int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType, int textureId);
	int registerTexture(const unsigned char* texels, int width, int height);
	void removeAllGraphicsInstances();
	void render(const btDiscreteDynamicsWorld* rbWorld);
	void resetCamera(float camDist, float yaw, float pitch, float camPosX, float camPosY, float camPosZ);
	void setUpAxis(int axis);
	void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld);
};

#endif  //BT_HEADLESS_INTERFACE_HELPER_H
