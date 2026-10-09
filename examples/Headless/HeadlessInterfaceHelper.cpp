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

/// After DummyGUIHelper
/// original author: Erwin Coumans
/// @author Ian Purvis <ian@purvisresearch.com>

#include "HeadlessInterfaceHelper.h"
#include "Headless/HeadlessParameters.h"
#include "Headless/HeadlessRenderer.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"

HeadlessInterfaceHelper::HeadlessInterfaceHelper()
{
	m_renderer = new HeadlessRenderer();
	m_parameters = new HeadlessParameters();
}

HeadlessInterfaceHelper::~HeadlessInterfaceHelper()
{
	delete m_renderer;
	delete m_parameters;
}

void HeadlessInterfaceHelper::autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) {}
void HeadlessInterfaceHelper::copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16], unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, float* depthBuffer, int depthBufferSizeInPixels, int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels, int startPixelIndex, int width, int height, int* numPixelsCopied) { *numPixelsCopied = 0; }
void HeadlessInterfaceHelper::createCollisionObjectGraphicsObject(btCollisionObject* obj, const btVector3& color) {}
void HeadlessInterfaceHelper::createCollisionShapeGraphicsObject(btCollisionShape* collisionShape) {}
void HeadlessInterfaceHelper::createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld) {}
void HeadlessInterfaceHelper::createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color) {}
Common2dCanvasInterface* HeadlessInterfaceHelper::get2dCanvasInterface() { return nullptr; }
CommonGraphicsApp* HeadlessInterfaceHelper::getAppInterface() { return nullptr; }
CommonParameterInterface* HeadlessInterfaceHelper::getParameterInterface() { return m_parameters; }
CommonRenderInterface* HeadlessInterfaceHelper::getRenderInterface() { return m_renderer; }
int HeadlessInterfaceHelper::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling) { return -1; }
int HeadlessInterfaceHelper::registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType, int textureId) { return -1; }
int HeadlessInterfaceHelper::registerTexture(const unsigned char* texels, int width, int height) { return -1; }
void HeadlessInterfaceHelper::removeAllGraphicsInstances() {}
void HeadlessInterfaceHelper::render(const btDiscreteDynamicsWorld* rbWorld) {}
void HeadlessInterfaceHelper::resetCamera(float camDist, float yaw, float pitch, float camPosX, float camPosY, float camPosZ) {}
void HeadlessInterfaceHelper::setUpAxis(int axis) {}
void HeadlessInterfaceHelper::syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld) {}
