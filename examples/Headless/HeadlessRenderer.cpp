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

#include "HeadlessRenderer.h"

void HeadlessRenderer::activateTexture(int textureIndex){};
void HeadlessRenderer::clearZBuffer(){};
void HeadlessRenderer::drawLine(const double from[4], const double to[4], const double color[4], double lineWidth){};
void HeadlessRenderer::drawLine(const float from[4], const float to[4], const float color[4], float lineWidth){};
void HeadlessRenderer::drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize){};
void HeadlessRenderer::drawPoint(const double* position, const double color[4], double pointDrawSize){};
void HeadlessRenderer::drawPoint(const float* position, const float color[4], float pointDrawSize){};
void HeadlessRenderer::drawTexturedTriangleMesh(float worldPosition[3], float worldOrientation[4], const float* vertices, int numvertices, const unsigned int* indices, int numIndices, float color[4], int textureIndex, int vertexLayout){};
CommonCameraInterface* HeadlessRenderer::getActiveCamera() { return nullptr; };
const CommonCameraInterface* HeadlessRenderer::getActiveCamera() const { return nullptr; };
GLInstanceRendererInternalData* HeadlessRenderer::getInternalData() { return nullptr; };
int HeadlessRenderer::getScreenHeight() { return 0; }
int HeadlessRenderer::getScreenWidth() { return 0; }
int HeadlessRenderer::getTotalNumInstances() const { return 1; };
void HeadlessRenderer::init(){};
bool HeadlessRenderer::readSingleInstanceTransformToCPU(float* position, float* orientation, int srcIndex) { return true; }
int HeadlessRenderer::registerGraphicsInstance(int shapeIndex, const double* position, const double* quaternion, const double* color, const double* scaling) { return 0; };
int HeadlessRenderer::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling) { return 0; };
int HeadlessRenderer::registerShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType, int textureIndex) { return 0; }
int HeadlessRenderer::registerTexture(const unsigned char* texels, int width, int height, bool flipPixelsY) { return 0; }
void HeadlessRenderer::removeAllInstances(){};
void HeadlessRenderer::removeGraphicsInstance(int instanceUid){};
void HeadlessRenderer::removeTexture(int textureIndex){};
void HeadlessRenderer::renderScene() {}
void HeadlessRenderer::renderSceneInternal(int renderMode){};
void HeadlessRenderer::resize(int width, int height){};
void HeadlessRenderer::setActiveCamera(CommonCameraInterface* cam){};
void HeadlessRenderer::setLightPosition(const double lightPos[3]){};
void HeadlessRenderer::setLightPosition(const float lightPos[3]){};
void HeadlessRenderer::setShadowMapIntensity(double shadowMapIntensity){};
void HeadlessRenderer::setShadowMapResolution(int shadowMapResolution){};
void HeadlessRenderer::setShadowMapWorldSize(float worldSize){};
void HeadlessRenderer::updateCamera(int upAxis){};
void HeadlessRenderer::updateShape(int shapeIndex, const float* vertices, int numVertices){};
void HeadlessRenderer::updateTexture(int textureIndex, const unsigned char* texels, bool flipPixelsY){};
void HeadlessRenderer::writeSingleInstanceColorToCPU(const double* color, int srcIndex){};
void HeadlessRenderer::writeSingleInstanceColorToCPU(const float* color, int srcIndex){};
void HeadlessRenderer::writeSingleInstanceFlagsToCPU(int flags, int srcIndex){};
void HeadlessRenderer::writeSingleInstanceScaleToCPU(const double* scale, int srcIndex){};
void HeadlessRenderer::writeSingleInstanceScaleToCPU(const float* scale, int srcIndex){};
void HeadlessRenderer::writeSingleInstanceSpecularColorToCPU(const double* specular, int srcIndex){};
void HeadlessRenderer::writeSingleInstanceSpecularColorToCPU(const float* specular, int srcIndex){};
void HeadlessRenderer::writeSingleInstanceTransformToCPU(const double* position, const double* orientation, int srcIndex){};
void HeadlessRenderer::writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex){};
void HeadlessRenderer::writeTransforms(){};
