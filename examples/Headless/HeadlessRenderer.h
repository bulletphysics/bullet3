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

#ifndef BT_HEADLESS_RENDERER_H
#define BT_HEADLESS_RENDERER_H
#include "CommonInterfaces/CommonRenderInterface.h"

struct HeadlessRenderer : public CommonRenderInterface
{
public:
	HeadlessRenderer() = default;
	~HeadlessRenderer() = default;

	void activateTexture(int textureIndex);
	void clearZBuffer();
	void drawLine(const double from[4], const double to[4], const double color[4], double lineWidth);
	void drawLine(const float from[4], const float to[4], const float color[4], float lineWidth);
	void drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize);
	void drawPoint(const double* position, const double color[4], double pointDrawSize);
	void drawPoint(const float* position, const float color[4], float pointDrawSize);
	void drawTexturedTriangleMesh(float worldPosition[3], float worldOrientation[4], const float* vertices, int numvertices, const unsigned int* indices, int numIndices, float color[4], int textureIndex = -1, int vertexLayout = 0);
	CommonCameraInterface* getActiveCamera();
	const CommonCameraInterface* getActiveCamera() const;
	GLInstanceRendererInternalData* getInternalData();
	int getScreenHeight();
	int getScreenWidth();
	int getTotalNumInstances() const;
	void init();
	bool readSingleInstanceTransformToCPU(float* position, float* orientation, int srcIndex);
	int registerGraphicsInstance(int shapeIndex, const double* position, const double* quaternion, const double* color, const double* scaling);
	int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling);
	int registerShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType = B3_GL_TRIANGLES, int textureIndex = -1);
	int registerTexture(const unsigned char* texels, int width, int height, bool flipPixelsY = true);
	void removeAllInstances();
	void removeGraphicsInstance(int instanceUid);
	void removeTexture(int textureIndex);
	void renderScene();
	void renderSceneInternal(int renderMode = B3_DEFAULT_RENDERMODE);
	void resize(int width, int height);
	void setActiveCamera(CommonCameraInterface* cam);
	void setLightPosition(const double lightPos[3]);
	void setLightPosition(const float lightPos[3]);
	void setShadowMapIntensity(double shadowMapIntensity);
	void setShadowMapResolution(int shadowMapResolution);
	void setShadowMapWorldSize(float worldSize);
	void updateCamera(int upAxis);
	void updateShape(int shapeIndex, const float* vertices, int numVertices);
	void updateTexture(int textureIndex, const unsigned char* texels, bool flipPixelsY = true);
	void writeSingleInstanceColorToCPU(const double* color, int srcIndex);
	void writeSingleInstanceColorToCPU(const float* color, int srcIndex);
	void writeSingleInstanceFlagsToCPU(int flags, int srcIndex);
	void writeSingleInstanceScaleToCPU(const double* scale, int srcIndex);
	void writeSingleInstanceScaleToCPU(const float* scale, int srcIndex);
	void writeSingleInstanceSpecularColorToCPU(const double* specular, int srcIndex);
	void writeSingleInstanceSpecularColorToCPU(const float* specular, int srcIndex);
	void writeSingleInstanceTransformToCPU(const double* position, const double* orientation, int srcIndex);
	void writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex);
	void writeTransforms();
};

#endif  //BT_HEADLESS_RENDERER_H
