/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#ifndef GL_INSTANCING_RENDERER_H
#define GL_INSTANCING_RENDERER_H

#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "SimpleCamera.h"





class GLInstancingRenderer : public CommonRenderInterface
{
	
	b3AlignedObjectArray<struct b3GraphicsInstance*> m_graphicsInstances;

	
	struct InternalDataRenderer* m_data;

	bool m_textureenabled;
	bool m_textureinitialized;

	int m_screenWidth;
	int m_screenHeight;
	
	int m_upAxis;
    
    
	int registerGraphicsInstanceInternal(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling);
	void rebuildGraphicsInstances();

	
public:
	GLInstancingRenderer(int m_maxObjectCapacity, int maxShapeCapacityInBytes = 56*1024*1024);
	virtual ~GLInstancingRenderer();

	virtual void init();

	virtual void renderScene();
	virtual void renderSceneInternal(int renderMode=B3_DEFAULT_RENDERMODE);

	void InitShaders();
	void CleanupShaders();
	virtual void removeAllInstances();
	virtual void removeGraphicsInstance(int instanceUid);

	virtual void updateShape(int shapeIndex, const float* vertices);

	///vertices must be in the format x,y,z, nx,ny,nz, u,v
	virtual int registerShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType=B3_GL_TRIANGLES, int textureIndex=-1);
	
	virtual int registerTexture(const unsigned char* texels, int width, int height, bool flipPixelsY=true);
    virtual void updateTexture(int textureIndex, const unsigned char* texels, bool flipPixelsY=true);
    virtual void activateTexture(int textureIndex);
	virtual void replaceTexture(int shapeIndex, int textureId);
	virtual int getShapeIndexFromInstance(int srcIndex);

	///position x,y,z, quaternion x,y,z,w, color r,g,b,a, scaling x,y,z
	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling);
	virtual int registerGraphicsInstance(int shapeIndex, const double* position, const double* quaternion, const double* color, const double* scaling);


	void writeTransforms();

	virtual bool readSingleInstanceTransformToCPU(float* position, float* orientation, int srcIndex);

	virtual void writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex);
	virtual void writeSingleInstanceTransformToCPU(const double* position, const double* orientation, int srcIndex)
    {
        float pos[4];
        float orn[4];
        pos[0] = (float)position[0];
        pos[1] = (float)position[1];
        pos[2] = (float)position[2];
        pos[3] = (float)position[3];
        orn[0] =(float)orientation[0];
        orn[1] =(float)orientation[1];
        orn[2] =(float)orientation[2];
        orn[3] =(float)orientation[3];
        writeSingleInstanceTransformToCPU(pos,orn,srcIndex);

    }


	virtual void readSingleInstanceTransformFromCPU(int srcIndex, float* position, float* orientation);

	virtual void writeSingleInstanceTransformToGPU(float* position, float* orientation, int srcIndex);

	virtual void writeSingleInstanceColorToCPU(const float* color, int srcIndex);
	virtual void writeSingleInstanceColorToCPU(const double* color, int srcIndex);

	virtual void writeSingleInstanceSpecularColorToCPU(const double* specular, int srcIndex2);
	virtual void writeSingleInstanceSpecularColorToCPU(const float* specular, int srcIndex2);

	virtual void writeSingleInstanceScaleToCPU(const float* scale, int srcIndex);
	virtual void writeSingleInstanceScaleToCPU(const double* scale, int srcIndex);

	
	virtual struct	GLInstanceRendererInternalData* getInternalData();

	virtual void drawLine(const float from[4], const float to[4], const float color[4], float lineWidth=1);
	virtual void drawLine(const double from[4], const double to[4], const double color[4], double lineWidth=1);
	virtual void drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize);
	virtual void drawPoints(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, float pointDrawSize);
	virtual void drawPoint(const float* position, const float color[4], float pointSize=1);
	virtual void drawPoint(const double* position, const double color[4], double pointDrawSize=1);
	virtual void drawTexturedTriangleMesh(float worldPosition[3], float worldOrientation[4], const float* vertices, int numvertices, const unsigned int* indices, int numIndices, float color[4], int textureIndex=-1, int vertexLayout=0);
	
	virtual void updateCamera(int upAxis=1);

	virtual const CommonCameraInterface* getActiveCamera() const;
	virtual CommonCameraInterface* getActiveCamera();
	virtual void setActiveCamera(CommonCameraInterface* cam);
	
	virtual void setLightPosition(const float lightPos[3]);
	virtual void setLightPosition(const double lightPos[3]);
	void setLightSpecularIntensity(const float lightSpecularIntensity[3]);

	virtual void	resize(int width, int height);
	virtual int	getScreenWidth()
	{
		return m_screenWidth;
	}
	virtual int getScreenHeight()
	{
		return m_screenHeight;
	}

	virtual int getMaxShapeCapacity() const;
	
	virtual int getInstanceCapacity() const;
	
	virtual int getTotalNumInstances() const;
	
	virtual void enableShadowMap();
    
	virtual void clearZBuffer();

	virtual void setRenderFrameBuffer(unsigned int renderFrameBuffer);
};

#endif //GL_INSTANCING_RENDERER_H
