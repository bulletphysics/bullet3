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
#include "OpenGLWindow/CommonRenderInterface.h"


void b3DefaultMouseButtonCallback( int button, int state, float x, float y);
void b3DefaultMouseMoveCallback(  float x, float y);
void b3DefaultKeyboardCallback(int key, int state);
void b3DefaultWheelCallback( float deltax, float deltay);



class GLInstancingRenderer : public CommonRenderInterface
{
	
	b3AlignedObjectArray<struct b3GraphicsInstance*> m_graphicsInstances;

	int		m_maxNumObjectCapacity;
	int		m_maxShapeCapacityInBytes;
	struct InternalDataRenderer* m_data;

	bool m_textureenabled;
	bool m_textureinitialized;

	int m_screenWidth;
	int m_screenHeight;
	
	int m_upAxis;
    bool m_enableBlend;
    
	void renderSceneInternal(int renderMode=B3_DEFAULT_RENDERMODE);

	
public:
	GLInstancingRenderer(int m_maxObjectCapacity, int maxShapeCapacityInBytes = 56*1024*1024);
	virtual ~GLInstancingRenderer();

	virtual void init();

	virtual void renderScene();

	void InitShaders();
	void CleanupShaders();
	void removeAllInstances();

	virtual void updateShape(int shapeIndex, const float* vertices);

	///vertices must be in the format x,y,z, nx,ny,nz, u,v
	virtual int registerShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType=B3_GL_TRIANGLES, int textureIndex=-1);
    

	virtual int	registerTexture(const unsigned char* texels, int width, int height);

	///position x,y,z, quaternion x,y,z,w, color r,g,b,a, scaling x,y,z
	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling);
	virtual int registerGraphicsInstance(int shapeIndex, const double* position, const double* quaternion, const double* color, const double* scaling);

	void writeTransforms();

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

	virtual void writeSingleInstanceTransformToGPU(float* position, float* orientation, int srcIndex);

	virtual void writeSingleInstanceColorToCPU(float* color, int srcIndex);

	virtual void getMouseDirection(float* dir, int mouseX, int mouseY);

	struct	GLInstanceRendererInternalData* getInternalData();

	virtual void drawLine(const float from[4], const float to[4], const float color[4], float lineWidth=1);
	virtual void drawLine(const double from[4], const double to[4], const double color[4], double lineWidth=1);
	virtual void drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize);
	virtual void drawPoints(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, float pointDrawSize);
	virtual void drawPoint(const float* position, const float color[4], float pointSize=1);
	virtual void drawPoint(const double* position, const double color[4], double pointDrawSize=1);
	virtual void updateCamera(int upAxis=1);

	virtual void	getCameraPosition(float cameraPos[4]);
	virtual void	getCameraPosition(double cameraPos[4])
    {
        float campos[4];
        getCameraPosition(campos);
        cameraPos[0] = campos[0];
        cameraPos[1] = campos[1];
        cameraPos[2] = campos[2];
        cameraPos[3] = campos[3];
    }

	virtual void	setCameraDistance(float dist);
	virtual float	getCameraDistance() const;

	//set the camera 'target'
	virtual void	setCameraTargetPosition(float x, float y, float z);
	virtual void	setCameraTargetPosition(float cameraPos[4]);
	virtual void	getCameraTargetPosition(float cameraPos[4]) const;
    virtual void	getCameraTargetPosition(double cameraPos[4]) const
    {
        float campos[4];
        getCameraTargetPosition(campos);
        cameraPos[0] = campos[0];
        cameraPos[1] = campos[1];
        cameraPos[2] = campos[2];
        cameraPos[3] = campos[3];
        
    }
	
	virtual void	setCameraYaw(float yaw);
	virtual void	setCameraPitch(float pitch);
	virtual float	getCameraYaw() const;
	virtual float	getCameraPitch() const;

	virtual void	resize(int width, int height);
	virtual int	getScreenWidth()
	{
		return m_screenWidth;
	}
	virtual int getScreenHeight()
	{
		return m_screenHeight;
	}

	virtual int getMaxShapeCapacity() const
	{
		return m_maxShapeCapacityInBytes;
	}
	virtual int getInstanceCapacity() const
	{
		return m_maxNumObjectCapacity;
	}
	virtual void enableShadowMap();
    virtual void enableBlend(bool blend)
    {
        m_enableBlend = blend;
    }

};

#endif //GL_INSTANCING_RENDERER_H
