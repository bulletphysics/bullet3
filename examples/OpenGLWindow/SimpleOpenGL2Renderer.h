
#ifndef SIMPLE_OPENGL2_RENDERER_H
#define SIMPLE_OPENGL2_RENDERER_H

#include "../CommonInterfaces/CommonRenderInterface.h"
#include "SimpleCamera.h"

struct SimpleOpenGL2Renderer : public CommonRenderInterface
{
    int m_width;
    int m_height;
    SimpleCamera	m_camera;
    
    SimpleOpenGL2Renderer(int width, int height);
    
    virtual void init();
    
    virtual void updateCamera(int upAxis);
    
	virtual const CommonCameraInterface* getActiveCamera() const;
	virtual CommonCameraInterface* getActiveCamera();
	virtual void setActiveCamera(CommonCameraInterface* cam);

	virtual void	resize(int width, int height)
	{
		m_width = width;
		m_height = height;
	}

    virtual void removeAllInstances();
    
    
    virtual void writeSingleInstanceColorToCPU(float* color, int srcIndex);
    virtual void writeSingleInstanceColorToCPU(double* color, int srcIndex);
	virtual void writeSingleInstanceScaleToCPU(float* scale, int srcIndex);
    virtual void writeSingleInstanceScaleToCPU(double* scale, int srcIndex);
    virtual void	getCameraViewMatrix(float viewMat[16]) const;
    virtual void	getCameraProjectionMatrix(float projMat[16]) const;

    
    virtual void renderScene();
    
    virtual int getScreenWidth()
    {
        return m_width;
    }
    virtual int getScreenHeight()
    {
        return m_height;
    }
	virtual int	registerTexture(const unsigned char* texels, int width, int height)
	{
		return -1;
	}
	virtual void    updateTexture(int textureIndex, const unsigned char* texels) {}
    virtual void activateTexture(int textureIndex) {}


    virtual int registerGraphicsInstance(int shapeIndex, const double* position, const double* quaternion, const double* color, const double* scaling);
    
    virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling);
    
    virtual void drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize);
    
    virtual void drawLine(const float from[4], const float to[4], const float color[4], float lineWidth);
    
    virtual int registerShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType=B3_GL_TRIANGLES, int textureIndex=-1);
    
    virtual void writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex);
    
    virtual void writeSingleInstanceTransformToCPU(const double* position, const double* orientation, int srcIndex);
    
    virtual int getTotalNumInstances() const;
    
    virtual void writeTransforms();
    
    virtual void drawLine(const double from[4], const double to[4], const double color[4], double lineWidth);
    
    virtual void drawPoint(const float* position, const float color[4], float pointDrawSize);
    
    virtual void drawPoint(const double* position, const double color[4], double pointDrawSize);
    
    virtual void updateShape(int shapeIndex, const float* vertices);
    
    virtual void enableBlend(bool blend);

	virtual void clearZBuffer();


	virtual struct	GLInstanceRendererInternalData* getInternalData()
	{
		return 0;
	}
    
};
#endif //SIMPLE_OPENGL2_RENDERER_H
