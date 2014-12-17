#ifndef COMMON_RENDER_INTERFACE_H
#define COMMON_RENDER_INTERFACE_H

enum
{
	B3_GL_TRIANGLES = 1,
	B3_GL_POINTS
};

enum 
{
	B3_DEFAULT_RENDERMODE=1,
	//B3_WIREFRAME_RENDERMODE,
	B3_CREATE_SHADOWMAP_RENDERMODE,
	B3_USE_SHADOWMAP_RENDERMODE,
};

struct CommonRenderInterface
{
	virtual void init()=0;
	virtual void updateCamera(int upAxis)=0;
	virtual void removeAllInstances() = 0;
	virtual void setCameraDistance(float dist) = 0;
	virtual void setCameraPitch(float pitch) = 0;
	virtual void setCameraTargetPosition(float x, float y, float z)=0;
	
	
	virtual void	getCameraPosition(float cameraPos[4])=0;
	virtual void	getCameraPosition(double cameraPos[4])=0;

	virtual void	setCameraTargetPosition(float cameraPos[4])=0;
	virtual void	getCameraTargetPosition(float cameraPos[4]) const=0;
    virtual void	getCameraTargetPosition(double cameraPos[4]) const=0;

	virtual void renderScene()=0;

	virtual int getScreenWidth() = 0;
	virtual int getScreenHeight() = 0;

	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)=0;
	virtual int registerGraphicsInstance(int shapeIndex, const double* position, const double* quaternion, const double* color, const double* scaling)=0;
	virtual void drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize)=0;
	virtual void drawLine(const float from[4], const float to[4], const float color[4], float lineWidth) = 0;
	virtual void drawLine(const double from[4], const double to[4], const double color[4], double lineWidth) = 0;
	virtual void drawPoint(const float* position, const float color[4], float pointDrawSize)=0;
	virtual void drawPoint(const double* position, const double color[4], double pointDrawSize)=0;
	virtual int registerShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType=B3_GL_TRIANGLES, int textureIndex=-1)=0;
    virtual void updateShape(int shapeIndex, const float* vertices)=0;
    
	virtual void writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex)=0;
	virtual void writeSingleInstanceTransformToCPU(const double* position, const double* orientation, int srcIndex)=0;
	virtual void writeTransforms()=0;
    virtual void enableBlend(bool blend)=0;
};
#endif//COMMON_RENDER_INTERFACE_H

