#ifndef COMMON_RENDER_INTERFACE_H
#define COMMON_RENDER_INTERFACE_H

struct CommonCameraInterface;

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
	virtual ~CommonRenderInterface() {}
	virtual void init()=0;
	virtual void updateCamera(int upAxis)=0;
	virtual void removeAllInstances() = 0;
	
	virtual const CommonCameraInterface* getActiveCamera() const =0;
	virtual CommonCameraInterface* getActiveCamera()=0;
	virtual void setActiveCamera(CommonCameraInterface* cam)=0;
	

	virtual void renderScene()=0;
	virtual void renderSceneInternal(int renderMode=B3_DEFAULT_RENDERMODE){};
	virtual int getScreenWidth() = 0;
	virtual int getScreenHeight() = 0;

	virtual void	resize(int width, int height) = 0;

	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)=0;
	virtual int registerGraphicsInstance(int shapeIndex, const double* position, const double* quaternion, const double* color, const double* scaling)=0;
	virtual void drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize)=0;
	virtual void drawLine(const float from[4], const float to[4], const float color[4], float lineWidth) = 0;
	virtual void drawLine(const double from[4], const double to[4], const double color[4], double lineWidth) = 0;
	virtual void drawPoint(const float* position, const float color[4], float pointDrawSize)=0;
	virtual void drawPoint(const double* position, const double color[4], double pointDrawSize)=0;
	virtual int registerShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType=B3_GL_TRIANGLES, int textureIndex=-1)=0;
    virtual void updateShape(int shapeIndex, const float* vertices)=0;
    
    virtual int registerTexture(const unsigned char* texels, int width, int height)=0;
    virtual void updateTexture(int textureIndex, const unsigned char* texels)=0;
    virtual void activateTexture(int textureIndex)=0;
    
	virtual void writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex)=0;
	virtual void writeSingleInstanceTransformToCPU(const double* position, const double* orientation, int srcIndex)=0;
	virtual void writeSingleInstanceColorToCPU(float* color, int srcIndex)=0;
	virtual void writeSingleInstanceColorToCPU(double* color, int srcIndex)=0;
	virtual void writeSingleInstanceScaleToCPU(float* scale, int srcIndex)=0;
	virtual void writeSingleInstanceScaleToCPU(double* scale, int srcIndex)=0;
    
    virtual int getTotalNumInstances() const = 0;
    
	virtual void writeTransforms()=0;
    virtual void enableBlend(bool blend)=0;
	virtual void clearZBuffer()=0;

	//This is internal access to OpenGL3+ features, mainly used for OpenCL-OpenGL interop
	//Only the GLInstancingRenderer supports it, just return 0 otherwise.
	virtual struct	GLInstanceRendererInternalData* getInternalData()=0;
};

template <typename T>
inline int projectWorldCoordToScreen(T objx, T objy, T objz,
		const T modelMatrix[16],
		const T projMatrix[16],
		const int viewport[4],
		T *winx, T *winy, T *winz)
{
	int i;
	T in2[4];
	T tmp[4];

	in2[0]=objx;
	in2[1]=objy;
	in2[2]=objz;
	in2[3]=T(1.0);

	for (i=0; i<4; i++) 
	{
		tmp[i] =   in2[0] * modelMatrix[0*4+i] +  in2[1] * modelMatrix[1*4+i] +  
			in2[2] * modelMatrix[2*4+i] +  in2[3] * modelMatrix[3*4+i];
	}

	T out[4];
	for (i=0; i<4; i++) 
	{
		out[i] =   tmp[0] * projMatrix[0*4+i] +  tmp[1] * projMatrix[1*4+i] +  tmp[2] * projMatrix[2*4+i] +  tmp[3] * projMatrix[3*4+i];
	}

	if (out[3] == T(0.0)) 
		return 0;
	out[0] /= out[3];
	out[1] /= out[3];
	out[2] /= out[3];
	/* Map x, y and z to range 0-1 */
	out[0] = out[0] * T(0.5) + T(0.5);
	out[1] = out[1] * T(0.5) + T(0.5);
	out[2] = out[2] * T(0.5) + T(0.5);

	/* Map x,y to viewport */
	out[0] = out[0] * viewport[2] + viewport[0];
	out[1] = out[1] * viewport[3] + viewport[1];

	*winx=out[0];
	*winy=out[1];
	*winz=out[2];
	return 1;
}

#endif//COMMON_RENDER_INTERFACE_H

