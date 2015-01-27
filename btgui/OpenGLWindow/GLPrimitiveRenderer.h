#ifndef _GL_PRIMITIVE_RENDERER_H
#define _GL_PRIMITIVE_RENDERER_H

//#include "OpenGLInclude.h"

struct PrimVec2
{
	PrimVec2(float x, float y)
	{
		p[0] = x;
		p[1] = y;
	}
	float p[2];
};

struct PrimVec4
{
	PrimVec4() {}
	PrimVec4(float x,float y, float z, float w)
	{
		p[0] = x;
		p[1] = y;
		p[2] = z;
		p[3] = w;
        
	}
    
	float p[4];
};

typedef struct
{
    PrimVec4 position;
    PrimVec4 colour;
	PrimVec2 uv;
} PrimVertex;


class GLPrimitiveRenderer
{
    int m_screenWidth;
    int m_screenHeight;

	struct PrimInternalData*	m_data;
    
    void loadBufferData();
    
public:
    
    GLPrimitiveRenderer(int screenWidth, int screenHeight);
    virtual ~GLPrimitiveRenderer();
	
	void drawRect(float x0, float y0, float x1, float y1, float color[4]);
    void drawTexturedRect(float x0, float y0, float x1, float y1, float color[4], float u0,float v0, float u1, float v1, int useRGBA=0);
	void drawTexturedRect3D(const PrimVertex& v0,const PrimVertex& v1,const PrimVertex& v2,const PrimVertex& v3,float viewMat[16],float projMat[16], bool useRGBA = true);
    void drawLine();//float from[4], float to[4], float color[4]);
    void setScreenSize(int width, int height);
    
	PrimInternalData*	getData()
	{
		return m_data;
	}

};

#endif//_GL_PRIMITIVE_RENDERER_H

