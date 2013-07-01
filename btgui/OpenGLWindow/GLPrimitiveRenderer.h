#ifndef _GL_PRIMITIVE_RENDERER_H
#define _GL_PRIMITIVE_RENDERER_H

#include "OpenGLInclude.h"

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
    void drawLine();//float from[4], float to[4], float color[4]);
    void setScreenSize(int width, int height);
    
	PrimInternalData*	getData()
	{
		return m_data;
	}

};

#endif//_GL_PRIMITIVE_RENDERER_H

