#ifndef _OPENGL_FONTSTASH_CALLBACKS_H
#define _OPENGL_FONTSTASH_CALLBACKS_H

#include "fontstash.h"
struct PrimInternalData;
class GLPrimitiveRenderer;

struct	InternalOpenGL2RenderCallbacks : public RenderCallbacks
{

	virtual PrimInternalData* getData()=0;

	virtual ~InternalOpenGL2RenderCallbacks();

	virtual void updateTexture(sth_texture* texture, sth_glyph* glyph, int textureWidth, int textureHeight);
	virtual void render(sth_texture* texture);
	
	void display2();
	
};

void dumpTextureToPng( int screenWidth, int screenHeight, const char* fileName);

struct SimpleOpenGL2RenderCallbacks : public InternalOpenGL2RenderCallbacks
{
	PrimInternalData* m_data;
	virtual PrimInternalData* getData()
	{
		return m_data;
	}
	SimpleOpenGL2RenderCallbacks(PrimInternalData* data)
		:m_data(data)
	{
	}
	virtual ~SimpleOpenGL2RenderCallbacks()
	{
	}
};


struct	OpenGL2RenderCallbacks : public InternalOpenGL2RenderCallbacks
{
	GLPrimitiveRenderer* m_primRender2;
	virtual PrimInternalData* getData();
	
	OpenGL2RenderCallbacks(GLPrimitiveRenderer* primRender);
	virtual ~OpenGL2RenderCallbacks();

	
};



#endif//_OPENGL_FONTSTASH_CALLBACKS_H

