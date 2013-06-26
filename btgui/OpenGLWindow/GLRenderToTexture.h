
#ifndef GL_RENDER_TO_TEXTURE_H
#define GL_RENDER_TO_TEXTURE_H

///See http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/
#include "OpenGLInclude.h"

struct GLRenderToTexture
{
	GLuint m_framebufferName;
	GLuint	m_renderedTexture;
	GLuint	m_depthrenderbuffer;
	bool	m_initialized;
public:
		
	GLRenderToTexture();
	
	void init(int width, int height);
	bool	enable();
	void	disable();
	
	virtual ~GLRenderToTexture();
	
};


#endif //GL_RENDER_TO_TEXTURE_H

