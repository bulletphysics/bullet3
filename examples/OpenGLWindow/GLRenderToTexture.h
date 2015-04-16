
#ifndef GL_RENDER_TO_TEXTURE_H
#define GL_RENDER_TO_TEXTURE_H

///See http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/
#include "OpenGLInclude.h"

enum 
{
	RENDERTEXTURE_COLOR=1,
	RENDERTEXTURE_DEPTH,
};
struct GLRenderToTexture
{
	GLuint m_framebufferName;
	GLuint	m_depthrenderbuffer;
	bool	m_initialized;
	int		m_renderTextureType;
public:
	GLRenderToTexture();
	
	void init(int width, int height, GLuint textureId, int renderTextureType=RENDERTEXTURE_COLOR);
	bool	enable();
	void	disable();
	
	virtual ~GLRenderToTexture();
	
};


#endif //GL_RENDER_TO_TEXTURE_H

