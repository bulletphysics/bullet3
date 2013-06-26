
///See http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/

#include "GLRenderToTexture.h"

GLRenderToTexture::GLRenderToTexture()
:m_framebufferName(0)
{
}
	
void GLRenderToTexture::init(int width, int height)
{
	glGenFramebuffers(1, &m_framebufferName);
	glBindFramebuffer(GL_FRAMEBUFFER, m_framebufferName);
	
	GLuint m_renderedTexture;
	glGenTextures(1, &m_renderedTexture);

	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, m_renderedTexture);

	// Give an empty image to OpenGL ( the last "0" )
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, width, height, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);


	// The depth buffer
	glGenRenderbuffers(1, &m_depthrenderbuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, m_depthrenderbuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthrenderbuffer);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, m_renderedTexture, 0);

}

bool GLRenderToTexture::enable()
{
	bool status = false;

		// Set the list of draw buffers.
	GLenum drawBuffers[2] = {GL_COLOR_ATTACHMENT0,0};
	glDrawBuffers(1, drawBuffers);

	// Always check that our framebuffer is ok
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)
	{
		status = true;
	}

	return status;

}

void GLRenderToTexture::disable()
{
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );
	glBindTexture(GL_TEXTURE_2D, 0);
}
	
GLRenderToTexture::~GLRenderToTexture()
{
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );

	if (m_depthrenderbuffer)
		glDeleteRenderbuffers(1,&m_depthrenderbuffer);

	if(m_renderedTexture) 
		glDeleteTextures(1, &m_renderedTexture);

	if( m_framebufferName)
	{ 
		glDeleteFramebuffers(1, &m_framebufferName);
	}
}

