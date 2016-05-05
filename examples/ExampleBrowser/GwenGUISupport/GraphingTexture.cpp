#include "GraphingTexture.h"
#include "../OpenGLWindow/OpenGLInclude.h"
#include <assert.h>

GraphingTexture::GraphingTexture()
:m_textureId(0),
m_width(0),
m_height(0)
{
}

GraphingTexture::~GraphingTexture()
{
	destroy();
}

void GraphingTexture::destroy()
{
	//TODO(erwincoumans) release memory etc...
	m_width = 0;
	m_height=0;
	glDeleteTextures(1,(GLuint*)&m_textureId);
	m_textureId=0;
}

bool GraphingTexture::create(int texWidth, int texHeight)
{
	m_width = texWidth;
	m_height = texHeight;
	glActiveTexture(GL_TEXTURE0);
	
	m_imageData.resize(texWidth*texHeight*4);
	for(int y=0;y<texHeight;++y)
	{
		//					const int	t=y>>5;
		GLubyte*	pi=&m_imageData[y*texWidth*4];
		for(int x=0;x<texWidth;++x)
		{
			if (x>=y)//x<2||y<2||x>253||y>253)
			{
				pi[0]=0;
				pi[1]=0;
				pi[2]=255;
				pi[3]=255;
			} else
			{
				pi[0]=255;
				pi[1]=0;
				pi[2]=0;
				pi[3]=255;
			}
			
			pi+=4;
		}
	}
	
	
	glGenTextures(1,(GLuint*)&m_textureId);
	
	uploadImageData();
	return true;
}

void GraphingTexture::uploadImageData()
{
	glBindTexture(GL_TEXTURE_2D,m_textureId);
	assert(glGetError()==GL_NO_ERROR);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_width,m_height,0,GL_RGBA,GL_UNSIGNED_BYTE,&m_imageData[0]);
	glGenerateMipmap(GL_TEXTURE_2D);
	
	assert(glGetError()==GL_NO_ERROR);
	
	
}


