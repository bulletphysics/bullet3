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

#if 0
//shift the image one pixel
for(int y=0;y<texHeight;++y)
{
	//					const int	t=y>>5;
	for(int x=1;x<texWidth;++x)
	{
		GLubyte*	org=image+(x+y*texWidth)*4;
		
		GLubyte*	dst=image+(x-1+y*texWidth)*4;
		
		dst[0] = org[0];
		dst[1] = org[1];
		dst[2] = org[2];
		dst[3] = org[3];
	}
}
//render a new row at the right
for(int y=0;y<texHeight;++y)
{
	GLubyte*	pi=image+(texWidth-1+y*texWidth)*4;
	pi[0]=255;
	pi[1]=255;
	pi[2]=255;
	pi[3]=255;
	if (y==texHeight*0.5)
	{
		pi[0]=200;
		pi[1]=200;
		pi[2]=200;
		pi[3]=255;
	}
	
}

{
	static float timer = 0.f;
	static int prevValue=0;
	timer+= 0.01;
	float value = 128+100*sinf(timer);
	MyClamp(value,0.f,float(texHeight-1));
	GLubyte*	org=image+(texWidth-1+(int)value*texWidth)*4;
	org[0] = 0;
	org[1] = 0;
	org[2] = 0;
	org[3] = 255;
	
	if (prevValue<value)
	{
		
	} else
	{
		
	}
	
}

{
	static float timer = 1.4f;
	timer+= 0.04;
	float value = 128+150*sinf(timer);
	MyClamp(value,0.f,float(texHeight-1));
	
	GLubyte*	org=image+(texWidth-1+(int)value*texWidth)*4;
	org[0] = 0;
	org[1] = 255;
	org[2] = 0;
	org[3] = 255;
}

{
	static float timer = 1.4f;
	timer+= 0.02;
	float value =256+400*sinf(timer);
	MyClamp(value,0.f,float(texHeight-1));
	static int prevValue = 0;
	
	GLubyte*	org=image+(texWidth-1+(int)value*texWidth)*4;
	org[0] = 0;
	org[1] = 0;
	org[2] = 255;
	org[3] = 255;
	
	if (prevValue<value)
	{
		for (int i=prevValue;i<value;i++)
		{
			GLubyte*	org=image+(texHeight-1+(int)i*texWidth)*4;
			org[0] = 0;
			org[1] = 0;
			org[2] = 255;
			org[3] = 255;
		}
	} else
	{
		for (int i=value;i<prevValue;i++)
		{
			GLubyte*	org=image+(texHeight-1+(int)i*texWidth)*4;
			org[0] = 0;
			org[1] = 0;
			org[2] = 255;
			org[3] = 255;
		}
	}
	prevValue = value;
}


glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texWidth,texHeight,0,GL_RGBA,GL_UNSIGNED_BYTE,image);
glGenerateMipmap(GL_TEXTURE_2D);

#endif

