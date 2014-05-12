/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef RENDER_TEXTURE_H
#define RENDER_TEXTURE_H

#include "LinearMath/btVector3.h"
#include "GLDebugFont.h"

///
///renderTexture provides a software-render context (setpixel/printf)
///
class renderTexture
{
	int m_height;
	int m_width;
	unsigned char*	m_buffer;

public:

	renderTexture(int width,int height);
	~renderTexture();

	///rgba input is in range [0..1] for each component
	inline void	setPixel(int x,int y,const btVector4& rgba)
	{
		unsigned char* pixel = &m_buffer[ (x+y*m_width) * 4];

		pixel[0] = (unsigned char)(255.*rgba.getX());
		pixel[1] = (unsigned char)(255.*rgba.getY());
		pixel[2] = (unsigned char)(255.*rgba.getZ());
		pixel[3] = (unsigned char)(255.*rgba.getW());
	}

	inline void	addPixel(int x,int y,const btVector4& rgba)
	{
		unsigned char* pixel = &m_buffer[ (x+y*m_width) * 4];
		pixel[0] = (unsigned char)btMin(btScalar(255.f),((btScalar)pixel[0] + btScalar(255.f)*rgba.getX()));
		pixel[1] = (unsigned char)btMin(btScalar(255.f),((btScalar)pixel[1] + btScalar(255.f)*rgba.getY()));
		pixel[2] = (unsigned char)btMin(btScalar(255.f),((btScalar)pixel[2] + btScalar(255.f)*rgba.getZ()));
//		pixel[3] = (unsigned char)btMin(btScalar(255.f),((btScalar)pixel[3] + btScalar(255.f)*rgba.getW()));
	}

	inline btVector4 getPixel(int x,int y)
	{
		unsigned char* pixel = &m_buffer[ (x+y*m_width) * 4];
		return btVector4(pixel[0]*1.f/255.f,
			pixel[1]*1.f/255.f,
			pixel[2]*1.f/255.f,
			pixel[3]*1.f/255.f);
	}

	const unsigned char*	getBuffer() const { return m_buffer;}
	int	getWidth() const { return m_width;}
	int	getHeight() const { return m_height;}
	void grapicalPrintf(char* str,	void* fontData, int startx = 0,int starty=0);

};

#endif //RENDER_TEXTURE_H

