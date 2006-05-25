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

#include "SimdVector3.h"
#include "BMF_FontData.h"

///
///RenderTexture provides a software-render context (setpixel/printf)
///
class RenderTexture
{
	int m_height;
	int m_width;
	unsigned char*	m_buffer;

public:

	RenderTexture(int width,int height);
	~RenderTexture();

	inline void	SetPixel(int x,int y,const SimdVector4& rgba)
	{
		unsigned char* pixel = &m_buffer[ (x+y*m_width) * 4];

		pixel[0] = (unsigned char)(255*rgba.getX());
		pixel[1] = (unsigned char)(255*rgba.getY());
		pixel[2] = (unsigned char)(255*rgba.getZ());
		pixel[3] = (unsigned char)(255*rgba.getW());
	}

	const unsigned char*	GetBuffer() const { return m_buffer;}
	int	GetWidth() const { return m_width;}
	int	GetHeight() const { return m_height;}
	void Printf(char* str,	BMF_FontData* fontData, int startx = 0,int starty=0);

};

#endif //RENDER_TEXTURE_H
