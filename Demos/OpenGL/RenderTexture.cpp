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

#include "RenderTexture.h"
#include <memory.h>


renderTexture::renderTexture(int width,int height)
:m_height(height),m_width(width)
{
	m_buffer = new unsigned char[m_width*m_height*4];
	
	//clear screen
	memset(m_buffer,0,m_width*m_height*4);

	//clear screen version 2
	for (int x=0;x<m_width;x++)
	{
		for (int y=0;y<m_height;y++)
		{
			setPixel(x,y,btVector4(float(x),float(y),0.f,1.f));
		}

	}

}

void renderTexture::grapicalPrintf(char* str,	void* fontData, int rasterposx,int rasterposy)
{
	unsigned char c;
	int x=0;
	int xx=0;

	while ((c = (unsigned char) *str++)) {
		
		x=xx;		
		unsigned char* fontPtr = (unsigned char*) fontData;
		char ch = c-32;

		int sx=ch%16;
		int sy=ch/16;
		
		
		for (int i=sx*16;i<(sx*16+16);i++)
		{
			int y=0;
			for (int j=sy*16;j<(sy*16+16);j++)
			{
				unsigned char packedColor = (fontPtr[i*3+255*256*3-(256*j)*3]);
				//float colorf = packedColor ? 0.f : 1.f;
				float colorf = packedColor/255.f;// ? 0.f : 1.f;
				btVector4 rgba(colorf,colorf,colorf,1.f);
				//if (colorf)
				{
					//setPixel(rasterposx+x,rasterposy+y,rgba);
					addPixel(rasterposx+x,rasterposy+y,rgba);
				}
				//bit >>=1;
				y++;
			}
			x++;
		}
		//xx+=16;
		xx+=10;
	}
}

renderTexture::~renderTexture()
{
	delete [] m_buffer;
}



