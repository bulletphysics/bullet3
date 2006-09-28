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

#include "renderTexture.h"
#include <memory.h>
#include "BMF_FontData.h"

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

void renderTexture::grapicalPrintf(char* str,	BMF_FontData* fontData, int startx,int starty)
{
	unsigned char c;
	int rasterposx = startx;
	int rasterposy = starty;
	while ((c = (unsigned char) *str++)) {
		BMF_CharData & cd = fontData->chars[c];
		
		if (cd.data_offset!=-1) {
			unsigned char* bitmap = &fontData->bitmap_data[cd.data_offset];
			for (int y=0;y<cd.height;y++)
			{
				int bit = 128;
				for (int x=0;x<cd.width;x++)
				{
					char packedColor = bitmap[y];
					float colorf = packedColor & bit ? 1.f : 0.f;
					btVector4 rgba(colorf,colorf,colorf,1.f);
					setPixel(rasterposx+x,rasterposy+8-y-1,rgba);
					bit >>=1;
				}
			}
		}
		rasterposx+= cd.advance;
	}
}

renderTexture::~renderTexture()
{
	delete [] m_buffer;
}



