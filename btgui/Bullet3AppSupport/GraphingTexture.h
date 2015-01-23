#ifndef GRAPHING_TEXTURE_H
#define GRAPHING_TEXTURE_H
#include "LinearMath/btAlignedObjectArray.h"

struct GraphingTexture
{
	int m_textureId;
	//assume rgba (8 bit per component, total of 32bit per pixel, for m_width*m_height pixels)
	btAlignedObjectArray<unsigned char> m_imageData;
	int m_width;
	int m_height;
	
	GraphingTexture();
	virtual ~GraphingTexture();
	
	bool create(int texWidth, int texHeight);
	void  destroy();
	
	void setPixel(int x, int y, unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha)
	{
		m_imageData[x*4+y*4*m_width+0] = red;
		m_imageData[x*4+y*4*m_width+1] = green;
		m_imageData[x*4+y*4*m_width+2] = blue;
		m_imageData[x*4+y*4*m_width+3] = alpha;
	}
	
	void uploadImageData();
	
	int getTextureId()
	{
		return m_textureId;
	}
};

#endif //GRAPHING_TEXTURE_H

