/*
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "stdafx.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <windows.h>

#include "GLFontData.h"
#include "GLFontRenderer.h"

bool GLFontRenderer::m_isInit=false;
unsigned int GLFontRenderer::m_textureObject=0;
int GLFontRenderer::m_screenWidth=640;
int GLFontRenderer::m_screenHeight=480;
float GLFontRenderer::m_color[4]={1.0f, 1.0f, 1.0f, 1.0f};

bool GLFontRenderer::init()
{
	glGenTextures(1, &m_textureObject);
	if(m_textureObject == 0) return false;

	glBindTexture(GL_TEXTURE_2D, m_textureObject);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// expand to rgba
	unsigned char* pNewSource = new unsigned char[OGL_FONT_TEXTURE_WIDTH*OGL_FONT_TEXTURE_HEIGHT*4];
	for(int i=0;i<OGL_FONT_TEXTURE_WIDTH*OGL_FONT_TEXTURE_HEIGHT;i++)
	{
		pNewSource[i*4+0]=255;
		pNewSource[i*4+1]=255;
		pNewSource[i*4+2]=255;
		pNewSource[i*4+3]=OGLFontData[i];
	}
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, OGL_FONT_TEXTURE_WIDTH, OGL_FONT_TEXTURE_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, pNewSource);
	delete[] pNewSource;

	return true;
}

void GLFontRenderer::print(float x, float y, float fontSize, const char* pString, bool forceMonoSpace, int monoSpaceWidth, bool doOrthoProj)
{
//	x = x*m_screenWidth;
//	y = y*m_screenHeight;
	fontSize = fontSize*m_screenHeight;

	if(!m_isInit)
	{
		m_isInit = init();
	}

	unsigned int num = (unsigned int)strlen(pString);
	if(m_isInit && num > 0)
	{
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, m_textureObject);


		if(doOrthoProj)
		{
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(0, m_screenWidth, 0, m_screenHeight, -1, 1);
		}
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glEnable(GL_BLEND);

		glColor4f(m_color[0], m_color[1], m_color[2], m_color[3]);

		const float glyphHeightUV = ((float)OGL_FONT_CHARS_PER_COL)/OGL_FONT_TEXTURE_HEIGHT*2-0.01f;

		float translate = 0.0f;

		float* pVertList = new float[num*3*6];
    	float* pTextureCoordList = new float[num*2*6];
		int vertIndex = 0;
    	int textureCoordIndex = 0;

		float translateDown = 0.0f;
		unsigned int count = 0;

		for(unsigned int i=0;i<num; i++)
		{
			const float glyphWidthUV = ((float)OGL_FONT_CHARS_PER_ROW)/OGL_FONT_TEXTURE_WIDTH;

			if (pString[i] == '\n') {
				translateDown-=0.005f*m_screenHeight+fontSize;
				translate = 0.0f;
				continue;
			}

			int c = pString[i]-OGL_FONT_CHAR_BASE;
			if (c < OGL_FONT_CHARS_PER_ROW*OGL_FONT_CHARS_PER_COL) {

				count++;

				float glyphWidth = (float)GLFontGlyphWidth[c];
				if(forceMonoSpace){
					glyphWidth = (float)monoSpaceWidth;
				}
				
				glyphWidth = glyphWidth*(fontSize/(((float)OGL_FONT_TEXTURE_WIDTH)/OGL_FONT_CHARS_PER_ROW))-0.01f;

				float cxUV = float((c)%OGL_FONT_CHARS_PER_ROW)/OGL_FONT_CHARS_PER_ROW+0.008f;
				float cyUV = float((c)/OGL_FONT_CHARS_PER_ROW)/OGL_FONT_CHARS_PER_COL+0.008f;

				pTextureCoordList[textureCoordIndex++] = cxUV;
				pTextureCoordList[textureCoordIndex++] = cyUV+glyphHeightUV;
				pVertList[vertIndex++] = x+0+translate;
				pVertList[vertIndex++] = y+0+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV+glyphWidthUV;
				pTextureCoordList[textureCoordIndex++] = cyUV;
				pVertList[vertIndex++] = x+fontSize+translate;
				pVertList[vertIndex++] = y+fontSize+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV;
				pTextureCoordList[textureCoordIndex++] = cyUV;
				pVertList[vertIndex++] = x+0+translate;
				pVertList[vertIndex++] = y+fontSize+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV;
				pTextureCoordList[textureCoordIndex++] = cyUV+glyphHeightUV;
				pVertList[vertIndex++] = x+0+translate;
				pVertList[vertIndex++] = y+0+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV+glyphWidthUV;
				pTextureCoordList[textureCoordIndex++] = cyUV+glyphHeightUV;
				pVertList[vertIndex++] = x+fontSize+translate;
				pVertList[vertIndex++] = y+0+translateDown;
				pVertList[vertIndex++] = 0;

				pTextureCoordList[textureCoordIndex++] = cxUV+glyphWidthUV;
				pTextureCoordList[textureCoordIndex++] = cyUV;
				pVertList[vertIndex++] = x+fontSize+translate;
				pVertList[vertIndex++] = y+fontSize+translateDown;
				pVertList[vertIndex++] = 0;

				translate+=glyphWidth;
			}
		}

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, pVertList);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(2, GL_FLOAT, 0, pTextureCoordList);
		glDrawArrays(GL_TRIANGLES, 0, count*6);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);

		delete[] pVertList;
		delete[] pTextureCoordList;

		if(doOrthoProj)
		{
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
		}
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);	
		glDisable(GL_BLEND);
	}
}

void GLFontRenderer::setScreenResolution(int screenWidth, int screenHeight)
{
	m_screenWidth = screenWidth;
	m_screenHeight = screenHeight;
}

void GLFontRenderer::setColor(float r, float g, float b, float a)
{
	m_color[0] = r;
	m_color[1] = g;
	m_color[2] = b;
	m_color[3] = a;
}
