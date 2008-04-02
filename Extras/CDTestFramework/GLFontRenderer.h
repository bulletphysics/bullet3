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
#ifndef GLFONTRENDERER_H
#define GLFONTRENDERER_H

class GLFontRenderer{
	
private:

	static bool m_isInit;
	static unsigned int m_textureObject;
	static int m_screenWidth;
	static int m_screenHeight;
	static float m_color[4];

public:
	
	static bool init();
	static void print(float x, float y, float fontSize, const char* pString, bool forceMonoSpace=false, int monoSpaceWidth=11, bool doOrthoProj=true);
	static void setScreenResolution(int screenWidth, int screenHeight);
	static void setColor(float r, float g, float b, float a);
	
};

#endif	// GLFONTRENDERER_H
