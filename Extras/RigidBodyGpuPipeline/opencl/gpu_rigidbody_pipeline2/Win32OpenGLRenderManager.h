/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans


#ifndef _WIN32_OPENGL_RENDER_MANAGER_H
#define _WIN32_OPENGL_RENDER_MANAGER_H


#define RM_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

RM_DECLARE_HANDLE(RenderObjectHandle);

struct InternalData2;

class Win32OpenGLWindow
{
	protected:
		
		struct InternalData2*	m_data;
		
		void enableOpenGL();
		
		void disableOpenGL();

		void pumpMessage();
	
		

public:

	Win32OpenGLWindow();

	virtual ~Win32OpenGLWindow();

	virtual	void	init(); //default implementation uses default settings for width/height/fullscreen

	void	init(int width,int height, bool fullscreen=false, int colorBitsPerPixel=0, void* windowHandle=0);
	
	void	switchFullScreen(bool fullscreen,int width=0,int height=0,int colorBitsPerPixel=0);

	virtual	void	exit();


	virtual	void	startRendering();

	virtual	void	renderAllObjects();

	virtual	void	endRendering();

	virtual	float	getTimeInSeconds();

	virtual void	setDebugMessage(int x,int y,const char* message);
	
	virtual bool requestedExit();

};

#endif //_WIN32_OPENGL_RENDER_MANAGER_H
