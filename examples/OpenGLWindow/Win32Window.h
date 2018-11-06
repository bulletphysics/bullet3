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

#ifndef _WIN32_WINDOW_H
#define _WIN32_WINDOW_H

struct InternalData2;

#include "../CommonInterfaces/CommonWindowInterface.h"

class Win32Window : public CommonWindowInterface
{
protected:
	struct InternalData2* m_data;

	void pumpMessage();

public:
	Win32Window();

	virtual ~Win32Window();

	virtual void createWindow(const b3gWindowConstructionInfo& ci);

	virtual void switchFullScreen(bool fullscreen, int width = 0, int height = 0, int colorBitsPerPixel = 0);

	virtual void closeWindow();

	virtual void runMainLoop();

	virtual void startRendering();

	virtual void renderAllObjects();

	virtual void endRendering();

	virtual float getTimeInSeconds();

	virtual void setDebugMessage(int x, int y, const char* message);

	virtual bool requestedExit() const;

	virtual void setRequestExit();

	virtual void getMouseCoordinates(int& x, int& y);

	virtual void setMouseMoveCallback(b3MouseMoveCallback mouseCallback);
	virtual void setMouseButtonCallback(b3MouseButtonCallback mouseCallback);
	virtual void setResizeCallback(b3ResizeCallback resizeCallback);
	virtual void setWheelCallback(b3WheelCallback wheelCallback);
	virtual void setKeyboardCallback(b3KeyboardCallback keyboardCallback);

	virtual b3MouseMoveCallback getMouseMoveCallback();
	virtual b3MouseButtonCallback getMouseButtonCallback();
	virtual b3ResizeCallback getResizeCallback();
	virtual b3WheelCallback getWheelCallback();
	virtual b3KeyboardCallback getKeyboardCallback();

	virtual void setRenderCallback(b3RenderCallback renderCallback);

	virtual void setWindowTitle(const char* title);

	virtual bool isModifierKeyPressed(int key);
};

#endif  //_WIN32_WINDOW_H