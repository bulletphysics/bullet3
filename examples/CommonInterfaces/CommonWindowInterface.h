#ifndef B3G_WINDOW_INTERFACE_H
#define B3G_WINDOW_INTERFACE_H

#include "CommonCallbacks.h"

struct b3gWindowConstructionInfo
{
	int m_width;
	int m_height;
	bool m_fullscreen;
	int m_colorBitsPerPixel;
	void* m_windowHandle;
	const char* m_title;
	int m_openglVersion;
	int m_renderDevice;

	b3gWindowConstructionInfo(int width = 1024, int height = 768)
		: m_width(width),
		  m_height(height),
		  m_fullscreen(false),
		  m_colorBitsPerPixel(32),
		  m_windowHandle(0),
		  m_title("title"),
		  m_openglVersion(3),
		  m_renderDevice(-1)
	{
	}
};

class CommonWindowInterface
{
public:
	virtual ~CommonWindowInterface()
	{
	}

	virtual void createDefaultWindow(int width, int height, const char* title)
	{
		b3gWindowConstructionInfo ci(width, height);
		ci.m_title = title;
		createWindow(ci);
	}

	virtual void createWindow(const b3gWindowConstructionInfo& ci) = 0;

	virtual void closeWindow() = 0;

	virtual void runMainLoop() = 0;
	virtual float getTimeInSeconds() = 0;

	virtual bool requestedExit() const = 0;
	virtual void setRequestExit() = 0;

	virtual void startRendering() = 0;

	virtual void endRendering() = 0;

	virtual bool isModifierKeyPressed(int key) = 0;

	virtual void setMouseMoveCallback(b3MouseMoveCallback mouseCallback) = 0;
	virtual b3MouseMoveCallback getMouseMoveCallback() = 0;

	virtual void setMouseButtonCallback(b3MouseButtonCallback mouseCallback) = 0;
	virtual b3MouseButtonCallback getMouseButtonCallback() = 0;

	virtual void setResizeCallback(b3ResizeCallback resizeCallback) = 0;
	virtual b3ResizeCallback getResizeCallback() = 0;

	virtual void setWheelCallback(b3WheelCallback wheelCallback) = 0;
	virtual b3WheelCallback getWheelCallback() = 0;

	virtual void setKeyboardCallback(b3KeyboardCallback keyboardCallback) = 0;
	virtual b3KeyboardCallback getKeyboardCallback() = 0;

	virtual void setRenderCallback(b3RenderCallback renderCallback) = 0;

	virtual void setWindowTitle(const char* title) = 0;

	virtual float getRetinaScale() const = 0;
	virtual void setAllowRetina(bool allow) = 0;

	virtual int getWidth() const = 0;
	virtual int getHeight() const = 0;

	virtual int fileOpenDialog(char* fileName, int maxFileNameLength) = 0;
};

#endif  //B3G_WINDOW_INTERFACE_H
