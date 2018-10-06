
#ifndef GLFW_OPENGL_WINDOW_H
#define GLFW_OPENGL_WINDOW_H

#ifdef B3_USE_GLFW

#include "../CommonInterfaces/CommonWindowInterface.h"

#define b3gDefaultOpenGLWindow GLFWOpenGLWindow

class GLFWOpenGLWindow : public CommonWindowInterface
{
	struct GLFWOpenGLWindowInternalData* m_data;

protected:
public:
	GLFWOpenGLWindow();

	virtual ~GLFWOpenGLWindow();

	virtual void createDefaultWindow(int width, int height, const char* title);

	virtual void createWindow(const b3gWindowConstructionInfo& ci);

	virtual void closeWindow();

	virtual void runMainLoop();
	virtual float getTimeInSeconds();

	virtual bool requestedExit() const;
	virtual void setRequestExit();

	virtual void startRendering();

	virtual void endRendering();

	virtual bool isModifierKeyPressed(int key);

	virtual void setMouseMoveCallback(b3MouseMoveCallback mouseCallback);
	virtual b3MouseMoveCallback getMouseMoveCallback();

	virtual void setMouseButtonCallback(b3MouseButtonCallback mouseCallback);
	virtual b3MouseButtonCallback getMouseButtonCallback();

	virtual void setResizeCallback(b3ResizeCallback resizeCallback);
	virtual b3ResizeCallback getResizeCallback();

	virtual void setWheelCallback(b3WheelCallback wheelCallback);
	virtual b3WheelCallback getWheelCallback();

	virtual void setKeyboardCallback(b3KeyboardCallback keyboardCallback);
	virtual b3KeyboardCallback getKeyboardCallback();

	virtual void setRenderCallback(b3RenderCallback renderCallback);

	virtual void setWindowTitle(const char* title);

	virtual float getRetinaScale() const;
	virtual void setAllowRetina(bool allow);

	virtual int getWidth() const;
	virtual int getHeight() const;

	virtual int fileOpenDialog(char* fileName, int maxFileNameLength);

	void keyboardCallbackInternal(int key, int state);
	void mouseButtonCallbackInternal(int button, int state);
	void mouseCursorCallbackInternal(double xPos, double yPos);
	void resizeInternal(int width, int height);
};
#endif  //B3_USE_GLFW
#endif  //GLFW_OPENGL_WINDOW_H
