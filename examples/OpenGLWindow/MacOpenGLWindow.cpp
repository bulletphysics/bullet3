#ifndef B3_USE_GLFW
#ifdef __APPLE__

#include "MacOpenGLWindow.h"

#include "OpenGLInclude.h"
#include "MacOpenGLWindowObjC.h"

#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

MacOpenGLWindow::MacOpenGLWindow()
	: m_internalData(0)
{
	m_internalData = Mac_createData();
}

MacOpenGLWindow::~MacOpenGLWindow()
{
	Mac_destroyData(m_internalData);
}

void MacOpenGLWindow::closeWindow()
{
	Mac_destroyData(m_internalData);
	m_internalData = Mac_createData();
}

bool MacOpenGLWindow::isModifierKeyPressed(int key)
{
	return Mac_isModifierKeyPressed(m_internalData, key);
}

float MacOpenGLWindow::getTimeInSeconds()
{
	return 0.f;
}

void MacOpenGLWindow::setRenderCallback(b3RenderCallback renderCallback)
{
}

void MacOpenGLWindow::setWindowTitle(const char* windowTitle)
{
	Mac_setWindowTitle(m_internalData, windowTitle);
}

void MacOpenGLWindow::createWindow(const b3gWindowConstructionInfo& ci)
{
	MacWindowConstructionInfo windowCI;
	windowCI.m_width = ci.m_width;
	windowCI.m_height = ci.m_height;
	windowCI.m_fullscreen = ci.m_fullscreen;
	windowCI.m_colorBitsPerPixel = ci.m_colorBitsPerPixel;
	windowCI.m_windowHandle = ci.m_windowHandle;
	windowCI.m_title = ci.m_title;
	windowCI.m_openglVersion = ci.m_openglVersion;
	windowCI.m_allowRetina = true;

	Mac_createWindow(m_internalData, &windowCI);
}

void MacOpenGLWindow::runMainLoop()
{
}

void MacOpenGLWindow::startRendering()
{
	Mac_updateWindow(m_internalData);
}

void MacOpenGLWindow::endRendering()
{
	Mac_swapBuffer(m_internalData);
}

bool MacOpenGLWindow::requestedExit() const
{
	return Mac_requestedExit(m_internalData);
}

void MacOpenGLWindow::setRequestExit()
{
	Mac_setRequestExit(m_internalData);
}

int MacOpenGLWindow::fileOpenDialog(char* filename, int maxNameLength)
{
	return Mac_fileOpenDialog(filename, maxNameLength);
}

void MacOpenGLWindow::getMouseCoordinates(int& x, int& y)
{
	int* xPtr = &x;
	int* yPtr = &y;

	Mac_getMouseCoordinates(m_internalData, xPtr, yPtr);
}

int MacOpenGLWindow::getWidth() const
{
	return Mac_getWidth(m_internalData);
}

int MacOpenGLWindow::getHeight() const
{
	return Mac_getHeight(m_internalData);
}

void MacOpenGLWindow::setResizeCallback(b3ResizeCallback resizeCallback)
{
	Mac_setResizeCallback(m_internalData, resizeCallback);
}

b3ResizeCallback MacOpenGLWindow::getResizeCallback()
{
	return Mac_getResizeCallback(m_internalData);
}

void MacOpenGLWindow::setMouseButtonCallback(b3MouseButtonCallback mouseCallback)
{
	Mac_setMouseButtonCallback(m_internalData, mouseCallback);
}

void MacOpenGLWindow::setMouseMoveCallback(b3MouseMoveCallback mouseCallback)
{
	Mac_setMouseMoveCallback(m_internalData, mouseCallback);
}

void MacOpenGLWindow::setKeyboardCallback(b3KeyboardCallback keyboardCallback)
{
	Mac_setKeyboardCallback(m_internalData, keyboardCallback);
}

b3MouseMoveCallback MacOpenGLWindow::getMouseMoveCallback()
{
	return Mac_getMouseMoveCallback(m_internalData);
}

b3MouseButtonCallback MacOpenGLWindow::getMouseButtonCallback()
{
	return Mac_getMouseButtonCallback(m_internalData);
}

void MacOpenGLWindow::setWheelCallback(b3WheelCallback wheelCallback)
{
	Mac_setWheelCallback(m_internalData, wheelCallback);
}

b3WheelCallback MacOpenGLWindow::getWheelCallback()
{
	return Mac_getWheelCallback(m_internalData);
}

b3KeyboardCallback MacOpenGLWindow::getKeyboardCallback()
{
	return Mac_getKeyboardCallback(m_internalData);
}

float MacOpenGLWindow::getRetinaScale() const
{
	return Mac_getRetinaScale(m_internalData);
}

void MacOpenGLWindow::setAllowRetina(bool allow)
{
	Mac_setAllowRetina(m_internalData, allow);
}

#endif  //__APPLE__
#endif  //B3_USE_GLFW
