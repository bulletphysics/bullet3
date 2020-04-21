
// portions of this file are copied from GLFW egl_context.c/egl_context.h

//========================================================================
// GLFW 3.3 EGL - www.glfw.org
//------------------------------------------------------------------------
// Copyright (c) 2002-2006 Marcus Geelnard
// Copyright (c) 2006-2016 Camilla Löwy <elmindreda@glfw.org>
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would
//    be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not
//    be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source
//    distribution.
//
//========================================================================

#ifdef BT_USE_EGL

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "OpenGLInclude.h"

#include "glad/egl.h"
#include "glad/gl.h"

#include "EGLOpenGLWindow.h"

struct EGLInternalData2
{
	bool m_isInitialized;

	int m_windowWidth;
	int m_windowHeight;
	int m_renderDevice;

	b3KeyboardCallback m_keyboardCallback;
	b3WheelCallback m_wheelCallback;
	b3ResizeCallback m_resizeCallback;
	b3MouseButtonCallback m_mouseButtonCallback;
	b3MouseMoveCallback m_mouseMoveCallback;

	EGLBoolean success;
	EGLint num_configs;
	EGLConfig egl_config;
	EGLSurface egl_surface;
	EGLContext egl_context;
	EGLDisplay egl_display;

	EGLInternalData2()
		: m_isInitialized(false),
		  m_windowWidth(0),
		  m_windowHeight(0),
		  m_keyboardCallback(0),
		  m_wheelCallback(0),
		  m_resizeCallback(0),
		  m_mouseButtonCallback(0),
		  m_mouseMoveCallback(0) {}
};

EGLOpenGLWindow::EGLOpenGLWindow() { m_data = new EGLInternalData2(); }

EGLOpenGLWindow::~EGLOpenGLWindow() { delete m_data; }

void EGLOpenGLWindow::createWindow(const b3gWindowConstructionInfo& ci)
{
	m_data->m_windowWidth = ci.m_width;
	m_data->m_windowHeight = ci.m_height;

	m_data->m_renderDevice = ci.m_renderDevice;

	EGLint egl_config_attribs[] = {EGL_RED_SIZE,
								   8,
								   EGL_GREEN_SIZE,
								   8,
								   EGL_BLUE_SIZE,
								   8,
								   EGL_DEPTH_SIZE,
								   8,
								   EGL_SURFACE_TYPE,
								   EGL_PBUFFER_BIT,
								   EGL_RENDERABLE_TYPE,
								   EGL_OPENGL_BIT,
								   EGL_NONE};

	EGLint egl_pbuffer_attribs[] = {
		EGL_WIDTH,
		m_data->m_windowWidth,
		EGL_HEIGHT,
		m_data->m_windowHeight,
		EGL_NONE,
	};

	// Load EGL functions
	int egl_version = gladLoaderLoadEGL(NULL);
	if (!egl_version)
	{
		fprintf(stderr, "failed to EGL with glad.\n");
		exit(EXIT_FAILURE);
	};

	// Query EGL Devices
	const int max_devices = 32;
	EGLDeviceEXT egl_devices[max_devices];
	EGLint num_devices = 0;
	EGLint egl_error = eglGetError();
	if (!eglQueryDevicesEXT(max_devices, egl_devices, &num_devices) ||
		egl_error != EGL_SUCCESS)
	{
		printf("eglQueryDevicesEXT Failed.\n");
		m_data->egl_display = EGL_NO_DISPLAY;
	}
	// Query EGL Screens
	if (m_data->m_renderDevice == -1)
	{
		// Chose default screen, by trying all
		for (EGLint i = 0; i < num_devices; ++i)
		{
			// Set display
			EGLDisplay display = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT,
														  egl_devices[i], NULL);
			if (eglGetError() == EGL_SUCCESS && display != EGL_NO_DISPLAY)
			{
				int major, minor;
				EGLBoolean initialized = eglInitialize(display, &major, &minor);
				if (eglGetError() == EGL_SUCCESS && initialized == EGL_TRUE)
				{
					m_data->egl_display = display;
					break;
				}
			}
			else
			{
				fprintf(stderr, "GetDisplay %d failed with error: %x\n", i, eglGetError());
			}
		}
	}
	else
	{
		// Chose specific screen, by using m_renderDevice
		if (m_data->m_renderDevice < 0 || m_data->m_renderDevice >= num_devices)
		{
			fprintf(stderr, "Invalid render_device choice: %d < %d.\n", m_data->m_renderDevice, num_devices);
			exit(EXIT_FAILURE);
		}

		// Set display
		EGLDisplay display = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT,
													  egl_devices[m_data->m_renderDevice], NULL);
		if (eglGetError() == EGL_SUCCESS && display != EGL_NO_DISPLAY)
		{
			int major, minor;
			EGLBoolean initialized = eglInitialize(display, &major, &minor);
			if (eglGetError() == EGL_SUCCESS && initialized == EGL_TRUE)
			{
				m_data->egl_display = display;
			}
		}
		else
		{
			fprintf(stderr, "GetDisplay %d failed with error: %x\n", m_data->m_renderDevice, eglGetError());
		}
	}

	if (!eglInitialize(m_data->egl_display, NULL, NULL))
	{
		fprintf(stderr, "Unable to initialize EGL\n");
		exit(EXIT_FAILURE);
	}

	egl_version = gladLoaderLoadEGL(m_data->egl_display);
	if (!egl_version)
	{
		fprintf(stderr, "Unable to reload EGL.\n");
		exit(EXIT_FAILURE);
	}
	printf("Loaded EGL %d.%d after reload.\n", GLAD_VERSION_MAJOR(egl_version),
		   GLAD_VERSION_MINOR(egl_version));

	m_data->success = eglBindAPI(EGL_OPENGL_API);
	if (!m_data->success)
	{
		// TODO: Properly handle this error (requires change to default window
		// API to change return on all window types to bool).
		fprintf(stderr, "Failed to bind OpenGL API.\n");
		exit(EXIT_FAILURE);
	}

	m_data->success =
		eglChooseConfig(m_data->egl_display, egl_config_attribs,
						&m_data->egl_config, 1, &m_data->num_configs);
	if (!m_data->success)
	{
		// TODO: Properly handle this error (requires change to default window
		// API to change return on all window types to bool).
		fprintf(stderr, "Failed to choose config (eglError: %d)\n", eglGetError());
		exit(EXIT_FAILURE);
	}
	if (m_data->num_configs != 1)
	{
		fprintf(stderr, "Didn't get exactly one config, but %d\n", m_data->num_configs);
		exit(EXIT_FAILURE);
	}

	m_data->egl_surface = eglCreatePbufferSurface(
		m_data->egl_display, m_data->egl_config, egl_pbuffer_attribs);
	if (m_data->egl_surface == EGL_NO_SURFACE)
	{
		fprintf(stderr, "Unable to create EGL surface (eglError: %d)\n", eglGetError());
		exit(EXIT_FAILURE);
	}

	m_data->egl_context = eglCreateContext(
		m_data->egl_display, m_data->egl_config, EGL_NO_CONTEXT, NULL);
	if (!m_data->egl_context)
	{
		fprintf(stderr, "Unable to create EGL context (eglError: %d)\n", eglGetError());
		exit(EXIT_FAILURE);
	}

	m_data->success =
		eglMakeCurrent(m_data->egl_display, m_data->egl_surface, m_data->egl_surface,
					   m_data->egl_context);
	if (!m_data->success)
	{
		fprintf(stderr, "Failed to make context current (eglError: %d)\n", eglGetError());
		exit(EXIT_FAILURE);
	}

	if (!gladLoadGL((GLADloadfunc)eglGetProcAddress))
	{
		fprintf(stderr, "failed to load GL with glad.\n");
		exit(EXIT_FAILURE);
	}

	const GLubyte* ven = glGetString(GL_VENDOR);
	printf("GL_VENDOR=%s\n", ven);

	const GLubyte* ren = glGetString(GL_RENDERER);
	printf("GL_RENDERER=%s\n", ren);
	const GLubyte* ver = glGetString(GL_VERSION);
	printf("GL_VERSION=%s\n", ver);
	const GLubyte* sl = glGetString(GL_SHADING_LANGUAGE_VERSION);
	printf("GL_SHADING_LANGUAGE_VERSION=%s\n", sl);
	glViewport(0,0,m_data->m_windowWidth, m_data->m_windowHeight);
	//int i = pthread_getconcurrency();
	//printf("pthread_getconcurrency()=%d\n", i);
}

void EGLOpenGLWindow::closeWindow()
{
	eglMakeCurrent(m_data->egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE,
				   EGL_NO_CONTEXT);
	eglDestroySurface(m_data->egl_display, m_data->egl_surface);
	eglDestroyContext(m_data->egl_display, m_data->egl_context);
	printf("Destroy EGL OpenGL window.\n");
}

void EGLOpenGLWindow::runMainLoop() {}

float EGLOpenGLWindow::getTimeInSeconds() { return 0.; }

bool EGLOpenGLWindow::requestedExit() const { return false; }

void EGLOpenGLWindow::setRequestExit() {}

void EGLOpenGLWindow::startRendering()
{
	// printf("EGL window start rendering.\n");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
}

void EGLOpenGLWindow::endRendering()
{
	// printf("EGL window end rendering.\n");
	eglSwapBuffers(m_data->egl_display, m_data->egl_surface);
}

bool EGLOpenGLWindow::isModifierKeyPressed(int key) { return false; }

void EGLOpenGLWindow::setMouseMoveCallback(b3MouseMoveCallback mouseCallback)
{
	m_data->m_mouseMoveCallback = mouseCallback;
}

b3MouseMoveCallback EGLOpenGLWindow::getMouseMoveCallback()
{
	return m_data->m_mouseMoveCallback;
}

void EGLOpenGLWindow::setMouseButtonCallback(
	b3MouseButtonCallback mouseCallback)
{
	m_data->m_mouseButtonCallback = mouseCallback;
}

b3MouseButtonCallback EGLOpenGLWindow::getMouseButtonCallback()
{
	return m_data->m_mouseButtonCallback;
}

void EGLOpenGLWindow::setResizeCallback(b3ResizeCallback resizeCallback)
{
	m_data->m_resizeCallback = resizeCallback;
}

b3ResizeCallback EGLOpenGLWindow::getResizeCallback()
{
	return m_data->m_resizeCallback;
}

void EGLOpenGLWindow::setWheelCallback(b3WheelCallback wheelCallback)
{
	m_data->m_wheelCallback = wheelCallback;
}

b3WheelCallback EGLOpenGLWindow::getWheelCallback()
{
	return m_data->m_wheelCallback;
}

void EGLOpenGLWindow::setKeyboardCallback(b3KeyboardCallback keyboardCallback)
{
	m_data->m_keyboardCallback = keyboardCallback;
}

b3KeyboardCallback EGLOpenGLWindow::getKeyboardCallback()
{
	return m_data->m_keyboardCallback;
}

void EGLOpenGLWindow::setRenderCallback(b3RenderCallback renderCallback) {}
void EGLOpenGLWindow::setWindowTitle(const char* title) {}

float EGLOpenGLWindow::getRetinaScale() const { return 1.f; }

void EGLOpenGLWindow::setAllowRetina(bool allow) {}

int EGLOpenGLWindow::getWidth() const { return m_data->m_windowWidth; }

int EGLOpenGLWindow::getHeight() const { return m_data->m_windowHeight; }

int EGLOpenGLWindow::fileOpenDialog(char* fileName, int maxFileNameLength)
{
	return 0;
}

#endif  // BT_USE_EGL
