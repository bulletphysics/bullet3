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

#ifndef __OPENGL_INCLUDE_H
#define __OPENGL_INCLUDE_H

#ifdef BT_NO_GLAD
#include "third_party/GL/gl/include/EGL/egl.h"
#include "third_party/GL/gl/include/EGL/eglext.h"
#include "third_party/GL/gl/include/GL/gl.h"
#else

#ifdef B3_USE_GLFW
#include "glad/gl.h"
#include <GLFW/glfw3.h>
#else
#include "glad/gl.h"
#endif  //B3_USE_GLFW
#endif  //BT_NO_GLAD

//disable glGetError
//#undef glGetError
//#define glGetError MyGetError
//
//GLenum inline MyGetError()
//{
//	return 0;
//}

///on Linux only glDrawElementsInstancedARB is defined?!?
//#ifdef __linux
//#define glDrawElementsInstanced glDrawElementsInstancedARB
//
//#endif //__linux

#endif  //__OPENGL_INCLUDE_H
