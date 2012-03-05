/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///original author: Erwin Coumans

#ifndef BT_OPENCL_GL_INTEROP_BUFFER_H
#define BT_OPENCL_GL_INTEROP_BUFFER_H

#include "btGlutInclude.h"

#include "../basic_initialize/btOpenCLInclude.h"

class btOpenCLGLInteropBuffer
{

	cl_context	m_clContext;
	cl_command_queue	m_commandQueue;
	cl_mem	m_buffer;
	GLuint m_openGLVBO;

public:
	
	btOpenCLGLInteropBuffer(cl_context	clContext, cl_command_queue	commandQueue,GLuint openGLVBO);
	virtual ~btOpenCLGLInteropBuffer();

	void	copyCL2GL();

	void	copyGL2CL();

	cl_mem	getCLBUffer()
	{
		return m_buffer;
	}
};

#endif //BT_OPENCL_GL_INTEROP_BUFFER_H

