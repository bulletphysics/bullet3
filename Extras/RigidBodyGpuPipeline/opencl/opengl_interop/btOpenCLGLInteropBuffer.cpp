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

#include "btOpenCLGLInteropBuffer.h"
	
btOpenCLGLInteropBuffer::btOpenCLGLInteropBuffer(cl_context	clContext, cl_command_queue	commandQueue,GLuint openGLVBO)
:m_clContext(clContext),
m_commandQueue(commandQueue),
m_openGLVBO(openGLVBO)
{
	cl_int ciErrNum = CL_SUCCESS;
//	m_buffer = clCreateFromGLBuffer(m_clContext, CL_MEM_WRITE_ONLY, m_openGLVBO, &ciErrNum);
	m_buffer = clCreateFromGLBuffer(m_clContext, CL_MEM_READ_WRITE, m_openGLVBO, &ciErrNum);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

}

btOpenCLGLInteropBuffer::~btOpenCLGLInteropBuffer()
{
	cl_int ciErrNum = CL_SUCCESS;
	clReleaseMemObject (m_buffer);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
}

void	btOpenCLGLInteropBuffer::copyCL2GL()
{
	cl_int ciErrNum = CL_SUCCESS;
	ciErrNum = clEnqueueAcquireGLObjects(m_commandQueue, 1, &m_buffer, 0, 0, NULL);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	//do some stuff




	ciErrNum = clEnqueueReleaseGLObjects(m_commandQueue, 1, &m_buffer, 0, 0, 0);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	//only wait if necessary
//	clFinish(m_commandQueue);

}

void	btOpenCLGLInteropBuffer::copyGL2CL()
{
}

