/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <cstdlib>
#include <cstdio>
#include <string.h>

#include <GL/glut.h>
#include <cuda_gl_interop.h>

#include "cutil_math.h"
#include "math_constants.h"

#include <vector_types.h>




#include "btCudaDefines.h"
#include "../../src/BulletMultiThreaded/btGpuUtilsSharedDefs.h"


void btCuda_exit(int val)
{
    fprintf(stderr, "Press ENTER key to terminate the program\n");
    getchar();
	exit(val);
}

void btCuda_allocateArray(void** devPtr, unsigned int size)
{
    BT_GPU_SAFE_CALL(cudaMalloc(devPtr, size));
}

void btCuda_freeArray(void* devPtr)
{
    BT_GPU_SAFE_CALL(cudaFree(devPtr));
}

void btCuda_copyArrayFromDevice(void* host, const void* device, unsigned int size)
{   
    BT_GPU_SAFE_CALL(cudaMemcpy(host, device, size, cudaMemcpyDeviceToHost));
}

void btCuda_copyArrayToDevice(void* device, const void* host, unsigned int size)
{
    BT_GPU_SAFE_CALL(cudaMemcpy((char*)device, host, size, cudaMemcpyHostToDevice));
}


void btCuda_registerGLBufferObject(unsigned int vbo)
{
    BT_GPU_SAFE_CALL(cudaGLRegisterBufferObject(vbo));
}

void* btCuda_mapGLBufferObject(unsigned int vbo)
{
    void *ptr;
    BT_GPU_SAFE_CALL(cudaGLMapBufferObject(&ptr, vbo));
    return ptr;
}

void btCuda_unmapGLBufferObject(unsigned int vbo)
{
    BT_GPU_SAFE_CALL(cudaGLUnmapBufferObject(vbo));
}



#include "../../src/BulletMultiThreaded/btGpuUtilsSharedCode.h"


