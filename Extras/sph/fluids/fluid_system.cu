/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include <cutil.h>
#include <cstdlib>
#include <cstdio>
#include <string.h>

#if defined(__APPLE__) || defined(MACOSX)
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif
#include <cuda_gl_interop.h>

#include "fluid_system_kern.cu"

extern "C"
{

// Compute number of blocks to create
int iDivUp (int a, int b) {
    return (a % b != 0) ? (a / b + 1) : (a / b);
}
void computeNumBlocks (int numPnts, int minThreads, int &numBlocks, int &numThreads)
{
    numThreads = min( minThreads, numPnts );
    numBlocks = iDivUp ( numPnts, numThreads );
}


void Grid_InsertParticlesCUDA ( uchar* data, uint stride, uint numPoints )
{
    int numThreads, numBlocks;
    computeNumBlocks (numPoints, 256, numBlocks, numThreads);

	// transfer point data to device
    char* pntData;
	size = numPoints * stride;
	cudaMalloc( (void**) &pntData, size);
	cudaMemcpy( pntData, data, size, cudaMemcpyHostToDevice);    

    // execute the kernel
    insertParticles<<< numBlocks, numThreads >>> ( pntData, stride );
    
    // transfer data back to host
    cudaMemcpy( data, pntData, cudaMemcpyDeviceToHost);
    
    // check if kernel invocation generated an error
    CUT_CHECK_ERROR("Kernel execution failed");
    CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vboPos));
}