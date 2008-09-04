/*
 * Copyright 1993-2006 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO USER:   
 *
 * This source code is subject to NVIDIA ownership rights under U.S. and 
 * international Copyright laws.  
 *
 * NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE 
 * CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR 
 * IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH 
 * REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF 
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.   
 * IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL, 
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS 
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE 
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE 
 * OR PERFORMANCE OF THIS SOURCE CODE.  
 *
 * U.S. Government End Users.  This source code is a "commercial item" as 
 * that term is defined at 48 C.F.R. 2.101 (OCT 1995), consisting  of 
 * "commercial computer software" and "commercial computer software 
 * documentation" as such terms are used in 48 C.F.R. 12.212 (SEPT 1995) 
 * and is provided to the U.S. Government only as a commercial end item.  
 * Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through 
 * 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the 
 * source code with only those rights set forth herein.
 */

//#include <cutil.h>
#include <cstdlib>
#include <cstdio>
#include <string.h>

#if defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <cuda_gl_interop.h>

#include "particles_kernel.cu"
#include "radixsort.cu"

    //! Check for CUDA error
#  define CUT_CHECK_ERROR(errorMessage) do {                                 \
    cudaError_t err = cudaGetLastError();                                    \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda error: %s in file '%s' in line %i : %s.\n",    \
                errorMessage, __FILE__, __LINE__, cudaGetErrorString( err) );\
        exit(EXIT_FAILURE);                                                  \
    }                                                                        \
    err = cudaThreadSynchronize();                                           \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda error: %s in file '%s' in line %i : %s.\n",    \
                errorMessage, __FILE__, __LINE__, cudaGetErrorString( err) );\
        exit(EXIT_FAILURE);                                                  \
    } } while (0)


#  define MY_CUDA_SAFE_CALL_NO_SYNC( call) do {                                 \
    cudaError err = call;                                                    \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",        \
                __FILE__, __LINE__, cudaGetErrorString( err) );              \
        exit(EXIT_FAILURE);                                                  \
    } } while (0)

#  define MY_CUDA_SAFE_CALL( call) do {                                         \
    MY_CUDA_SAFE_CALL_NO_SYNC(call);                                            \
    cudaError err = cudaThreadSynchronize();                                 \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",        \
                __FILE__, __LINE__, cudaGetErrorString( err) );              \
        exit(EXIT_FAILURE);                                                  \
    } } while (0)


extern "C"
{

void cudaInit(int argc, char **argv)
{   
    //CUT_DEVICE_INIT(argc, argv);
}

void allocateArray(void **devPtr, size_t size)
{
    MY_CUDA_SAFE_CALL(cudaMalloc(devPtr, size));
}

void freeArray(void *devPtr)
{
    MY_CUDA_SAFE_CALL(cudaFree(devPtr));
}

void threadSync()
{
    MY_CUDA_SAFE_CALL(cudaThreadSynchronize());
}

void copyArrayFromDevice(void* host, const void* device, unsigned int vbo, int size)
{   
    if (vbo)
        MY_CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&device, vbo));
    MY_CUDA_SAFE_CALL(cudaMemcpy(host, device, size, cudaMemcpyDeviceToHost));
    if (vbo)
        MY_CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vbo));
}

void copyArrayToDevice(void* device, const void* host, int offset, int size)
{
    MY_CUDA_SAFE_CALL(cudaMemcpy((char *) device + offset, host, size, cudaMemcpyHostToDevice));
}

void registerGLBufferObject(uint vbo)
{
    MY_CUDA_SAFE_CALL(cudaGLRegisterBufferObject(vbo));
}

void unregisterGLBufferObject(uint vbo)
{
    MY_CUDA_SAFE_CALL(cudaGLUnregisterBufferObject(vbo));
}

void setParameters(SimParams *hostParams)
{
    // copy parameters to constant memory
    MY_CUDA_SAFE_CALL( cudaMemcpyToSymbol(params, hostParams, sizeof(SimParams)) );
}

//Round a / b to nearest higher integer value
int iDivUp(int a, int b){
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

// compute grid and thread block size for a given number of elements
void computeGridSize(int n, int blockSize, int &numBlocks, int &numThreads)
{
    numThreads = min(blockSize, n);
    numBlocks = iDivUp(n, numThreads);
}

void 
integrateSystem(uint vboOldPos, uint vboNewPos, 
                float* oldVel, float* newVel, 
                float deltaTime,
                int numBodies)
{
    int numThreads, numBlocks;
    computeGridSize(numBodies, 256, numBlocks, numThreads);

    float *oldPos, *newPos;
    MY_CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&oldPos, vboOldPos));
    MY_CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&newPos, vboNewPos));

    // execute the kernel
    integrate<<< numBlocks, numThreads >>>((float4*)newPos, (float4*)newVel,
                                           (float4*)oldPos, (float4*)oldVel,
                                           deltaTime);
    
    // check if kernel invocation generated an error
    CUT_CHECK_ERROR("integrate kernel execution failed");

    MY_CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vboOldPos));
    MY_CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vboNewPos));
}

void 
updateGrid(uint    vboPos, 
           uint*   gridCounters,
           uint*   gridCells,
           uint    numBodies,
           uint    numCells)
{
    int numThreads, numBlocks;
    computeGridSize(numBodies, 256, numBlocks, numThreads);

    float *pos;
    MY_CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&pos, vboPos));

    MY_CUDA_SAFE_CALL(cudaMemset(gridCounters, 0, numCells*sizeof(uint)));

    // execute the kernel
    updateGridD<<< numBlocks, numThreads >>>((float4 *) pos,
                                             gridCounters,
                                             gridCells);
    
    // check if kernel invocation generated an error
    CUT_CHECK_ERROR("Kernel execution failed");

    MY_CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vboPos));
}


void 
calcHash(uint    vboPos, 
         uint*   particleHash,
         int     numBodies)
{
    int numThreads, numBlocks;
    computeGridSize(numBodies, 256, numBlocks, numThreads);

    float *pos;
    MY_CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&pos, vboPos));

    // execute the kernel
    calcHashD<<< numBlocks, numThreads >>>((float4 *) pos,
                                           (uint2 *) particleHash);
    
    // check if kernel invocation generated an error
    CUT_CHECK_ERROR("Kernel execution failed");

    MY_CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vboPos));
}

void 
reorderDataAndFindCellStart(uint*  particleHash,
							uint   vboOldPos,
							float* oldVel,
							float* sortedPos,
							float* sortedVel,
							uint*  cellStart,
							uint   numBodies,
							uint   numCells)
{
    int numThreads, numBlocks;
    computeGridSize(numBodies, 256, numBlocks, numThreads);

	MY_CUDA_SAFE_CALL(cudaMemset(cellStart, 0xffffffff, numCells*sizeof(uint)));

    float *oldPos;
    MY_CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&oldPos, vboOldPos));

#if USE_TEX
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, oldPosTex, oldPos, numBodies*sizeof(float4)));
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, oldVelTex, oldVel, numBodies*sizeof(float4)));
#endif

    reorderDataAndFindCellStartD<<< numBlocks, numThreads >>>(
		(uint2 *)  particleHash,
        (float4 *) oldPos,
        (float4 *) oldVel,
        (float4 *) sortedPos,
        (float4 *) sortedVel,
        (uint *)   cellStart);
    CUT_CHECK_ERROR("Kernel execution failed: reorderDataAndFindCellStartD");

#if USE_TEX
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(oldPosTex));
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(oldVelTex));
#endif

    MY_CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vboOldPos));
}

void
collide(uint   vboOldPos, uint vboNewPos,
        float* sortedPos, float* sortedVel,
        float* oldVel, float* newVel,
        uint*  gridCounters,
        uint*  gridCells,
        uint*  particleHash,
        uint*  cellStart,
        uint   numBodies,
        uint   numCells,
        uint   maxParticlesPerCell)
{
    float4 *oldPos, *newPos;
    MY_CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&oldPos, vboOldPos));
    MY_CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&newPos, vboNewPos));

#if USE_TEX

#if USE_SORT
    // use sorted arrays
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, oldPosTex, sortedPos, numBodies*sizeof(float4)));
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, oldVelTex, sortedVel, numBodies*sizeof(float4)));

    MY_CUDA_SAFE_CALL(cudaBindTexture(0, particleHashTex, particleHash, numBodies*sizeof(uint2)));
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, cellStartTex, cellStart, numCells*sizeof(uint)));
#else

    MY_CUDA_SAFE_CALL(cudaBindTexture(0, oldPosTex, oldPos, numBodies*sizeof(float4)));
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, oldVelTex, oldVel, numBodies*sizeof(float4)));

    MY_CUDA_SAFE_CALL(cudaBindTexture(0, gridCountersTex, gridCounters,numCells*sizeof(uint)));
    MY_CUDA_SAFE_CALL(cudaBindTexture(0, gridCellsTex, gridCells, numCells*maxParticlesPerCell*sizeof(uint)));
#endif

#endif

    // thread per particle
    int numThreads, numBlocks;
    computeGridSize(numBodies, BLOCKDIM, numBlocks, numThreads);

    // execute the kernel
    collideD<<< numBlocks, numThreads >>>((float4*)newPos, (float4*)newVel,
#if USE_SORT
                                          (float4*)sortedPos, (float4*)sortedVel,
                                          (uint2 *) particleHash,
                                          cellStart
#else
                                          (float4*)oldPos, (float4*)oldVel,
                                          gridCounters,
                                          gridCells
#endif
                                          );

    // check if kernel invocation generated an error
    CUT_CHECK_ERROR("Kernel execution failed");

    MY_CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vboNewPos));
    MY_CUDA_SAFE_CALL(cudaGLUnmapBufferObject(vboOldPos));

#if USE_TEX
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(oldPosTex));
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(oldVelTex));

#if USE_SORT
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(particleHashTex));
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(cellStartTex));
#else
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(gridCountersTex));
    MY_CUDA_SAFE_CALL(cudaUnbindTexture(gridCellsTex));
#endif
#endif
}

}   // extern "C"
