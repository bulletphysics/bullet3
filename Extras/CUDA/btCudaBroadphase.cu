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

#include <cstdlib>
#include <cstdio>
#include <string.h>

#include "cutil_math.h"
#include "math_constants.h"

#if defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <cuda_gl_interop.h>

#include "btCudaBroadphaseKernel.h"
//#include "radixsort.cu"


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//               K E R N E L    F U N C T I O N S 
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

#ifdef __DEVICE_EMULATION__
	#define B_CUDA_USE_TEX 0
#else
	#define B_CUDA_USE_TEX 1
#endif

__device__ inline btCuda3F1U tex_fetch3F1U(float4 a) { return *((btCuda3F1U*)(&a)); }

#if B_CUDA_USE_TEX
	#define BT3DGRIDFETCH(t, i) tex_fetch3F1U(tex1Dfetch(t##Tex, i))
#else
	#define BT3DGRIDFETCH(t, i) t[i]
#endif

texture<uint2, 1, cudaReadModeElementType> particleHashTex;
texture<uint, 1, cudaReadModeElementType> cellStartTex;
texture<float4, 1, cudaReadModeElementType> pAABBTex;

//----------------------------------------------------------------------------------------

__constant__ btCudaBroadphaseParams params;

//----------------------------------------------------------------------------------------

#define BT3DGRID__device__ __device__
#define BT3DGRIDmax(a, b) max(a, b)
#define BT3DGRIDmin(a, b) min(a, b)
#define BT3DGRIDparams params
#define BT3DGRID__mul24(a, b) __mul24(a, b)
#define BT3DGRID__global__ __global__
#define BT3DGRID__shared__ __shared__
#define BT3DGRID__syncthreads() __syncthreads()
#define BT3DGRIDmake_uint2(x, y) make_uint2(x, y)
#define BT3DGRIDmake_int3(x, y, z) make_int3(x, y, z)
#define BT3DGRIDPREF(func) btCuda_##func
#define BT3DGPRDMemset cudaMemset
#define BT3DGRIDblockIdx blockIdx
#define BT3DGRIDblockDim blockDim
#define BT3DGRIDthreadIdx threadIdx
#define BT3DGRIDEXECKERNEL(numb, numt, kfunc, args) kfunc<<<numb, numt>>>args

//----------------------------------------------------------------------------------------

//! Check for CUDA error
#  define CUT_CHECK_ERROR(errorMessage) do {                                 \
    cudaError_t err = cudaGetLastError();                                    \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda error: %s in file '%s' in line %i : %s.\n",    \
                errorMessage, __FILE__, __LINE__, cudaGetErrorString( err) );\
        btCuda_exit(EXIT_FAILURE);                                           \
    }                                                                        \
    err = cudaThreadSynchronize();                                           \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda error: %s in file '%s' in line %i : %s.\n",    \
                errorMessage, __FILE__, __LINE__, cudaGetErrorString( err) );\
        btCuda_exit(EXIT_FAILURE);                                           \
    } } while (0)


#  define MY_CUDA_SAFE_CALL_NO_SYNC( call) do {                              \
    cudaError err = call;                                                    \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",        \
                __FILE__, __LINE__, cudaGetErrorString( err) );              \
        btCuda_exit(EXIT_FAILURE);                                           \
    } } while (0)

#  define MY_CUDA_SAFE_CALL( call) do {                                      \
    MY_CUDA_SAFE_CALL_NO_SYNC(call);                                         \
    cudaError err = cudaThreadSynchronize();                                 \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "Cuda errorSync in file '%s' in line %i : %s.\n",    \
                __FILE__, __LINE__, cudaGetErrorString( err) );              \
        btCuda_exit(EXIT_FAILURE);                                           \
    } } while (0)

//----------------------------------------------------------------------------------------

void btCuda_exit(int val)
{
	exit(val);
}

void btCuda_allocateArray(void** devPtr, unsigned int size)
{
    MY_CUDA_SAFE_CALL(cudaMalloc(devPtr, size));
}

void btCuda_freeArray(void* devPtr)
{
    MY_CUDA_SAFE_CALL(cudaFree(devPtr));
}

void btCuda_copyArrayFromDevice(void* host, const void* device, unsigned int size)
{   
    MY_CUDA_SAFE_CALL(cudaMemcpy(host, device, size, cudaMemcpyDeviceToHost));
}

void btCuda_copyArrayToDevice(void* device, const void* host, unsigned int size)
{
    MY_CUDA_SAFE_CALL(cudaMemcpy((char*)device, host, size, cudaMemcpyHostToDevice));
}

void btCuda_setParameters(btCudaBroadphaseParams* hostParams)
{
    // copy parameters to constant memory
    MY_CUDA_SAFE_CALL(cudaMemcpyToSymbol(params, hostParams, sizeof(btCudaBroadphaseParams)));
}

//----------------------------------------------------------------------------------------

#include "bt3DGridBroadphaseFunc.h"

//----------------------------------------------------------------------------------------

