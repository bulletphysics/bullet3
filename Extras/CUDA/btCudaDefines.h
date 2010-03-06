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



// Common preprocessor definitions for CUDA compiler



#ifndef BTCUDADEFINES_H
#define BTCUDADEFINES_H



#ifdef __DEVICE_EMULATION__
	#define B_CUDA_USE_TEX 0
#else
	#define B_CUDA_USE_TEX 1
#endif


#if B_CUDA_USE_TEX
	#define BT_GPU_FETCH(t, i) tex_fetch3F1U(tex1Dfetch(t##Tex, i))
	#define BT_GPU_FETCH4(t, i) tex1Dfetch(t##Tex, i)
#else
	#define BT_GPU_FETCH(t, i) t[i]
	#define BT_GPU_FETCH4(t, i) t[i]
#endif



#define BT_GPU___device__ __device__
#define BT_GPU___devdata__ __device__
#define BT_GPU___constant__ __constant__
#define BT_GPU_max(a, b) max(a, b)
#define BT_GPU_min(a, b) min(a, b)
#define BT_GPU_params params
#define BT_GPU___mul24(a, b) __mul24(a, b)
#define BT_GPU___global__ __global__
#define BT_GPU___shared__ __shared__
#define BT_GPU___syncthreads() __syncthreads()
#define BT_GPU_make_uint2(x, y) make_uint2(x, y)
#define BT_GPU_make_int3(x, y, z) make_int3(x, y, z)
#define BT_GPU_make_float3(x, y, z) make_float3(x, y, z)
#define BT_GPU_make_float34(x) make_float3(x)
#define BT_GPU_make_float31(x) make_float3(x)
#define BT_GPU_make_float42(a, b) make_float4(a, b) 
#define BT_GPU_make_float44(a, b, c, d) make_float4(a, b, c, d) 
#define BT_GPU_PREF(func) btCuda_##func
#define BT_GPU_Memset cudaMemset
#define BT_GPU_MemcpyToSymbol(a, b, c) cudaMemcpyToSymbol(a, b, c)
#define BT_GPU_blockIdx blockIdx
#define BT_GPU_blockDim blockDim
#define BT_GPU_threadIdx threadIdx
#define BT_GPU_dot(a, b) dot(a, b)
#define BT_GPU_dot4(a, b) dot(a, b)
#define BT_GPU_cross(a, b) cross(a, b)
#define BT_GPU_BindTexture(a, b, c, d) cudaBindTexture(a, b, c, d) 
#define BT_GPU_UnbindTexture(a) cudaUnbindTexture(a) 
#define BT_GPU_EXECKERNEL(numb, numt, kfunc, args) kfunc<<<numb, numt>>>args



//! Check for CUDA error
#define BT_GPU_CHECK_ERROR(errorMessage)									\
	do																		\
	{																		\
		cudaError_t err = cudaGetLastError();								\
		if(err != cudaSuccess)												\
		{																	\
			fprintf(stderr,"Cuda error: %s in file '%s' in line %i : %s.\n",\
				errorMessage, __FILE__, __LINE__, cudaGetErrorString( err));\
			btCuda_exit(EXIT_FAILURE);                                      \
		}                                                                   \
		err = cudaThreadSynchronize();                                      \
		if(err != cudaSuccess)												\
		{																	\
			fprintf(stderr,"Cuda error: %s in file '%s' in line %i : %s.\n",\
				errorMessage, __FILE__, __LINE__, cudaGetErrorString( err));\
			btCuda_exit(EXIT_FAILURE);										\
		}																	\
	}																		\
	while(0)



#define BT_GPU_SAFE_CALL_NO_SYNC(call)										\
	do																		\
	{																		\
		cudaError err = call;												\
		if(err != cudaSuccess)												\
		{																	\
			fprintf(stderr, "Cuda error in file '%s' in line %i : %s.\n",	\
				__FILE__, __LINE__, cudaGetErrorString( err) );             \
			btCuda_exit(EXIT_FAILURE);										\
		}																	\
	}																		\
	while(0)



#define BT_GPU_SAFE_CALL(call)												\
	do																		\
	{																		\
		BT_GPU_SAFE_CALL_NO_SYNC(call);										\
		cudaError err = cudaThreadSynchronize();							\
		if(err != cudaSuccess)												\
		{																	\
			fprintf(stderr,"Cuda errorSync in file '%s' in line %i : %s.\n",\
				__FILE__, __LINE__, cudaGetErrorString( err) );				\
			btCuda_exit(EXIT_FAILURE);										\
		}																	\
	} while (0)



extern "C" void btCuda_exit(int val);



#endif // BTCUDADEFINES_H





