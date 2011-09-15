/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2010 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <string.h>

#include "btOclCommon.h"


static const char* spPlatformVendor = 
#if defined(CL_PLATFORM_MINI_CL)
"MiniCL, SCEA";
#elif defined(CL_PLATFORM_INTEL)
"Intel(R) Corporation";
#elif defined(CL_PLATFORM_AMD)
"Advanced Micro Devices, Inc.";
#elif defined(CL_PLATFORM_NVIDIA)
"NVIDIA Corporation";
#else
"Unknown Vendor";
#endif

#ifndef CL_PLATFORM_MINI_CL
#ifdef _WIN32
#include "CL/cl_gl.h"
#endif //_WIN32
#endif

cl_context btOclCommon::createContextFromType(cl_device_type deviceType, cl_int* pErrNum, void* pGLContext, void* pGLDC )
{
    cl_uint numPlatforms;    
	cl_platform_id platform = NULL;    
	cl_int ciErrNum = clGetPlatformIDs(0, NULL, &numPlatforms);
	if(ciErrNum != CL_SUCCESS)
	{
		if(pErrNum != NULL) *pErrNum = ciErrNum;
		return NULL;
	}
	if(numPlatforms > 0)     
	{        
		cl_platform_id* platforms = new cl_platform_id[numPlatforms];
		ciErrNum = clGetPlatformIDs(numPlatforms, platforms, NULL);
		if(ciErrNum != CL_SUCCESS)
		{
			if(pErrNum != NULL) *pErrNum = ciErrNum;
			return NULL;
		}
		for (unsigned i = 0; i < numPlatforms; ++i)         
		{            
			char pbuf[128];            
			ciErrNum = clGetPlatformInfo(	platforms[i],
											CL_PLATFORM_VENDOR,                                       
											sizeof(pbuf),                                       
											pbuf,                                       
											NULL);
			if(ciErrNum != CL_SUCCESS)
			{
				if(pErrNum != NULL) *pErrNum = ciErrNum;
				return NULL;
			}
            platform = platforms[i];            
			if(!strcmp(pbuf, spPlatformVendor))             
			{                
				break;            
			}        
		}        
		delete[] platforms;    
	}
    /*     
	 * If we could find our platform, use it. Otherwise pass a NULL and get whatever the     
	 * implementation thinks we should be using.     
	 */
    cl_context_properties cps[7] =     
	{        
		CL_CONTEXT_PLATFORM,         
		(cl_context_properties)platform, 
		0,
		0,
		0,
		0,
		0
	};    
#ifndef CL_PLATFORM_MINI_CL
#ifdef _WIN32
	// If we have a gl context then enable interop
	if( pGLContext )
	{
		cps[2] = CL_GL_CONTEXT_KHR;
		cps[3] = (cl_context_properties)pGLContext;
		cps[4] = CL_WGL_HDC_KHR;
		cps[5] = (cl_context_properties)pGLDC;
	}
#endif
#endif //CL_PLATFORM_MINI_CL

	/* Use NULL for backward compatibility */    
	cl_context_properties* cprops = (NULL == platform) ? NULL : cps;
    cl_context retContext = clCreateContextFromType(cprops, 
													deviceType,                  
													NULL,                  
													NULL,                  
													&ciErrNum);
	if(pErrNum != NULL) *pErrNum = ciErrNum;
	return retContext;
}

