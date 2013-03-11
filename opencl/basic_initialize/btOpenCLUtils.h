/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2011 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//original author: Roman Ponomarev
//cleanup by Erwin Coumans

#ifndef BT_OPENCL_UTILS_H
#define BT_OPENCL_UTILS_H

#include "btOpenCLInclude.h"

#ifdef __cplusplus
extern "C" {
#endif


///C API for OpenCL utilities: convenience functions, see below for C++ API

/// CL Context optionally takes a GL context. This is a generic type because we don't really want this code
/// to have to understand GL types. It is a HGLRC in _WIN32 or a GLXContext otherwise.
cl_context 	btOpenCLUtils_createContextFromType(cl_device_type deviceType, cl_int* pErrNum, void* pGLCtx , void* pGLDC , int preferredDeviceIndex , int preferredPlatformIndex, cl_platform_id* platformId);
	
int btOpenCLUtils_getNumDevices(cl_context cxMainContext);

cl_device_id btOpenCLUtils_getDevice(cl_context cxMainContext, int nr);

void btOpenCLUtils_printDeviceInfo(cl_device_id device);

cl_kernel btOpenCLUtils_compileCLKernelFromString( cl_context clContext,cl_device_id device, const char* kernelSource, const char* kernelName, cl_int* pErrNum, cl_program prog,const char* additionalMacros);

//optional
cl_program btOpenCLUtils_compileCLProgramFromString( cl_context clContext,cl_device_id device, const char* kernelSource, cl_int* pErrNum,const char* additionalMacros  , const char* srcFileNameForCaching);

//the following optional APIs provide access using specific platform information
int btOpenCLUtils_getNumPlatforms(cl_int* pErrNum);

///get the nr'th platform, where nr is in the range [0..getNumPlatforms)
cl_platform_id btOpenCLUtils_getPlatform(int nr, cl_int* pErrNum);

void btOpenCLUtils_printPlatformInfo(cl_platform_id platform);

const char* btOpenCLUtils_getSdkVendorName();

cl_context 	btOpenCLUtils_createContextFromPlatform(cl_platform_id platform, cl_device_type deviceType, cl_int* pErrNum, void* pGLCtx , void* pGLDC ,int preferredDeviceIndex , int preferredPlatformIndex);

#ifdef __cplusplus
}

#define BT_MAX_STRING_LENGTH 1024

typedef struct
{
	char m_deviceName[BT_MAX_STRING_LENGTH];
	char m_deviceVendor[BT_MAX_STRING_LENGTH];
	char m_driverVersion[BT_MAX_STRING_LENGTH];
	char m_deviceExtensions[BT_MAX_STRING_LENGTH];

	cl_device_type		m_deviceType;
	cl_uint 				m_computeUnits;
	size_t 					m_workitemDims;
	size_t 					m_workItemSize[3];
	size_t 					m_image2dMaxWidth;
	size_t 					m_image2dMaxHeight;
	size_t 					m_image3dMaxWidth;
	size_t 					m_image3dMaxHeight;
	size_t 					m_image3dMaxDepth;
	size_t 					m_workgroupSize;
	cl_uint 				m_clockFrequency;
	cl_ulong				m_constantBufferSize;
	cl_ulong				m_localMemSize;
	cl_ulong				m_globalMemSize;
    cl_bool					m_errorCorrectionSupport;
	cl_device_local_mem_type m_localMemType;
	cl_uint					m_maxReadImageArgs;
	cl_uint					m_maxWriteImageArgs;



	cl_uint 				m_addressBits;
	cl_ulong				m_maxMemAllocSize;
	cl_command_queue_properties m_queueProperties;
	cl_bool					m_imageSupport;
	cl_uint					m_vecWidthChar;
	cl_uint					m_vecWidthShort;
	cl_uint					m_vecWidthInt;
	cl_uint					m_vecWidthLong;
	cl_uint					m_vecWidthFloat;
	cl_uint					m_vecWidthDouble;

} btOpenCLDeviceInfo;

typedef struct
{
	char m_platformVendor[BT_MAX_STRING_LENGTH];
	char m_platformName[BT_MAX_STRING_LENGTH];
	char m_platformVersion[BT_MAX_STRING_LENGTH];
} btOpenCLPlatformInfo;


///C++ API for OpenCL utilities: convenience functions
struct btOpenCLUtils
{
	/// CL Context optionally takes a GL context. This is a generic type because we don't really want this code
	/// to have to understand GL types. It is a HGLRC in _WIN32 or a GLXContext otherwise.
	static inline cl_context 	createContextFromType(cl_device_type deviceType, cl_int* pErrNum, void* pGLCtx = 0, void* pGLDC = 0, int preferredDeviceIndex = -1, int preferredPlatformIndex= - 1, cl_platform_id* platformId=0)
	{
		return btOpenCLUtils_createContextFromType(deviceType, pErrNum, pGLCtx , pGLDC , preferredDeviceIndex, preferredPlatformIndex, platformId);
	}
	
	static inline int getNumDevices(cl_context cxMainContext)
	{
		return btOpenCLUtils_getNumDevices(cxMainContext);
	}
	static inline cl_device_id getDevice(cl_context cxMainContext, int nr)
	{
		return btOpenCLUtils_getDevice(cxMainContext,nr);
	}

	static void getDeviceInfo(cl_device_id device, btOpenCLDeviceInfo* info);

	static inline void printDeviceInfo(cl_device_id device)
	{
		btOpenCLUtils_printDeviceInfo(device);
	}

	static inline cl_kernel compileCLKernelFromString( cl_context clContext,cl_device_id device, const char* kernelSource, const char* kernelName, cl_int* pErrNum=0, cl_program prog=0,const char* additionalMacros = "" )
	{
		return btOpenCLUtils_compileCLKernelFromString(clContext,device, kernelSource,  kernelName, pErrNum, prog,additionalMacros);
	}

	//optional
	static inline cl_program compileCLProgramFromString( cl_context clContext,cl_device_id device, const char* kernelSource, cl_int* pErrNum=0,const char* additionalMacros = "" , const char* srcFileNameForCaching=0)
	{
		return btOpenCLUtils_compileCLProgramFromString(clContext,device, kernelSource, pErrNum,additionalMacros, srcFileNameForCaching);
	}

	//the following optional APIs provide access using specific platform information
	static inline int getNumPlatforms(cl_int* pErrNum=0)
	{
		return btOpenCLUtils_getNumPlatforms(pErrNum);
	}
	///get the nr'th platform, where nr is in the range [0..getNumPlatforms)
	static inline cl_platform_id getPlatform(int nr, cl_int* pErrNum=0)
	{
		return btOpenCLUtils_getPlatform(nr,pErrNum);
	}
	
	static void getPlatformInfo(cl_platform_id platform, btOpenCLPlatformInfo* platformInfo);

	static inline void printPlatformInfo(cl_platform_id platform)
	{
		btOpenCLUtils_printPlatformInfo(platform);
	}

	static inline const char* getSdkVendorName()
	{
		return btOpenCLUtils_getSdkVendorName();
	}
	static inline cl_context 	createContextFromPlatform(cl_platform_id platform, cl_device_type deviceType, cl_int* pErrNum, void* pGLCtx = 0, void* pGLDC = 0,int preferredDeviceIndex = -1, int preferredPlatformIndex= -1)
	{
		return btOpenCLUtils_createContextFromPlatform(platform, deviceType, pErrNum, pGLCtx,pGLDC,preferredDeviceIndex, preferredPlatformIndex);
	}
};

#endif //__cplusplus

#endif // BT_OPENCL_UTILS_H
