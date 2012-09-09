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

#include <string.h>

#include "btOpenCLUtils.h"
#include <stdio.h>
#include <stdlib.h>

#define BT_MAX_CL_DEVICES 16 //who needs 16 devices?
//#define BT_USE_CACHE_DIR
#ifdef _WIN32
#include <Windows.h>
#endif //_WIN32

#include <assert.h>
#define btAssert assert

//Set the preferred platform vendor using the OpenCL SDK
static const char* spPlatformVendor =
#if defined(CL_PLATFORM_MINI_CL)
"MiniCL, SCEA";
#elif defined(CL_PLATFORM_AMD)
"Advanced Micro Devices, Inc.";
#elif defined(CL_PLATFORM_NVIDIA)
"NVIDIA Corporation";
#elif defined(CL_PLATFORM_INTEL)
"Intel(R) Corporation";
#else
"Unknown Vendor";
#endif

#ifndef CL_PLATFORM_MINI_CL
#ifdef _WIN32
#include "CL/cl_gl.h"
#endif //_WIN32
#endif

int btOpenCLUtils::getNumPlatforms(cl_int* pErrNum)
{
	cl_uint numPlatforms=0;
	cl_int ciErrNum = clGetPlatformIDs(0, NULL, &numPlatforms);

	if(ciErrNum != CL_SUCCESS)
	{
		if(pErrNum != NULL) 
			*pErrNum = ciErrNum;
	}
	return numPlatforms;
}

const char* btOpenCLUtils::getSdkVendorName()
{
	return spPlatformVendor;
}

cl_platform_id btOpenCLUtils::getPlatform(int platformIndex, cl_int* pErrNum)
{
	cl_platform_id platform = 0;

	cl_uint numPlatforms;
	cl_int ciErrNum = clGetPlatformIDs(0, NULL, &numPlatforms);
	
	if (platformIndex>=0 && platformIndex<numPlatforms)
	{
		cl_platform_id* platforms = new cl_platform_id[numPlatforms];
		ciErrNum = clGetPlatformIDs(numPlatforms, platforms, NULL);
		if(ciErrNum != CL_SUCCESS)
		{
			if(pErrNum != NULL) 
				*pErrNum = ciErrNum;
			return platform;
		}

		platform = platforms[platformIndex];

		delete[] platforms;
	}

	return platform;
}

void btOpenCLUtils::getPlatformInfo(cl_platform_id platform, btOpenCLPlatformInfo& platformInfo)
{
	cl_int ciErrNum;

	ciErrNum = clGetPlatformInfo(	platform,CL_PLATFORM_VENDOR,BT_MAX_STRING_LENGTH,platformInfo.m_platformVendor,NULL);
	oclCHECKERROR(ciErrNum,CL_SUCCESS);
	ciErrNum = clGetPlatformInfo(	platform,CL_PLATFORM_NAME,BT_MAX_STRING_LENGTH,platformInfo.m_platformName,NULL);
	oclCHECKERROR(ciErrNum,CL_SUCCESS);
	ciErrNum = clGetPlatformInfo(	platform,CL_PLATFORM_VERSION,BT_MAX_STRING_LENGTH,platformInfo.m_platformVersion,NULL);
	oclCHECKERROR(ciErrNum,CL_SUCCESS);
}

cl_context btOpenCLUtils::createContextFromPlatform(cl_platform_id platform, cl_device_type deviceType, cl_int* pErrNum, void* pGLContext, void* pGLDC, int preferredDeviceIndex, int preferredPlatformIndex)
{
	cl_context retContext = 0;
	cl_int ciErrNum=0;

	/*     
	* If we could find our platform, use it. Otherwise pass a NULL and get whatever the     
	* implementation thinks we should be using.     
	*/
	cl_context_properties cps[7] = {0,0,0,0,0,0,0};
	cps[0] = CL_CONTEXT_PLATFORM;
	cps[1] = (cl_context_properties)platform;
	if (pGLContext && pGLDC)
	{
#if defined(CL_PLATFORM_AMD) || defined(CL_PLATFORM_NVIDIA)
		cps[2] = CL_GL_CONTEXT_KHR;
		cps[3] = (cl_context_properties)pGLContext;
		cps[4] = CL_WGL_HDC_KHR;
		cps[5] = (cl_context_properties)pGLDC;
#endif
	}

	cl_uint num_entries = BT_MAX_CL_DEVICES;
 	cl_device_id devices[BT_MAX_CL_DEVICES];

	cl_uint num_devices=-1;

	ciErrNum = clGetDeviceIDs(	
		platform,
		deviceType,
 		num_entries,
 		devices,
 		&num_devices);

	cl_context_properties* cprops = (NULL == platform) ? NULL : cps;

	if (pGLContext)
	{
		//search for the GPU that relates to the OpenCL context
		for (int i=0;i<num_devices;i++)
		{
			retContext = clCreateContext(cprops,1,&devices[i],NULL,NULL,&ciErrNum);
			if (ciErrNum==CL_SUCCESS)
				break;
		}
	}
	else
	{
		if (preferredDeviceIndex>=0 && preferredDeviceIndex<num_devices)
		{
			//create a context of the preferred device index
			retContext = clCreateContext(cprops,1,&devices[preferredDeviceIndex],NULL,NULL,&ciErrNum);
		} else
		{
			//create a context of all devices
			retContext = clCreateContext(cprops,num_devices,devices,NULL,NULL,&ciErrNum);
		}
	}
	if(pErrNum != NULL) 
	{
		*pErrNum = ciErrNum;
	};

	return retContext;
}

cl_context btOpenCLUtils::createContextFromType(cl_device_type deviceType, cl_int* pErrNum, void* pGLContext, void* pGLDC , int preferredDeviceIndex, int preferredPlatformIndex)
{
	cl_uint numPlatforms;
	cl_context retContext = 0;
	
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
		int i;


		for ( i = 0; i < numPlatforms; ++i)         
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

			if (preferredPlatformIndex>=0 && i==preferredPlatformIndex)
			{
				cl_platform_id tmpPlatform = platforms[0];
				platforms[0] = platforms[i];
				platforms[i] = tmpPlatform;
				break;
			} else
			{
				if(!strcmp(pbuf, spPlatformVendor))
				{
					cl_platform_id tmpPlatform = platforms[0];
					platforms[0] = platforms[i];
					platforms[i] = tmpPlatform;
					break;
				}
			}
		}

		for (i = 0; i < numPlatforms; ++i)         
		{
			cl_platform_id platform = platforms[i];
			assert(platform);

			retContext = btOpenCLUtils::createContextFromPlatform(platform,deviceType,pErrNum,pGLContext,pGLDC,preferredDeviceIndex);

			if (retContext)
			{
//				printf("OpenCL platform details:\n");
				btOpenCLPlatformInfo platformInfo;

				btOpenCLUtils::getPlatformInfo(platform, platformInfo);

				printf("  CL_PLATFORM_VENDOR: \t\t\t%s\n",platformInfo.m_platformVendor);
				printf("  CL_PLATFORM_NAME: \t\t\t%s\n",platformInfo.m_platformName);
				printf("  CL_PLATFORM_VERSION: \t\t\t%s\n",platformInfo.m_platformVersion);

				break;
			}
		}

		delete[] platforms;    
	}
	return retContext;
}


//////////////////////////////////////////////////////////////////////////////
//! Gets the id of the nth device from the context
//!
//! @return the id or -1 when out of range
//! @param cxMainContext         OpenCL context
//! @param device_idx            index of the device of interest
//////////////////////////////////////////////////////////////////////////////
cl_device_id btOpenCLUtils::getDevice(cl_context cxMainContext, int deviceIndex)
{
	size_t szParmDataBytes;
	cl_device_id* cdDevices;

	// get the list of devices associated with context
	clGetContextInfo(cxMainContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);

	if( szParmDataBytes / sizeof(cl_device_id) < deviceIndex ) {
		return (cl_device_id)-1;
	}

	cdDevices = (cl_device_id*) malloc(szParmDataBytes);

	clGetContextInfo(cxMainContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);

	cl_device_id device = cdDevices[deviceIndex];
	free(cdDevices);

	return device;
}

int btOpenCLUtils::getNumDevices(cl_context cxMainContext)
{
	size_t szParamDataBytes;
	clGetContextInfo(cxMainContext, CL_CONTEXT_DEVICES, 0, NULL, &szParamDataBytes);
	int device_count = (int) szParamDataBytes/ sizeof(cl_device_id);
	return device_count;
}

void btOpenCLUtils::printDeviceInfo(cl_device_id device)
{
	btOpenCLDeviceInfo info;
	getDeviceInfo(device,info);

	printf("  CL_DEVICE_NAME: \t\t\t%s\n", info.m_deviceName);
	printf("  CL_DEVICE_VENDOR: \t\t\t%s\n", info.m_deviceVendor);
	printf("  CL_DRIVER_VERSION: \t\t\t%s\n", info.m_driverVersion);

	if( info.m_deviceType & CL_DEVICE_TYPE_CPU )
		printf("  CL_DEVICE_TYPE:\t\t\t%s\n", "CL_DEVICE_TYPE_CPU");
	if( info.m_deviceType & CL_DEVICE_TYPE_GPU )
		printf("  CL_DEVICE_TYPE:\t\t\t%s\n", "CL_DEVICE_TYPE_GPU");
	if( info.m_deviceType & CL_DEVICE_TYPE_ACCELERATOR )
		printf("  CL_DEVICE_TYPE:\t\t\t%s\n", "CL_DEVICE_TYPE_ACCELERATOR");
	if( info.m_deviceType & CL_DEVICE_TYPE_DEFAULT )
		printf("  CL_DEVICE_TYPE:\t\t\t%s\n", "CL_DEVICE_TYPE_DEFAULT");

	printf("  CL_DEVICE_MAX_COMPUTE_UNITS:\t\t%u\n", info.m_computeUnits);
	printf("  CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS:\t%zu\n", info.m_workitemDims);
	printf("  CL_DEVICE_MAX_WORK_ITEM_SIZES:\t%zu / %zu / %zu \n", info.m_workItemSize[0], info.m_workItemSize[1], info.m_workItemSize[2]);
	printf("  CL_DEVICE_MAX_WORK_GROUP_SIZE:\t%zu\n", info.m_workgroupSize);
	printf("  CL_DEVICE_MAX_CLOCK_FREQUENCY:\t%u MHz\n", info.m_clockFrequency);
	printf("  CL_DEVICE_ADDRESS_BITS:\t\t%u\n", info.m_addressBits);
	printf("  CL_DEVICE_MAX_MEM_ALLOC_SIZE:\t\t%u MByte\n", (unsigned int)(info.m_maxMemAllocSize/ (1024 * 1024)));
	printf("  CL_DEVICE_GLOBAL_MEM_SIZE:\t\t%u MByte\n", (unsigned int)(info.m_globalMemSize/ (1024 * 1024)));
	printf("  CL_DEVICE_ERROR_CORRECTION_SUPPORT:\t%s\n", info.m_errorCorrectionSupport== CL_TRUE ? "yes" : "no");
	printf("  CL_DEVICE_LOCAL_MEM_TYPE:\t\t%s\n", info.m_localMemType == 1 ? "local" : "global");
	printf("  CL_DEVICE_LOCAL_MEM_SIZE:\t\t%u KByte\n", (unsigned int)(info.m_localMemSize / 1024));
	printf("  CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE:\t%u KByte\n", (unsigned int)(info.m_constantBufferSize / 1024));
	if( info.m_queueProperties  & CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE )
		printf("  CL_DEVICE_QUEUE_PROPERTIES:\t\t%s\n", "CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE");    
	if( info.m_queueProperties & CL_QUEUE_PROFILING_ENABLE )
		printf("  CL_DEVICE_QUEUE_PROPERTIES:\t\t%s\n", "CL_QUEUE_PROFILING_ENABLE");

	printf("  CL_DEVICE_IMAGE_SUPPORT:\t\t%u\n", info.m_imageSupport);

	printf("  CL_DEVICE_MAX_READ_IMAGE_ARGS:\t%u\n", info.m_maxReadImageArgs);
	printf("  CL_DEVICE_MAX_WRITE_IMAGE_ARGS:\t%u\n", info.m_maxWriteImageArgs);
	printf("\n  CL_DEVICE_IMAGE <dim>"); 
	printf("\t\t\t2D_MAX_WIDTH\t %zu\n", info.m_image2dMaxWidth);
	printf("\t\t\t\t\t2D_MAX_HEIGHT\t %zu\n", info.m_image2dMaxHeight);
	printf("\t\t\t\t\t3D_MAX_WIDTH\t %zu\n", info.m_image3dMaxWidth);
	printf("\t\t\t\t\t3D_MAX_HEIGHT\t %zu\n", info.m_image3dMaxHeight);
	printf("\t\t\t\t\t3D_MAX_DEPTH\t %zu\n", info.m_image3dMaxDepth);
	if (info.m_deviceExtensions != 0) 
		printf("\n  CL_DEVICE_EXTENSIONS:%s\n",info.m_deviceExtensions);
	else 
		printf("  CL_DEVICE_EXTENSIONS: None\n");
	printf("  CL_DEVICE_PREFERRED_VECTOR_WIDTH_<t>\t"); 
	printf("CHAR %u, SHORT %u, INT %u,LONG %u, FLOAT %u, DOUBLE %u\n\n\n", 
		info.m_vecWidthChar, info.m_vecWidthShort, info.m_vecWidthInt, info.m_vecWidthLong,info.m_vecWidthFloat, info.m_vecWidthDouble); 


}

void btOpenCLUtils::getDeviceInfo(cl_device_id device, btOpenCLDeviceInfo& info)
{

	// CL_DEVICE_NAME
	clGetDeviceInfo(device, CL_DEVICE_NAME, BT_MAX_STRING_LENGTH, &info.m_deviceName, NULL);

	// CL_DEVICE_VENDOR
	clGetDeviceInfo(device, CL_DEVICE_VENDOR, BT_MAX_STRING_LENGTH, &info.m_deviceVendor, NULL);

	// CL_DRIVER_VERSION
	clGetDeviceInfo(device, CL_DRIVER_VERSION, BT_MAX_STRING_LENGTH, &info.m_driverVersion, NULL);

	// CL_DEVICE_INFO
	clGetDeviceInfo(device, CL_DEVICE_TYPE, sizeof(cl_device_type), &info.m_deviceType, NULL);

	// CL_DEVICE_MAX_COMPUTE_UNITS
	clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(info.m_computeUnits), &info.m_computeUnits, NULL);

	// CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS
	clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(info.m_workitemDims), &info.m_workitemDims, NULL);

	// CL_DEVICE_MAX_WORK_ITEM_SIZES
	clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(info.m_workItemSize), &info.m_workItemSize, NULL);

	// CL_DEVICE_MAX_WORK_GROUP_SIZE
	clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(info.m_workgroupSize), &info.m_workgroupSize, NULL);

	// CL_DEVICE_MAX_CLOCK_FREQUENCY
	clGetDeviceInfo(device, CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(info.m_clockFrequency), &info.m_clockFrequency, NULL);

	// CL_DEVICE_ADDRESS_BITS
	clGetDeviceInfo(device, CL_DEVICE_ADDRESS_BITS, sizeof(info.m_addressBits), &info.m_addressBits, NULL);

	// CL_DEVICE_MAX_MEM_ALLOC_SIZE
	clGetDeviceInfo(device, CL_DEVICE_MAX_MEM_ALLOC_SIZE, sizeof(info.m_maxMemAllocSize), &info.m_maxMemAllocSize, NULL);

	// CL_DEVICE_GLOBAL_MEM_SIZE
	clGetDeviceInfo(device, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(info.m_globalMemSize), &info.m_globalMemSize, NULL);

	// CL_DEVICE_ERROR_CORRECTION_SUPPORT
	clGetDeviceInfo(device, CL_DEVICE_ERROR_CORRECTION_SUPPORT, sizeof(info.m_errorCorrectionSupport), &info.m_errorCorrectionSupport, NULL);

	// CL_DEVICE_LOCAL_MEM_TYPE
	clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_TYPE, sizeof(info.m_localMemType), &info.m_localMemType, NULL);

	// CL_DEVICE_LOCAL_MEM_SIZE
	clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(info.m_localMemSize), &info.m_localMemSize, NULL);

	// CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE
	clGetDeviceInfo(device, CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE, sizeof(info.m_constantBufferSize), &info.m_constantBufferSize, NULL);

	// CL_DEVICE_QUEUE_PROPERTIES
	clGetDeviceInfo(device, CL_DEVICE_QUEUE_PROPERTIES, sizeof(info.m_queueProperties), &info.m_queueProperties, NULL);

	// CL_DEVICE_IMAGE_SUPPORT
	clGetDeviceInfo(device, CL_DEVICE_IMAGE_SUPPORT, sizeof(info.m_imageSupport), &info.m_imageSupport, NULL);

	// CL_DEVICE_MAX_READ_IMAGE_ARGS
	clGetDeviceInfo(device, CL_DEVICE_MAX_READ_IMAGE_ARGS, sizeof(info.m_maxReadImageArgs), &info.m_maxReadImageArgs, NULL);

	// CL_DEVICE_MAX_WRITE_IMAGE_ARGS
	clGetDeviceInfo(device, CL_DEVICE_MAX_WRITE_IMAGE_ARGS, sizeof(info.m_maxWriteImageArgs), &info.m_maxWriteImageArgs, NULL);

	// CL_DEVICE_IMAGE2D_MAX_WIDTH, CL_DEVICE_IMAGE2D_MAX_HEIGHT, CL_DEVICE_IMAGE3D_MAX_WIDTH, CL_DEVICE_IMAGE3D_MAX_HEIGHT, CL_DEVICE_IMAGE3D_MAX_DEPTH
	clGetDeviceInfo(device, CL_DEVICE_IMAGE2D_MAX_WIDTH, sizeof(size_t), &info.m_image2dMaxWidth, NULL);
	clGetDeviceInfo(device, CL_DEVICE_IMAGE2D_MAX_HEIGHT, sizeof(size_t), &info.m_image2dMaxHeight, NULL);
	clGetDeviceInfo(device, CL_DEVICE_IMAGE3D_MAX_WIDTH, sizeof(size_t), &info.m_image3dMaxWidth, NULL);
	clGetDeviceInfo(device, CL_DEVICE_IMAGE3D_MAX_HEIGHT, sizeof(size_t), &info.m_image3dMaxHeight, NULL);
	clGetDeviceInfo(device, CL_DEVICE_IMAGE3D_MAX_DEPTH, sizeof(size_t), &info.m_image3dMaxDepth, NULL);

	// CL_DEVICE_EXTENSIONS: get device extensions, and if any then parse & log the string onto separate lines
	clGetDeviceInfo(device, CL_DEVICE_EXTENSIONS, BT_MAX_STRING_LENGTH, &info.m_deviceExtensions, NULL);

	// CL_DEVICE_PREFERRED_VECTOR_WIDTH_<type>
	clGetDeviceInfo(device, CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR, sizeof(cl_uint), &info.m_vecWidthChar, NULL);
	clGetDeviceInfo(device, CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT, sizeof(cl_uint), &info.m_vecWidthShort, NULL);
	clGetDeviceInfo(device, CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT, sizeof(cl_uint), &info.m_vecWidthInt, NULL);
	clGetDeviceInfo(device, CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG, sizeof(cl_uint), &info.m_vecWidthLong, NULL);
	clGetDeviceInfo(device, CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT, sizeof(cl_uint), &info.m_vecWidthFloat, NULL);
	clGetDeviceInfo(device, CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE, sizeof(cl_uint), &info.m_vecWidthDouble, NULL);
}

static char* strip1(char* name, const char* pattern,int* numOccurences=0)
{
	  size_t const patlen = strlen(pattern);
	  char * oriptr;
	  char * patloc;
		// find how many times the pattern occurs in the original string
	  for (oriptr = name; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
	  {
		  if (numOccurences)
			  (*numOccurences)++;
	  }
	  return oriptr;
}
static const char* strip2(const char* name, const char* pattern,int* numOccurences=0)
{
	  size_t const patlen = strlen(pattern);
	  const char * oriptr;
	  const char * patloc;
		// find how many times the pattern occurs in the original string
	  for (oriptr = name; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
	  {
		  if (numOccurences)
			  (*numOccurences)++;
	  }
	  return oriptr;
}

cl_program btOpenCLUtils::compileCLProgramFromString(cl_context clContext, cl_device_id device, const char* kernelSource, cl_int* pErrNum, const char* additionalMacros)
{

	cl_int localErrNum;
	size_t program_length = strlen(kernelSource);

	cl_program m_cpProgram = clCreateProgramWithSource(clContext, 1, (const char**)&kernelSource, &program_length, &localErrNum);
	if (localErrNum!= CL_SUCCESS)
	{
		if (pErrNum)
			*pErrNum = localErrNum;
		return 0;
	}

	// Build the program with 'mad' Optimization option


#ifdef MAC
	char* flags = "-cl-mad-enable -DMAC -DGUID_ARG";
#else
	//const char* flags = "-DGUID_ARG= -fno-alias";
	const char* flags = "-DGUID_ARG= ";
#endif

	char* compileFlags = new char[strlen(additionalMacros) + strlen(flags) + 5];
	sprintf(compileFlags, "%s %s", flags, additionalMacros);
	localErrNum = clBuildProgram(m_cpProgram, 1, &device, compileFlags, NULL, NULL);
	if (localErrNum!= CL_SUCCESS)
	{
		char *build_log;
		size_t ret_val_size;
		clGetProgramBuildInfo(m_cpProgram, device, CL_PROGRAM_BUILD_LOG, 0, NULL, &ret_val_size);
		build_log = new char[ret_val_size+1];
		clGetProgramBuildInfo(m_cpProgram, device, CL_PROGRAM_BUILD_LOG, ret_val_size, build_log, NULL);

		// to be carefully, terminate with \0
		// there's no information in the reference whether the string is 0 terminated or not
		build_log[ret_val_size] = '\0';


		printf("Error in clBuildProgram, Line %u in file %s, Log: \n%s\n !!!\n\n", __LINE__, __FILE__, build_log);
		delete[] build_log;
		if (pErrNum)
			*pErrNum = localErrNum;
		return 0;
	}
	delete[] compileFlags;
	return m_cpProgram;
}

cl_program btOpenCLUtils::compileCLProgramFromFile(cl_context clContext, cl_device_id device, cl_int* pErrNum, const char* additionalMacros , const char* clFileNameForCaching)
{

	cl_program m_cpProgram=0;
	cl_int status;
	char binaryFileName[522];

	if (clFileNameForCaching)
	{
#ifdef _WIN32	
		char deviceName[256];
		char driverVersion[256];
		clGetDeviceInfo(device, CL_DEVICE_NAME, 256, &deviceName, NULL);
		clGetDeviceInfo(device, CL_DRIVER_VERSION, 256, &driverVersion, NULL);

		
		const char* strippedName = strip2(clFileNameForCaching,"\\");
		strippedName = strip2(strippedName,"/");
#ifdef BT_USE_CACHE_DIR
		sprintf_s(binaryFileName,"cache/%s.%s.%s.bin",strippedName, deviceName,driverVersion );
#else
		sprintf_s(binaryFileName,"%s.%s.%s.bin",strippedName, deviceName,driverVersion );
#endif
		
	//printf("searching for %s\n", binaryFileName);

		bool fileUpToDate = false;
		bool binaryFileValid=false;

		FILETIME modtimeBinary; 

#ifdef BT_USE_CACHE_DIR
		CreateDirectory("cache",0);
#endif //BT_USE_CACHE_DIR
		{
			
			HANDLE binaryFileHandle = CreateFile(binaryFileName,GENERIC_READ,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
			if (binaryFileHandle ==INVALID_HANDLE_VALUE)
			{
				DWORD errorCode;
				errorCode = GetLastError();
				switch (errorCode)
				{
				case ERROR_FILE_NOT_FOUND:
					{
						printf("\nCached file not found %s\n", binaryFileName);
						break;
					}
				case ERROR_PATH_NOT_FOUND:
					{
						printf("\nCached file path not found %s\n", binaryFileName);
						break;
					}
				default:
					{
						printf("\nFailed reading cached file with errorCode = %d\n", errorCode);
					}
				}
			} else
			{
				if (GetFileTime(binaryFileHandle, NULL, NULL, &modtimeBinary)==0)
				{
					DWORD errorCode;
					errorCode = GetLastError();
					printf("\nGetFileTime errorCode = %d\n", errorCode);
				} else
				{
					binaryFileValid = true;
				}
				CloseHandle(binaryFileHandle);
			}

			if (binaryFileValid)
			{
				HANDLE srcFileHandle = CreateFile(clFileNameForCaching,GENERIC_READ,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
				if (srcFileHandle!=INVALID_HANDLE_VALUE)
				{
					FILETIME modtimeSrc; 
					if (GetFileTime(srcFileHandle, NULL, NULL, &modtimeSrc)==0)
					{
						DWORD errorCode;
						errorCode = GetLastError();
						printf("\nGetFileTime errorCode = %d\n", errorCode);
					}
					if (  ( modtimeSrc.dwHighDateTime < modtimeBinary.dwHighDateTime)
						||(( modtimeSrc.dwHighDateTime == modtimeBinary.dwHighDateTime)&&(modtimeSrc.dwLowDateTime <= modtimeBinary.dwLowDateTime)))
					{
						fileUpToDate=true;
					} else
					{
						printf("\nCached binary file out-of-date (%s)\n",binaryFileName);
					}
					CloseHandle(srcFileHandle);
				} 
				else
				{
					DWORD errorCode;
					errorCode = GetLastError();
					switch (errorCode)
					{
					case ERROR_FILE_NOT_FOUND:
						{
							printf("\nSrc file not found %s\n", clFileNameForCaching);
							break;
						}
					case ERROR_PATH_NOT_FOUND:
						{
							printf("\nSrc path not found %s\n", clFileNameForCaching);
							break;
						}
					default:
						{
							printf("\nnSrc file reading errorCode = %d\n", errorCode);
						}
					}

					//we should make sure the src file exists so we can verify the timestamp with binary
					fileUpToDate = false;
				}
			}
			

		}

		if( fileUpToDate)
		{
			FILE* file = fopen(binaryFileName, "rb");
			if (file)
			{
				fseek( file, 0L, SEEK_END );
				size_t binarySize = ftell( file );
				rewind( file );
				char* binary = new char[binarySize];
				fread( binary, sizeof(char), binarySize, file );
				fclose( file );

				m_cpProgram = clCreateProgramWithBinary( clContext, 1,&device, &binarySize, (const unsigned char**)&binary, 0, &status );
				btAssert( status == CL_SUCCESS );
				status = clBuildProgram( m_cpProgram, 1, &device, additionalMacros, 0, 0 );
				btAssert( status == CL_SUCCESS );

				if( status != CL_SUCCESS )
				{
					char *build_log;
					size_t ret_val_size;
					clGetProgramBuildInfo(m_cpProgram, device, CL_PROGRAM_BUILD_LOG, 0, NULL, &ret_val_size);
					build_log = new char[ret_val_size+1];
					clGetProgramBuildInfo(m_cpProgram, device, CL_PROGRAM_BUILD_LOG, ret_val_size, build_log, NULL);
					build_log[ret_val_size] = '\0';
					printf("%s\n", build_log);
					delete build_log;
					btAssert(0);
					m_cpProgram = 0;
				}
				delete[] binary;
			}
		}
#endif //_WIN32
		
	}
	
	if (!m_cpProgram)
	{
		
		FILE* file = fopen(clFileNameForCaching, "r");
		if (file)
		{
			fseek( file, 0L, SEEK_END );
			size_t fileSize= ftell( file );
			rewind( file );
			char* kernelSource2 = new char[fileSize+1];
			fread( kernelSource2, sizeof(char), fileSize, file );
			fclose( file );
			kernelSource2[fileSize]=0;
			int numOccurences = 0;
			///patch/remove the MSTRINGIFY( and );
			 char* kernelSource = strip1(kernelSource2,"MSTRINGIFY(",&numOccurences);
			 int newlen = strlen(kernelSource);
			if (numOccurences)
			{
				int i=newlen-1;

				for (;i>=0;i--)
				{
					if (kernelSource[i] == ';')
					{
						kernelSource[i] = 0;//' ';
						break;
					}
				}
				for (;i>=0;i--)
				{
					if (kernelSource[i] == ')')
					{
						kernelSource[i] = 0;//' ';
						break;
					}
				}
			}
		
			m_cpProgram = compileCLProgramFromString(clContext,device,kernelSource,pErrNum,additionalMacros);
			
			if( clFileNameForCaching )
			{	//	write to binary

				cl_uint numAssociatedDevices;
				status = clGetProgramInfo( m_cpProgram, CL_PROGRAM_NUM_DEVICES, sizeof(cl_uint), &numAssociatedDevices, 0 );
				btAssert( status == CL_SUCCESS );
				if (numAssociatedDevices==1)
				{

					size_t binarySize;
					status = clGetProgramInfo( m_cpProgram, CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &binarySize, 0 );
					btAssert( status == CL_SUCCESS );

					char* binary = new char[binarySize];

					status = clGetProgramInfo( m_cpProgram, CL_PROGRAM_BINARIES, sizeof(char*), &binary, 0 );
					btAssert( status == CL_SUCCESS );

					{
						FILE* file = fopen(binaryFileName, "wb");
						if (file)
						{
							fwrite( binary, sizeof(char), binarySize, file );
							fclose( file );
						} else
						{
							printf("cannot write file %s\n", binaryFileName);
						}
					}

					delete [] binary;
				}
			}
		}
	}

	return m_cpProgram;
}


cl_kernel btOpenCLUtils::compileCLKernelFromString(cl_context clContext, cl_device_id device, const char* kernelSource, const char* kernelName, cl_int* pErrNum, cl_program prog, const char* additionalMacros )
{
	printf("compiling kernel %s ",kernelName);
	cl_kernel kernel;
	cl_int localErrNum;
	//size_t program_length = strlen(kernelSource);


	cl_program m_cpProgram = prog;
	if (!m_cpProgram)
	{
		m_cpProgram = compileCLProgramFromString(clContext,device,kernelSource,pErrNum, additionalMacros);
	}


	// Create the kernel
	kernel = clCreateKernel(m_cpProgram, kernelName, &localErrNum);
	if (localErrNum != CL_SUCCESS)
	{
		printf("Error in clCreateKernel, Line %u in file %s, cannot find kernel function %s !!!\n\n", __LINE__, __FILE__, kernelName);
		if (pErrNum)
			*pErrNum = localErrNum;
		return 0;
	}

	if (!prog && m_cpProgram)
	{
		clReleaseProgram(m_cpProgram);
	}
	printf("ready. \n");


	if (pErrNum)
			*pErrNum = CL_SUCCESS;
	return kernel;

}
