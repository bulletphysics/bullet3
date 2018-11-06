/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
All rights reserved.


Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifdef OPENCL_FOUND

#include "oclHelper.h"
#include <assert.h>
#include <sstream>

bool OCLHelper::InitPlatform(const unsigned int platformIndex)
{
	cl_uint numPlatforms;
	m_lastError = clGetPlatformIDs(1, NULL, &numPlatforms);
	if (m_lastError != CL_SUCCESS || platformIndex >= numPlatforms)
		return false;

	cl_platform_id* platforms = new cl_platform_id[numPlatforms];
	m_lastError = clGetPlatformIDs(numPlatforms, platforms, NULL);
	if (m_lastError != CL_SUCCESS)
	{
		delete[] platforms;
		return false;
	}

	m_platform = platforms[platformIndex];
	delete[] platforms;
	return true;
}
bool OCLHelper::GetPlatformsInfo(std::vector<std::string>& info, const std::string& indentation)
{
	const char* platformInfoParameters[] = {"CL_PLATFORM_NAME",
											"CL_PLATFORM_VENDOR",
											"CL_PLATFORM_VERSION",
											"CL_PLATFORM_PROFILE",
											"CL_PLATFORM_EXTENSIONS"};

	cl_uint numPlatforms;
	m_lastError = clGetPlatformIDs(1, NULL, &numPlatforms);
	if (m_lastError != CL_SUCCESS)
		return false;

	cl_platform_id* platforms = new cl_platform_id[numPlatforms];
	m_lastError = clGetPlatformIDs(numPlatforms, platforms, NULL);
	if (m_lastError != CL_SUCCESS)
	{
		delete[] platforms;
		return false;
	}

	size_t bufferSize = 4096;
	char* buffer = new char[bufferSize];
	size_t size;
	info.resize(numPlatforms);
	for (cl_uint i = 0; i < numPlatforms; ++i)
	{
		for (int j = CL_PLATFORM_PROFILE; j <= CL_PLATFORM_EXTENSIONS; ++j)
		{
			info[i] += indentation + platformInfoParameters[j - CL_PLATFORM_PROFILE] + std::string(": ");
			m_lastError = clGetPlatformInfo(platforms[i], j, 0, NULL, &size);
			if (m_lastError != CL_SUCCESS)
			{
				delete[] buffer;
				delete[] platforms;
				return false;
			}
			if (bufferSize < size)
			{
				delete[] buffer;
				bufferSize = size;
				buffer = new char[bufferSize];
			}
			m_lastError = clGetPlatformInfo(platforms[i], j, size, buffer, NULL);
			if (m_lastError != CL_SUCCESS)
			{
				delete[] buffer;
				delete[] platforms;
				return false;
			}
			info[i] += buffer + std::string("\n");
		}
	}
	delete[] platforms;
	delete[] buffer;
	return true;
}
bool OCLHelper::InitDevice(const unsigned int deviceIndex)
{
	cl_uint numDevices;
	m_lastError = clGetDeviceIDs(m_platform, CL_DEVICE_TYPE_ALL, 0, NULL, &numDevices);
	if (m_lastError != CL_SUCCESS || deviceIndex >= numDevices)
		return false;

	cl_device_id* devices = new cl_device_id[numDevices];
	m_lastError = clGetDeviceIDs(m_platform, CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);
	if (m_lastError != CL_SUCCESS)
	{
		delete[] devices;
		return false;
	}
	m_device = devices[deviceIndex];
	delete[] devices;
	return true;
}
bool OCLHelper::GetDevicesInfo(std::vector<std::string>& info, const std::string& indentation)
{
	enum
	{
		DATA_TYPE_CL_UINT,
		DATA_TYPE_CL_BOOL,
		DATA_TYPE_STRING,
		DATA_TYPE_CL_ULONG,
		DATA_TYPE_CL_DEVICE_FP_CONFIG,
		DATA_TYPE_CL_DEVICE_EXEC_CAPABILITIES,
		DATA_TYPE_CL_DEVICE_MEM_CACHE_TYPE,
		DATA_TYPE_CL_DEVICE_MEM_LOCAL_TYPE,
		DATA_TYPE_CL_DEVICE_CMD_QUEUE_PROP,
		DATA_TYPE_CL_DEVICE_TYPE,
		DATA_TYPE_SIZE_T,
		DATA_TYPE_SIZE_T_3,
	};
	typedef struct
	{
		cl_device_info id;
		const char* name;
		int type;
	} DeviceInfoParam;
	const int numDeviceInfoParameters = 49;
	const DeviceInfoParam deviceInfoParameters[numDeviceInfoParameters] = {
		{CL_DEVICE_NAME, "CL_DEVICE_NAME", DATA_TYPE_STRING},
		{CL_DEVICE_PROFILE, "CL_DEVICE_PROFILE", DATA_TYPE_STRING},
		{CL_DEVICE_VENDOR, "CL_DEVICE_VENDOR", DATA_TYPE_STRING},
		{CL_DEVICE_VERSION, "CL_DEVICE_VERSION", DATA_TYPE_STRING},
		{CL_DRIVER_VERSION, "CL_DRIVER_VERSION", DATA_TYPE_STRING},
		{CL_DEVICE_EXTENSIONS, "CL_DEVICE_EXTENSIONS", DATA_TYPE_STRING},
		{CL_DEVICE_VERSION, "CL_DEVICE_VERSION", DATA_TYPE_STRING},
		{CL_DEVICE_ADDRESS_BITS, "CL_DEVICE_ADDRESS_BITS", DATA_TYPE_CL_UINT},
		{CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE, "CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MAX_CLOCK_FREQUENCY, "CL_DEVICE_MAX_CLOCK_FREQUENCY", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MAX_COMPUTE_UNITS, "CL_DEVICE_MAX_COMPUTE_UNITS", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MAX_CONSTANT_ARGS, "CL_DEVICE_MAX_CONSTANT_ARGS", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MAX_READ_IMAGE_ARGS, "CL_DEVICE_MAX_READ_IMAGE_ARGS", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MAX_SAMPLERS, "CL_DEVICE_MAX_SAMPLERS", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, "CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MAX_WRITE_IMAGE_ARGS, "CL_DEVICE_MAX_WRITE_IMAGE_ARGS", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MEM_BASE_ADDR_ALIGN, "CL_DEVICE_MEM_BASE_ADDR_ALIGN", DATA_TYPE_CL_UINT},
		{CL_DEVICE_MIN_DATA_TYPE_ALIGN_SIZE, "CL_DEVICE_MIN_DATA_TYPE_ALIGN_SIZE", DATA_TYPE_CL_UINT},
		{CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR, "CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR", DATA_TYPE_CL_UINT},
		{CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT, "CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT", DATA_TYPE_CL_UINT},
		{CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT, "CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT", DATA_TYPE_CL_UINT},
		{CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG, "CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG", DATA_TYPE_CL_UINT},
		{CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT, "CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT", DATA_TYPE_CL_UINT},
		{CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE, "CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE", DATA_TYPE_CL_UINT},
		{CL_DEVICE_VENDOR_ID, "CL_DEVICE_VENDOR_ID", DATA_TYPE_CL_UINT},
		{CL_DEVICE_AVAILABLE, "CL_DEVICE_AVAILABLE", DATA_TYPE_CL_BOOL},
		{CL_DEVICE_COMPILER_AVAILABLE, "CL_DEVICE_COMPILER_AVAILABLE", DATA_TYPE_CL_BOOL},
		{CL_DEVICE_ENDIAN_LITTLE, "CL_DEVICE_ENDIAN_LITTLE", DATA_TYPE_CL_BOOL},
		{CL_DEVICE_ERROR_CORRECTION_SUPPORT, "CL_DEVICE_ERROR_CORRECTION_SUPPORT", DATA_TYPE_CL_BOOL},
		{CL_DEVICE_IMAGE_SUPPORT, "CL_DEVICE_IMAGE_SUPPORT", DATA_TYPE_CL_BOOL},
		{CL_DEVICE_EXECUTION_CAPABILITIES, "CL_DEVICE_EXECUTION_CAPABILITIES", DATA_TYPE_CL_DEVICE_EXEC_CAPABILITIES},
		{CL_DEVICE_GLOBAL_MEM_CACHE_SIZE, "CL_DEVICE_GLOBAL_MEM_CACHE_SIZE", DATA_TYPE_CL_ULONG},
		{CL_DEVICE_GLOBAL_MEM_SIZE, "CL_DEVICE_GLOBAL_MEM_SIZE", DATA_TYPE_CL_ULONG},
		{CL_DEVICE_LOCAL_MEM_SIZE, "CL_DEVICE_LOCAL_MEM_SIZE", DATA_TYPE_CL_ULONG},
		{CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE, "CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE", DATA_TYPE_CL_ULONG},
		{CL_DEVICE_MAX_MEM_ALLOC_SIZE, "CL_DEVICE_MAX_MEM_ALLOC_SIZE", DATA_TYPE_CL_ULONG},
		{CL_DEVICE_GLOBAL_MEM_CACHE_TYPE, "CL_DEVICE_GLOBAL_MEM_CACHE_TYPE", DATA_TYPE_CL_DEVICE_MEM_CACHE_TYPE},
		{CL_DEVICE_IMAGE2D_MAX_HEIGHT, "CL_DEVICE_IMAGE2D_MAX_HEIGHT", DATA_TYPE_SIZE_T},
		{CL_DEVICE_IMAGE2D_MAX_WIDTH, "CL_DEVICE_IMAGE2D_MAX_WIDTH", DATA_TYPE_SIZE_T},
		{CL_DEVICE_IMAGE3D_MAX_DEPTH, "CL_DEVICE_IMAGE3D_MAX_DEPTH", DATA_TYPE_SIZE_T},
		{CL_DEVICE_IMAGE3D_MAX_HEIGHT, "CL_DEVICE_IMAGE3D_MAX_HEIGHT", DATA_TYPE_SIZE_T},
		{CL_DEVICE_IMAGE3D_MAX_WIDTH, "CL_DEVICE_IMAGE3D_MAX_WIDTH", DATA_TYPE_SIZE_T},
		{CL_DEVICE_MAX_PARAMETER_SIZE, "CL_DEVICE_MAX_PARAMETER_SIZE", DATA_TYPE_SIZE_T},
		{CL_DEVICE_MAX_WORK_GROUP_SIZE, "CL_DEVICE_MAX_WORK_GROUP_SIZE", DATA_TYPE_SIZE_T},
		{CL_DEVICE_PROFILING_TIMER_RESOLUTION, "CL_DEVICE_PROFILING_TIMER_RESOLUTION", DATA_TYPE_SIZE_T},
		{CL_DEVICE_QUEUE_PROPERTIES, "CL_DEVICE_QUEUE_PROPERTIES", DATA_TYPE_CL_DEVICE_CMD_QUEUE_PROP},
		{CL_DEVICE_TYPE, "CL_DEVICE_TYPE", DATA_TYPE_CL_DEVICE_TYPE},
		{CL_DEVICE_LOCAL_MEM_TYPE, "CL_DEVICE_LOCAL_MEM_TYPE", DATA_TYPE_CL_DEVICE_MEM_LOCAL_TYPE},
		{CL_DEVICE_MAX_WORK_ITEM_SIZES, "CL_DEVICE_MAX_WORK_ITEM_SIZES", DATA_TYPE_SIZE_T_3}
		//        { CL_DEVICE_DOUBLE_FP_CONFIG, "CL_DEVICE_DOUBLE_FP_CONFIG", DATA_TYPE_CL_DEVICE_FP_CONFIG },
		//
	};
	cl_uint numDevices;
	m_lastError = clGetDeviceIDs(m_platform, CL_DEVICE_TYPE_ALL, 0, NULL, &numDevices);
	if (m_lastError != CL_SUCCESS)
		return false;

	cl_device_id* devices = new cl_device_id[numDevices];
	m_lastError = clGetDeviceIDs(m_platform, CL_DEVICE_TYPE_ALL, numDevices, devices, NULL);
	if (m_lastError != CL_SUCCESS)
	{
		delete[] devices;
		return false;
	}
	size_t bufferSize = 4096;
	char* buffer = new char[bufferSize];
	size_t size;
	info.resize(numDevices);

	for (cl_uint i = 0; i < numDevices; ++i)
	{
		for (int j = 0; j < numDeviceInfoParameters; ++j)
		{
			const DeviceInfoParam& infoParam = deviceInfoParameters[j];
			info[i] += indentation + infoParam.name + std::string(": ");

			if (infoParam.type == DATA_TYPE_STRING)
			{
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, 0, NULL, &size);
				if (m_lastError == CL_SUCCESS)
				{
					if (bufferSize < size)
					{
						delete[] buffer;
						bufferSize = size;
						buffer = new char[bufferSize];
					}
					m_lastError = clGetDeviceInfo(devices[i], infoParam.id, size, buffer, NULL);
					if (m_lastError != CL_SUCCESS)
					{
						delete[] devices;
						delete[] buffer;
						return false;
					}
					info[i] += buffer + std::string("\n");
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_UINT)
			{
				cl_uint value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_uint), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_BOOL)
			{
				cl_bool value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_bool), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_ULONG)
			{
				cl_ulong value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_ulong), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_DEVICE_FP_CONFIG)
			{
				cl_device_fp_config value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_device_fp_config), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_DEVICE_EXEC_CAPABILITIES)
			{
				cl_device_exec_capabilities value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_device_exec_capabilities), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_DEVICE_MEM_CACHE_TYPE)
			{
				cl_device_mem_cache_type value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_device_mem_cache_type), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_DEVICE_MEM_LOCAL_TYPE)
			{
				cl_device_local_mem_type value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_device_local_mem_type), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_DEVICE_CMD_QUEUE_PROP)
			{
				cl_command_queue_properties value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_command_queue_properties), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_CL_DEVICE_TYPE)
			{
				cl_device_type value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(cl_device_type), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_SIZE_T)
			{
				size_t value;
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, sizeof(size_t), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << value;
					info[i] += svalue.str() + "\n";
				}
			}
			else if (infoParam.type == DATA_TYPE_SIZE_T_3)
			{
				size_t value[3];
				m_lastError = clGetDeviceInfo(devices[i], infoParam.id, 3 * sizeof(size_t), &value, &size);
				if (m_lastError == CL_SUCCESS)
				{
					std::ostringstream svalue;
					svalue << "(" << value[0] << ", " << value[1] << ", " << value[2] << ")";
					info[i] += svalue.str() + "\n";
				}
			}
			else
			{
				assert(0);
			}
		}
	}
	delete[] devices;
	delete[] buffer;
	return true;
}
#endif  // OPENCL_FOUND
