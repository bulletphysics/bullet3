/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada


#ifndef ADL_H
#define ADL_H

#pragma warning( disable : 4996 )
#include <Adl/AdlConfig.h>
#include <Adl/AdlError.h>
#include <algorithm>

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

namespace adl
{

enum DeviceType
{
	TYPE_CL = 0,
	TYPE_DX11 = 1,
	TYPE_HOST,
};


struct Device;

struct BufferBase
{
	enum BufferType
	{
		BUFFER,

		//	for dx
		BUFFER_CONST,
		BUFFER_STAGING,
		BUFFER_APPEND,
		BUFFER_RAW,
		BUFFER_W_COUNTER,
		BUFFER_INDEX,
		BUFFER_VERTEX,

		//	for cl
		BUFFER_ZERO_COPY,

	};
};

class DeviceUtils
{
	public:
		struct Config
		{
			enum DeviceType
			{
				DEVICE_GPU,
				DEVICE_CPU,
			};

			//	for CL
			enum DeviceVendor
			{
				VD_AMD,
				VD_INTEL,
				VD_NV,
			};

			Config() : m_type(DEVICE_GPU), m_deviceIdx(0), m_vendor(VD_AMD){}

			DeviceType m_type;
			int m_deviceIdx;
			DeviceVendor m_vendor;
		};

		__inline
		static
		int getNDevices( DeviceType type );
		__inline
		static Device* allocate( DeviceType type, Config& cfg );
		__inline
		static void deallocate( Device* deviceData );
		__inline
		static void waitForCompletion( const Device* deviceData );
};

//==========================
//	DeviceData
//==========================
struct Kernel;

struct Device
{
	typedef DeviceUtils::Config Config;

	Device( DeviceType type ) : m_type( type ), m_memoryUsage(0)
	{
	}

	virtual void* getContext() const { return 0; }
	virtual void initialize(const Config& cfg){}
	virtual void release(){}
	virtual void waitForCompletion() const {}
	virtual void getDeviceName( char nameOut[128] ) const {}
	virtual Kernel* getKernel(const char* fileName, const char* funcName, const char* option = NULL, const char* src = NULL, bool cacheKernel = true ) const { ADLASSERT(0); return 0;}
	virtual unsigned int getUsedMemory() const { return m_memoryUsage; }

	DeviceType m_type;
	unsigned int m_memoryUsage;
};

//==========================
//	Buffer
//==========================

template<typename T>
struct HostBuffer;
//	overload each deviceDatas
template<typename T>
struct Buffer : public BufferBase
{
	__inline
	Buffer();
	__inline
	Buffer(const Device* device, int nElems, BufferType type = BUFFER );
	__inline
	virtual ~Buffer();
	
	__inline
	void setRawPtr( const Device* device, T* ptr, int size, BufferType type = BUFFER );
	__inline
	void allocate(const Device* device, int nElems, BufferType type = BUFFER );
	__inline
	void write(T* hostSrcPtr, int nElems, int dstOffsetNElems = 0);
	__inline
	void read(T* hostDstPtr, int nElems, int srcOffsetNElems = 0) const;
	__inline
	void write(Buffer<T>& src, int nElems);
	__inline
	void read(Buffer<T>& dst, int nElems) const;
//	__inline
//	Buffer<T>& operator = (const Buffer<T>& buffer);
	__inline
	int getSize() const { return m_size; }

	DeviceType getType() const { ADLASSERT( m_device ); return m_device->m_type; }


	const Device* m_device;
	int m_size;
	T* m_ptr;
	//	for DX11
	void* m_uav;
	void* m_srv;
	bool m_allocated;	//	todo. move this to a bit
};

class BufferUtils
{
public:
	template<DeviceType TYPE, bool COPY, typename T>
	__inline
	static
	typename Buffer<T>* map(const Device* device, const Buffer<T>* in, int copySize = -1);

	template<bool COPY, typename T>
	__inline
	static
	void unmap( Buffer<T>* native, const Buffer<T>* orig, int copySize = -1 );
};

//==========================
//	HostBuffer
//==========================
struct DeviceHost;

template<typename T>
struct HostBuffer : public Buffer<T>
{
	__inline
	HostBuffer():Buffer<T>(){}
	__inline
	HostBuffer(const Device* device, int nElems, BufferType type = BUFFER ) : Buffer<T>(device, nElems, type) {}
//	HostBuffer(const Device* deviceData, T* rawPtr, int nElems);


	__inline
	T& operator[](int idx);
	__inline
	const T& operator[](int idx) const;
	__inline
	T* begin() { return m_ptr; }

	__inline
	HostBuffer<T>& operator = (const Buffer<T>& device);
};

};

#include <Adl/AdlKernel.h>
#if defined(ADL_ENABLE_CL)
	#include <Adl/CL/AdlCL.inl>
#endif
#if defined(ADL_ENABLE_DX11)
	#include <Adl/DX11/AdlDX11.inl>
#endif

#include <Adl/Host/AdlHost.inl>
#include <Adl/AdlKernel.inl>
#include <Adl/Adl.inl>


#include <Adl/AdlStopwatch.h>

#include <Adl/Host/AdlStopwatchHost.inl>
#include <Adl/AdlStopwatch.inl>

#endif
