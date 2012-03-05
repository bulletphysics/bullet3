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


#ifndef ADL_KERNEL_H
#define ADL_KERNEL_H

#include <map>
#include <string>
#include <fstream>

namespace adl
{

//==========================
//	Kernel
//==========================
struct Kernel
{
	DeviceType m_type;
	void* m_kernel;
};

//==========================
//	KernelManager
//==========================
class KernelManager
{
	public:
		typedef std::map<std::string, Kernel*> KMap;

		__inline
		~KernelManager();

		__inline
//		static
		Kernel* query(const Device* dd, const char* fileName, const char* funcName, const char* option = NULL, const char* src = NULL,
			bool cacheKernel = true);

	public:
		KMap m_map;
};

//==========================
//	Launcher
//==========================
class Launcher
{
	public:
		struct BufferInfo
		{
			BufferInfo(){}
			template<typename T>
			BufferInfo(Buffer<T>* buff, bool isReadOnly = false): m_buffer(buff), m_isReadOnly(isReadOnly){}

			void* m_buffer;
			bool m_isReadOnly;
		};

		__inline
		Launcher(const Device* dd, char* fileName, char* funcName, char* option = NULL);
		__inline
		Launcher(const Device* dd, Kernel* kernel);
		__inline
		void setBuffers( BufferInfo* buffInfo, int n );
		template<typename T>
		__inline
		void setConst( Buffer<T>& constBuff, const T& consts );
		__inline
		void launch1D( int numThreads, int localSize = 64 );
		__inline
		void launch2D( int numThreadsX, int numThreadsY, int localSizeX = 8, int localSizeY = 8 );

	public:
		enum
		{
			CONST_BUFFER_SIZE = 512,
		};

		const Device* m_deviceData;
		Kernel* m_kernel;
		int m_idx;
		int m_idxRw;
};

template<DeviceType TYPE>
class KernelBuilder
{
	public:

		__inline
		KernelBuilder(): m_ptr(0){}
		
		__inline
		void setFromFile( const Device* deviceData, const char* fileName, const char* option = NULL, bool addExtension = false,
			bool cacheKernel = true);

		__inline
		void setFromSrc( const Device* deviceData, const char* src, const char* option = NULL );

		__inline
		void setFromSrcCached( const Device* deviceData, const char* src, const char* fileName, const char* option );


		__inline
		void createKernel( const char* funcName, Kernel& kernelOut );

		__inline
		~KernelBuilder();
		//	todo. implemement in kernel destructor?
		__inline
		static void deleteKernel( Kernel& kernel );

	private:
		enum
		{
			MAX_PATH_LENGTH = 260,
		};
		const Device* m_deviceData;
#ifdef UNICODE
		wchar_t m_path[MAX_PATH_LENGTH];
#else
		char m_path[MAX_PATH_LENGTH];
#endif
		void* m_ptr;
};

};

#endif //ADL_KERNEL_H
