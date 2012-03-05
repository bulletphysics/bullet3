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





namespace adl
{

struct KernelCL : public Kernel
{
	cl_kernel& getKernel() { return (cl_kernel&)m_kernel; }
};

static const char* strip(const char* name, const char* pattern)
{
	  size_t const patlen = strlen(pattern);
  	size_t patcnt = 0;
	  const char * oriptr;
	  const char * patloc;
		// find how many times the pattern occurs in the original string
	  for (oriptr = name; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
	  {
		patcnt++;
	  }
	  return oriptr;
}

static bool isFileUpToDate(const char* binaryFileName,const char* srcFileName)

{
	bool fileUpToDate = false;

	bool binaryFileValid=false;
	FILETIME modtimeBinary; 

	int nameLength = (int)strlen(binaryFileName)+1;
#ifdef UNICODE
	WCHAR* fName = new WCHAR[nameLength];
	MultiByteToWideChar(CP_ACP,0,binaryFileName,-1, fName, nameLength);
	HANDLE binaryFileHandle = CreateFile(fName,GENERIC_READ,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
	delete [] fName;
#else
	HANDLE binaryFileHandle = CreateFile(binaryFileName,GENERIC_READ,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
#endif
	if (binaryFileHandle ==INVALID_HANDLE_VALUE)
	{
		DWORD errorCode;
		errorCode = GetLastError();
		switch (errorCode)
		{
		case ERROR_FILE_NOT_FOUND:
			{
				debugPrintf("\nCached file not found %s\n", binaryFileName);
				break;
			}
		case ERROR_PATH_NOT_FOUND:
			{
				debugPrintf("\nCached file path not found %s\n", binaryFileName);
				break;
			}
		default:
			{
				debugPrintf("\nFailed reading cached file with errorCode = %d\n", errorCode);
			}
		}
	} else
	{
		if (GetFileTime(binaryFileHandle, NULL, NULL, &modtimeBinary)==0)
		{
			DWORD errorCode;
			errorCode = GetLastError();
			debugPrintf("\nGetFileTime errorCode = %d\n", errorCode);
		} else
		{
			binaryFileValid = true;
		}
		CloseHandle(binaryFileHandle);
	}

	if (binaryFileValid)
	{
#ifdef UNICODE
		int nameLength = (int)strlen(srcFileName)+1;
		WCHAR* fName = new WCHAR[nameLength];
		MultiByteToWideChar(CP_ACP,0,srcFileName,-1, fName, nameLength);
		HANDLE srcFileHandle = CreateFile(fName,GENERIC_READ,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
		delete [] fName;
#else
		HANDLE srcFileHandle = CreateFile(srcFileName,GENERIC_READ,0,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
#endif
		if (srcFileHandle!=INVALID_HANDLE_VALUE)
		{
			FILETIME modtimeSrc; 
			if (GetFileTime(srcFileHandle, NULL, NULL, &modtimeSrc)==0)
			{
				DWORD errorCode;
				errorCode = GetLastError();
				debugPrintf("\nGetFileTime errorCode = %d\n", errorCode);
			}
			if (  ( modtimeSrc.dwHighDateTime < modtimeBinary.dwHighDateTime)
				||(( modtimeSrc.dwHighDateTime == modtimeBinary.dwHighDateTime)&&(modtimeSrc.dwLowDateTime <= modtimeBinary.dwLowDateTime)))
			{
				fileUpToDate=true;
			} else
			{
				debugPrintf("\nCached binary file found (%s), but out-of-date\n",binaryFileName);
			}
			CloseHandle(srcFileHandle);
		} 
		else
		{
#ifdef _DEBUG
			DWORD errorCode;
			errorCode = GetLastError();
			switch (errorCode)
			{
			case ERROR_FILE_NOT_FOUND:
				{
					debugPrintf("\nSrc file not found %s\n", srcFileName);
					break;
				}
			case ERROR_PATH_NOT_FOUND:
				{
					debugPrintf("\nSrc path not found %s\n", srcFileName);
					break;
				}
			default:
				{
					debugPrintf("\nnSrc file reading errorCode = %d\n", errorCode);
				}
			}
			ADLASSERT(0);
#else
			//if we cannot find the source, assume it is OK in release builds
			fileUpToDate = true;
#endif
		}
	}
			

	return fileUpToDate;
}

template<>
void KernelBuilder<TYPE_CL>::setFromFile( const Device* deviceData, const char* fileName, const char* option, bool addExtension,
	bool cacheKernel)
{
	m_deviceData = deviceData;

	char fileNameWithExtension[256];

	if( addExtension )
		sprintf_s( fileNameWithExtension, "%s.cl", fileName );
	else
		sprintf_s( fileNameWithExtension, "%s", fileName );

	class File
	{
		public:
			__inline
			bool open(const char* fileNameWithExtension)
			{
				size_t      size;
				char*       str;

				// Open file stream
				std::fstream f(fileNameWithExtension, (std::fstream::in | std::fstream::binary));

				// Check if we have opened file stream
				if (f.is_open()) {
					size_t  sizeFile;
					// Find the stream size
					f.seekg(0, std::fstream::end);
					size = sizeFile = (size_t)f.tellg();
					f.seekg(0, std::fstream::beg);

					str = new char[size + 1];
					if (!str) {
						f.close();
						return  NULL;
					}

					// Read file
					f.read(str, sizeFile);
					f.close();
					str[size] = '\0';

					m_source  = str;

					delete[] str;

					return true;
				}

				return false;
			}
			const std::string& getSource() const {return m_source;}

		private:
			std::string m_source;
	};

	cl_program& program = (cl_program&)m_ptr;
	cl_int status = 0;

	bool cacheBinary = cacheKernel;
#if defined(ADL_CL_FORCE_UNCACHE_KERNEL)
	cacheBinary = false;
#endif

	char binaryFileName[512];
	{
		char deviceName[256];
		deviceData->getDeviceName(deviceName);
		char driverVersion[256];
		const DeviceCL* dd = (const DeviceCL*) deviceData;
		clGetDeviceInfo(dd->m_deviceIdx, CL_DRIVER_VERSION, 256, &driverVersion, NULL);
		const char* strippedFileName = strip(fileName,"\\");
		strippedFileName = strip(strippedFileName,"/");

		sprintf_s(binaryFileName,"cache/%s.%s.%s.bin",strippedFileName, deviceName,driverVersion );
	}

	bool upToDate = isFileUpToDate(binaryFileName,fileNameWithExtension);

	if( cacheBinary && upToDate)
	{
		FILE* file = fopen(binaryFileName, "rb");

		if( file )
		{
			fseek( file, 0L, SEEK_END );
			size_t binarySize = ftell( file );

			rewind( file );
			char* binary = new char[binarySize];
			fread( binary, sizeof(char), binarySize, file );
			fclose( file );

			if (binarySize)
			{
				const DeviceCL* dd = (const DeviceCL*) deviceData;
				program = clCreateProgramWithBinary( dd->m_context, 1, &dd->m_deviceIdx, &binarySize, (const unsigned char**)&binary, 0, &status );
				ADLASSERT( status == CL_SUCCESS );
				status = clBuildProgram( program, 1, &dd->m_deviceIdx, option, 0, 0 );
				ADLASSERT( status == CL_SUCCESS );
			if( status != CL_SUCCESS )
			{
				char *build_log;
				size_t ret_val_size;
				clGetProgramBuildInfo(program, dd->m_deviceIdx, CL_PROGRAM_BUILD_LOG, 0, NULL, &ret_val_size);
				build_log = new char[ret_val_size+1];
				clGetProgramBuildInfo(program, dd->m_deviceIdx, CL_PROGRAM_BUILD_LOG, ret_val_size, build_log, NULL);

				build_log[ret_val_size] = '\0';

				debugPrintf("%s\n", build_log);

				delete build_log;
				ADLASSERT(0);
				}

			}
		}
	}
	if( !m_ptr )
	{
		File kernelFile;
		ADLASSERT( kernelFile.open( fileNameWithExtension ) );
		const char* source = kernelFile.getSource().c_str();
		setFromSrc( m_deviceData, source, option );

		if( cacheBinary )
		{	//	write to binary
			size_t binarySize;
			status = clGetProgramInfo( program, CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &binarySize, 0 );
			ADLASSERT( status == CL_SUCCESS );

			char* binary = new char[binarySize];

			status = clGetProgramInfo( program, CL_PROGRAM_BINARIES, sizeof(char*), &binary, 0 );
			ADLASSERT( status == CL_SUCCESS );

			{
				FILE* file = fopen(binaryFileName, "wb");
				if (file)
				{
					fwrite( binary, sizeof(char), binarySize, file );
					fclose( file );
				}
			}

			delete [] binary;
		}
	}
}



template<>
void KernelBuilder<TYPE_CL>::setFromSrcCached( const Device* deviceData, const char* src, const char* fileName, const char* option )
{
	m_deviceData = deviceData;

	bool cacheBinary = true;
	cl_program& program = (cl_program&)m_ptr;
	cl_int status = 0;	
	
	char binaryFileName[512];
	{
		char deviceName[256];
		deviceData->getDeviceName(deviceName);
		char driverVersion[256];
		const DeviceCL* dd = (const DeviceCL*) deviceData;
		clGetDeviceInfo(dd->m_deviceIdx, CL_DRIVER_VERSION, 256, &driverVersion, NULL);
		
		const char* strippedFileName = strip(fileName,"\\");
		strippedFileName = strip(strippedFileName,"/");

		sprintf_s(binaryFileName,"cache/%s.%s.%s.bin",strippedFileName, deviceName,driverVersion );
	}

	
	char fileNameWithExtension[256];
	sprintf_s(fileNameWithExtension,"%s.cl",fileName, ".cl");

	bool upToDate = isFileUpToDate(binaryFileName,fileNameWithExtension);


	if( cacheBinary )
	{
		
		bool fileUpToDate = isFileUpToDate(binaryFileName,fileNameWithExtension);

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

				const DeviceCL* dd = (const DeviceCL*) deviceData;
				program = clCreateProgramWithBinary( dd->m_context, 1, &dd->m_deviceIdx, &binarySize, (const unsigned char**)&binary, 0, &status );
				ADLASSERT( status == CL_SUCCESS );
				status = clBuildProgram( program, 1, &dd->m_deviceIdx, option, 0, 0 );
				ADLASSERT( status == CL_SUCCESS );

				if( status != CL_SUCCESS )
				{
					char *build_log;
					size_t ret_val_size;
					clGetProgramBuildInfo(program, dd->m_deviceIdx, CL_PROGRAM_BUILD_LOG, 0, NULL, &ret_val_size);
					build_log = new char[ret_val_size+1];
					clGetProgramBuildInfo(program, dd->m_deviceIdx, CL_PROGRAM_BUILD_LOG, ret_val_size, build_log, NULL);

					build_log[ret_val_size] = '\0';

					debugPrintf("%s\n", build_log);

					delete build_log;
					ADLASSERT(0);
				}
				delete[] binary;
			}
		}
	}


	if( !m_ptr )
	{
		
		setFromSrc( deviceData, src, option );

		if( cacheBinary )
		{	//	write to binary
			cl_uint numAssociatedDevices;
			status = clGetProgramInfo( program, CL_PROGRAM_NUM_DEVICES, sizeof(cl_uint), &numAssociatedDevices, 0 );
			ADLASSERT( status == CL_SUCCESS );
			if (numAssociatedDevices==1)
			{
			

				size_t binarySize;
				status = clGetProgramInfo( program, CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &binarySize, 0 );
				ADLASSERT( status == CL_SUCCESS );

				char* binary = new char[binarySize];

				status = clGetProgramInfo( program, CL_PROGRAM_BINARIES, sizeof(char*), &binary, 0 );
				ADLASSERT( status == CL_SUCCESS );

				{
					FILE* file = fopen(binaryFileName, "wb");
					if (file)
					{
						fwrite( binary, sizeof(char), binarySize, file );
						fclose( file );
					}
				}

				delete [] binary;
			}
		}
	}
}


template<>
void KernelBuilder<TYPE_CL>::setFromSrc( const Device* deviceData, const char* src, const char* option )
{
	ADLASSERT( deviceData->m_type == TYPE_CL );
	m_deviceData = deviceData;
	const DeviceCL* dd = (const DeviceCL*) deviceData;

	cl_program& program = (cl_program&)m_ptr;
	cl_int status = 0;
	size_t srcSize[] = {strlen( src )};
	program = clCreateProgramWithSource( dd->m_context, 1, &src, srcSize, &status );
	ADLASSERT( status == CL_SUCCESS );
	status = clBuildProgram( program, 1, &dd->m_deviceIdx, option, NULL, NULL );
	if( status != CL_SUCCESS )
	{
		char *build_log;
		size_t ret_val_size;
		clGetProgramBuildInfo(program, dd->m_deviceIdx, CL_PROGRAM_BUILD_LOG, 0, NULL, &ret_val_size);
		build_log = new char[ret_val_size+1];
		clGetProgramBuildInfo(program, dd->m_deviceIdx, CL_PROGRAM_BUILD_LOG, ret_val_size, build_log, NULL);

		build_log[ret_val_size] = '\0';

		debugPrintf("%s\n", build_log);
		printf("%s\n", build_log);

		ADLASSERT(0);
		delete build_log;
		
	}
}

template<>
KernelBuilder<TYPE_CL>::~KernelBuilder()
{
	cl_program program = (cl_program)m_ptr;
	clReleaseProgram( program );
}

template<>
void KernelBuilder<TYPE_CL>::createKernel( const char* funcName, Kernel& kernelOut )
{
	KernelCL* clKernel = (KernelCL*)&kernelOut;

	cl_program program = (cl_program)m_ptr;
	cl_int status = 0;
	clKernel->getKernel() = clCreateKernel(program, funcName, &status );
	ADLASSERT( status == CL_SUCCESS );

	kernelOut.m_type = TYPE_CL;
}

template<>
void KernelBuilder<TYPE_CL>::deleteKernel( Kernel& kernel )
{
	KernelCL* clKernel = (KernelCL*)&kernel;
	clReleaseKernel( clKernel->getKernel() );
}



class LauncherCL
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		__inline
		static void setBuffers( Launcher* launcher, BufferInfo* buffInfo, int n );
		template<typename T>
		__inline
		static void setConst( Launcher* launcher, Buffer<T>& constBuff, const T& consts );
		__inline
		static void launch2D( Launcher* launcher, int numThreadsX, int numThreadsY, int localSizeX, int localSizeY );
};

void LauncherCL::setBuffers( Launcher* launcher, BufferInfo* buffInfo, int n )
{
	KernelCL* clKernel = (KernelCL*)launcher->m_kernel;
	for(int i=0; i<n; i++)
	{
		Buffer<int>* buff = (Buffer<int>*)buffInfo[i].m_buffer;
		cl_int status = clSetKernelArg( clKernel->getKernel(), launcher->m_idx++, sizeof(cl_mem), &buff->m_ptr );
		ADLASSERT( status == CL_SUCCESS );
	}
}

template<typename T>
void LauncherCL::setConst( Launcher* launcher, Buffer<T>& constBuff, const T& consts )
{
	KernelCL* clKernel = (KernelCL*)launcher->m_kernel;
	int sz=sizeof(T);
	cl_int status = clSetKernelArg( clKernel->getKernel(), launcher->m_idx++, sz, &consts );
	ADLASSERT( status == CL_SUCCESS );
}

void LauncherCL::launch2D( Launcher* launcher, int numThreadsX, int numThreadsY, int localSizeX, int localSizeY )
{
	KernelCL* clKernel = (KernelCL*)launcher->m_kernel;
	const DeviceCL* ddcl = (const DeviceCL*)launcher->m_deviceData;
	size_t gRange[3] = {1,1,1};
	size_t lRange[3] = {1,1,1};
	lRange[0] = localSizeX;
	lRange[1] = localSizeY;
	gRange[0] = max((size_t)1, (numThreadsX/lRange[0])+(!(numThreadsX%lRange[0])?0:1));
	gRange[0] *= lRange[0];
	gRange[1] = max((size_t)1, (numThreadsY/lRange[1])+(!(numThreadsY%lRange[1])?0:1));
	gRange[1] *= lRange[1];

	cl_int status = clEnqueueNDRangeKernel( ddcl->m_commandQueue, 
		clKernel->getKernel(), 2, NULL, gRange, lRange, 0,0,0 );
	ADLASSERT( status == CL_SUCCESS );
}


};