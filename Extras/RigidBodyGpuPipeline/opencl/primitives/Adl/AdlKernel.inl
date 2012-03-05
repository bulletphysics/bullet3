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



#ifdef ADL_ENABLE_CL
	#include <Adl/CL/AdlKernelUtilsCL.inl>
#endif
#ifdef ADL_ENABLE_DX11
	#include <Adl/DX11/AdlKernelUtilsDX11.inl>
#endif

namespace adl
{

//==========================
//	KernelManager
//==========================
Kernel* KernelManager::query(const Device* dd, const char* fileName, const char* funcName, const char* option, const char* src,
	bool cacheKernel)
{
	printf("compiling kernel %s",funcName);
	const int charSize = 1024*2;
	KernelManager* s_kManager = this;

	char fullFineName[charSize];
	switch( dd->m_type )
	{
	case TYPE_CL:
#if defined(ADL_ENABLE_CL)
		sprintf_s(fullFineName,charSize,"%s.cl", fileName);
		break;
#endif
#if defined(ADL_ENABLE_DX11)
	case TYPE_DX11:
		sprintf_s(fullFineName,charSize,"%s.hlsl", fileName);
		break;
#endif
	default:
		ADLASSERT(0);
		break;
	};

	char mapName[charSize];
	{
		if( option )
			sprintf_s(mapName, charSize, "%d%s%s%s", (int)dd->getContext(), fullFineName, funcName, option);
		else
			sprintf_s(mapName, charSize, "%d%s%s", (int)dd->getContext(), fullFineName, funcName);
	}

	std::string str(mapName);

	KMap::iterator iter = s_kManager->m_map.find( str );

	Kernel* kernelOut;
	if( iter == s_kManager->m_map.end() )
	{
		kernelOut = new Kernel();

		switch( dd->m_type )
		{
#if defined(ADL_ENABLE_CL)
		case TYPE_CL:
			{
				KernelBuilder<TYPE_CL> builder;
				if( src )
					if (cacheKernel)
					{
						builder.setFromSrcCached( dd, src, fileName, option );
					} else
					{
						builder.setFromSrc( dd, src, option );
					}
				else
					builder.setFromFile( dd, fileName, option, true, cacheKernel );
				builder.createKernel( funcName, *kernelOut );
			}
			break;
#endif
#if defined(ADL_ENABLE_DX11)
		case TYPE_DX11:
			{
				KernelBuilder<TYPE_DX11> builder;
				if( src )
					builder.setFromSrc( dd, src, option );
				else
					builder.setFromFile( dd, fileName, option, true, cacheKernel );
				builder.createKernel( funcName, *kernelOut );
			}
			break;
#endif
		default:
			ADLASSERT(0);
			break;
		};
		s_kManager->m_map.insert( KMap::value_type(str,kernelOut) );
	}
	else
	{
		kernelOut = iter->second;
	}

	printf(" ready\n");
	return kernelOut;
}

KernelManager::~KernelManager()
{
	for(KMap::iterator iter = m_map.begin(); iter != m_map.end(); iter++)
	{
		Kernel* k = iter->second;
		switch( k->m_type )
		{
#if defined(ADL_ENABLE_CL)
		case TYPE_CL:
			KernelBuilder<TYPE_CL>::deleteKernel( *k );
			delete k;
			break;
#endif
#if defined(ADL_ENABLE_DX11)
		case TYPE_DX11:
			KernelBuilder<TYPE_DX11>::deleteKernel( *k );
			delete k;
			break;
#endif
		default:
			ADLASSERT(0);
			break;
		};
	}
}

//==========================
//	Launcher
//==========================

#if defined(ADL_ENABLE_DX11)
	#if defined(ADL_ENABLE_CL)
	#define SELECT_LAUNCHER( type, func ) \
		switch( type ) \
		{ \
		case TYPE_CL: LauncherCL::func; break; \
		case TYPE_DX11: LauncherDX11::func; break; \
		default: ADLASSERT(0); break; \
		};
	#else
	#define SELECT_LAUNCHER( type, func ) \
		switch( type ) \
		{ \
		case TYPE_DX11: LauncherDX11::func; break; \
		default: ADLASSERT(0); break; \
		};
	#endif
#else
	#if defined(ADL_ENABLE_CL)
	#define SELECT_LAUNCHER( type, func ) \
		switch( type ) \
		{ \
		case TYPE_CL: LauncherCL::func; break; \
		default: ADLASSERT(0); break; \
		};
	#else
	#define SELECT_LAUNCHER( type, func ) \
		switch( type ) \
		{ \
		default: ADLASSERT(0); break; \
		};
	#endif
#endif

Launcher::Launcher(const Device *dd, char *fileName, char *funcName, char *option)
{
	m_kernel = dd->getKernel( fileName, funcName, option );
	m_deviceData = dd;
	m_idx = 0;
	m_idxRw = 0;
}

Launcher::Launcher(const Device* dd, Kernel* kernel)
{
	m_kernel = kernel;
	m_deviceData = dd;
	m_idx = 0;
	m_idxRw = 0;
}

void Launcher::setBuffers( BufferInfo* buffInfo, int n )
{
	SELECT_LAUNCHER( m_deviceData->m_type, setBuffers( this, buffInfo, n ) );
}

template<typename T>
void Launcher::setConst( Buffer<T>& constBuff, const T& consts )
{
	SELECT_LAUNCHER( m_deviceData->m_type, setConst( this, constBuff, consts ) );
}

void Launcher::launch1D( int numThreads, int localSize )
{
	SELECT_LAUNCHER( m_deviceData->m_type, launch2D( this, numThreads, 1, localSize, 1 ) );
}

void Launcher::launch2D(  int numThreadsX, int numThreadsY, int localSizeX, int localSizeY )
{
	SELECT_LAUNCHER( m_deviceData->m_type, launch2D( this, numThreadsX, numThreadsY, localSizeX, localSizeY ) );
}

#undef SELECT_LAUNCHER

};