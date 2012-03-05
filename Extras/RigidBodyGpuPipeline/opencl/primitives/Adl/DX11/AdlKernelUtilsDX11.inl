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

#define SAFE_RELEASE(p)      { if(p) { (p)->Release(); (p)=NULL; } }

struct KernelDX11 : public Kernel
{
	ID3D11ComputeShader* getKernel() { return (ID3D11ComputeShader*)m_kernel; }
	ID3D11ComputeShader** getKernelPtr() { return (ID3D11ComputeShader**)&m_kernel; }
};


__inline
#ifdef UNICODE
HRESULT FindDXSDKShaderFileCch( __in_ecount(cchDest) WCHAR* strDestPath,
                                int cchDest, 
                                __in LPCWSTR strFilename )
#else
HRESULT FindDXSDKShaderFileCch( __in_ecount(cchDest) CHAR* strDestPath,
                                int cchDest, 
                                __in LPCSTR strFilename )
#endif
{
    if( NULL == strFilename || strFilename[0] == 0 || NULL == strDestPath || cchDest < 10 )
        return E_INVALIDARG;

    // Get the exe name, and exe path
#ifdef UNICODE
    WCHAR strExePath[MAX_PATH] =
#else
    CHAR strExePath[MAX_PATH] =
#endif
    {
        0
    };
#ifdef UNICODE
    WCHAR strExeName[MAX_PATH] =
#else
    CHAR strExeName[MAX_PATH] =
#endif
    {
        0
    };
#ifdef UNICODE
    WCHAR* strLastSlash = NULL;
#else
    CHAR* strLastSlash = NULL;
#endif
    GetModuleFileName( NULL, strExePath, MAX_PATH );
    strExePath[MAX_PATH - 1] = 0;
#ifdef UNICODE
    strLastSlash = wcsrchr( strExePath, TEXT( '\\' ) );
#else
    strLastSlash = strrchr( strExePath, TEXT( '\\' ) );
#endif
    if( strLastSlash )
    {
#ifdef UNICODE
        wcscpy_s( strExeName, MAX_PATH, &strLastSlash[1] );
#else

#endif
        // Chop the exe name from the exe path
        *strLastSlash = 0;

        // Chop the .exe from the exe name
#ifdef UNICODE
        strLastSlash = wcsrchr( strExeName, TEXT( '.' ) );
#else
        strLastSlash = strrchr( strExeName, TEXT( '.' ) );
#endif
        if( strLastSlash )
            *strLastSlash = 0;
    }

    // Search in directories:
    //      .\
    //      %EXE_DIR%\..\..\%EXE_NAME%
#ifdef UNICODE
    wcscpy_s( strDestPath, cchDest, strFilename );
#else
	strcpy_s( strDestPath, cchDest, strFilename );
#endif
    if( GetFileAttributes( strDestPath ) != 0xFFFFFFFF )
        return S_OK;

//    swprintf_s( strDestPath, cchDest, L"%s\\..\\..\\%s\\%s", strExePath, strExeName, strFilename );
#ifdef UNICODE
    swprintf_s( strDestPath, cchDest, L"%s\\..\\%s\\%s", strExePath, strExeName, strFilename );
#else
    sprintf_s( strDestPath, cchDest, "%s\\..\\%s\\%s", strExePath, strExeName, strFilename );
#endif
    if( GetFileAttributes( strDestPath ) != 0xFFFFFFFF )
        return S_OK;    

    // On failure, return the file as the path but also return an error code
#ifdef UNICODE
    wcscpy_s( strDestPath, cchDest, strFilename );
#else
    strcpy_s( strDestPath, cchDest, strFilename );
#endif

	ADLASSERT( 0 );

    return E_FAIL;
}




template<>
void KernelBuilder<TYPE_DX11>::setFromFile( const Device* deviceData, const char* fileName, const char* option, bool addExtension,
	bool cacheKernel)
{
	char fileNameWithExtension[256];

	if( addExtension )
		sprintf_s( fileNameWithExtension, "%s.hlsl", fileName );
	else
		sprintf_s( fileNameWithExtension, "%s", fileName );

	m_deviceData = deviceData;

	int nameLength = (int)strlen(fileNameWithExtension)+1;
#ifdef UNICODE
	WCHAR* wfileNameWithExtension = new WCHAR[nameLength];
#else
	CHAR* wfileNameWithExtension = new CHAR[nameLength];
#endif
	memset(wfileNameWithExtension,0,nameLength);
#ifdef UNICODE
	MultiByteToWideChar(CP_ACP,0,fileNameWithExtension,-1, wfileNameWithExtension, nameLength);
#else
	sprintf_s(wfileNameWithExtension, nameLength, "%s", fileNameWithExtension);
#endif
//			swprintf_s(wfileNameWithExtension, nameLength*2, L"%s", fileNameWithExtension);

	HRESULT hr;

	// Finds the correct path for the shader file.
	// This is only required for this sample to be run correctly from within the Sample Browser,
	// in your own projects, these lines could be removed safely
	hr = FindDXSDKShaderFileCch( m_path, MAX_PATH, wfileNameWithExtension );

	delete [] wfileNameWithExtension;

	ADLASSERT( hr == S_OK );
}

template<>
void KernelBuilder<TYPE_DX11>::setFromSrc( const Device* deviceData, const char* src, const char* option )
{
	m_deviceData = deviceData;
	m_ptr = (void*)src;
	m_path[0] = '0';
}

template<>
KernelBuilder<TYPE_DX11>::~KernelBuilder()
{

}

template<>
void KernelBuilder<TYPE_DX11>::createKernel( const char* funcName, Kernel& kernelOut )
{
	const DeviceDX11* deviceData = (const DeviceDX11*)m_deviceData;
	KernelDX11* dxKernel = (KernelDX11*)&kernelOut;
	HRESULT hr;

	DWORD dwShaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
#if defined( DEBUG ) || defined( _DEBUG )
	// Set the D3DCOMPILE_DEBUG flag to embed debug information in the shaders.
	// Setting this flag improves the shader debugging experience, but still allows 
	// the shaders to be optimized and to run exactly the way they will run in 
	// the release configuration of this program.
	dwShaderFlags |= D3DCOMPILE_DEBUG;
#endif

	const D3D_SHADER_MACRO defines[] = 
	{
#ifdef USE_STRUCTURED_BUFFERS
		"USE_STRUCTURED_BUFFERS", "1",
#endif

#ifdef TEST_DOUBLE
		"TEST_DOUBLE", "1",
#endif
		NULL, NULL
	};

	// We generally prefer to use the higher CS shader profile when possible as CS 5.0 is better performance on 11-class hardware
	LPCSTR pProfile = ( deviceData->m_device->GetFeatureLevel() >= D3D_FEATURE_LEVEL_11_0 ) ? "cs_5_0" : "cs_4_0";

	ID3DBlob* pErrorBlob = NULL;
	ID3DBlob* pBlob = NULL;
	if( m_path[0] == '0' )
	{
		char* src = (char*)m_ptr;
		hr = D3DX11CompileFromMemory( src, strlen(src), 0, defines, NULL, funcName, pProfile, 
			dwShaderFlags, NULL, NULL, &pBlob, &pErrorBlob, NULL );
	}
	else
	{
		hr = D3DX11CompileFromFile( m_path, defines, NULL, funcName, pProfile, 
			dwShaderFlags, NULL, NULL, &pBlob, &pErrorBlob, NULL );
	}

	if ( FAILED(hr) )
	{
		debugPrintf("%s", (char*)pErrorBlob->GetBufferPointer());
	}
	ADLASSERT( hr == S_OK );

	hr = deviceData->m_device->CreateComputeShader( pBlob->GetBufferPointer(), pBlob->GetBufferSize(), NULL, 
		dxKernel->getKernelPtr() );

#if defined(DEBUG) || defined(PROFILE)
	if ( kernelOut.m_kernel )
		kernelOut.m_kernel->SetPrivateData( WKPDID_D3DDebugObjectName, lstrlenA(pFunctionName), pFunctionName );
#endif

	SAFE_RELEASE( pErrorBlob );
	SAFE_RELEASE( pBlob );

	kernelOut.m_type = TYPE_DX11;
}

template<>
void KernelBuilder<TYPE_DX11>::deleteKernel( Kernel& kernel )
{
	KernelDX11* dxKernel = (KernelDX11*)&kernel;

	if( kernel.m_kernel )
	{
		dxKernel->getKernel()->Release();
		kernel.m_kernel = NULL;
	}
}



class LauncherDX11
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

void LauncherDX11::setBuffers( Launcher* launcher, BufferInfo* buffInfo, int n )
{
	KernelDX11* dxKernel = (KernelDX11*)launcher->m_kernel;
	const DeviceDX11* dddx = (const DeviceDX11*)launcher->m_deviceData;

	for(int i=0; i<n; i++)
	{
		BufferDX11<int>* dBuf = (BufferDX11<int>*)buffInfo[i].m_buffer;
		if( buffInfo[i].m_isReadOnly )
		{
			dddx->m_context->CSSetShaderResources( launcher->m_idx++, 1, dBuf->getSRVPtr() );
		}
		else
		{
			//	todo. cannot initialize append buffer with proper counter value which is the last arg
			dddx->m_context->CSSetUnorderedAccessViews( launcher->m_idxRw++, 1, dBuf->getUAVPtr(), 0 );
		}
	}
}

template<typename T>
void LauncherDX11::setConst( Launcher* launcher, Buffer<T>& constBuff, const T& consts )
{
	KernelDX11* dxKernel = (KernelDX11*)launcher->m_kernel;
	const DeviceDX11* dddx = (const DeviceDX11*)launcher->m_deviceData;
	BufferDX11<T>* dBuf = (BufferDX11<T>*)&constBuff;
/*
    D3D11_MAPPED_SUBRESOURCE MappedResource;
	dddx->m_context->Map( dBuf->getBuffer(), 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
    memcpy( MappedResource.pData, &consts, sizeof(T) );
	dddx->m_context->Unmap( dBuf->getBuffer(), 0 );
*/

	dddx->m_context->UpdateSubresource( dBuf->getBuffer(), 0, NULL, &consts, 0, 0 );

	dddx->m_context->CSSetConstantBuffers( 0, 1, dBuf->getBufferPtr() );
}

void LauncherDX11::launch2D( Launcher* launcher, int numThreadsX, int numThreadsY, int localSizeX, int localSizeY )
{
	KernelDX11* dxKernel = (KernelDX11*)launcher->m_kernel;
	const DeviceDX11* dddx = (const DeviceDX11*)launcher->m_deviceData;

	dddx->m_context->CSSetShader( dxKernel->getKernel(), NULL, 0 );

	int nx, ny, nz;
	nx = max( 1, (numThreadsX/localSizeX)+(!(numThreadsX%localSizeX)?0:1) );
	ny = max( 1, (numThreadsY/localSizeY)+(!(numThreadsY%localSizeY)?0:1) );
	nz = 1;

	dddx->m_context->Dispatch( nx, ny, nz );

	//	set 0 to registers
	{
	    dddx->m_context->CSSetShader( NULL, NULL, 0 );

		if( launcher->m_idxRw )
		{
			ID3D11UnorderedAccessView* aUAViewsNULL[ 16 ] = { 0 };
			dddx->m_context->CSSetUnorderedAccessViews( 0, 
				min( (unsigned int)launcher->m_idxRw, sizeof(aUAViewsNULL)/sizeof(*aUAViewsNULL) ), aUAViewsNULL, NULL );
		}

		if( launcher->m_idx )
		{
			ID3D11ShaderResourceView* ppSRVNULL[16] = { 0 };
			dddx->m_context->CSSetShaderResources( 0, 
				min( (unsigned int)launcher->m_idx, sizeof(ppSRVNULL)/sizeof(*ppSRVNULL) ), ppSRVNULL );
		}
	}
}

#undef SAFE_RELEASE

};
