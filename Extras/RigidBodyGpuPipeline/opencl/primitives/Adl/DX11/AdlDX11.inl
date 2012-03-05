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


#include <windows.h>
#include <d3d11.h>
#include <d3dx11.h>
#include <d3dcompiler.h>
#include <DXGI.h>
#pragma comment(lib,"d3dx11.lib")
#pragma comment(lib,"d3d11.lib")
#pragma comment(lib,"DXGI.lib")

namespace adl
{

#define u32 unsigned int

struct DeviceDX11 : public Device
{
	typedef DeviceUtils::Config Config;


	__inline
	DeviceDX11() : Device( TYPE_DX11 ), m_kernelManager(0){}
	__inline
	void* getContext() const { return m_context; }
	__inline
	void initialize(const Config& cfg);
	__inline
	void release();

	template<typename T>
	__inline
	void allocate(Buffer<T>* buf, int nElems, BufferBase::BufferType type);

	template<typename T>
	__inline
	void deallocate(Buffer<T>* buf);

	template<typename T>
	__inline
	void copy(Buffer<T>* dst, const Buffer<T>* src, int nElems);

	template<typename T>
	__inline
	void copy(T* dst, const Buffer<T>* src, int nElems, int srcOffsetNElems = 0);

	template<typename T>
	__inline
	void copy(Buffer<T>* dst, const T* src, int nElems, int dstOffsetNElems = 0);

	__inline
	void waitForCompletion() const;

	__inline
	void getDeviceName( char nameOut[128] ) const;

	__inline
	static
	int getNDevices();

	__inline
	Kernel* getKernel(const char* fileName, const char* funcName, const char* option = NULL, const char* src = NULL, bool cacheKernel = true )const;


	ID3D11DeviceContext* m_context;
	ID3D11Device* m_device;
	IDXGISwapChain* m_swapChain;

	KernelManager* m_kernelManager;
};

template<typename T>
struct BufferDX11 : public Buffer<T>
{
	ID3D11Buffer* getBuffer() { return (ID3D11Buffer*)m_ptr; }
	ID3D11UnorderedAccessView* getUAV() { return (ID3D11UnorderedAccessView*)m_uav; }
	ID3D11ShaderResourceView* getSRV() { return (ID3D11ShaderResourceView*)m_srv; }

	ID3D11Buffer** getBufferPtr() { return (ID3D11Buffer**)&m_ptr; }
	ID3D11UnorderedAccessView** getUAVPtr() { return (ID3D11UnorderedAccessView**)&m_uav; }
	ID3D11ShaderResourceView** getSRVPtr() { return (ID3D11ShaderResourceView**)&m_srv; }
};

#define SAFE_RELEASE(p)      { if(p) { (p)->Release(); (p)=NULL; } }


void DeviceDX11::initialize(const Config& cfg)
{
	DeviceDX11* deviceData = this;

	HRESULT hr = S_OK;
	UINT createDeviceFlg = 0;
#ifdef _DEBUG
	createDeviceFlg |= D3D11_CREATE_DEVICE_DEBUG;
#endif
	D3D_FEATURE_LEVEL fl[] = {
		D3D_FEATURE_LEVEL_11_0,
		D3D_FEATURE_LEVEL_10_1,
		D3D_FEATURE_LEVEL_10_0
	};

typedef HRESULT (WINAPI * LPD3D11CREATEDEVICE)( IDXGIAdapter*, D3D_DRIVER_TYPE, HMODULE, u32, D3D_FEATURE_LEVEL*, UINT, u32, ID3D11Device**, D3D_FEATURE_LEVEL*, ID3D11DeviceContext** );

	HMODULE moduleD3D11 = 0; 
#ifdef UNICODE
	moduleD3D11 = LoadLibrary( L"d3d11.dll" );
#else
	moduleD3D11 = LoadLibrary( "d3d11.dll" );
#endif
	ADLASSERT( moduleD3D11 );

	LPD3D11CREATEDEVICE _DynamicD3D11CreateDevice; 
	_DynamicD3D11CreateDevice = ( LPD3D11CREATEDEVICE )GetProcAddress( moduleD3D11, "D3D11CreateDevice" );

	D3D_DRIVER_TYPE type = D3D_DRIVER_TYPE_HARDWARE;
	//	http://msdn.microsoft.com/en-us/library/ff476082(v=VS.85).aspx
	//	If you set the pAdapter parameter to a non-NULL value, you must also set the DriverType parameter to the D3D_DRIVER_TYPE_UNKNOWN value. If you set the pAdapter parameter to a non-NULL value and the DriverType parameter to the D3D_DRIVER_TYPE_HARDWARE value, D3D11CreateDevice returns an HRESULT of E_INVALIDARG.
	type = D3D_DRIVER_TYPE_UNKNOWN;
/*
	// Create a hardware Direct3D 11 device
	hr = _DynamicD3D11CreateDevice( NULL, 
		type, NULL, createDeviceFlg,
		fl, _countof(fl), D3D11_SDK_VERSION, &deviceData->m_device, NULL, &deviceData->m_context );
*/
	IDXGIAdapter* adapter = NULL;
	{//	get adapter of the index
		IDXGIFactory* factory = NULL;
		int targetAdapterIdx = cfg.m_deviceIdx;//min( cfg.m_deviceIdx, getNDevices()-1 );
		CreateDXGIFactory( __uuidof(IDXGIFactory), (void**)&factory );

		u32 i = 0;
		while( factory->EnumAdapters( i, &adapter ) != DXGI_ERROR_NOT_FOUND )
		{
			if( i== targetAdapterIdx ) break;
			i++;
		}
		factory->Release();
	}

	// Create a hardware Direct3D 11 device
	hr = D3D11CreateDevice( adapter, 
		type, 
		NULL, createDeviceFlg,
		fl, _countof(fl), D3D11_SDK_VERSION, &deviceData->m_device, NULL, &deviceData->m_context );

	ADLASSERT( hr == S_OK );

   // Check if the hardware device supports Compute Shader 4.0
    D3D11_FEATURE_DATA_D3D10_X_HARDWARE_OPTIONS hwopts;
    deviceData->m_device->CheckFeatureSupport(D3D11_FEATURE_D3D10_X_HARDWARE_OPTIONS, &hwopts, sizeof(hwopts));

	if( !hwopts.ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x )
	{
		SAFE_RELEASE( deviceData->m_context );
		SAFE_RELEASE( deviceData->m_device );

		debugPrintf("DX11 GPU is not present\n");
		ADLASSERT( 0 );
	}

	m_kernelManager = new KernelManager;
}

void DeviceDX11::release()
{
	SAFE_RELEASE( m_context );
	SAFE_RELEASE( m_device );

	if( m_kernelManager ) delete m_kernelManager;
}

template<typename T>
void DeviceDX11::allocate(Buffer<T>* buf, int nElems, BufferBase::BufferType type)
{
	ADLASSERT( type != BufferBase::BUFFER_ZERO_COPY );

	DeviceDX11* deviceData = this;
	buf->m_device = deviceData;
	buf->m_size = nElems;
	BufferDX11<T>* dBuf = (BufferDX11<T>*)buf;

//	if( type & BufferBase::BUFFER )
	{
		HRESULT hr = S_OK;

		if( type == BufferBase::BUFFER_CONST )
		{
			ADLASSERT( nElems == 1 );
			D3D11_BUFFER_DESC constant_buffer_desc;
			ZeroMemory( &constant_buffer_desc, sizeof(constant_buffer_desc) );
//			constant_buffer_desc.ByteWidth = NEXTMULTIPLEOF( sizeof(T), 16 );
			constant_buffer_desc.ByteWidth = (((sizeof(T))/(16) + (((sizeof(T))%(16)==0)?0:1))*(16));
//			constant_buffer_desc.Usage = D3D11_USAGE_DYNAMIC;
//			constant_buffer_desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
//			constant_buffer_desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			constant_buffer_desc.Usage = D3D11_USAGE_DEFAULT;
			constant_buffer_desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			constant_buffer_desc.CPUAccessFlags = 0;

			hr = deviceData->m_device->CreateBuffer( &constant_buffer_desc, NULL, dBuf->getBufferPtr() );
			ADLASSERT( hr == S_OK );
			return;
		}

		D3D11_BUFFER_DESC buffer_desc;
		ZeroMemory(&buffer_desc, sizeof(buffer_desc));
		buffer_desc.ByteWidth = nElems * sizeof(T);

		if( type != BufferBase::BUFFER_RAW )
		{
			buffer_desc.StructureByteStride = sizeof(T);
//		    buffer_desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
		}

		if( type == BufferBase::BUFFER_STAGING )
		{
			buffer_desc.Usage = D3D11_USAGE_STAGING;
		    buffer_desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
		}
		else if( type == BufferBase::BUFFER_INDEX )
		{
			buffer_desc.Usage = D3D11_USAGE_DEFAULT;
			buffer_desc.BindFlags = D3D11_BIND_INDEX_BUFFER;
		}
		else if( type == BufferBase::BUFFER_VERTEX )
		{
			buffer_desc.Usage = D3D11_USAGE_DEFAULT;
			buffer_desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		}
		else
		{
			buffer_desc.Usage = D3D11_USAGE_DEFAULT;
			
			buffer_desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
			buffer_desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;

//	check this
			if(type == BufferBase::BUFFER_RAW)
			{
//				buffer_desc.BindFlags |= D3D11_BIND_INDEX_BUFFER | D3D11_BIND_VERTEX_BUFFER;
				buffer_desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS | D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS; // need this to be used for DispatchIndirect
			}
		}
		hr = deviceData->m_device->CreateBuffer(&buffer_desc, NULL, dBuf->getBufferPtr());

		ADLASSERT( hr == S_OK );

		if( type == BufferBase::BUFFER_INDEX ) return;

		if( type == BufferBase::BUFFER || 
			type == BufferBase::BUFFER_RAW || 
			type == BufferBase::BUFFER_W_COUNTER )
		{
			// Create UAVs for all CS buffers
			D3D11_UNORDERED_ACCESS_VIEW_DESC uavbuffer_desc;
			ZeroMemory(&uavbuffer_desc, sizeof(uavbuffer_desc));
			uavbuffer_desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;

			if( type == BufferBase::BUFFER_RAW )
			{
				uavbuffer_desc.Format = DXGI_FORMAT_R32_TYPELESS;
				uavbuffer_desc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
				uavbuffer_desc.Buffer.NumElements = buffer_desc.ByteWidth / 4; 
			}
			else
			{
				uavbuffer_desc.Format = DXGI_FORMAT_UNKNOWN;
				uavbuffer_desc.Buffer.NumElements = nElems;
			}

			if( type == BufferBase::BUFFER_W_COUNTER )
			{
				uavbuffer_desc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_COUNTER;
			}

			hr = deviceData->m_device->CreateUnorderedAccessView(dBuf->getBuffer(), &uavbuffer_desc, dBuf->getUAVPtr());
			ADLASSERT( hr == S_OK );

			// Create SRVs for all CS buffers
			D3D11_SHADER_RESOURCE_VIEW_DESC srvbuffer_desc;
			ZeroMemory(&srvbuffer_desc, sizeof(srvbuffer_desc));
			if( type == BufferBase::BUFFER_RAW )
			{
				ADLASSERT( sizeof(T) <= 16 );
				srvbuffer_desc.Format = DXGI_FORMAT_R32_UINT;
				srvbuffer_desc.Buffer.ElementWidth = nElems;
//			if ( buffer_desc.MiscFlags & D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS )
//			{
//				srvbuffer_desc.Format = DXGI_FORMAT_R32_TYPELESS;
//				srvbuffer_desc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
//				srvbuffer_desc.BufferEx.NumElements = buffer_desc.ByteWidth / 4;
			}
			else
			{
				srvbuffer_desc.Format = DXGI_FORMAT_UNKNOWN;
				srvbuffer_desc.Buffer.ElementWidth = nElems;
			}
			srvbuffer_desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;

			hr = deviceData->m_device->CreateShaderResourceView(dBuf->getBuffer(), &srvbuffer_desc, dBuf->getSRVPtr());
			ADLASSERT( hr == S_OK );
		}
		else if( type == BufferBase::BUFFER_APPEND )
		{
			D3D11_UNORDERED_ACCESS_VIEW_DESC desc;
			ZeroMemory( &desc, sizeof(desc) );
			desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
			desc.Buffer.FirstElement = 0;

			desc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_APPEND;

			desc.Format = DXGI_FORMAT_UNKNOWN;      // Format must be must be DXGI_FORMAT_UNKNOWN, when creating a View of a Structured Buffer
			desc.Buffer.NumElements = buffer_desc.ByteWidth / buffer_desc.StructureByteStride; 

			hr = deviceData->m_device->CreateUnorderedAccessView( dBuf->getBuffer(), &desc, dBuf->getUAVPtr() );
			ADLASSERT( hr == S_OK );
		}
	}
//	else
//	{
//		ADLASSERT(0);
//	}
}

template<typename T>
void DeviceDX11::deallocate(Buffer<T>* buf)
{
	BufferDX11<T>* dBuf = (BufferDX11<T>*)buf;

	if( dBuf->getBuffer() )
	{
		dBuf->getBuffer()->Release();
		dBuf->m_ptr = NULL;
	}
	if( dBuf->getUAV() )
	{
		dBuf->getUAV()->Release();
		dBuf->m_uav = NULL;
	}
	if( dBuf->getSRV() )
	{
		dBuf->getSRV()->Release();
		dBuf->m_srv = NULL;
	}
	buf->m_device = 0;
}

template<typename T>
void DeviceDX11::copy(Buffer<T>* dst, const Buffer<T>* src, int nElems)
{
	if( dst->m_device->m_type == TYPE_DX11 || src->m_device->m_type == TYPE_DX11 )
	{
		DeviceDX11* deviceData = this;
		BufferDX11<T>* dDst = (BufferDX11<T>*)dst;
		BufferDX11<T>* dSrc = (BufferDX11<T>*)src;

		D3D11_MAPPED_SUBRESOURCE MappedVelResource = {0};

		D3D11_BOX destRegion;
		destRegion.left = 0*sizeof(T);
		destRegion.front = 0;
		destRegion.top = 0;
		destRegion.bottom = 1;
		destRegion.back = 1;
		destRegion.right = (0+nElems)*sizeof(T);

		deviceData->m_context->CopySubresourceRegion(
				dDst->getBuffer(),
				0, 0, 0, 0,
				dSrc->getBuffer(),
				0,
				&destRegion );

	}
	else if( src->m_device->m_type == TYPE_HOST )
	{
		ADLASSERT( dst->getType() == TYPE_DX11 );
		dst->write( src->m_ptr, nElems );
	}
	else if( dst->m_device->m_type == TYPE_HOST )
	{
		ADLASSERT( src->getType() == TYPE_DX11 );
		src->read( dst->m_ptr, nElems );
	}
	else
	{
		ADLASSERT( 0 );
	}
}

template<typename T>
void DeviceDX11::copy(T* dst, const Buffer<T>* src, int nElems, int srcOffsetNElems)
{
	DeviceDX11* deviceData = this;
	BufferDX11<T>* dSrc = (BufferDX11<T>*)src;
	Buffer<T> sBuf( deviceData, nElems, BufferBase::BUFFER_STAGING );
	BufferDX11<T>* dStagingBuf = (BufferDX11<T>*)&sBuf;


	ID3D11Buffer *StagingBuffer = dStagingBuf->getBuffer();
    D3D11_MAPPED_SUBRESOURCE MappedVelResource = {0};

    D3D11_BOX destRegion;
    destRegion.left = srcOffsetNElems*sizeof(T);
    destRegion.front = 0;
    destRegion.top = 0;
    destRegion.bottom = 1;
    destRegion.back = 1;
    destRegion.right = (srcOffsetNElems+nElems)*sizeof(T);

    deviceData->m_context->CopySubresourceRegion(
            StagingBuffer,
            0, 0, 0, 0,
			dSrc->getBuffer(),
            0,
            &destRegion);

    deviceData->m_context->Map(StagingBuffer, 0, D3D11_MAP_READ, 0, &MappedVelResource);
    memcpy(dst, MappedVelResource.pData, nElems*sizeof(T));
    deviceData->m_context->Unmap(StagingBuffer, 0);
}

template<typename T>
void DeviceDX11::copy(Buffer<T>* dst, const T* src, int nElems, int dstOffsetNElems)
{
	BufferDX11<T>* dBuf = (BufferDX11<T>*)dst;

	DeviceDX11* deviceData = this;

    D3D11_BOX destRegion;
    destRegion.left = dstOffsetNElems*sizeof(T);
    destRegion.front = 0;
    destRegion.top = 0;
    destRegion.bottom = 1;
    destRegion.back = 1;
    destRegion.right = (dstOffsetNElems+nElems)*sizeof(T);
	deviceData->m_context->UpdateSubresource(dBuf->getBuffer(), 0, &destRegion, src, 0, 0);
}

void DeviceDX11::waitForCompletion() const
{
	const DeviceDX11* deviceData = this;

	ID3D11Query* syncQuery;
	D3D11_QUERY_DESC qDesc;
	qDesc.Query = D3D11_QUERY_EVENT;
	qDesc.MiscFlags = 0;
	deviceData->m_device->CreateQuery( &qDesc, &syncQuery );
	deviceData->m_context->End( syncQuery );
	while( deviceData->m_context->GetData( syncQuery, 0,0,0 ) == S_FALSE ){}
	syncQuery->Release();
}

int DeviceDX11::getNDevices()
{
	IDXGIFactory1* factory = NULL;
	IDXGIAdapter1* adapter = NULL;
	CreateDXGIFactory1( __uuidof(IDXGIFactory1), (void**)&factory );

	u32 i = 0;
	while( factory->EnumAdapters1( i, &adapter ) != DXGI_ERROR_NOT_FOUND )
	{
		i++;
	}

	factory->Release();
	return i;
}

void DeviceDX11::getDeviceName( char nameOut[128] ) const
{
	IDXGIAdapter* adapter;// = getAdapterFromDevice( this );
	{
		IDXGIDevice* pDXGIDevice;

		ADLASSERT( m_device->QueryInterface(__uuidof(IDXGIDevice), (void **)&pDXGIDevice) == S_OK );
		ADLASSERT( pDXGIDevice->GetParent(__uuidof(IDXGIAdapter), (void **)&adapter) == S_OK );

		pDXGIDevice->Release();
	}
	DXGI_ADAPTER_DESC adapterDesc;
	adapter->GetDesc( &adapterDesc );

//	wcstombs( nameOut, adapterDesc.Description, 128 );
	size_t	i;
	wcstombs_s( &i, nameOut, 128, adapterDesc.Description, 128 );
}

Kernel* DeviceDX11::getKernel(const char* fileName, const char* funcName, const char* option, const char* src, bool cacheKernel ) const
{
	return m_kernelManager->query( this, fileName, funcName, option, src, cacheKernel );
}

#undef u32

#undef SAFE_RELEASE

};
