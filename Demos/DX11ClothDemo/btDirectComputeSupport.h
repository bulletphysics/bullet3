/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2010 Advanced Micro Devices

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_DIRECT_COMPUTE_SUPPORT_HPP
#define BT_DIRECT_COMPUTE_SUPPORT_HPP

// DX11 support
#include <windows.h>
#include <crtdbg.h>
#include <d3d11.h>
#include <d3dx11.h>
#include <d3dcompiler.h>

#ifndef SAFE_RELEASE
#define SAFE_RELEASE(p)      { if(p) { (p)->Release(); (p)=NULL; } }
#endif

namespace BTAcceleratedSoftBody
{

	/**
	 * Class to provide basic DX11 support to an application, wrapping the device creation functionality and necessary pointers.
	 */
	class DX11SupportHelper
	{
	private:

		ID3D11Device*           m_pd3dDevice;
		ID3D11DeviceContext*	m_pd3dImmediateContext;
		typedef HRESULT			(WINAPI * LPD3D11CREATEDEVICE)( IDXGIAdapter*, D3D_DRIVER_TYPE, HMODULE, UINT32, D3D_FEATURE_LEVEL*, UINT, UINT32, ID3D11Device**, D3D_FEATURE_LEVEL*, ID3D11DeviceContext** );
		HMODULE					m_s_hModD3D11;
		LPD3D11CREATEDEVICE		m_s_DynamicD3D11CreateDevice;


		bool Dynamic_EnsureD3D11APIs( void )
		{
			// If both modules are non-NULL, this function has already been called.  Note
			// that this doesn't guarantee that all ProcAddresses were found.
			if( m_s_hModD3D11 != NULL )
				return true;

		    // This may fail if Direct3D 11 isn't installed
			m_s_hModD3D11 = LoadLibraryA( "d3d11.dll" );
			if( m_s_hModD3D11 != NULL )
			{
				m_s_DynamicD3D11CreateDevice = ( LPD3D11CREATEDEVICE )GetProcAddress( m_s_hModD3D11, "D3D11CreateDevice" );
			}
		
			return ( m_s_hModD3D11 != NULL );
		}

		// Helper to call D3D11CreateDevice from d3d11.dll
		HRESULT WINAPI Dynamic_D3D11CreateDevice( IDXGIAdapter* pAdapter,
												  D3D_DRIVER_TYPE DriverType,
												  HMODULE Software,
												  UINT32 Flags,
												  D3D_FEATURE_LEVEL* pFeatureLevels,
												  UINT FeatureLevels,
												  UINT32 SDKVersion,
												  ID3D11Device** ppDevice,
												  D3D_FEATURE_LEVEL* pFeatureLevel,
												  ID3D11DeviceContext** ppImmediateContext )
		{
			if( Dynamic_EnsureD3D11APIs() && m_s_DynamicD3D11CreateDevice != NULL )
			{
				return m_s_DynamicD3D11CreateDevice( 
						pAdapter, 
						DriverType, 
						Software, 
						Flags, 
						pFeatureLevels, 
						FeatureLevels,
						SDKVersion, 
						ppDevice, 
						pFeatureLevel, 
						ppImmediateContext );
			}
			else
			{
				MessageBoxA( 0, "Could not locate resources need by d3d11."\
					"  Please install the latest DirectX SDK.", "Error", MB_ICONEXCLAMATION );
				return E_FAIL;
			}
		}

	public:
		DX11SupportHelper()
		{
			m_s_hModD3D11 = 0;
			m_s_DynamicD3D11CreateDevice = 0;
			m_pd3dDevice = 0;
			m_pd3dImmediateContext = 0;
		}

		virtual ~DX11SupportHelper()
		{
			SAFE_RELEASE( m_pd3dDevice );
			SAFE_RELEASE( m_pd3dImmediateContext );
		}

		ID3D11Device* getDevice()
		{
			return m_pd3dDevice;
		}

		ID3D11DeviceContext* getContext()
		{
			return m_pd3dImmediateContext;
		}		
		
		/**
		 * Do a simplistic initialisation of the first DirectCompute device in the system.
		 * Clearly the user can do their own version for more flexibility.
		 */
		bool InitComputeShaderDevice()
		{
			HRESULT hr = S_OK;

			UINT createDeviceFlags = 0;
		#ifdef _DEBUG
			createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
		#endif

			D3D_FEATURE_LEVEL fl[] = {
				D3D_FEATURE_LEVEL_11_0,
				D3D_FEATURE_LEVEL_10_1,
				D3D_FEATURE_LEVEL_10_0
			};

			// Create a hardware Direct3D 11 device
			hr = Dynamic_D3D11CreateDevice( NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, createDeviceFlags,
				fl, _countof(fl), D3D11_SDK_VERSION, &m_pd3dDevice, NULL, &m_pd3dImmediateContext );

			// Check if the hardware device supports Compute Shader 4.0
			D3D11_FEATURE_DATA_D3D10_X_HARDWARE_OPTIONS hwopts;
			m_pd3dDevice->CheckFeatureSupport(D3D11_FEATURE_D3D10_X_HARDWARE_OPTIONS, &hwopts, sizeof(hwopts));
			if( !hwopts.ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x ) {
				SAFE_RELEASE( m_pd3dImmediateContext );
				SAFE_RELEASE( m_pd3dDevice );

				int result = MessageBoxA(0, "This program needs to use the Direct3D 11 reference device.  This device implements the entire Direct3D 11 feature set, but runs very slowly.  Do you wish to continue?", "Compute Shader Sort", MB_ICONINFORMATION | MB_YESNO);
				if( result == IDNO )
					return false;//E_FAIL;
		        
				// Create a reference device if hardware is not available
				hr = Dynamic_D3D11CreateDevice( NULL, D3D_DRIVER_TYPE_REFERENCE, NULL, createDeviceFlags,
					fl, _countof(fl), D3D11_SDK_VERSION, &m_pd3dDevice, NULL, &m_pd3dImmediateContext );
				if( FAILED( hr ) )
					return (hr==S_OK);

				printf("Using Direct3D 11 Reference Device\n");
			}

			bool returnVal = (hr==S_OK);


			return returnVal;

		} // InitComputeShaderDevice

	}; // DX11SupportHelper

} // namespace BTAcceleratedSoftBody


#endif // #ifndef BT_DIRECT_COMPUTE_SUPPORT_HPP