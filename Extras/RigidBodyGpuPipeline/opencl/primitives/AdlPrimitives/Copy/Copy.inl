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



#define PATH "..\\..\\opencl\\primitives\\AdlPrimitives\\Copy\\CopyKernels"
#define KERNEL0 "Copy1F4Kernel"
#define KERNEL1 "Copy2F4Kernel"
#define KERNEL2 "Copy4F4Kernel"
#define KERNEL3 "CopyF1Kernel"
#define KERNEL4 "CopyF2Kernel"

#include <AdlPrimitives/Copy/CopyKernelsCL.h>
#include <AdlPrimitives/Copy/CopyKernelsDX11.h>


template<DeviceType TYPE>
typename Copy<TYPE>::Data* Copy<TYPE>::allocate( const Device* device )
{
	ADLASSERT( TYPE == device->m_type );


	const char* src[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
	{copyKernelsCL, copyKernelsDX11};
//	ADLASSERT(0);
#else
	{0,0};
#endif	

	Data* data = new Data;
	data->m_device = device;
	data->m_copy1F4Kernel = device->getKernel( PATH, KERNEL0, 0, src[TYPE] );
	data->m_copy2F4Kernel = device->getKernel( PATH, KERNEL1, 0, src[TYPE] );
	data->m_copy4F4Kernel = device->getKernel( PATH, KERNEL2, 0, src[TYPE] );
	data->m_copyF1Kernel = device->getKernel( PATH, KERNEL3, 0, src[TYPE] );
	data->m_copyF2Kernel = device->getKernel( PATH, KERNEL4, 0, src[TYPE] );
	data->m_constBuffer = new Buffer<int4>( device, 1, BufferBase::BUFFER_CONST );

	return data;
}

template<DeviceType TYPE>
void Copy<TYPE>::deallocate( Data* data )
{
	delete data->m_constBuffer;
	delete data;
}

template<DeviceType TYPE>
void Copy<TYPE>::execute( Data* data, Buffer<float4>& dst, Buffer<float4>& src, int n, Option option )
{
	ADLASSERT( TYPE == dst.getType() );
	ADLASSERT( TYPE == src.getType() );

	int4 constBuffer;
	constBuffer.x = n;

	switch (option)
	{
	case PER_WI_1:
		{
			BufferInfo bInfo[] = { BufferInfo( &dst ), BufferInfo( &src, true ) };

			Launcher launcher( data->m_device, data->m_copy1F4Kernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer, constBuffer );
			launcher.launch1D( n/1 );
		}
		break;
	case PER_WI_2:
		{
			ADLASSERT( n%2 == 0 );
			BufferInfo bInfo[] = { BufferInfo( &dst ), BufferInfo( &src, true ) };

			Launcher launcher( data->m_device, data->m_copy2F4Kernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer, constBuffer );
			launcher.launch1D( n/2 );
		}
		break;
	case PER_WI_4:
		{
			ADLASSERT( n%4 == 0 );
			BufferInfo bInfo[] = { BufferInfo( &dst ), BufferInfo( &src, true ) };

			Launcher launcher( data->m_device, data->m_copy4F4Kernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer, constBuffer );
			launcher.launch1D( n/4 );
		}
		break;
	default:
		ADLASSERT(0);
		break;
	};
}

template<DeviceType TYPE>
void Copy<TYPE>::execute( Data* data, Buffer<float2>& dst, Buffer<float2>& src, int n )
{
	ADLASSERT( TYPE == dst.getType() );
	ADLASSERT( TYPE == src.getType() );

	int4 constBuffer;
	constBuffer.x = n;

	BufferInfo bInfo[] = { BufferInfo( &dst ), BufferInfo( &src, true ) };

	Launcher launcher( data->m_device, data->m_copyF2Kernel );
	launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
	launcher.setConst( *data->m_constBuffer, constBuffer );
	launcher.launch1D( n/1 );
}

template<DeviceType TYPE>
void Copy<TYPE>::execute( Data* data, Buffer<float>& dst, Buffer<float>& src, int n )
{
	ADLASSERT( TYPE == dst.getType() );
	ADLASSERT( TYPE == src.getType() );

	int4 constBuffer;
	constBuffer.x = n;

	BufferInfo bInfo[] = { BufferInfo( &dst ), BufferInfo( &src, true ) };

	Launcher launcher( data->m_device, data->m_copyF1Kernel );
	launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
	launcher.setConst( *data->m_constBuffer, constBuffer );
	launcher.launch1D( n/1 );
}


#undef PATH
#undef KERNEL0
#undef KERNEL1
#undef KERNEL2
#undef KERNEL3
#undef KERNEL4
