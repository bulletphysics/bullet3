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


//#define PATH "..\\..\\AdlPrimitives\\Fill\\FillKernels"
#define PATH "..\\..\\opencl\\primitives\\AdlPrimitives\\Fill\\FillKernels"
#define KERNEL0 "FillIntKernel"
#define KERNEL1 "FillInt2Kernel"
#define KERNEL2 "FillInt4Kernel"

#include <AdlPrimitives/Fill/FillKernelsCL.h>
#include <AdlPrimitives/Fill/FillKernelsDX11.h>


template<DeviceType TYPE>
typename Fill<TYPE>::Data* Fill<TYPE>::allocate( const Device* device )
{
	ADLASSERT( TYPE == device->m_type );

	const char* src[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
		{fillKernelsCL, fillKernelsDX11};
#else
		{0,0};
#endif

	Data* data = new Data;
	data->m_device = device;
	data->m_fillIntKernel = device->getKernel( PATH, KERNEL0, 0, src[TYPE] );
	data->m_fillInt2Kernel = device->getKernel( PATH, KERNEL1, 0, src[TYPE] );
	data->m_fillInt4Kernel = device->getKernel( PATH, KERNEL2, 0, src[TYPE] );
	data->m_constBuffer = new Buffer<ConstData>( device, 1, BufferBase::BUFFER_CONST );

	return data;
}

template<DeviceType TYPE>
void Fill<TYPE>::deallocate( Data* data )
{
	delete data->m_constBuffer;
	delete data;
}

template<DeviceType TYPE>
void Fill<TYPE>::execute(Data* data, Buffer<int>& src, const int& value, int n, int offset)
{
	ADLASSERT( n>0 );
	ConstData constBuffer;
	{
		constBuffer.m_offset = offset;
		constBuffer.m_n = n;
		constBuffer.m_data = make_int4( value );
	}

	{
		BufferInfo bInfo[] = { BufferInfo( &src ) };

		Launcher launcher( data->m_device, data->m_fillIntKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( *data->m_constBuffer, constBuffer );
		launcher.launch1D( n );
	}
}

template<DeviceType TYPE>
void Fill<TYPE>::execute(Data* data, Buffer<int2>& src, const int2& value, int n, int offset)
{
	ADLASSERT( n>0 );
	ConstData constBuffer;
	{
		constBuffer.m_offset = offset;
		constBuffer.m_n = n;
		constBuffer.m_data = make_int4( value.x, value.y, 0, 0 );
	}

	{
		BufferInfo bInfo[] = { BufferInfo( &src ) };

		Launcher launcher( data->m_device, data->m_fillInt2Kernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( *data->m_constBuffer, constBuffer );
		launcher.launch1D( n );
	}
}

template<DeviceType TYPE>
void Fill<TYPE>::execute(Data* data, Buffer<int4>& src, const int4& value, int n, int offset)
{
	ADLASSERT( n>0 );
	ConstData constBuffer;
	{
		constBuffer.m_offset = offset;
		constBuffer.m_n = n;
		constBuffer.m_data = value;
	}

	{
		BufferInfo bInfo[] = { BufferInfo( &src ) };

		Launcher launcher( data->m_device, data->m_fillInt4Kernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( *data->m_constBuffer, constBuffer );
		launcher.launch1D( n );
	}
}

#undef PATH
#undef KERNEL0
#undef KERNEL1
#undef KERNEL2

