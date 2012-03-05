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

#define PATH "..\\..\\opencl\\primitives\\AdlPrimitives\\Search\\BoundSearchKernels"
#define KERNEL0 "SearchSortDataLowerKernel"
#define KERNEL1 "SearchSortDataUpperKernel"
#define KERNEL2 "SubtractKernel"

#include <AdlPrimitives/Search/BoundSearchKernelsCL.h>
#include <AdlPrimitives/Search/BoundSearchKernelsDX11.h>

template<DeviceType TYPE>
typename BoundSearch<TYPE>::Data* BoundSearch<TYPE>::allocate(const Device* device, int maxSize)
{
	ADLASSERT( TYPE == device->m_type );

	const char* src[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
		{boundSearchKernelsCL, boundSearchKernelsDX11};
#else
		{0,0};
#endif

	Data* data = new Data;

	data->m_device = device;
	data->m_lowerSortDataKernel = device->getKernel( PATH, KERNEL0, 0, src[TYPE] );
	data->m_upperSortDataKernel = device->getKernel( PATH, KERNEL1, 0, src[TYPE] );
	data->m_constBuffer = new Buffer<int4>( device, 1, BufferBase::BUFFER_CONST );
	if( maxSize )
	{
		data->m_subtractKernel = device->getKernel( PATH, KERNEL2, 0, src[TYPE] );
	}
	data->m_lower = (maxSize == 0)? 0: new Buffer<u32>( device, maxSize );
	data->m_upper = (maxSize == 0)? 0: new Buffer<u32>( device, maxSize );
	data->m_fillData = (maxSize == 0)? 0: Fill<TYPE>::allocate( device );

	return data;
}

template<DeviceType TYPE>
void BoundSearch<TYPE>::deallocate(Data* data)
{
	delete data->m_constBuffer;
	if( data->m_lower ) delete data->m_lower;
	if( data->m_upper ) delete data->m_upper;
	if( data->m_fillData ) Fill<TYPE>::deallocate( data->m_fillData );
	delete data;
}

template<DeviceType TYPE>
void BoundSearch<TYPE>::execute(Data* data, Buffer<SortData>& src, u32 nSrc, Buffer<u32>& dst, u32 nDst, Option option )
{
	int4 constBuffer;
	constBuffer.x = nSrc;
	constBuffer.y = nDst;

	Buffer<SortData>* srcNative = BufferUtils::map<TYPE, true>( data->m_device, &src );
	Buffer<u32>* dstNative = BufferUtils::map<TYPE, false>( data->m_device, &dst );

	if( option == BOUND_LOWER )
	{
		BufferInfo bInfo[] = { BufferInfo( srcNative, true ), BufferInfo( dstNative ) };

		Launcher launcher( data->m_device, data->m_lowerSortDataKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( *data->m_constBuffer, constBuffer );
		launcher.launch1D( nSrc, 64 );
	}
	else if( option == BOUND_UPPER )
	{
		BufferInfo bInfo[] = { BufferInfo( srcNative, true ), BufferInfo( dstNative ) };

		Launcher launcher( data->m_device, data->m_upperSortDataKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( *data->m_constBuffer, constBuffer );
		launcher.launch1D( nSrc+1, 64 );
	}
	else if( option == COUNT )
	{
		ADLASSERT( data->m_lower );
		ADLASSERT( data->m_upper );
		ADLASSERT( data->m_lower->getSize() <= (int)nDst );
		ADLASSERT( data->m_upper->getSize() <= (int)nDst );

		int zero = 0;
		Fill<TYPE>::execute( data->m_fillData, (Buffer<int>&)*data->m_lower, zero, nDst );
		Fill<TYPE>::execute( data->m_fillData, (Buffer<int>&)*data->m_upper, zero, nDst );

		execute( data, src, nSrc, *data->m_lower, nDst, BOUND_LOWER );
		execute( data, src, nSrc, *data->m_upper, nDst, BOUND_UPPER );

		{
			BufferInfo bInfo[] = { BufferInfo( data->m_upper, true ), BufferInfo( data->m_lower, true ), BufferInfo( dstNative ) };

			Launcher launcher( data->m_device, data->m_subtractKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer, constBuffer );
			launcher.launch1D( nDst, 64 );
		}
	}
	else
	{
		ADLASSERT( 0 );
	}

	BufferUtils::unmap<false>( srcNative, &src );
	BufferUtils::unmap<true>( dstNative, &dst );
}


#undef PATH
#undef KERNEL0
#undef KERNEL1
#undef KERNEL2

