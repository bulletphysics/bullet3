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


#include <stdio.h>

#include <Adl/Adl.h>
#include <AdlPrimitives/Math/Math.h>

#include "UnitTests.h"
#include "RadixSortBenchmark.h"
#include "LaunchOverheadBenchmark.h"


#undef NUM_TESTS


struct ConstBuffer
{
	float4 m_a;
	float4 m_b;
	float4 m_c;
};

int main()
{
	if(0)
	{	//	radix sort test
		Device* deviceHost;
		Device* deviceGPU;
		{
			DeviceUtils::Config cfg;

		// Choose AMD or NVidia
#ifdef CL_PLATFORM_AMD
	cfg.m_vendor = DeviceUtils::Config::VD_AMD;
#endif

#ifdef CL_PLATFORM_INTEL
	cfg.m_vendor = DeviceUtils::Config::VD_INTEL;
#endif

#ifdef CL_PLATFORM_NVIDIA
	cfg.m_vendor = adl::DeviceUtils::Config::VD_NV;
#endif
			deviceGPU = DeviceUtils::allocate( TYPE_DX11, cfg );
			deviceHost = DeviceUtils::allocate( TYPE_HOST, cfg );
		}

		{
		int maxSize = 512*20;
		int size = maxSize;

		HostBuffer<SortData> buf0( deviceHost, maxSize );
		HostBuffer<SortData> buf1( deviceHost, maxSize );
		Buffer<SortData> buf2( deviceGPU, maxSize );

		RadixSort<TYPE_HOST>::Data* dataH = RadixSort<TYPE_HOST>::allocate( deviceHost, maxSize, RadixSortBase::SORT_STANDARD );
		RadixSort<TYPE_DX11>::Data* dataC = RadixSort<TYPE_DX11>::allocate( deviceGPU, maxSize, RadixSortBase::SORT_ADVANCED );

		{
			size = NEXTMULTIPLEOF( size, 512 );

			for(int i=0; i<size; i++) buf0[i] = SortData( getRandom(0,0xfff), i );
			buf2.write( buf0.m_ptr, size );
			DeviceUtils::waitForCompletion( deviceGPU );

			RadixSort<TYPE_HOST>::execute( dataH, buf0, size );
			RadixSort<TYPE_DX11>::execute( dataC, buf2, size );

			buf2.read( buf1.m_ptr, size );
			DeviceUtils::waitForCompletion( deviceGPU );
			for(int i=0; i<size; i++) ADLASSERT( buf0[i].m_value == buf1[i].m_value && buf0[i].m_key == buf1[i].m_key );
		}

		RadixSort<TYPE_HOST>::deallocate( dataH );
		RadixSort<TYPE_DX11>::deallocate( dataC );
		}

		DeviceUtils::deallocate( deviceHost );
		DeviceUtils::deallocate( deviceGPU );
	}

	if(0)
	{
		launchOverheadBenchmark();
	}

	if(0)
	{
		radixSortBenchmark<TYPE_DX11>();
	}

	if(0)
	{
		radixSortBenchmark<TYPE_CL>();
	}

	if(1)
	{
		runAllTest();
	}
	printf("End, press <enter>\n");
	getchar();
}

