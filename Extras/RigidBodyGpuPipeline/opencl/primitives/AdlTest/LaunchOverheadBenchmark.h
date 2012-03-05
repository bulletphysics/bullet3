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


#include <AdlPrimitives/Copy/Copy.h>



template<DeviceType TYPE>
__inline
void copyTest( Device* device )
{
	int size = 65*1024;

	Buffer<float4> buf0( device, size );
	Buffer<float4> buf1( device, size );

	Stopwatch sw( device );
	
	Copy<TYPE>::Data* data = Copy<TYPE>::allocate( device );

	for(int i=0; i<10; i++)
		Copy<TYPE>::execute( data, buf1, buf0, size, CopyBase::PER_WI_1 );
	DeviceUtils::waitForCompletion( device );

	{
		const int nTests = 12;

		float t[nTests];

		for(int ii=0; ii<nTests; ii++)
		{
			int iter = 1<<ii;

			DeviceUtils::waitForCompletion( device );
			sw.start();
			for(int i=0; i<iter; i++)
			{
				Copy<TYPE>::execute( data, buf1, buf0, size, CopyBase::PER_WI_1 );
			}
			DeviceUtils::waitForCompletion( device );
			sw.stop();

			t[ii] = sw.getMs()/(float)iter;
		}

		for(int ii=0; ii<nTests; ii++)
		{
			printf("%d:	%3.4fms	(%3.2fGB/s)\n", (1<<ii), t[ii], size*16*2/1024.f/1024.f/t[ii]);
		}
		printf("\n");

	}
	
	Copy<TYPE>::deallocate( data );
}

void launchOverheadBenchmark()
{
	printf("LaunchOverheadBenchmark\n");


	Device* ddcl;
#if defined(ADL_ENABLE_DX11)
	Device* dddx;
#endif
	{
		DeviceUtils::Config cfg;
		ddcl = DeviceUtils::allocate( TYPE_CL, cfg );
#if defined(ADL_ENABLE_DX11)
		dddx = DeviceUtils::allocate( TYPE_DX11, cfg );
#endif
	}

	{
		printf("CL\n");
		copyTest<TYPE_CL>( ddcl );
	}
#ifdef ADL_ENABLE_DX11
	{
		printf("DX11\n");
		copyTest<TYPE_DX11>( dddx );
	}
#endif


}


//1, 2, 4, 8, 16, 32, 64, 128, 256, 

