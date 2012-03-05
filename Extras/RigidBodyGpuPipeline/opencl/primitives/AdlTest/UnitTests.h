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


#include <AdlPrimitives/Scan/PrefixScan.h>
#include <AdlPrimitives/Sort/RadixSort.h>
#include <AdlPrimitives/Sort/RadixSort32.h>
#include <AdlPrimitives/Search/BoundSearch.h>
#include <AdlPrimitives/Fill/Fill.h>
#include <AdlPrimitives/Copy/Copy.h>

#include <time.h>

using namespace adl;

#define NUM_TESTS 10

int g_nPassed = 0;
int g_nFailed = 0;
bool g_testFailed = 0;

//#define TEST_INIT bool g_testFailed = 0;
#define TEST_INIT g_testFailed = 0;
#define TEST_ASSERT(x) if( !(x) ){g_testFailed = 1;}
//#define TEST_ASSERT(x) if( !(x) ){g_testFailed = 1;ADLASSERT(x);}
#define TEST_REPORT(testName) printf("[%s] %s\n",(g_testFailed)?"X":"O", testName); if(g_testFailed) g_nFailed++; else g_nPassed++;

void memCpyTest( Device* deviceData )
{
	TEST_INIT;
	int maxSize = 64*1024;
	Buffer<u32> buff( deviceData, maxSize );

	u32* hostBuff = new u32[maxSize];

	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = getRandom( 1024, maxSize );

		for(int i=0; i<size; i++) hostBuff[i] = i;

		buff.write( hostBuff, size );

		DeviceUtils::waitForCompletion( deviceData );
		for(int i=0; i<size; i++) hostBuff[i] = 0;

		buff.read( hostBuff, size );

		DeviceUtils::waitForCompletion( deviceData );
		for(int i=0; i<size; i++) TEST_ASSERT( hostBuff[i] == i );
	}

	delete [] hostBuff;
	TEST_REPORT( "memCpyTest" );
}

void kernelTest( Device* deviceData )
{
	TEST_INIT;

	KernelManager* manager = new KernelManager();

	Kernel* kernel = manager->query(deviceData, ".\\Kernel", "VectorAddKernel" );

	{
		int size = 1024;
		Buffer<int> buf0( deviceData, size );
		Buffer<int> buf1( deviceData, size );
		Buffer<float4> cBuf( deviceData, 1, BufferBase::BUFFER_CONST );
		int* hostBuf0 = new int[size];
		int* hostBuf1 = new int[size];
		for(int i=0; i<size; i++) { hostBuf0[i] = i; hostBuf1[i] = 1; }
		buf0.write( hostBuf0, size );
		buf1.write( hostBuf1, size );
		DeviceUtils::waitForCompletion( deviceData );

		float4 constBuffer;
		constBuffer.x = (float)size;
		constBuffer.y = 2.f;
		constBuffer.z = 0.f;
		constBuffer.w = 0.f;
		{
			Launcher::BufferInfo bInfo[] = { Launcher::BufferInfo( (Buffer<float>*)&buf0 ), Launcher::BufferInfo( (Buffer<float>*)&buf1, true ) };

			Launcher launcher( deviceData, kernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( (Buffer<float4>&)cBuf, constBuffer );
			launcher.launch1D( size );

			buf0.read( hostBuf0, size );
			buf1.read( hostBuf1, size );
			DeviceUtils::waitForCompletion( deviceData );
		}

		for(int i=0; i<size; i++) { TEST_ASSERT( hostBuf0[i] == i+1+2 ); }

		delete [] hostBuf0;
		delete [] hostBuf1;
	}
	TEST_REPORT( "kernelTest" );
}

void stopwatchTest( Device* deviceData )
{
	{
		Stopwatch sw( deviceData );

		sw.start();
		Sleep(2);
		sw.split();
		Sleep(2);
		sw.stop();

		float t[2];
		sw.getMs( t, 2 );
	}
}

template<DeviceType type>
void scanTest( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;

	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<u32> buf0( deviceHost, maxSize );
	HostBuffer<u32> buf1( deviceHost, maxSize );
	Buffer<u32> buf2( deviceGPU, maxSize );
	Buffer<u32> buf3( deviceGPU, maxSize );

	PrefixScan<type>::Data* data0 = PrefixScan<type>::allocate( deviceGPU, maxSize );
	PrefixScan<TYPE_HOST>::Data* data1 = PrefixScan<TYPE_HOST>::allocate( deviceHost, maxSize );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize );

		for(int i=0; i<size; i++) buf0[i] = 1;
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		u32 sumHost, sumGPU;
		PrefixScan<TYPE_HOST>::execute( data1, buf0, buf1, size, &sumHost );
		PrefixScan<type>::execute( data0, buf2, buf3, size, &sumGPU );

		buf3.read( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
		TEST_ASSERT( sumHost == sumGPU );
		for(int i=0; i<size; i++) TEST_ASSERT( buf1[i] == buf0[i] );
	}

	PrefixScan<TYPE_HOST>::deallocate( data1 );
	PrefixScan<type>::deallocate( data0 );

	TEST_REPORT( "scanTest" );
}

template<DeviceType type, RadixSortBase::Option SORT_TYPE>
bool radixSortTest( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<SortData> buf0( deviceHost, maxSize );
	HostBuffer<SortData> buf1( deviceHost, maxSize );
	Buffer<SortData> buf2( deviceGPU, maxSize );

	RadixSort<TYPE_HOST>::Data* dataH = RadixSort<TYPE_HOST>::allocate( deviceHost, maxSize, RadixSortBase::SORT_SIMPLE );
	RadixSort<type>::Data* dataC = RadixSort<type>::allocate( deviceGPU, maxSize, SORT_TYPE );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize-512 );
		size = NEXTMULTIPLEOF( size, 512 );

		for(int i=0; i<size; i++) buf0[i] = SortData( getRandom(0,0xff), i );
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		RadixSort<TYPE_HOST>::execute( dataH, buf0, size );
		RadixSort<type>::execute( dataC, buf2, size );

		buf2.read( buf1.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
		for(int i=0; i<size; i++) TEST_ASSERT( buf0[i].m_value == buf1[i].m_value && buf0[i].m_key == buf1[i].m_key );
	}

	RadixSort<TYPE_HOST>::deallocate( dataH );
	RadixSort<type>::deallocate( dataC );

	return g_testFailed;
}

template<DeviceType type>
void radixSortSimpleTest( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	g_testFailed = radixSortTest<type, RadixSortBase::SORT_SIMPLE>(deviceGPU, deviceHost);
	TEST_REPORT( "radixSortSimpleTest" );
}

template<DeviceType type>
void radixSortStandardTest( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	g_testFailed = radixSortTest<type, RadixSortBase::SORT_STANDARD>(deviceGPU, deviceHost);
	TEST_REPORT( "radixSortStandardTest" );
}

template<DeviceType type>
void radixSortAdvancedTest( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	g_testFailed = radixSortTest<type, RadixSortBase::SORT_ADVANCED>(deviceGPU, deviceHost);
	TEST_REPORT( "radixSortAdvancedTest" );
}

template<DeviceType type>
void boundSearchTest( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;

	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;
	int bucketSize = 256;

	HostBuffer<SortData> buf0( deviceHost, maxSize );
	HostBuffer<u32> lowerH( deviceHost, maxSize );
	HostBuffer<u32> upperH( deviceHost, maxSize );

	Buffer<SortData> buf( deviceGPU, maxSize );
	Buffer<u32> lower( deviceGPU, maxSize );
	Buffer<u32> upper( deviceGPU, maxSize );

	BoundSearch<type>::Data* dataH = BoundSearch<type>::allocate( deviceGPU );
	RadixSort<TYPE_HOST>::Data* dataHSort = RadixSort<TYPE_HOST>::allocate( deviceHost, maxSize, RadixSortBase::SORT_SIMPLE );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize );
		for(int i=0; i<size; i++) buf0[i] = SortData( getRandom(0,bucketSize), i );
		RadixSort<TYPE_HOST>::execute( dataHSort, buf0, size );
		buf.write( buf0.m_ptr, size );
		{
			u32* host = new u32[size];
			for(int i=0; i<size; i++) host[i] = -1;
			lower.write( host, size );
			upper.write( host, size );
		}
		DeviceUtils::waitForCompletion( deviceGPU );

		BoundSearch<type>::execute( dataH, buf, size, lower, bucketSize, BoundSearchBase::BOUND_LOWER );
		BoundSearch<type>::execute( dataH, buf, size, upper, bucketSize, BoundSearchBase::BOUND_UPPER );

		lower.read( lowerH.m_ptr, bucketSize );
		upper.read( upperH.m_ptr, bucketSize );
		DeviceUtils::waitForCompletion( deviceGPU );
/*
		for(u32 i=1; i<(u32)bucketSize; i++)
		{
			for(u32 j=lowerH[i-1]; j<lowerH[i]; j++)
			{
				TEST_ASSERT( buf0[j].m_key < i );
			}
		}

		for(u32 i=0; i<(u32)bucketSize; i++)
		{
			int jMin = (i==0)?0:upperH[i-1];
			for(u32 j=jMin; j<upperH[i]; j++)
			{
				TEST_ASSERT( buf0[j].m_key <= i );
			}
		}
*/
		for(u32 i=0; i<(u32)bucketSize; i++)
		{
			for(u32 j=lowerH[i]; j<upperH[i]; j++)
			{
				if ( buf0[j].m_key != i )
				{
					printf("error %d != %d\n",buf0[j].m_key,i);
				}
				TEST_ASSERT( buf0[j].m_key == i );
			}
		}

	}
	
	BoundSearch<type>::deallocate( dataH );
	RadixSort<TYPE_HOST>::deallocate( dataHSort );

	TEST_REPORT( "boundSearchTest" );
}

template<DeviceType type>
void fillIntTest( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;

	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<int> buf0( deviceHost, maxSize );
	HostBuffer<int> buf1( deviceHost, maxSize );
	Buffer<int> buf2( deviceGPU, maxSize );

	Fill<TYPE_HOST>::Data* data0 = Fill<TYPE_HOST>::allocate( deviceHost );
	Fill<type>::Data* data1 = Fill<type>::allocate( deviceGPU );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize );
		for(int i=0; i<size; i++) buf0[i] = -1;
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		Fill<TYPE_HOST>::execute( data0, buf0, 12, size );
		Fill<type>::execute( data1, buf2, 12, size );

		buf2.read( buf1.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
		for(int i=0; i<size; i++) TEST_ASSERT( buf1[i] == buf0[i] );
	}

	Fill<TYPE_HOST>::deallocate( data0 );
	Fill<type>::deallocate( data1 );

	TEST_REPORT( "fillIntTest" );
}

template<DeviceType type>
void fillInt2Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;

	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<int2> buf0( deviceHost, maxSize );
	HostBuffer<int2> buf1( deviceHost, maxSize );
	Buffer<int2> buf2( deviceGPU, maxSize );

	Fill<TYPE_HOST>::Data* data0 = Fill<TYPE_HOST>::allocate( deviceHost );
	Fill<type>::Data* data1 = Fill<type>::allocate( deviceGPU );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize );
		for(int i=0; i<size; i++) buf0[i] = make_int2( -1, -1 );
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		Fill<TYPE_HOST>::execute( data0, buf0, make_int2( 12, 12 ), size );
		Fill<type>::execute( data1, buf2, make_int2( 12, 12 ), size );

		buf2.read( buf1.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
		for(int i=0; i<size; i++) TEST_ASSERT( buf1[i] == buf0[i] );
	}

	Fill<TYPE_HOST>::deallocate( data0 );
	Fill<type>::deallocate( data1 );

	TEST_REPORT( "fillInt2Test" );
}

template<DeviceType type>
void fillInt4Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;

	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<int4> buf0( deviceHost, maxSize );
	HostBuffer<int4> buf1( deviceHost, maxSize );
	Buffer<int4> buf2( deviceGPU, maxSize );

	Fill<TYPE_HOST>::Data* data0 = Fill<TYPE_HOST>::allocate( deviceHost );
	Fill<type>::Data* data1 = Fill<type>::allocate( deviceGPU );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize );
		for(int i=0; i<size; i++) buf0[i] = make_int4( -1 );
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		Fill<TYPE_HOST>::execute( data0, buf0, make_int4( 12 ), size );
		Fill<type>::execute( data1, buf2, make_int4( 12 ), size );

		buf2.read( buf1.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
		for(int i=0; i<size; i++) TEST_ASSERT( buf1[i] == buf0[i] );
	}

	Fill<TYPE_HOST>::deallocate( data0 );
	Fill<type>::deallocate( data1 );

	TEST_REPORT( "fillInt4Test" );
}


template<DeviceType type, CopyBase::Option OPTION>
bool CopyF4Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<float4> buf0( deviceHost, maxSize );
	HostBuffer<float4> buf1( deviceHost, maxSize );
	Buffer<float4> buf2( deviceGPU, maxSize );
	Buffer<float4> buf3( deviceGPU, maxSize );
	HostBuffer<float4> devResult( deviceHost, maxSize );

	Copy<TYPE_HOST>::Data* data0 = Copy<TYPE_HOST>::allocate( deviceHost );
	Copy<type>::Data* data1 = Copy<type>::allocate( deviceGPU );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize-4 );
		size = NEXTMULTIPLEOF( size, 4 );
		float r = 10000.f;
		for(int i=0; i<size; i++) buf0[i] = make_float4( getRandom( -r, r ), getRandom( -r, r ), getRandom( -r, r ), getRandom( -r, r ) );
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		Copy<TYPE_HOST>::execute( data0, buf1, buf0, size, OPTION );
		Copy<type>::execute( data1, buf3, buf2, size, OPTION );

		buf3.read( devResult.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
		for(int i=0; i<size; i++)
		{
			TEST_ASSERT( buf1[i] == devResult[i] );
			TEST_ASSERT( buf0[i] == devResult[i] );
		}
	}

	Copy<TYPE_HOST>::deallocate( data0 );
	Copy<type>::deallocate( data1 );

	return g_testFailed;
}

template<DeviceType type>
void Copy1F4Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	g_testFailed = CopyF4Test<type, CopyBase::PER_WI_1>( deviceGPU, deviceHost );
	TEST_REPORT( "Copy1F4Test" );
}

template<DeviceType type>
void Copy2F4Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	g_testFailed = CopyF4Test<type, CopyBase::PER_WI_2>( deviceGPU, deviceHost );
	TEST_REPORT( "Copy2F4Test" );
}

template<DeviceType type>
void Copy4F4Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	g_testFailed = CopyF4Test<type, CopyBase::PER_WI_4>( deviceGPU, deviceHost );
	TEST_REPORT( "Copy4F4Test" );
}


template<DeviceType type>
void CopyF1Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<float> buf0( deviceHost, maxSize );
	HostBuffer<float> buf1( deviceHost, maxSize );
	Buffer<float> buf2( deviceGPU, maxSize );
	Buffer<float> buf3( deviceGPU, maxSize );
	HostBuffer<float> devResult( deviceHost, maxSize );

	Copy<TYPE_HOST>::Data* data0 = Copy<TYPE_HOST>::allocate( deviceHost );
	Copy<type>::Data* data1 = Copy<type>::allocate( deviceGPU );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize-4 );
		size = NEXTMULTIPLEOF( size, 4 );
		float r = 10000.f;
		for(int i=0; i<size; i++) buf0[i] = getRandom( -r, r );
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		Copy<TYPE_HOST>::execute( data0, buf1, buf0, size );
		Copy<type>::execute( data1, buf3, buf2, size );

		buf3.read( devResult.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
		for(int i=0; i<size; i++)
		{
			TEST_ASSERT( buf1[i] == devResult[i] );
			TEST_ASSERT( buf0[i] == devResult[i] );
		}
	}

	Copy<TYPE_HOST>::deallocate( data0 );
	Copy<type>::deallocate( data1 );

	TEST_REPORT( "CopyF1Test" );
}

template<DeviceType type>
void CopyF2Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<float2> buf0( deviceHost, maxSize );
	HostBuffer<float2> buf1( deviceHost, maxSize );
	Buffer<float2> buf2( deviceGPU, maxSize );
	Buffer<float2> buf3( deviceGPU, maxSize );
	HostBuffer<float2> devResult( deviceHost, maxSize );

	Copy<TYPE_HOST>::Data* data0 = Copy<TYPE_HOST>::allocate( deviceHost );
	Copy<type>::Data* data1 = Copy<type>::allocate( deviceGPU );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize-4 );
		size = NEXTMULTIPLEOF( size, 4 );
		float r = 10000.f;
		for(int i=0; i<size; i++) buf0[i] = make_float2( getRandom( -r, r ), getRandom( -r, r ) );
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		Copy<TYPE_HOST>::execute( data0, buf1, buf0, size );
		Copy<type>::execute( data1, buf3, buf2, size );

		buf3.read( devResult.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
		for(int i=0; i<size; i++)
		{
			TEST_ASSERT( buf1[i] == devResult[i] );
			TEST_ASSERT( buf0[i] == devResult[i] );
		}
	}

	Copy<TYPE_HOST>::deallocate( data0 );
	Copy<type>::deallocate( data1 );

	TEST_REPORT( "CopyF2Test" );
}

template<DeviceType type>
void radixSort32Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;

	HostBuffer<u32> buf0( deviceHost, maxSize );
	HostBuffer<u32> buf1( deviceHost, maxSize );
	Buffer<u32> buf2( deviceGPU, maxSize );

	RadixSort32<TYPE_HOST>::Data* dataH = RadixSort32<TYPE_HOST>::allocate( deviceHost, maxSize );
	RadixSort32<type>::Data* dataC = RadixSort32<type>::allocate( deviceGPU, maxSize );

	int dx = maxSize/NUM_TESTS;
	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize-512 );
		size = NEXTMULTIPLEOF( size, 512 );

		for(int i=0; i<size; i++) buf0[i] = getRandom(0u,0xffffffffu);
		buf2.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		RadixSort32<TYPE_HOST>::execute( dataH, buf0, size, 32 );
		RadixSort32<type>::execute( dataC, buf2, size, 32 );

		buf2.read( buf1.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );
//		for(int i=0; i<size-1; i++) TEST_ASSERT( buf1[i] <= buf1[i+1] );
		for(int i=0; i<size; i++) TEST_ASSERT( buf0[i] == buf1[i] );
	}

	RadixSort32<TYPE_HOST>::deallocate( dataH );
	RadixSort32<type>::deallocate( dataC );

	TEST_REPORT( "RadixSort32Test" );
}

template<DeviceType type>
void radixSortKeyValue32Test( Device* deviceGPU, Device* deviceHost )
{
	TEST_INIT;
	ADLASSERT( type == deviceGPU->m_type );

	int maxSize = 1024*256;
	
	// Host buffers
	HostBuffer<u32> buf0( deviceHost, maxSize ); // Buffer for keys in host and will be sorted by host.
	HostBuffer<u32> buf1( deviceHost, maxSize ); // Buffer for keys in host and will be saved by device after sorting in device.  
	HostBuffer<u32> buf2( deviceHost, maxSize ); // Buffer for values in host. This buffer is paired with buf0.
	HostBuffer<u32> buf3( deviceHost, maxSize ); // Buffer for values in host and will be saved by device after sorting. It is paired with buf1.
	
	// Device buffers
	Buffer<u32> buf4( deviceGPU, maxSize ); // Buffer for input keys for device.
	Buffer<u32> buf5( deviceGPU, maxSize ); // Buffer for output keys from device and will be sorted by device. This key data will be saved to buf1 to be compared with a result(buf0) from host.
	Buffer<u32> buf6( deviceGPU, maxSize ); // Buffer for input values in device.
	Buffer<u32> buf7( deviceGPU, maxSize ); // Buffer for output values in device.

	RadixSort32<TYPE_HOST>::Data* dataH = RadixSort32<TYPE_HOST>::allocate( deviceHost, maxSize );
	RadixSort32<type>::Data* dataC = RadixSort32<type>::allocate( deviceGPU, maxSize );

	int dx = maxSize/NUM_TESTS;

	for(int iter=0; iter<NUM_TESTS; iter++)
	{
		int size = min2( 128+dx*iter, maxSize-512 );
		size = NEXTMULTIPLEOF( size, 512 );

		// keys
		seedRandom((int)time(NULL)/2);
		for(int i=0; i<size; i++) buf0[i] = getRandom(0u,0xffffffffu);
		buf4.write( buf0.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		// values
		seedRandom((int)time(NULL)/2);
		for(int i=0; i<size; i++) buf2[i] = getRandom(0u,0xffffffffu);
		buf6.write( buf2.m_ptr, size );
		DeviceUtils::waitForCompletion( deviceGPU );

		RadixSort32<TYPE_HOST>::execute( dataH, buf0, buf2, size, 32 );
		RadixSort32<type>::execute( dataC, buf4, buf5, buf6, buf7, size, 32 );
		buf5.read( buf1.m_ptr, size );
		buf7.read( buf3.m_ptr, size );

		DeviceUtils::waitForCompletion( deviceGPU );
		
		for(int i=0; i<size; i++) 
		{
			// Comparing keys. One is done by Host and the other is done by Device.
			TEST_ASSERT( buf0[i] == buf1[i] );

			// Comparing values. One is done by Host and the other is done by Device.
			TEST_ASSERT( buf2[i] == buf3[i] );
		}
	}

	RadixSort32<TYPE_HOST>::deallocate( dataH );
	RadixSort32<type>::deallocate( dataC );

	TEST_REPORT( "RadixSortKeyValue32Test" );
}

#if defined(ADL_ENABLE_DX11)
	#define RUN_GPU( func ) func(ddcl); func(dddx);
	#define RUN_GPU_TEMPLATE( func ) func<TYPE_CL>( ddcl, ddhost ); func<TYPE_DX11>( dddx, ddhost );
	#define RUN_CL_TEMPLATE( func ) func<TYPE_CL>( ddcl, ddhost );
#else
	#define RUN_GPU( func ) func(ddcl);
	#define RUN_GPU_TEMPLATE( func ) func<TYPE_CL>( ddcl, ddhost ); 
#endif
#define RUN_ALL( func ) RUN_GPU( func ); func(ddhost);

void runAllTest()
{
	g_nPassed = 0;
	g_nFailed = 0;

	
	Device* ddcl;
	Device* ddhost;
#if defined(ADL_ENABLE_DX11)
	Device* dddx;
#endif

	{
		DeviceUtils::Config cfg;

				// Choose AMD or NVidia
#ifdef CL_PLATFORM_AMD
		cfg.m_vendor = adl::DeviceUtils::Config::VD_AMD;
#endif

#ifdef CL_PLATFORM_INTEL
		cfg.m_vendor = adl::DeviceUtils::Config::VD_INTEL;
		cfg.m_type = DeviceUtils::Config::DEVICE_CPU;
#endif
		

#ifdef CL_PLATFORM_NVIDIA
		cfg.m_vendor = adl::DeviceUtils::Config::VD_NV;
#endif


		ddcl = DeviceUtils::allocate( TYPE_CL, cfg );
		ddhost = DeviceUtils::allocate( TYPE_HOST, cfg );
//		cfg.m_type = DeviceUtils::Config::DEVICE_GPU;
#if defined(ADL_ENABLE_DX11)
		dddx = DeviceUtils::allocate( TYPE_DX11, cfg );
#endif
	}

	{
		char name[128];
		ddcl->getDeviceName( name );
		printf("CL: %s\n", name);
#ifdef ADL_ENABLE_DX11
		dddx->getDeviceName( name );
		printf("DX11: %s\n", name);
#endif
	}

	RUN_GPU_TEMPLATE( radixSort32Test );
	RUN_GPU_TEMPLATE( radixSortKeyValue32Test );

	if (1)
	{
		RUN_GPU_TEMPLATE( CopyF1Test );
		RUN_GPU_TEMPLATE( CopyF2Test );

		boundSearchTest<TYPE_HOST>( ddhost, ddhost );
//		fillTest<TYPE_HOST>( ddhost, ddhost );
//		fillTest<TYPE_CL>( ddcl, ddhost );


	

		RUN_GPU_TEMPLATE( boundSearchTest );

		RUN_GPU_TEMPLATE( fillIntTest );
		RUN_GPU_TEMPLATE( fillInt2Test );
		RUN_GPU_TEMPLATE( fillInt4Test );

		RUN_ALL( stopwatchTest );
		RUN_ALL( memCpyTest );
//		RUN_GPU( kernelTest );
		RUN_GPU_TEMPLATE( scanTest );
		RUN_GPU_TEMPLATE( radixSortSimpleTest );

		RUN_GPU_TEMPLATE( radixSortStandardTest );

		RUN_GPU_TEMPLATE( radixSort32Test );
		
//		RUN_GPU_TEMPLATE( boundSearchTest );
		RUN_GPU_TEMPLATE( Copy1F4Test );
		RUN_GPU_TEMPLATE( Copy2F4Test );
		RUN_GPU_TEMPLATE( Copy4F4Test );
	}

	DeviceUtils::deallocate( ddcl );
	DeviceUtils::deallocate( ddhost );
#if defined(ADL_ENABLE_DX11)
	DeviceUtils::deallocate( dddx );
#endif

	printf("=========\n%d Passed\n%d Failed\n", g_nPassed, g_nFailed);


}