/*
		2011 Takahiro Harada
*/

#define PATH "..\\..\\opencl\\primitives\\AdlPrimitives\\Sort\\RadixSort32Kernels"
#define RADIXSORT32_KERNEL0 "StreamCountKernel"
#define RADIXSORT32_KERNEL1 "PrefixScanKernel"
#define RADIXSORT32_KERNEL2 "SortAndScatterKernel"
#define RADIXSORT32_KERNEL3 "SortAndScatterKeyValueKernel"
#define RADIXSORT32_KERNEL4 "SortAndScatterSortDataKernel"
#define RADIXSORT32_KERNEL5 "StreamCountSortDataKernel"

#include "RadixSort32KernelsCL.h"
#include "RadixSort32KernelsDX11.h"

//	todo. Shader compiler (2010JuneSDK) doesn't allow me to place Barriers in SortAndScatterKernel... 
//	So it only works on a GPU with 64 wide SIMD. 

template<DeviceType TYPE>
typename RadixSort32<TYPE>::Data* RadixSort32<TYPE>::allocate( const Device* device, int maxSize )
{
	ADLASSERT( TYPE == device->m_type );

	const char* src[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
	{radixSort32KernelsCL, radixSort32KernelsDX11};
#else
	{0,0};
#endif
	
	Data* data = new Data;
	data->m_device = device;
	data->m_maxSize = maxSize;
	data->m_streamCountKernel = device->getKernel( PATH, RADIXSORT32_KERNEL0, 0, src[TYPE] );
	data->m_streamCountSortDataKernel = device->getKernel( PATH, RADIXSORT32_KERNEL5, 0, src[TYPE] );

	

	data->m_prefixScanKernel = device->getKernel( PATH, RADIXSORT32_KERNEL1, 0, src[TYPE] );
	data->m_sortAndScatterKernel = device->getKernel( PATH, RADIXSORT32_KERNEL2, 0, src[TYPE] );
	data->m_sortAndScatterKeyValueKernel = device->getKernel( PATH, RADIXSORT32_KERNEL3, 0, src[TYPE] );
	data->m_sortAndScatterSortDataKernel = device->getKernel( PATH, RADIXSORT32_KERNEL4, 0, src[TYPE] );

	int wtf = NUM_WGS*(1<<BITS_PER_PASS);

	data->m_workBuffer0 = new Buffer<u32>( device, maxSize );
	data->m_workBuffer1 = new Buffer<u32>( device , wtf );
	data->m_workBuffer2 = new Buffer<u32>( device, maxSize );
	data->m_workBuffer3 = new Buffer<SortData>(device,maxSize);


	for(int i=0; i<32/BITS_PER_PASS; i++)
		data->m_constBuffer[i] = new Buffer<ConstData>( device, 1, BufferBase::BUFFER_CONST );

	data->m_copyData = Copy<TYPE>::allocate( device );

	return data;
}

template<DeviceType TYPE>
void RadixSort32<TYPE>::deallocate( Data* data )
{
	delete data->m_workBuffer0;
	delete data->m_workBuffer1;
	delete data->m_workBuffer2;
	delete data->m_workBuffer3;

	for(int i=0; i<32/BITS_PER_PASS; i++)
		delete data->m_constBuffer[i];

	Copy<TYPE>::deallocate( data->m_copyData );

	delete data;
}

template<DeviceType TYPE>
void RadixSort32<TYPE>::execute(Data* data, Buffer<u32>& inout, int n, int sortBits /* = 32 */ )
{
	ADLASSERT( n%DATA_ALIGNMENT == 0 );
	ADLASSERT( n <= data->m_maxSize );
//	ADLASSERT( ELEMENTS_PER_WORK_ITEM == 4 );
	ADLASSERT( BITS_PER_PASS == 4 );
	ADLASSERT( WG_SIZE == 64 );
	ADLASSERT( (sortBits&0x3) == 0 );

	Buffer<u32>* src = &inout;
	Buffer<u32>* dst = data->m_workBuffer0;
	Buffer<u32>* histogramBuffer = data->m_workBuffer1;

	int nWGs = NUM_WGS;
	ConstData cdata;
	{
		int nBlocks = (n+ELEMENTS_PER_WORK_ITEM*WG_SIZE-1)/(ELEMENTS_PER_WORK_ITEM*WG_SIZE);

		cdata.m_n = n;
		cdata.m_nWGs = NUM_WGS;
		cdata.m_startBit = 0;
		cdata.m_nBlocksPerWG = (nBlocks + cdata.m_nWGs - 1)/cdata.m_nWGs;

		if( nBlocks < NUM_WGS )
		{
			cdata.m_nBlocksPerWG = 1;
			nWGs = nBlocks;
		}
	}

	for(int ib=0; ib<sortBits; ib+=4)
	{
		cdata.m_startBit = ib;
		{
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( histogramBuffer ) };
			Launcher launcher( data->m_device, data->m_streamCountKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( NUM_WGS*WG_SIZE, WG_SIZE );
		}
		{//	prefix scan group histogram
			BufferInfo bInfo[] = { BufferInfo( histogramBuffer ) };
			Launcher launcher( data->m_device, data->m_prefixScanKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( 128, 128 );
		}
		{//	local sort and distribute
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( histogramBuffer, true ), BufferInfo( dst ) };
			Launcher launcher( data->m_device, data->m_sortAndScatterKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( nWGs*WG_SIZE, WG_SIZE );
		}
		swap2( src, dst );
	}

	if( src != &inout )
	{
		Copy<TYPE>::execute( data->m_copyData, (Buffer<float>&)inout, (Buffer<float>&)*src, n );
	}
}

template<DeviceType TYPE>
void RadixSort32<TYPE>::execute(Data* data, Buffer<u32>& in, Buffer<u32>& out, int n, int sortBits /* = 32 */ )
{
	ADLASSERT( n%DATA_ALIGNMENT == 0 );
	ADLASSERT( n <= data->m_maxSize );
//	ADLASSERT( ELEMENTS_PER_WORK_ITEM == 4 );
	ADLASSERT( BITS_PER_PASS == 4 );
	ADLASSERT( WG_SIZE == 64 );
	ADLASSERT( (sortBits&0x3) == 0 );

	Buffer<u32>* src = &in;
	Buffer<u32>* dst = data->m_workBuffer0;
	Buffer<u32>* histogramBuffer = data->m_workBuffer1;

	int nWGs = NUM_WGS;
	ConstData cdata;
	{
		int nBlocks = (n+ELEMENTS_PER_WORK_ITEM*WG_SIZE-1)/(ELEMENTS_PER_WORK_ITEM*WG_SIZE);
		cdata.m_n = n;
		cdata.m_nWGs = NUM_WGS;
		cdata.m_startBit = 0;
		cdata.m_nBlocksPerWG = (nBlocks + cdata.m_nWGs - 1)/cdata.m_nWGs;
		if( nBlocks < NUM_WGS )
		{
			cdata.m_nBlocksPerWG = 1;
			nWGs = nBlocks;
		}
	}

	if( sortBits == 4 ) dst = &out;

	for(int ib=0; ib<sortBits; ib+=4)
	{
		if( ib==4 )
		{
			dst = &out;
		}

		cdata.m_startBit = ib;
		{
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( histogramBuffer ) };
			Launcher launcher( data->m_device, data->m_streamCountKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( NUM_WGS*WG_SIZE, WG_SIZE );
		}
		{//	prefix scan group histogram
			BufferInfo bInfo[] = { BufferInfo( histogramBuffer ) };
			Launcher launcher( data->m_device, data->m_prefixScanKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( 128, 128 );
		}
		{//	local sort and distribute
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( histogramBuffer, true ), BufferInfo( dst ) };
			Launcher launcher( data->m_device, data->m_sortAndScatterKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( nWGs*WG_SIZE, WG_SIZE );
		}
		swap2( src, dst );
	}
}

template<DeviceType TYPE>
void RadixSort32<TYPE>::execute(Data* data, Buffer<u32>& keysIn, Buffer<u32>& keysOut, Buffer<u32>& valuesIn, Buffer<u32>& valuesOut, int n, int sortBits /* = 32 */)
{
	ADLASSERT( n%DATA_ALIGNMENT == 0 );
	ADLASSERT( n <= data->m_maxSize );
//	ADLASSERT( ELEMENTS_PER_WORK_ITEM == 4 );
	ADLASSERT( BITS_PER_PASS == 4 );
	ADLASSERT( WG_SIZE == 64 );
	ADLASSERT( (sortBits&0x3) == 0 );

	Buffer<u32>* src = &keysIn;
	Buffer<u32>* srcVal = &valuesIn;
	Buffer<u32>* dst = data->m_workBuffer0;
	Buffer<u32>* dstVal = data->m_workBuffer2;
	Buffer<u32>* histogramBuffer = data->m_workBuffer1;

	int nWGs = NUM_WGS;
	ConstData cdata;
	{
		int nBlocks = (n+ELEMENTS_PER_WORK_ITEM*WG_SIZE-1)/(ELEMENTS_PER_WORK_ITEM*WG_SIZE);
		cdata.m_n = n;
		cdata.m_nWGs = NUM_WGS;
		cdata.m_startBit = 0;
		cdata.m_nBlocksPerWG = (nBlocks + cdata.m_nWGs - 1)/cdata.m_nWGs;
		if( nBlocks < NUM_WGS )
		{
			cdata.m_nBlocksPerWG = 1;
			nWGs = nBlocks;
		}
	}

	if( sortBits == 4 ) 
	{
		dst = &keysOut;
		dstVal = &valuesOut;
	}

	for(int ib=0; ib<sortBits; ib+=4)
	{
		if( ib==4 )
		{
			dst = &keysOut;
			dstVal = &valuesOut;
		}

		cdata.m_startBit = ib;
		{
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( histogramBuffer ) };
			Launcher launcher( data->m_device, data->m_streamCountKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( NUM_WGS*WG_SIZE, WG_SIZE );
		}
		{//	prefix scan group histogram
			BufferInfo bInfo[] = { BufferInfo( histogramBuffer ) };
			Launcher launcher( data->m_device, data->m_prefixScanKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( 128, 128 );
		}
		{//	local sort and distribute
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( srcVal, true ), BufferInfo( histogramBuffer, true ), BufferInfo( dst ), BufferInfo( dstVal ) };
			Launcher launcher( data->m_device, data->m_sortAndScatterKeyValueKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( nWGs*WG_SIZE, WG_SIZE );
		}
		swap2( src, dst );
		swap2( srcVal, dstVal );
	}
}

template<DeviceType TYPE>
void RadixSort32<TYPE>::execute(Data* data, Buffer<SortData>& keyValuesInOut, int n, int sortBits /* = 32 */)
{
	ADLASSERT( n%DATA_ALIGNMENT == 0 );
	ADLASSERT( n <= data->m_maxSize );
//	ADLASSERT( ELEMENTS_PER_WORK_ITEM == 4 );
	ADLASSERT( BITS_PER_PASS == 4 );
	ADLASSERT( WG_SIZE == 64 );
	ADLASSERT( (sortBits&0x3) == 0 );

	Buffer<SortData>* src = &keyValuesInOut;
	Buffer<SortData>* dst = data->m_workBuffer3;

	Buffer<u32>* histogramBuffer = data->m_workBuffer1;

	int nWGs = NUM_WGS;
	ConstData cdata;
	{
		int nBlocks = (n+ELEMENTS_PER_WORK_ITEM*WG_SIZE-1)/(ELEMENTS_PER_WORK_ITEM*WG_SIZE);
		cdata.m_n = n;
		cdata.m_nWGs = NUM_WGS;
		cdata.m_startBit = 0;
		cdata.m_nBlocksPerWG = (nBlocks + cdata.m_nWGs - 1)/cdata.m_nWGs;
		if( nBlocks < NUM_WGS )
		{
			cdata.m_nBlocksPerWG = 1;
			nWGs = nBlocks;
		}
	}

	int count=0;
	for(int ib=0; ib<sortBits; ib+=4)
	{
		cdata.m_startBit = ib;
		{
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( histogramBuffer ) };
			Launcher launcher( data->m_device, data->m_streamCountSortDataKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( NUM_WGS*WG_SIZE, WG_SIZE );
		}
		{//	prefix scan group histogram
			BufferInfo bInfo[] = { BufferInfo( histogramBuffer ) };
			Launcher launcher( data->m_device, data->m_prefixScanKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( 128, 128 );
		}
		{//	local sort and distribute
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( histogramBuffer, true ), BufferInfo( dst )};
			Launcher launcher( data->m_device, data->m_sortAndScatterSortDataKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[ib/4], cdata );
			launcher.launch1D( nWGs*WG_SIZE, WG_SIZE );
		}
		swap2( src, dst );
		count++;
	}
	
	if (count&1)
	{
		ADLASSERT(0);//need to copy from workbuffer to keyValuesInOut

	}
}
#undef PATH
#undef RADIXSORT32_KERNEL0
#undef RADIXSORT32_KERNEL1
#undef RADIXSORT32_KERNEL2
#undef RADIXSORT32_KERNEL3

