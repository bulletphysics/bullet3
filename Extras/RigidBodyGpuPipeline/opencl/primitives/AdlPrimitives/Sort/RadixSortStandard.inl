/*
		2011 Takahiro Harada
*/

#define PATH "..\\..\\opencl\\primitives\\AdlPrimitives\\Sort\\RadixSortStandardKernels"
#define KERNEL0 "LocalSortKernel"
#define KERNEL1 "ScatterKernel"
#define KERNEL2 "CopyKernel"

#include <AdlPrimitives/Sort/RadixSortStandardKernelsCL.h>
#include <AdlPrimitives/Sort/RadixSortStandardKernelsDX11.h>

template<DeviceType type>
class RadixSortStandard : public RadixSortBase
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		enum
		{
			WG_SIZE = 128,
			NUM_PER_WI = 4,

			BITS_PER_PASS = 4,
		};

		struct Data : public RadixSort<type>::Data
		{
			Kernel* m_localSortKernel;
			Kernel* m_scatterKernel;
			Kernel* m_copyKernel;

			Buffer<u32>* m_workBuffer0;
			Buffer<u32>* m_workBuffer1;
			Buffer<u32>* m_workBuffer2;
			Buffer<SortData>* m_workBuffer3;
			Buffer<int4>* m_constBuffer[32/BITS_PER_PASS];
		};
		

		static
		Data* allocate(const Device* deviceData, int maxSize, Option option = SORT_NORMAL);

		static
		void deallocate(void* data);

		static
		void execute(void* data, Buffer<SortData>& inout, int n, int sortBits);
};

template<DeviceType type>
typename RadixSortStandard<type>::Data* RadixSortStandard<type>::allocate(const Device* deviceData, int maxSize, Option option)
{
	ADLASSERT( type == deviceData->m_type );

	u32 maxNumGroups = (maxSize+WG_SIZE*NUM_PER_WI-1)/(WG_SIZE*NUM_PER_WI);

	const char* src[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
	{radixSortStandardKernelsCL,radixSortStandardKernelsDX11};
//	ADLASSERT(0);
#else
	{0,0};
#endif	

	Data* data = new Data;
	data->m_option = option;
	data->m_deviceData = deviceData;

	data->m_localSortKernel = deviceData->getKernel( PATH, KERNEL0, 0, src[type] );
	data->m_scatterKernel = deviceData->getKernel( PATH, KERNEL1, 0, src[type] );
	data->m_copyKernel = deviceData->getKernel( PATH, KERNEL2, 0, src[type] );

	//	is this correct?
	data->m_scanData = PrefixScan<type>::allocate( deviceData, maxNumGroups*(1<<BITS_PER_PASS) );

	data->m_workBuffer0 = new Buffer<u32>( deviceData, maxNumGroups*(1<<BITS_PER_PASS) );
	data->m_workBuffer1 = new Buffer<u32>( deviceData, maxNumGroups*(1<<BITS_PER_PASS) );
	data->m_workBuffer2 = new Buffer<u32>( deviceData, maxNumGroups*(1<<BITS_PER_PASS) );
	data->m_workBuffer3 = new Buffer<SortData>( deviceData, maxSize );
	for(int i=0; i<32/BITS_PER_PASS; i++)
		data->m_constBuffer[i] = new Buffer<int4>( deviceData, 1, BufferBase::BUFFER_CONST );
	data->m_maxSize = maxSize;

	return data;
}

template<DeviceType type>
void RadixSortStandard<type>::deallocate(void* rawData)
{
	Data* data = (Data*)rawData;

	delete data->m_workBuffer0;
	delete data->m_workBuffer1;
	delete data->m_workBuffer2;
	delete data->m_workBuffer3;
	for(int i=0; i<32/BITS_PER_PASS; i++)
		delete data->m_constBuffer[i];
	
	PrefixScan<type>::deallocate( data->m_scanData );

	delete data;
}

template<DeviceType type>
void RadixSortStandard<type>::execute(void* rawData, Buffer<SortData>& inout, int n, int sortBits)
{
	Data* data = (Data*)rawData;

	ADLASSERT( n%512 == 0 );
	ADLASSERT( n <= data->m_maxSize );
	ADLASSERT( NUM_PER_WI == 4 );

	Buffer<SortData>* src = BufferUtils::map<type, true>( data->m_deviceData, &inout );
	Buffer<SortData>* dst = data->m_workBuffer3;

	const Device* deviceData = data->m_deviceData;

	int numGroups = (n+WG_SIZE*NUM_PER_WI-1)/(WG_SIZE*NUM_PER_WI);

	int4 constBuffer;

	int iPass = 0;
	for(int startBit=0; startBit<sortBits; startBit+=BITS_PER_PASS, iPass++)
	{
		constBuffer.x = startBit;
		constBuffer.y = numGroups;
		constBuffer.z = WG_SIZE;

		{
			BufferInfo bInfo[] = { BufferInfo( src ), BufferInfo( data->m_workBuffer0 ), BufferInfo( data->m_workBuffer1 ) };

			Launcher launcher( deviceData, data->m_localSortKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[iPass], constBuffer );
			launcher.launch1D( WG_SIZE*numGroups, WG_SIZE );
		}

		PrefixScan<type>::execute( data->m_scanData, *data->m_workBuffer0, *data->m_workBuffer2, numGroups*(1<<BITS_PER_PASS) );

		{
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( data->m_workBuffer2, true ), BufferInfo( data->m_workBuffer1, true ),
				BufferInfo( dst ) };

			Launcher launcher( deviceData, data->m_scatterKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[iPass], constBuffer );
			launcher.launch1D( WG_SIZE*numGroups, WG_SIZE );
		}

		if(0)
		{
			BufferInfo bInfo[] = { BufferInfo( dst, true ), BufferInfo( src ) };

			Launcher launcher( deviceData, data->m_copyKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.launch1D( n, WG_SIZE );
		}
		swap2( src, dst );
	}

	if( src != &inout )
	{
		BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( dst ) };

		Launcher launcher( deviceData, data->m_copyKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.launch1D( n, WG_SIZE );
	}

	BufferUtils::unmap<true>( src, &inout );
}

#undef PATH
#undef KERNEL0
#undef KERNEL1
#undef KERNEL2
