/*
		2011 Takahiro Harada
*/

#define PATH "..\\..\\opencl\\primitives\\AdlPrimitives\\Sort\\RadixSortSimpleKernels"
#define KERNEL0 "LocalCountKernel"
#define KERNEL1 "ScatterKernel"

#include <AdlPrimitives/Sort/RadixSortSimpleCL.h>
#include <AdlPrimitives/Sort/RadixSortSimpleDX11.h>

template<DeviceType type>
class RadixSortSimple : public RadixSortBase
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		enum
		{
			WG_SIZE = 128,
			NUM_PER_WI = 4,
		};

		struct Data : public RadixSort<type>::Data
		{
			Kernel* m_localCountKernel;
			Kernel* m_scatterKernel;

			Buffer<u32>* m_workBuffer0;
			Buffer<u32>* m_workBuffer1;
			Buffer<SortData>* m_workBuffer2;
			Buffer<int4>* m_constBuffer[4];
		};
		

		static
		Data* allocate(const Device* deviceData, int maxSize, Option option = SORT_NORMAL);

		static
		void deallocate(void* data);

		static
		void execute(void* data, Buffer<SortData>& inout, int n, int sortBits);
};

template<DeviceType type>
typename RadixSortSimple<type>::Data* RadixSortSimple<type>::allocate(const Device* deviceData, int maxSize, Option option)
{
	ADLASSERT( type == deviceData->m_type );

	const char* src[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
		{radixSortSimpleKernelsCL, radixSortSimpleKernelsDX11};
#else
		{ 0, 0 };
#endif
	u32 maxNumGroups = (maxSize+WG_SIZE*NUM_PER_WI-1)/(WG_SIZE*NUM_PER_WI);

	Data* data = new Data;
	data->m_option = option;
	data->m_deviceData = deviceData;

	data->m_localCountKernel = deviceData->getKernel( PATH, KERNEL0, 0, src[type] );
	data->m_scatterKernel = deviceData->getKernel( PATH, KERNEL1, 0, src[type] );

	data->m_scanData = PrefixScan<type>::allocate( deviceData, maxSize );

	data->m_workBuffer0 = new Buffer<u32>( deviceData, maxNumGroups*256 );
	data->m_workBuffer1 = new Buffer<u32>( deviceData, maxNumGroups*256 );
	data->m_workBuffer2 = new Buffer<SortData>( deviceData, maxSize );
	data->m_constBuffer[0] = new Buffer<int4>( deviceData, 1, BufferBase::BUFFER_CONST );
	data->m_constBuffer[1] = new Buffer<int4>( deviceData, 1, BufferBase::BUFFER_CONST );
	data->m_constBuffer[2] = new Buffer<int4>( deviceData, 1, BufferBase::BUFFER_CONST );
	data->m_constBuffer[3] = new Buffer<int4>( deviceData, 1, BufferBase::BUFFER_CONST );
	data->m_maxSize = maxSize;

	return data;
}

template<DeviceType type>
void RadixSortSimple<type>::deallocate(void* rawData)
{
	Data* data = (Data*)rawData;

	delete data->m_workBuffer0;
	delete data->m_workBuffer1;
	delete data->m_workBuffer2;
	delete data->m_constBuffer[0];
	delete data->m_constBuffer[1];
	delete data->m_constBuffer[2];
	delete data->m_constBuffer[3];
	
	PrefixScan<type>::deallocate( data->m_scanData );

	delete data;
}

template<DeviceType type>
void RadixSortSimple<type>::execute(void* rawData, Buffer<SortData>& inout, int n, int sortBits)
{
	Data* data = (Data*)rawData;

	ADLASSERT( sortBits == 32 );
	ADLASSERT( n%512 == 0 );
	ADLASSERT( n <= data->m_maxSize );

	Buffer<SortData>* src = &inout;
	Buffer<SortData>* dst = data->m_workBuffer2;

	const Device* deviceData = data->m_deviceData;

	int numGroups = (n+WG_SIZE*NUM_PER_WI-1)/(WG_SIZE*NUM_PER_WI);

	int4 constBuffer;

	int iPass = 0;
	for(int startBit=0; startBit<32; startBit+=8, iPass++)
	{
		constBuffer.x = startBit;
		constBuffer.y = numGroups;
		constBuffer.z = WG_SIZE;

		{
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( data->m_workBuffer0 ) };

			Launcher launcher( deviceData, data->m_localCountKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[iPass], constBuffer );
			launcher.launch1D( WG_SIZE*numGroups, WG_SIZE );
		}

		PrefixScan<type>::execute( data->m_scanData, *data->m_workBuffer0, *data->m_workBuffer1, numGroups*256 );

		{
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( dst ), BufferInfo( data->m_workBuffer1 ) };

			Launcher launcher( deviceData, data->m_scatterKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[iPass], constBuffer );
			launcher.launch1D( WG_SIZE*numGroups, WG_SIZE );
		}

		swap2( src, dst );
	}
}

#undef PATH
#undef KERNEL0
#undef KERNEL1
