/*
		2011 Takahiro Harada
*/

#define PATH "..\\..\\AdlPrimitives\\Sort\\RadixSortAdvancedKernels"
#define KERNEL0 "StreamCountKernel"
#define KERNEL1 "SortAndScatterKernel1"
#define KERNEL2 "PrefixScanKernel"

template<DeviceType type>
class RadixSortAdvanced : public RadixSortBase
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		enum
		{
			WG_SIZE = 128,
			NUM_PER_WI = 4,
			MAX_NUM_WORKGROUPS = 60,
		};

		struct Data : public RadixSort<type>::Data
		{
			Kernel* m_localCountKernel;
			Kernel* m_scatterKernel;
			Kernel* m_scanKernel;

			Buffer<u32>* m_workBuffer0;
			Buffer<SortData>* m_workBuffer1;
			Buffer<int4>* m_constBuffer[32/4];
		};
		

		static
		Data* allocate(const Device* deviceData, int maxSize, Option option = SORT_NORMAL);

		static
		void deallocate(void* data);

		static
		void execute(void* data, Buffer<SortData>& inout, int n, int sortBits);
};

template<DeviceType type>
typename RadixSortAdvanced<type>::Data* RadixSortAdvanced<type>::allocate(const Device* deviceData, int maxSize, Option option)
{
	ADLASSERT( type == deviceData->m_type );

	const char* src[] = { 0, 0, 0 };

	Data* data = new Data;
	data->m_option = option;
	data->m_deviceData = deviceData;

	data->m_localCountKernel = deviceData->getKernel( PATH, KERNEL0, 0, src[type] );
	data->m_scatterKernel = deviceData->getKernel( PATH, KERNEL1, 0, src[type] );
	data->m_scanKernel = deviceData->getKernel( PATH, KERNEL2, 0, src[type] );

	data->m_workBuffer0 = new Buffer<u32>( deviceData, MAX_NUM_WORKGROUPS*16 );
	data->m_workBuffer1 = new Buffer<SortData>( deviceData, maxSize );
	for(int i=0; i<32/4; i++)
		data->m_constBuffer[i] = new Buffer<int4>( deviceData, 1, BufferBase::BUFFER_CONST );
	data->m_maxSize = maxSize;

	return data;
}

template<DeviceType type>
void RadixSortAdvanced<type>::deallocate(void* rawData)
{
	Data* data = (Data*)rawData;

	delete data->m_workBuffer0;
	delete data->m_workBuffer1;
	for(int i=0; i<32/4; i++)
		delete data->m_constBuffer[i];
	
	delete data;
}

template<DeviceType type>
void RadixSortAdvanced<type>::execute(void* rawData, Buffer<SortData>& inout, int n, int sortBits)
{
	Data* data = (Data*)rawData;

	ADLASSERT( sortBits == 32 );

	ADLASSERT( NUM_PER_WI == 4 );
	ADLASSERT( n%(WG_SIZE*NUM_PER_WI) == 0 );
	ADLASSERT( MAX_NUM_WORKGROUPS < 128*8/16 );

	Buffer<SortData>* src = &inout;
	Buffer<SortData>* dst = data->m_workBuffer1;

	const Device* deviceData = data->m_deviceData;

	int nBlocks = n/(NUM_PER_WI*WG_SIZE);
	const int nWorkGroupsToExecute = min2((int)MAX_NUM_WORKGROUPS, nBlocks);
	int nBlocksPerGroup = (nBlocks+nWorkGroupsToExecute-1)/nWorkGroupsToExecute;
	ADLASSERT( nWorkGroupsToExecute <= MAX_NUM_WORKGROUPS );

	int4 constBuffer = make_int4(0, nBlocks, nWorkGroupsToExecute, nBlocksPerGroup);

	int iPass = 0;
	int startBit = 0;
	for(int startBit=0; startBit<32; startBit+=4, iPass++)
	{
		constBuffer.x = startBit;

		{
			BufferInfo bInfo[] = { BufferInfo( src, true ), BufferInfo( data->m_workBuffer0 ) };

			Launcher launcher( deviceData, data->m_localCountKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[iPass], constBuffer );
			launcher.launch1D( WG_SIZE* nWorkGroupsToExecute, WG_SIZE );
		}


		{
			BufferInfo bInfo[] = { BufferInfo( data->m_workBuffer0 ) };

			Launcher launcher( deviceData, data->m_scanKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[iPass], constBuffer );
			launcher.launch1D( WG_SIZE, WG_SIZE );
		}

		{
			BufferInfo bInfo[] = { BufferInfo( data->m_workBuffer0, true ), BufferInfo( src ), BufferInfo( dst ) };

			Launcher launcher( deviceData, data->m_scatterKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( *data->m_constBuffer[iPass], constBuffer );
			launcher.launch1D( WG_SIZE*nWorkGroupsToExecute, WG_SIZE );
		}

		swap2( src, dst );
	}
}

#undef PATH
#undef KERNEL0
#undef KERNEL1
#undef KERNEL2
