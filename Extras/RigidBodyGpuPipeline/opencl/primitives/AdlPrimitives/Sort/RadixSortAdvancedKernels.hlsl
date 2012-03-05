/*
		2011 Takahiro Harada
*/

typedef uint u32;

#define GET_GROUP_IDX groupIdx.x
#define GET_LOCAL_IDX localIdx.x
#define GET_GLOBAL_IDX globalIdx.x
#define GROUP_LDS_BARRIER GroupMemoryBarrierWithGroupSync()
#define DEFAULT_ARGS uint3 globalIdx : SV_DispatchThreadID, uint3 localIdx : SV_GroupThreadID, uint3 groupIdx : SV_GroupID
#define AtomInc(x) InterlockedAdd(x, 1)
#define AtomInc1(x, out) InterlockedAdd(x, 1, out)

#define min2 min
#define max2 max


cbuffer CB0 : register( b0 )
{
	int m_startBit;
	int m_totalBlocks;
	int m_nWorkGroupsToExecute;
	int m_nBlocksPerGroup;

};


typedef struct {
    unsigned int key;
    unsigned int value;
} KeyValuePair;


StructuredBuffer<u32> rHistogram : register(t0);

RWStructuredBuffer<KeyValuePair> dataToSort : register( u0 );
RWStructuredBuffer<KeyValuePair> dataToSortOut : register( u1 );



#define WG_SIZE 128
#define ELEMENTS_PER_WORK_ITEM 4
#define BITS_PER_PASS 4
#define NUM_BUCKET (1<<BITS_PER_PASS)


groupshared u32 sorterSharedMemory[max(WG_SIZE*2*2, WG_SIZE*ELEMENTS_PER_WORK_ITEM*2)];
groupshared u32 localHistogramToCarry[NUM_BUCKET];
groupshared u32 localHistogram[NUM_BUCKET*2];
groupshared u32 localHistogramMat[NUM_BUCKET*WG_SIZE];
groupshared u32 localPrefixSum[NUM_BUCKET];



#define SET_LOCAL_SORT_DATA(idx, sortDataIn) sorterSharedMemory[2*(idx)+0] = sortDataIn.key; sorterSharedMemory[2*(idx)+1] = sortDataIn.value; 
#define GET_LOCAL_SORT_DATA(idx, sortDataOut) sortDataOut.key = sorterSharedMemory[2*(idx)+0]; sortDataOut.value = sorterSharedMemory[2*(idx)+1];



uint4 prefixScanVector( uint4 data )
{
	data.y += data.x;
	data.w += data.z;
	data.z += data.y;
	data.w += data.y;
	return data;
}

uint prefixScanVectorEx( inout uint4 data )
{
	uint4 backup = data;
	data.y += data.x;
	data.w += data.z;
	data.z += data.y;
	data.w += data.y;
	uint sum = data.w;
	data -= backup;
	return sum;
}

uint localPrefixScan128( uint pData, uint lIdx, inout uint totalSum )
{
	{	//	Set data
		sorterSharedMemory[lIdx] = 0;
		sorterSharedMemory[lIdx+WG_SIZE] = pData;
	}

	GROUP_LDS_BARRIER;

	{	//	Prefix sum
		int idx = 2*lIdx + (WG_SIZE+1);
		if( lIdx < 64 )
		{
			sorterSharedMemory[idx] += sorterSharedMemory[idx-1];
			sorterSharedMemory[idx] += sorterSharedMemory[idx-2];					
			sorterSharedMemory[idx] += sorterSharedMemory[idx-4];
			sorterSharedMemory[idx] += sorterSharedMemory[idx-8];			
			sorterSharedMemory[idx] += sorterSharedMemory[idx-16];
			sorterSharedMemory[idx] += sorterSharedMemory[idx-32];		
			sorterSharedMemory[idx] += sorterSharedMemory[idx-64];
		}
		if( lIdx < 64 ) sorterSharedMemory[idx-1] += sorterSharedMemory[idx-2];
	}

	GROUP_LDS_BARRIER;

	totalSum = sorterSharedMemory[WG_SIZE*2-1];
	return sorterSharedMemory[lIdx+127];
}

void localPrefixScan128Dual( uint pData0, uint pData1, uint lIdx, 
							inout uint rank0, inout uint rank1,
							inout uint totalSum0, inout uint totalSum1 )
{
	{	//	Set data
		sorterSharedMemory[lIdx] = 0;
		sorterSharedMemory[lIdx+WG_SIZE] = pData0;
		sorterSharedMemory[2*WG_SIZE+lIdx] = 0;
		sorterSharedMemory[2*WG_SIZE+lIdx+WG_SIZE] = pData1;
	}

	GROUP_LDS_BARRIER;

//	if( lIdx < 128 ) // todo. assert wg size is 128
	{	//	Prefix sum
		int blockIdx = lIdx/64;
		int groupIdx = lIdx%64;
		int idx = 2*groupIdx + (WG_SIZE+1) + (2*WG_SIZE)*blockIdx;

		sorterSharedMemory[idx] += sorterSharedMemory[idx-1];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-2];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-4];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-8];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-16];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-32];		
		sorterSharedMemory[idx] += sorterSharedMemory[idx-64];

		sorterSharedMemory[idx-1] += sorterSharedMemory[idx-2];
	}

	GROUP_LDS_BARRIER;

	totalSum0 = sorterSharedMemory[WG_SIZE*2-1];
	rank0 = sorterSharedMemory[lIdx+127];
	totalSum1 = sorterSharedMemory[2*WG_SIZE+WG_SIZE*2-1];
	rank1 = sorterSharedMemory[2*WG_SIZE+lIdx+127];
}

uint4 localPrefixSum128V( uint4 pData, uint lIdx, inout uint totalSum )
{
	{	//	Set data
		sorterSharedMemory[lIdx] = 0;
		sorterSharedMemory[lIdx+WG_SIZE] = prefixScanVectorEx( pData );
	}

	GROUP_LDS_BARRIER;

	{	//	Prefix sum
		int idx = 2*lIdx + (WG_SIZE+1);
		if( lIdx < 64 )
		{
			sorterSharedMemory[idx] += sorterSharedMemory[idx-1];
			sorterSharedMemory[idx] += sorterSharedMemory[idx-2];					
			sorterSharedMemory[idx] += sorterSharedMemory[idx-4];
			sorterSharedMemory[idx] += sorterSharedMemory[idx-8];			
			sorterSharedMemory[idx] += sorterSharedMemory[idx-16];
			sorterSharedMemory[idx] += sorterSharedMemory[idx-32];		
			sorterSharedMemory[idx] += sorterSharedMemory[idx-64];

			sorterSharedMemory[idx-1] += sorterSharedMemory[idx-2];
		}
	}

	GROUP_LDS_BARRIER;

	totalSum = sorterSharedMemory[WG_SIZE*2-1];
	uint addValue = sorterSharedMemory[lIdx+127];
	return pData + uint4(addValue, addValue, addValue, addValue);
}

void localPrefixSum128Dual( uint4 pData0, uint4 pData1, uint lIdx, 
						   inout uint4 dataOut0, inout uint4 dataOut1, 
						   inout uint totalSum0, inout uint totalSum1 )
{
/*
	dataOut0 = localPrefixSum128V( pData0, lIdx, totalSum0 );
	GROUP_LDS_BARRIER;
	dataOut1 = localPrefixSum128V( pData1, lIdx, totalSum1 );
	return;
*/

	uint4 backup0 = pData0;
	uint4 backup1 = pData1;

	{	// Prefix sum in a vector
		pData0 = prefixScanVector( pData0 );
		pData1 = prefixScanVector( pData1 );
	}

	{	//	Set data
		sorterSharedMemory[lIdx] = 0;
		sorterSharedMemory[lIdx+WG_SIZE] = pData0.w;
		sorterSharedMemory[2*WG_SIZE+lIdx] = 0;
		sorterSharedMemory[2*WG_SIZE+lIdx+WG_SIZE] = pData1.w;
	}

	GROUP_LDS_BARRIER;

//	if( lIdx < 128 ) // todo. assert wg size is 128
	{	//	Prefix sum
		int blockIdx = lIdx/64;
		int groupIdx = lIdx%64;
		int idx = 2*groupIdx + (WG_SIZE+1) + (2*WG_SIZE)*blockIdx;

		sorterSharedMemory[idx] += sorterSharedMemory[idx-1];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-2];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-4];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-8];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-16];
		sorterSharedMemory[idx] += sorterSharedMemory[idx-32];		
		sorterSharedMemory[idx] += sorterSharedMemory[idx-64];

		sorterSharedMemory[idx-1] += sorterSharedMemory[idx-2];
	}

	GROUP_LDS_BARRIER;

	totalSum0 = sorterSharedMemory[WG_SIZE*2-1];
	{
		uint addValue = sorterSharedMemory[lIdx+127];
		dataOut0 = pData0 + uint4(addValue, addValue, addValue, addValue) - backup0;
	}

	totalSum1 = sorterSharedMemory[2*WG_SIZE+WG_SIZE*2-1];
	{
		uint addValue = sorterSharedMemory[2*WG_SIZE+lIdx+127];
		dataOut1 = pData1 + uint4(addValue, addValue, addValue, addValue) - backup1;
	}
}

uint4 extractKeys(uint4 data, uint targetKey)
{
	uint4 key;
	key.x = data.x == targetKey ? 1:0;
	key.y = data.y == targetKey ? 1:0;
	key.z = data.z == targetKey ? 1:0;
	key.w = data.w == targetKey ? 1:0;
	return key;
}

uint4 extractKeysByBits(uint4 data, uint targetKey)
{
	uint4 key;
	uint mask = 1<<targetKey;
	key.x = (data.x & mask) >> targetKey;
	key.y = (data.y & mask) >> targetKey;
	key.z = (data.z & mask) >> targetKey;
	key.w = (data.w & mask) >> targetKey;
	return key;
}

uint packKeys(uint lower, uint upper)
{
	return lower|(upper<<16);
}

uint4 packKeys(uint4 lower, uint4 upper)
{
	return uint4( lower.x|(upper.x<<16), lower.y|(upper.y<<16), lower.z|(upper.z<<16), lower.w|(upper.w<<16) );
}

uint extractLower( uint data )
{
	return data&0xffff;
}

uint extractUpper( uint data )
{
	return (data>>16)&0xffff;
}

uint4 extractLower( uint4 data )
{
	return uint4( data.x&0xffff, data.y&0xffff, data.z&0xffff, data.w&0xffff );
}

uint4 extractUpper( uint4 data )
{
	return uint4( (data.x>>16)&0xffff, (data.y>>16)&0xffff, (data.z>>16)&0xffff, (data.w>>16)&0xffff );
}

[numthreads(WG_SIZE, 1, 1)]
void SortAndScatterKernel( DEFAULT_ARGS )        
{
	u32 lIdx = GET_LOCAL_IDX;
	u32 wgIdx = GET_GROUP_IDX;

	if( lIdx < (NUM_BUCKET) )
	{
		localHistogramToCarry[lIdx] = rHistogram[lIdx*m_nWorkGroupsToExecute + wgIdx];
	}

	GROUP_LDS_BARRIER;

	for(uint igroup=wgIdx*m_nBlocksPerGroup; igroup<min2(m_totalBlocks,(wgIdx+1)*m_nBlocksPerGroup); igroup++)
	{
		u32 myHistogram;
		if( lIdx < (NUM_BUCKET) )
		{
			localPrefixSum[lIdx] = 0.f;
		}

		u32 newOffset[4];
		KeyValuePair myData[4];
		{	//	read data
			int numLocalElements = WG_SIZE*ELEMENTS_PER_WORK_ITEM;
			uint startAddress = igroup*numLocalElements + lIdx*4;

			myData[0] = dataToSort[startAddress+0];
			myData[1] = dataToSort[startAddress+1];
			myData[2] = dataToSort[startAddress+2];
			myData[3] = dataToSort[startAddress+3];

			newOffset[0] = newOffset[1] = newOffset[2] = newOffset[3] = 0;
		}

		int localOffset = 0;
		uint4 b = uint4((myData[0].key>>m_startBit) & 0xf, (myData[1].key>>m_startBit) & 0xf, (myData[2].key>>m_startBit) & 0xf, (myData[3].key>>m_startBit) & 0xf);
		for(uint targetKey=0; targetKey<(NUM_BUCKET); targetKey+=4)
		{
			uint4 key[4];
			uint keySet[2];
			{	//	pack 4
				uint4 scannedKey[4];
				key[0] = scannedKey[0] = extractKeys( b, targetKey+0 );
				key[1] = scannedKey[1] = extractKeys( b, targetKey+1 );
				key[2] = scannedKey[2] = extractKeys( b, targetKey+2 );
				key[3] = scannedKey[3] = extractKeys( b, targetKey+3 );
				{
					uint s[4];
					s[0] = prefixScanVectorEx( scannedKey[0] );
					s[1] = prefixScanVectorEx( scannedKey[1] );
					s[2] = prefixScanVectorEx( scannedKey[2] );
					s[3] = prefixScanVectorEx( scannedKey[3] );
					keySet[0] = packKeys( s[0], s[1] );
					keySet[1] = packKeys( s[2], s[3] );
				}
			}

			uint dstAddressBase[4];
			{

				uint totalSumPacked[2];
				uint dstAddressPacked[2];

				localPrefixScan128Dual( keySet[0], keySet[1], lIdx, dstAddressPacked[0], dstAddressPacked[1], totalSumPacked[0], totalSumPacked[1] );

				dstAddressBase[0] = extractLower( dstAddressPacked[0] );
				dstAddressBase[1] = extractUpper( dstAddressPacked[0] );
				dstAddressBase[2] = extractLower( dstAddressPacked[1] );
				dstAddressBase[3] = extractUpper( dstAddressPacked[1] );

				uint4 histogram;
				histogram.x = extractLower(totalSumPacked[0]);
				histogram.y = extractUpper(totalSumPacked[0]);
				histogram.z = extractLower(totalSumPacked[1]);
				histogram.w = extractUpper(totalSumPacked[1]);

				if( lIdx == targetKey + 0 ) myHistogram = histogram.x;
				else if( lIdx == targetKey + 1 ) myHistogram = histogram.y;
				else if( lIdx == targetKey + 2 ) myHistogram = histogram.z;
				else if( lIdx == targetKey + 3 ) myHistogram = histogram.w;
				
				uint histogramSum = prefixScanVectorEx( histogram );

				if( lIdx == targetKey + 0 ) localPrefixSum[targetKey+0] = localOffset+histogram.x;
				else if( lIdx == targetKey + 1 ) localPrefixSum[targetKey+1] = localOffset+histogram.y;
				else if( lIdx == targetKey + 2 ) localPrefixSum[targetKey+2] = localOffset+histogram.z;
				else if( lIdx == targetKey + 3 ) localPrefixSum[targetKey+3] = localOffset+histogram.w;

				localOffset += histogramSum;
			}
			
			GROUP_LDS_BARRIER;


			for(int ie=0; ie<4; ie++)
			{
				uint4 scannedKey = key[ie];
				prefixScanVectorEx( scannedKey );

				uint offset = localPrefixSum[targetKey + ie] + dstAddressBase[ie];
				uint4 dstAddress = uint4( offset, offset, offset, offset ) + scannedKey;

				newOffset[0] += dstAddress.x*key[ie].x;
				newOffset[1] += dstAddress.y*key[ie].y;
				newOffset[2] += dstAddress.z*key[ie].z;
				newOffset[3] += dstAddress.w*key[ie].w;
			}
		}

		{	//	local scatter
			SET_LOCAL_SORT_DATA(newOffset[0], myData[0]);
			SET_LOCAL_SORT_DATA(newOffset[1], myData[1]);
			SET_LOCAL_SORT_DATA(newOffset[2], myData[2]);
			SET_LOCAL_SORT_DATA(newOffset[3], myData[3]);
		}

		GROUP_LDS_BARRIER;

		{	//	write data
			for(int i=0; i<ELEMENTS_PER_WORK_ITEM; i++)
			{
				int dataIdx = 4*lIdx+i;
				KeyValuePair localData; GET_LOCAL_SORT_DATA( dataIdx, localData );
				int binIdx = (localData.key >> m_startBit) & 0xf;
				int groupOffset = localHistogramToCarry[binIdx];
				int myIdx = dataIdx - localPrefixSum[binIdx];

				dataToSortOut[ groupOffset + myIdx ] = localData;
			}
		}

		GROUP_LDS_BARRIER;
		if( lIdx < NUM_BUCKET )
		{
			localHistogramToCarry[lIdx] += myHistogram;
		}
		GROUP_LDS_BARRIER;
	}
}


[numthreads(WG_SIZE, 1, 1)]
void SortAndScatterKernel1( DEFAULT_ARGS )
{
	u32 lIdx = GET_LOCAL_IDX;
	u32 wgIdx = GET_GROUP_IDX;

	if( lIdx < (NUM_BUCKET) )
	{
		localHistogramToCarry[lIdx] = rHistogram[lIdx*m_nWorkGroupsToExecute + wgIdx.x];
	}

	GROUP_LDS_BARRIER;

	for(uint igroup=wgIdx.x*m_nBlocksPerGroup; igroup<min2(m_totalBlocks,(wgIdx.x+1)*m_nBlocksPerGroup); igroup++)
	{
		u32 myHistogram;

		KeyValuePair myData[4];
		uint startAddrBlock;
		{	//	read data
			int numLocalElements = WG_SIZE*ELEMENTS_PER_WORK_ITEM;
			startAddrBlock = lIdx*4;
			uint startAddress = igroup*numLocalElements + startAddrBlock;

			myData[0] = dataToSort[startAddress+0];
			myData[1] = dataToSort[startAddress+1];
			myData[2] = dataToSort[startAddress+2];
			myData[3] = dataToSort[startAddress+3];
		}

		//	local sort
		for(int ib=m_startBit; ib<m_startBit+BITS_PER_PASS; ib++)
		{
			uint4 keys = uint4(~(myData[0].key>>ib) & 0x1, ~(myData[1].key>>ib) & 0x1, ~(myData[2].key>>ib) & 0x1, ~(myData[3].key>>ib) & 0x1);
			uint total;
			uint4 rankOfP = localPrefixSum128V( keys, lIdx, total );
			uint4 rankOfN = uint4(startAddrBlock, startAddrBlock+1, startAddrBlock+2, startAddrBlock+3) - rankOfP + uint4( total, total, total, total );

			uint4 myAddr = (keys==uint4(1,1,1,1))? rankOfP: rankOfN;
			
			GROUP_LDS_BARRIER;

			SET_LOCAL_SORT_DATA( myAddr.x, myData[0] );
			SET_LOCAL_SORT_DATA( myAddr.y, myData[1] );
			SET_LOCAL_SORT_DATA( myAddr.z, myData[2] );
			SET_LOCAL_SORT_DATA( myAddr.w, myData[3] );

			GROUP_LDS_BARRIER;
			
			GET_LOCAL_SORT_DATA( startAddrBlock+0, myData[0] );
			GET_LOCAL_SORT_DATA( startAddrBlock+1, myData[1] );
			GET_LOCAL_SORT_DATA( startAddrBlock+2, myData[2] );
			GET_LOCAL_SORT_DATA( startAddrBlock+3, myData[3] );
		}

		{//	create histogram -> prefix sum
			if( lIdx < NUM_BUCKET )
			{
				localHistogram[lIdx] = 0;
				localHistogram[NUM_BUCKET+lIdx] = 0;
			}
			GROUP_LDS_BARRIER;
			uint4 keys = uint4((myData[0].key>>m_startBit) & 0xf, (myData[1].key>>m_startBit) & 0xf, (myData[2].key>>m_startBit) & 0xf, (myData[3].key>>m_startBit) & 0xf);
			
			InterlockedAdd( localHistogram[NUM_BUCKET+keys.x], 1 );
			InterlockedAdd( localHistogram[NUM_BUCKET+keys.y], 1 );
			InterlockedAdd( localHistogram[NUM_BUCKET+keys.z], 1 );
			InterlockedAdd( localHistogram[NUM_BUCKET+keys.w], 1 );
			
			GROUP_LDS_BARRIER;
			
			uint hIdx = NUM_BUCKET+lIdx;
			if( lIdx < NUM_BUCKET )
			{
				myHistogram = localHistogram[hIdx];
			}
			GROUP_LDS_BARRIER;
	
			if( lIdx < NUM_BUCKET )
			{
				localHistogram[hIdx] = localHistogram[hIdx-1];

				localHistogram[hIdx] += localHistogram[hIdx-1];
				localHistogram[hIdx] += localHistogram[hIdx-2];
				localHistogram[hIdx] += localHistogram[hIdx-4];
				localHistogram[hIdx] += localHistogram[hIdx-8];
			}

			GROUP_LDS_BARRIER;
		}
/*
		{//	write back
			int numLocalElements = WG_SIZE*ELEMENTS_PER_WORK_ITEM;
			startAddrBlock = lIdx*4;
			uint startAddress = igroup*numLocalElements + startAddrBlock;

			for(int ie=0; ie<ELEMENTS_PER_WORK_ITEM; ie++)
			{
				dataToSortOut[ startAddress+ie ] = myData[ie];
			}
		}
*/
		{
			for(int ie=0; ie<ELEMENTS_PER_WORK_ITEM; ie++)
			{
				int dataIdx = startAddrBlock+ie;
				int binIdx = (myData[ie].key>>m_startBit)&0xf;
				int groupOffset = localHistogramToCarry[binIdx];
				int myIdx = dataIdx - localHistogram[NUM_BUCKET+binIdx];
				dataToSortOut[ groupOffset + myIdx ] = myData[ie];
			}
		}
		
		GROUP_LDS_BARRIER;
		if( lIdx < NUM_BUCKET )
		{
			localHistogramToCarry[lIdx] += myHistogram;
		}
		GROUP_LDS_BARRIER;
	
	}
}

/*
[numthreads(WG_SIZE, 1, 1)]
void SortAndScatterKernel1( uint3 gIdx : SV_GroupID, uint3 lIdx : SV_GroupThreadID )
{
	if( lIdx.x < (NUM_BUCKET) )
	{
		localHistogramToCarry[lIdx.x] = rHistogram[lIdx.x*m_nWorkGroupsToExecute + gIdx.x];
	}

	GROUP_LDS_BARRIER;

	for(uint igroup=gIdx.x*m_nBlocksPerGroup; igroup<min2(m_totalBlocks,(gIdx.x+1)*m_nBlocksPerGroup); igroup++)
	{
		u32 myHistogram;

		KeyValuePair myData[4];
		uint startAddrBlock;
		{	//	read data
			int numLocalElements = WG_SIZE*ELEMENTS_PER_WORK_ITEM;
			startAddrBlock = lIdx.x*4;
			uint startAddress = igroup*numLocalElements + startAddrBlock;

			myData[0] = dataToSort[startAddress+0];
			myData[1] = dataToSort[startAddress+1];
			myData[2] = dataToSort[startAddress+2];
			myData[3] = dataToSort[startAddress+3];
		}

		for(int ib=m_startBit; ib<m_startBit+BITS_PER_PASS; ib++)
		{
			uint4 keys = uint4(~(myData[0].key>>ib) & 0x1, ~(myData[1].key>>ib) & 0x1, ~(myData[2].key>>ib) & 0x1, ~(myData[3].key>>ib) & 0x1);
			uint total;
			uint4 rankOfP = localPrefixSum128V( keys, lIdx.x, total );
			uint4 rankOfN = uint4(startAddrBlock, startAddrBlock+1, startAddrBlock+2, startAddrBlock+3) - rankOfP + uint4( total, total, total, total );

			uint4 myAddr = (keys==uint4(1,1,1,1))? rankOfP: rankOfN;
			
			GROUP_LDS_BARRIER;

			SET_LOCAL_SORT_DATA( myAddr.x, myData[0] );
			SET_LOCAL_SORT_DATA( myAddr.y, myData[1] );
			SET_LOCAL_SORT_DATA( myAddr.z, myData[2] );
			SET_LOCAL_SORT_DATA( myAddr.w, myData[3] );

			GROUP_LDS_BARRIER;
			
			GET_LOCAL_SORT_DATA( startAddrBlock+0, myData[0] );
			GET_LOCAL_SORT_DATA( startAddrBlock+1, myData[1] );
			GET_LOCAL_SORT_DATA( startAddrBlock+2, myData[2] );
			GET_LOCAL_SORT_DATA( startAddrBlock+3, myData[3] );
		}
		
		{//	create histogram -> prefix sum
			if( lIdx.x < NUM_BUCKET )
			{
				localHistogram[lIdx.x] = 0;
				localHistogram[NUM_BUCKET+lIdx.x] = 0;
			}
			GROUP_LDS_BARRIER;
			uint4 keys = uint4((myData[0].key>>m_startBit) & 0xf, (myData[1].key>>m_startBit) & 0xf, (myData[2].key>>m_startBit) & 0xf, (myData[3].key>>m_startBit) & 0xf);
			
			InterlockedAdd( localHistogram[NUM_BUCKET+keys.x], 1 );
			InterlockedAdd( localHistogram[NUM_BUCKET+keys.y], 1 );
			InterlockedAdd( localHistogram[NUM_BUCKET+keys.z], 1 );
			InterlockedAdd( localHistogram[NUM_BUCKET+keys.w], 1 );
			
			GROUP_LDS_BARRIER;
			
			uint hIdx = NUM_BUCKET+lIdx.x;
			if( lIdx.x < NUM_BUCKET )
			{
				myHistogram = localHistogram[hIdx];
			}
			GROUP_LDS_BARRIER;
	

			if( lIdx.x < NUM_BUCKET )
			{
				localHistogram[hIdx] = localHistogram[hIdx-1];

				localHistogram[hIdx] += localHistogram[hIdx-1];
				localHistogram[hIdx] += localHistogram[hIdx-2];
				localHistogram[hIdx] += localHistogram[hIdx-4];
				localHistogram[hIdx] += localHistogram[hIdx-8];
			}

			GROUP_LDS_BARRIER;
		}
		{//	write back
			for(int ie=0; ie<ELEMENTS_PER_WORK_ITEM; ie++)
			{
				int dataIdx = startAddrBlock+ie;
				int binIdx = (myData[ie].key>>m_startBit)&0xf;
				int groupOffset = localHistogramToCarry[binIdx];
				int myIdx = dataIdx - localHistogram[NUM_BUCKET+binIdx];
				
				dataToSortOut[ groupOffset + myIdx ] = myData[ie];
			}
		}
		
		GROUP_LDS_BARRIER;
		if( lIdx.x < NUM_BUCKET )
		{
			localHistogramToCarry[lIdx.x] += myHistogram;
		}
		GROUP_LDS_BARRIER;
	
	}
}
*/

StructuredBuffer<KeyValuePair> dataToSort1 : register( t0 );
RWStructuredBuffer<u32> wHistogram1 : register(u0);

#define MY_HISTOGRAM(idx) localHistogramMat[(idx)*WG_SIZE+lIdx.x]

[numthreads(WG_SIZE, 1, 1)]
void StreamCountKernel( DEFAULT_ARGS )        
{
	u32 lIdx = GET_LOCAL_IDX;
	u32 wgIdx = GET_GROUP_IDX;

	int myHistogram[NUM_BUCKET];

	for(int i=0; i<NUM_BUCKET; i++)
	{
		MY_HISTOGRAM(i) = 0;
	}

	for(uint igroup=wgIdx.x*m_nBlocksPerGroup; igroup<min2(m_totalBlocks,(wgIdx.x+1)*m_nBlocksPerGroup); igroup++)
	{
		uint localKeys[4];
		{	//	read data
			int numLocalElements = WG_SIZE*ELEMENTS_PER_WORK_ITEM;

			uint4 localAddress = uint4(lIdx, lIdx, lIdx, lIdx)*4+uint4(0,1,2,3);
			uint4 globalAddress = uint4(igroup,igroup,igroup,igroup)*numLocalElements + localAddress;

			KeyValuePair localData0 = dataToSort1[globalAddress.x];
			KeyValuePair localData1 = dataToSort1[globalAddress.y];
			KeyValuePair localData2 = dataToSort1[globalAddress.z];
			KeyValuePair localData3 = dataToSort1[globalAddress.w];

			localKeys[0] = (localData0.key >> m_startBit) & 0xf;
			localKeys[1] = (localData1.key >> m_startBit) & 0xf;
			localKeys[2] = (localData2.key >> m_startBit) & 0xf;
			localKeys[3] = (localData3.key >> m_startBit) & 0xf;
		}

		MY_HISTOGRAM( localKeys[0] )++;
		MY_HISTOGRAM( localKeys[1] )++;
		MY_HISTOGRAM( localKeys[2] )++;
		MY_HISTOGRAM( localKeys[3] )++;
	}

	GROUP_LDS_BARRIER;

	{	//	reduce to 1
		if( lIdx < 64 )//WG_SIZE/2 )
		{
			for(int i=0; i<NUM_BUCKET/2; i++)
			{
				int idx = lIdx;
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+64];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+32];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+16];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+8];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+4];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+2];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+1];
			}
		}
		else if( lIdx < 128 )
		{
			for(int i=NUM_BUCKET/2; i<NUM_BUCKET; i++)
			{
				int idx = lIdx-64;
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+64];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+32];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+16];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+8];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+4];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+2];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+1];
			}
		}
	}

	GROUP_LDS_BARRIER;

	{	//	write data
		if( lIdx < NUM_BUCKET )
		{
			wHistogram1[ lIdx*m_nWorkGroupsToExecute + wgIdx.x ] = localHistogramMat[ lIdx*WG_SIZE+0 ];
		}
	}
}

/*
[numthreads(WG_SIZE, 1, 1)]
void StreamCountKernel( uint3 gIdx : SV_GroupID, uint3 lIdx : SV_GroupThreadID )        
{
	int myHistogram[NUM_BUCKET];

	for(int i=0; i<NUM_BUCKET; i++)
	{
		myHistogram[i] = 0;
	}

	for(uint igroup=gIdx.x*m_nBlocksPerGroup; igroup<min2(m_totalBlocks,(gIdx.x+1)*m_nBlocksPerGroup); igroup++)
	{
		uint localKeys[4];
		{	//	read data
			int numLocalElements = WG_SIZE*ELEMENTS_PER_WORK_ITEM;

			uint4 localAddress = uint4(lIdx.x, lIdx.x, lIdx.x, lIdx.x)*4+uint4(0,1,2,3);
			uint4 globalAddress = uint4(igroup,igroup,igroup,igroup)*numLocalElements + localAddress;

			KeyValuePair localData0 = dataToSort1[globalAddress.x];
			KeyValuePair localData1 = dataToSort1[globalAddress.y];
			KeyValuePair localData2 = dataToSort1[globalAddress.z];
			KeyValuePair localData3 = dataToSort1[globalAddress.w];

			localKeys[0] = (localData0.key >> m_startBit) & 0xf;
			localKeys[1] = (localData1.key >> m_startBit) & 0xf;
			localKeys[2] = (localData2.key >> m_startBit) & 0xf;
			localKeys[3] = (localData3.key >> m_startBit) & 0xf;
		}

		myHistogram[ localKeys[0] ]++;
		myHistogram[ localKeys[1] ]++;
		myHistogram[ localKeys[2] ]++;
		myHistogram[ localKeys[3] ]++;
	}

	{	//	move to shared
		for(int i=0; i<NUM_BUCKET; i++)
		{
			localHistogramMat[i*WG_SIZE+lIdx.x] = myHistogram[i];
		}
	}

	GROUP_LDS_BARRIER;

	{	//	reduce to 1
		if( lIdx.x < 64 )//WG_SIZE/2 )
		{
			for(int i=0; i<NUM_BUCKET/2; i++)
			{
				int idx = lIdx.x;
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+64];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+32];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+16];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+8];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+4];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+2];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+1];
			}
		}
		else if( lIdx.x < 128 )
		{
			for(int i=NUM_BUCKET/2; i<NUM_BUCKET; i++)
			{
				int idx = lIdx.x-64;
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+64];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+32];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+16];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+8];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+4];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+2];
				localHistogramMat[i*WG_SIZE+idx] += localHistogramMat[i*WG_SIZE+idx+1];
			}
		}
	}

	GROUP_LDS_BARRIER;

	{	//	write data
		if( lIdx.x < NUM_BUCKET )
		{
			wHistogram1[ lIdx.x*m_nWorkGroupsToExecute + gIdx.x ] = localHistogramMat[ lIdx.x*WG_SIZE+0 ];
		}
	}
}
*/

/*
//	for MAX_WG_SIZE 20
[numthreads(WG_SIZE, 1, 1)]
void PrefixScanKernel( uint3 gIdx : SV_GroupID, uint3 lIdx : SV_GroupThreadID )        
{
	uint4 myData = uint4(0,0,0,0);
	if( 4*lIdx.x+0 < NUM_BUCKET*m_nWorkGroupsToExecute )
		myData.x = wHistogram1[4*lIdx.x+0];
	if( 4*lIdx.x+1 < NUM_BUCKET*m_nWorkGroupsToExecute )
		myData.y = wHistogram1[4*lIdx.x+1];
	if( 4*lIdx.x+2 < NUM_BUCKET*m_nWorkGroupsToExecute )
		myData.z = wHistogram1[4*lIdx.x+2];
	if( 4*lIdx.x+3 < NUM_BUCKET*m_nWorkGroupsToExecute )
		myData.w = wHistogram1[4*lIdx.x+3];

	uint totalSum;

	uint4 scanned = localPrefixSum128V( myData, lIdx.x, totalSum );

	wHistogram1[4*lIdx.x+0] = scanned.x;
	wHistogram1[4*lIdx.x+1] = scanned.y;
	wHistogram1[4*lIdx.x+2] = scanned.z;
	wHistogram1[4*lIdx.x+3] = scanned.w;
}
*/

//	for MAX_WG_SIZE 80
//	can hold up to WG_SIZE*12 (128*12 > 80*16 )
[numthreads(WG_SIZE, 1, 1)]
void PrefixScanKernel( DEFAULT_ARGS )
{
	u32 lIdx = GET_LOCAL_IDX;
	u32 wgIdx = GET_GROUP_IDX;

	uint data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	for(int i=0; i<12; i++)
	{
		if( int(12*lIdx+i) < NUM_BUCKET*m_nWorkGroupsToExecute )
			data[i] = wHistogram1[12*lIdx+i];
	}

	uint4 myData = uint4(0,0,0,0);
	myData.x = data[0] + data[1];
	myData.y = data[2] + data[3];
	myData.z = data[4] + data[5];
	myData.w = data[6] + data[7];


	uint totalSum;
	uint4 scanned = localPrefixSum128V( myData, lIdx, totalSum );

	data[11] = scanned.w + data[9] + data[10];
	data[10] = scanned.w + data[9];
	data[9] = scanned.w;
	data[8] = scanned.z + data[6] + data[7];
	data[7] = scanned.z + data[6];
	data[6] = scanned.z;
	data[5] = scanned.y + data[3] + data[4];
	data[4] = scanned.y + data[3];
	data[3] = scanned.y;
	data[2] = scanned.x + data[0] + data[1];
	data[1] = scanned.x + data[0];
	data[0] = scanned.x;

	for(int i=0; i<12; i++)
	{
		wHistogram1[12*lIdx+i] = data[i];
	}
}
/*
[numthreads(WG_SIZE, 1, 1)]
void PrefixScanKernel( DEFAULT_ARGS )
{
	u32 lIdx = GET_LOCAL_IDX;
	u32 wgIdx = GET_GROUP_IDX;

	uint data[8] = {0,0,0,0,0,0,0,0};
	for(int i=0; i<8; i++)
	{
		if( int(8*lIdx+i) < NUM_BUCKET*m_nWorkGroupsToExecute )
			data[i] = wHistogram1[8*lIdx+i];
	}

	uint4 myData = uint4(0,0,0,0);
	myData.x = data[0] + data[1];
	myData.y = data[2] + data[3];
	myData.z = data[4] + data[5];
	myData.w = data[6] + data[7];


	uint totalSum;
	uint4 scanned = localPrefixSum128V( myData, lIdx, totalSum );

	data[7] = scanned.w + data[6];
	data[6] = scanned.w;// + data[5];
	data[5] = scanned.z + data[4];
	data[4] = scanned.z;// + data[3];
	data[3] = scanned.y + data[2];
	data[2] = scanned.y;// + data[1];
	data[1] = scanned.x + data[0];
	data[0] = scanned.x;

	for(int i=0; i<8; i++)
	{
		wHistogram1[8*lIdx+i] = data[i];
	}
}
*/


[numthreads(WG_SIZE, 1, 1)]
void CopyKernel( DEFAULT_ARGS )
{
	u32 lIdx = GET_LOCAL_IDX;
	u32 wgIdx = GET_GROUP_IDX;

	for(uint igroup=wgIdx.x*m_nBlocksPerGroup; igroup<min2(m_totalBlocks,(wgIdx.x+1)*m_nBlocksPerGroup); igroup++)
	{
		KeyValuePair myData[4];
		uint startAddrBlock;
		{	//	read data
			int numLocalElements = WG_SIZE*ELEMENTS_PER_WORK_ITEM;
			startAddrBlock = lIdx*4;
			uint startAddress = igroup*numLocalElements + startAddrBlock;

			myData[0] = dataToSort[startAddress+0];
			myData[1] = dataToSort[startAddress+1];
			myData[2] = dataToSort[startAddress+2];
			myData[3] = dataToSort[startAddress+3];
		}

		{
			int numLocalElements = WG_SIZE*ELEMENTS_PER_WORK_ITEM;
			uint startAddress = igroup*numLocalElements + startAddrBlock;

			dataToSortOut[startAddress+0] = myData[0];
			dataToSortOut[startAddress+1] = myData[1];
			dataToSortOut[startAddress+2] = myData[2];
			dataToSortOut[startAddress+3] = myData[3];
		}
	}
}