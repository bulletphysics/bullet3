/*
		2011 Takahiro Harada
*/

typedef uint u32;

#define GET_GROUP_IDX groupIdx.x
#define GET_LOCAL_IDX localIdx.x
#define GET_GLOBAL_IDX globalIdx.x
#define GROUP_LDS_BARRIER GroupMemoryBarrierWithGroupSync()
#define GROUP_MEM_FENCE
#define DEFAULT_ARGS uint3 globalIdx : SV_DispatchThreadID, uint3 localIdx : SV_GroupThreadID, uint3 groupIdx : SV_GroupID
#define AtomInc(x) InterlockedAdd(x, 1)
#define AtomInc1(x, out) InterlockedAdd(x, 1, out)

#define make_uint4 uint4
#define make_uint2 uint2

uint4 SELECT_UINT4(uint4 b,uint4 a,uint4 condition ){ return  make_uint4( ((condition).x)?a.x:b.x, ((condition).y)?a.y:b.y, ((condition).z)?a.z:b.z, ((condition).w)?a.w:b.w ); }

//	takahiro end
#define WG_SIZE 128
#define NUM_PER_WI 4

#define GET_GROUP_SIZE WG_SIZE

typedef struct
{
	u32 m_key; 
	u32 m_value;
}SortData;

cbuffer SortCB : register( b0 )
{
	u32 m_startBit;
	u32 m_numGroups;
	u32 m_padding[2];
};

#define BITS_PER_PASS 4


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



RWStructuredBuffer<SortData> sortDataIn : register( u0 );
RWStructuredBuffer<u32> ldsHistogramOut0 : register( u1 );
RWStructuredBuffer<u32> ldsHistogramOut1 : register( u2 );

groupshared u32 ldsSortData[ WG_SIZE*NUM_PER_WI + 16 ];


uint4 localPrefixSum128V( uint4 pData, uint lIdx, inout uint totalSum )
{
	{	//	Set data
		ldsSortData[lIdx] = 0;
		ldsSortData[lIdx+WG_SIZE] = prefixScanVectorEx( pData );
	}

	GROUP_LDS_BARRIER;

	{	//	Prefix sum
		int idx = 2*lIdx + (WG_SIZE+1);
		if( lIdx < 64 )
		{
			ldsSortData[idx] += ldsSortData[idx-1];
			GROUP_MEM_FENCE;
			ldsSortData[idx] += ldsSortData[idx-2];					
			GROUP_MEM_FENCE;
			ldsSortData[idx] += ldsSortData[idx-4];
			GROUP_MEM_FENCE;
			ldsSortData[idx] += ldsSortData[idx-8];
			GROUP_MEM_FENCE;
			ldsSortData[idx] += ldsSortData[idx-16];
			GROUP_MEM_FENCE;
			ldsSortData[idx] += ldsSortData[idx-32];		
			GROUP_MEM_FENCE;
			ldsSortData[idx] += ldsSortData[idx-64];
			GROUP_MEM_FENCE;

			ldsSortData[idx-1] += ldsSortData[idx-2];
			GROUP_MEM_FENCE;
		}
	}

	GROUP_LDS_BARRIER;

	totalSum = ldsSortData[WG_SIZE*2-1];
	uint addValue = ldsSortData[lIdx+127];
	return pData + make_uint4(addValue, addValue, addValue, addValue);
}

void generateHistogram(u32 lIdx, u32 wgIdx, 
		uint4 sortedData)
{
    if( lIdx < (1<<BITS_PER_PASS) )
    {
    	ldsSortData[lIdx] = 0;
    }

	int mask = ((1<<BITS_PER_PASS)-1);
	uint4 keys = make_uint4( (sortedData.x)&mask, (sortedData.y)&mask, (sortedData.z)&mask, (sortedData.w)&mask );

	GROUP_LDS_BARRIER;
	
	AtomInc( ldsSortData[keys.x] );
	AtomInc( ldsSortData[keys.y] );
	AtomInc( ldsSortData[keys.z] );
	AtomInc( ldsSortData[keys.w] );
}

[numthreads(WG_SIZE, 1, 1)]
void LocalSortKernel( DEFAULT_ARGS )
{
	int nElemsPerWG = WG_SIZE*NUM_PER_WI;
	u32 lIdx = GET_LOCAL_IDX;
	u32 wgIdx = GET_GROUP_IDX;
	u32 wgSize = GET_GROUP_SIZE;

    uint4 localAddr = make_uint4(lIdx*4+0,lIdx*4+1,lIdx*4+2,lIdx*4+3);


	SortData sortData[NUM_PER_WI];

	{
		u32 offset = nElemsPerWG*wgIdx;
		sortData[0] = sortDataIn[offset+localAddr.x];
		sortData[1] = sortDataIn[offset+localAddr.y];
		sortData[2] = sortDataIn[offset+localAddr.z];
		sortData[3] = sortDataIn[offset+localAddr.w];
	}

	int bitIdx = m_startBit;
	do
	{
//	what is this?
//		if( lIdx == wgSize-1 ) ldsSortData[256] = sortData[3].m_key;
		u32 mask = (1<<bitIdx);
		uint4 cmpResult = make_uint4( sortData[0].m_key & mask, sortData[1].m_key & mask, sortData[2].m_key & mask, sortData[3].m_key & mask );
		uint4 prefixSum = SELECT_UINT4( make_uint4(1,1,1,1), make_uint4(0,0,0,0), cmpResult != make_uint4(0,0,0,0) );
		u32 total;
		prefixSum = localPrefixSum128V( prefixSum, lIdx, total );

		{
			uint4 dstAddr = localAddr - prefixSum + make_uint4( total, total, total, total );
			dstAddr = SELECT_UINT4( prefixSum, dstAddr, cmpResult != make_uint4(0, 0, 0, 0) );

			GROUP_LDS_BARRIER;

			ldsSortData[dstAddr.x] = sortData[0].m_key;
			ldsSortData[dstAddr.y] = sortData[1].m_key;
			ldsSortData[dstAddr.z] = sortData[2].m_key;
			ldsSortData[dstAddr.w] = sortData[3].m_key;

			GROUP_LDS_BARRIER;

			sortData[0].m_key = ldsSortData[localAddr.x];
			sortData[1].m_key = ldsSortData[localAddr.y];
			sortData[2].m_key = ldsSortData[localAddr.z];
			sortData[3].m_key = ldsSortData[localAddr.w];

			GROUP_LDS_BARRIER;

			ldsSortData[dstAddr.x] = sortData[0].m_value;
			ldsSortData[dstAddr.y] = sortData[1].m_value;
			ldsSortData[dstAddr.z] = sortData[2].m_value;
			ldsSortData[dstAddr.w] = sortData[3].m_value;

			GROUP_LDS_BARRIER;

			sortData[0].m_value = ldsSortData[localAddr.x];
			sortData[1].m_value = ldsSortData[localAddr.y];
			sortData[2].m_value = ldsSortData[localAddr.z];
			sortData[3].m_value = ldsSortData[localAddr.w];

			GROUP_LDS_BARRIER;
		}
		bitIdx ++;
	}
	while( bitIdx <(m_startBit+BITS_PER_PASS) );

	{	//	generate historgram
		uint4 localKeys = make_uint4( sortData[0].m_key>>m_startBit, sortData[1].m_key>>m_startBit, 
			sortData[2].m_key>>m_startBit, sortData[3].m_key>>m_startBit );

		generateHistogram( lIdx, wgIdx, localKeys );

		GROUP_LDS_BARRIER;

		int nBins = (1<<BITS_PER_PASS);
		if( lIdx < nBins )
		{
     		u32 histValues = ldsSortData[lIdx];

     		u32 globalAddresses = nBins*wgIdx + lIdx;
     		u32 globalAddressesRadixMajor = m_numGroups*lIdx + wgIdx;
		
     		ldsHistogramOut0[globalAddressesRadixMajor] = histValues;
     		ldsHistogramOut1[globalAddresses] = histValues;
		}
	}

	{	//	write
		u32 offset = nElemsPerWG*wgIdx;
		uint4 dstAddr = make_uint4(offset+localAddr.x, offset+localAddr.y, offset+localAddr.z, offset+localAddr.w );

		sortDataIn[ dstAddr.x + 0 ] = sortData[0];
		sortDataIn[ dstAddr.x + 1 ] = sortData[1];
		sortDataIn[ dstAddr.x + 2 ] = sortData[2];
		sortDataIn[ dstAddr.x + 3 ] = sortData[3];
	}
}

StructuredBuffer<SortData> src : register( t0 );
StructuredBuffer<u32> histogramGlobalRadixMajor : register( t1 );
StructuredBuffer<u32> histogramLocalGroupMajor : register( t2 );

RWStructuredBuffer<SortData> dst : register( u0 );

groupshared u32 ldsLocalHistogram[ 2*(1<<BITS_PER_PASS) ];
groupshared u32 ldsGlobalHistogram[ (1<<BITS_PER_PASS) ];


[numthreads(WG_SIZE, 1, 1)]
void ScatterKernel( DEFAULT_ARGS )
{
	u32 lIdx = GET_LOCAL_IDX;
	u32 wgIdx = GET_GROUP_IDX;
	u32 ldsOffset = (1<<BITS_PER_PASS);

	//	load and prefix scan local histogram
	if( lIdx < ((1<<BITS_PER_PASS)/2) )
	{
		uint2 myIdx = make_uint2(lIdx, lIdx+8);

		ldsLocalHistogram[ldsOffset+myIdx.x] = histogramLocalGroupMajor[(1<<BITS_PER_PASS)*wgIdx + myIdx.x];
		ldsLocalHistogram[ldsOffset+myIdx.y] = histogramLocalGroupMajor[(1<<BITS_PER_PASS)*wgIdx + myIdx.y];
		ldsLocalHistogram[ldsOffset+myIdx.x-(1<<BITS_PER_PASS)] = 0;
		ldsLocalHistogram[ldsOffset+myIdx.y-(1<<BITS_PER_PASS)] = 0;

		int idx = ldsOffset+2*lIdx;
		ldsLocalHistogram[idx] += ldsLocalHistogram[idx-1];
		GROUP_MEM_FENCE;
		ldsLocalHistogram[idx] += ldsLocalHistogram[idx-2];
		GROUP_MEM_FENCE;
		ldsLocalHistogram[idx] += ldsLocalHistogram[idx-4];
		GROUP_MEM_FENCE;
		ldsLocalHistogram[idx] += ldsLocalHistogram[idx-8];
		GROUP_MEM_FENCE;

		// Propagate intermediate values through
		ldsLocalHistogram[idx-1] += ldsLocalHistogram[idx-2];
		GROUP_MEM_FENCE;

		// Grab and propagate for whole WG - loading the - 1 value
		uint2 localValues;
		localValues.x = ldsLocalHistogram[ldsOffset+myIdx.x-1];
		localValues.y = ldsLocalHistogram[ldsOffset+myIdx.y-1];

		ldsLocalHistogram[myIdx.x] = localValues.x;
		ldsLocalHistogram[myIdx.y] = localValues.y;


		ldsGlobalHistogram[myIdx.x] = histogramGlobalRadixMajor[m_numGroups*myIdx.x + wgIdx];
		ldsGlobalHistogram[myIdx.y] = histogramGlobalRadixMajor[m_numGroups*myIdx.y + wgIdx];
	}

	GROUP_LDS_BARRIER;

    uint4 localAddr = make_uint4(lIdx*4+0,lIdx*4+1,lIdx*4+2,lIdx*4+3);

	SortData sortData[4];
	{
	    uint4 globalAddr = wgIdx*WG_SIZE*NUM_PER_WI + localAddr;
		sortData[0] = src[globalAddr.x];
		sortData[1] = src[globalAddr.y];
		sortData[2] = src[globalAddr.z];
		sortData[3] = src[globalAddr.w];
	}

	uint cmpValue = ((1<<BITS_PER_PASS)-1);
	uint4 radix = make_uint4( (sortData[0].m_key>>m_startBit)&cmpValue, (sortData[1].m_key>>m_startBit)&cmpValue, 
		(sortData[2].m_key>>m_startBit)&cmpValue, (sortData[3].m_key>>m_startBit)&cmpValue );;

	//	data is already sorted. So simply subtract local prefix sum
	uint4 dstAddr;
	dstAddr.x = ldsGlobalHistogram[radix.x] + (localAddr.x - ldsLocalHistogram[radix.x]);
	dstAddr.y = ldsGlobalHistogram[radix.y] + (localAddr.y - ldsLocalHistogram[radix.y]);
	dstAddr.z = ldsGlobalHistogram[radix.z] + (localAddr.z - ldsLocalHistogram[radix.z]);
	dstAddr.w = ldsGlobalHistogram[radix.w] + (localAddr.w - ldsLocalHistogram[radix.w]);

	dst[dstAddr.x] = sortData[0];
	dst[dstAddr.y] = sortData[1];
	dst[dstAddr.z] = sortData[2];
	dst[dstAddr.w] = sortData[3];
}

[numthreads(WG_SIZE, 1, 1)]
void CopyKernel( DEFAULT_ARGS )
{
	dst[ GET_GLOBAL_IDX ] = src[ GET_GLOBAL_IDX ];
}
