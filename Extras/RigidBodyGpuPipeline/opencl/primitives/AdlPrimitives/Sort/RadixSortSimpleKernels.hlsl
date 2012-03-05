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
 
StructuredBuffer<SortData> sortData : register( t0 );
RWStructuredBuffer<u32> ldsHistogramOut : register( u0 );

groupshared u32 ldsHistogram[16][256];

[numthreads(WG_SIZE, 1, 1)]
void LocalCountKernel( DEFAULT_ARGS )
{
	int lIdx = GET_LOCAL_IDX;
	int gIdx = GET_GLOBAL_IDX;
	
	for(int i=0; i<16; i++)
	{
		ldsHistogram[i][lIdx] = 0.f;
		ldsHistogram[i][lIdx+128] = 0.f;
	}
	
	GROUP_LDS_BARRIER;
	
	SortData datas[NUM_PER_WI];
	datas[0] = sortData[gIdx*NUM_PER_WI+0];
	datas[1] = sortData[gIdx*NUM_PER_WI+1];
	datas[2] = sortData[gIdx*NUM_PER_WI+2];
	datas[3] = sortData[gIdx*NUM_PER_WI+3];

	datas[0].m_key = (datas[0].m_key >> m_startBit) & 0xff;
	datas[1].m_key = (datas[1].m_key >> m_startBit) & 0xff;
	datas[2].m_key = (datas[2].m_key >> m_startBit) & 0xff;
	datas[3].m_key = (datas[3].m_key >> m_startBit) & 0xff;

	int tableIdx = lIdx%16;
	
	AtomInc(ldsHistogram[tableIdx][datas[0].m_key]);
	AtomInc(ldsHistogram[tableIdx][datas[1].m_key]);
	AtomInc(ldsHistogram[tableIdx][datas[2].m_key]);
	AtomInc(ldsHistogram[tableIdx][datas[3].m_key]);

	GROUP_LDS_BARRIER;
	
	u32 sum0, sum1;
	sum0 = sum1 = 0;
	for(int i=0; i<16; i++)
	{
		sum0 += ldsHistogram[i][lIdx];
		sum1 += ldsHistogram[i][lIdx+128];
	}

	ldsHistogramOut[lIdx*m_numGroups+GET_GROUP_IDX] = sum0;
	ldsHistogramOut[(lIdx+128)*m_numGroups+GET_GROUP_IDX] = sum1;
}


RWStructuredBuffer<SortData> sortDataOut : register( u0 );
RWStructuredBuffer<u32> scannedHistogram : register( u1 );

groupshared u32 ldsCurrentLocation[256];

[numthreads(WG_SIZE, 1, 1)]
void ScatterKernel( DEFAULT_ARGS )
{
	int lIdx = GET_LOCAL_IDX;
	int gIdx = GET_GLOBAL_IDX;
	
	{
		ldsCurrentLocation[lIdx] = scannedHistogram[lIdx*m_numGroups+GET_GROUP_IDX];
		ldsCurrentLocation[lIdx+128] = scannedHistogram[(lIdx+128)*m_numGroups+GET_GROUP_IDX];
	}

	GROUP_LDS_BARRIER;
	
	SortData datas[NUM_PER_WI];
	int keys[NUM_PER_WI];
	datas[0] = sortData[gIdx*NUM_PER_WI+0];
	datas[1] = sortData[gIdx*NUM_PER_WI+1];
	datas[2] = sortData[gIdx*NUM_PER_WI+2];
	datas[3] = sortData[gIdx*NUM_PER_WI+3];

	keys[0] = (datas[0].m_key >> m_startBit) & 0xff;
	keys[1] = (datas[1].m_key >> m_startBit) & 0xff;
	keys[2] = (datas[2].m_key >> m_startBit) & 0xff;
	keys[3] = (datas[3].m_key >> m_startBit) & 0xff;

	int dst[NUM_PER_WI];
	for(int i=0; i<WG_SIZE; i++)
//	for(int i=0; i<m_padding[0]; i++)	//	to reduce compile time
	{
		if( i==lIdx )
		{
			AtomInc1(ldsCurrentLocation[keys[0]], dst[0]);
			AtomInc1(ldsCurrentLocation[keys[1]], dst[1]);
			AtomInc1(ldsCurrentLocation[keys[2]], dst[2]);
			AtomInc1(ldsCurrentLocation[keys[3]], dst[3]);
		}
		GROUP_LDS_BARRIER;
	}
	sortDataOut[dst[0]] = datas[0];
	sortDataOut[dst[1]] = datas[1];
	sortDataOut[dst[2]] = datas[2];
	sortDataOut[dst[3]] = datas[3];
}
