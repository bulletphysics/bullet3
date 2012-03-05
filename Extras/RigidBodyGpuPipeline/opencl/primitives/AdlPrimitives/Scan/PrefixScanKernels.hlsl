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


typedef uint u32;

#define GET_GROUP_IDX groupIdx.x
#define GET_LOCAL_IDX localIdx.x
#define GET_GLOBAL_IDX globalIdx.x
#define GROUP_LDS_BARRIER GroupMemoryBarrierWithGroupSync()

//	takahiro end
#define WG_SIZE 128

#define GET_GROUP_SIZE WG_SIZE


cbuffer SortCB : register( b0 )
{
	int m_numElems;
	int m_numBlocks;
	int m_numScanBlocks;
};
 
RWStructuredBuffer<uint> dst : register( u0 );
RWStructuredBuffer<uint> src : register( u1 );
RWStructuredBuffer<uint> sumBuffer : register( u2 );


groupshared u32 ldsData[2048];

u32 ScanExclusive(u32 n, int lIdx, int lSize)
{
	u32 blocksum;
    int offset = 1;
    for(int nActive=n>>1; nActive>0; nActive>>=1, offset<<=1)
    {
        GROUP_LDS_BARRIER;
        for(int iIdx=lIdx; iIdx<nActive; iIdx+=lSize)
        {
            int ai = offset*(2*iIdx+1)-1;
            int bi = offset*(2*iIdx+2)-1;
            ldsData[bi] += ldsData[ai];
        }
	}

    GROUP_LDS_BARRIER;

    if( lIdx == 0 )
	{
		blocksum = ldsData[ n-1 ];
        ldsData[ n-1 ] = 0;
	}

	GROUP_LDS_BARRIER;

	offset >>= 1;
    for(int nActive=1; nActive<n; nActive<<=1, offset>>=1 )
    {
        GROUP_LDS_BARRIER;
        for( int iIdx = lIdx; iIdx<nActive; iIdx += lSize )
        {
            int ai = offset*(2*iIdx+1)-1;
            int bi = offset*(2*iIdx+2)-1;
            u32 temp = ldsData[ai];
            ldsData[ai] = ldsData[bi];
            ldsData[bi] += temp;
        }
	}
	GROUP_LDS_BARRIER;

	return blocksum;
}

[numthreads(WG_SIZE, 1, 1)]
void LocalScanKernel(uint3 globalIdx : SV_DispatchThreadID, uint3 localIdx : SV_GroupThreadID, uint3 groupIdx : SV_GroupID)
{
	int gIdx = GET_GLOBAL_IDX;
	int lIdx = GET_LOCAL_IDX;

	ldsData[2*lIdx]     = ( 2*gIdx < m_numElems )? src[2*gIdx]: 0;
	ldsData[2*lIdx + 1] = ( 2*gIdx+1 < m_numElems )? src[2*gIdx + 1]: 0;

	u32 sum = ScanExclusive(WG_SIZE*2, GET_LOCAL_IDX, GET_GROUP_SIZE);

	if( lIdx == 0 ) sumBuffer[GET_GROUP_IDX] = sum;

	if( (2*gIdx) < m_numElems )
    {
        dst[2*gIdx]     = ldsData[2*lIdx];
	}
	if( (2*gIdx + 1) < m_numElems )
	{
        dst[2*gIdx + 1] = ldsData[2*lIdx + 1];
    }
}

[numthreads(WG_SIZE, 1, 1)]
void TopLevelScanKernel(uint3 globalIdx : SV_DispatchThreadID, uint3 localIdx : SV_GroupThreadID, uint3 groupIdx : SV_GroupID)
{
	int gIdx = GET_GLOBAL_IDX;
	int lIdx = GET_LOCAL_IDX;
	int lSize = GET_GROUP_SIZE;

	for(int i=lIdx; i<m_numScanBlocks; i+=lSize )
	{
		ldsData[i] = (i<m_numBlocks)? dst[i]:0;
	}

	GROUP_LDS_BARRIER;

	u32 sum = ScanExclusive(m_numScanBlocks, GET_LOCAL_IDX, GET_GROUP_SIZE);

	for(int i=lIdx; i<m_numBlocks; i+=lSize )
	{
		dst[i] = ldsData[i];
	}

	if( gIdx == 0 )
	{
		dst[m_numBlocks] = sum;
	}
}


 
RWStructuredBuffer<uint> blockSum2 : register( u1 );

[numthreads(WG_SIZE, 1, 1)]
void AddOffsetKernel(uint3 globalIdx : SV_DispatchThreadID, uint3 localIdx : SV_GroupThreadID, uint3 groupIdx : SV_GroupID)
{
	const u32 blockSize = WG_SIZE*2;

	int myIdx = GET_GROUP_IDX+1;
	int llIdx = GET_LOCAL_IDX;

	u32 iBlockSum = blockSum2[myIdx];

	int endValue = min((myIdx+1)*(blockSize), m_numElems);
	for(int i=myIdx*blockSize+llIdx; i<endValue; i+=GET_GROUP_SIZE)
	{
		dst[i] += iBlockSum;
	}
}

