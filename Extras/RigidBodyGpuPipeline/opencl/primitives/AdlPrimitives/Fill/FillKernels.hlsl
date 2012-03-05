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
#define GROUP_MEM_FENCE
#define DEFAULT_ARGS uint3 globalIdx : SV_DispatchThreadID, uint3 localIdx : SV_GroupThreadID, uint3 groupIdx : SV_GroupID
#define AtomInc(x) InterlockedAdd(x, 1)
#define AtomInc1(x, out) InterlockedAdd(x, 1, out)

#define make_uint4 uint4
#define make_uint2 uint2
#define make_int2 int2


cbuffer CB : register( b0 )
{
	int4 m_data;
	int m_offset;
	int m_n;
	int m_padding[2];
};


RWStructuredBuffer<int> dstInt : register( u0 );

[numthreads(64, 1, 1)]
void FillIntKernel( DEFAULT_ARGS )
{
	int gIdx = GET_GLOBAL_IDX;

	if( gIdx < m_n )
	{
		dstInt[ m_offset+gIdx ] = m_data.x;
	}
}

RWStructuredBuffer<int2> dstInt2 : register( u0 );

[numthreads(64, 1, 1)]
void FillInt2Kernel( DEFAULT_ARGS )
{
	int gIdx = GET_GLOBAL_IDX;

	if( gIdx < m_n )
	{
		dstInt2[ m_offset+gIdx ] = make_int2( m_data.x, m_data.y );
	}
}

RWStructuredBuffer<int4> dstInt4 : register( u0 );

[numthreads(64, 1, 1)]
void FillInt4Kernel( DEFAULT_ARGS )
{
	int gIdx = GET_GLOBAL_IDX;

	if( gIdx < m_n )
	{
		dstInt4[ m_offset+gIdx ] = m_data;
	}
}
