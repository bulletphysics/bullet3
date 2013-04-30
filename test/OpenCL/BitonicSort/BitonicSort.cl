MSTRINGIFY(
/*
 * Copyright 1993-2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and 
 * proprietary rights in and to this software and related documentation. 
 * Any use, reproduction, disclosure, or distribution of this software 
 * and related documentation without an express license agreement from
 * NVIDIA Corporation is strictly prohibited.
 *
 * Please refer to the applicable NVIDIA end user license agreement (EULA) 
 * associated with this source code for terms and conditions that govern 
 * your use of this NVIDIA software.
 * 
 */



inline void ComparatorPrivate(int2* keyA, int2* keyB, uint dir)
{
    if((keyA[0].x > keyB[0].x) == dir)
    {
		int2 tmp = *keyA;
		*keyA = *keyB;
		*keyB = tmp;
    }
}

inline void ComparatorLocal(__local int2* keyA, __local int2* keyB, uint dir)
{
    if((keyA[0].x > keyB[0].x) == dir)
    {
		int2 tmp = *keyA;
		*keyA = *keyB;
		*keyB = tmp;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Monolithic bitonic sort kernel for short arrays fitting into local memory
////////////////////////////////////////////////////////////////////////////////
__kernel void kBitonicSortCellIdLocal(__global int2* pKey, uint arrayLength, uint dir GUID_ARG)
{
    __local int2 l_key[1024U];
    int localSizeLimit = get_local_size(0) * 2;

    //Offset to the beginning of subbatch and load data
    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

    for(uint size = 2; size < arrayLength; size <<= 1)
    {
        //Bitonic merge
        uint ddd = dir ^ ( (get_local_id(0) & (size / 2)) != 0 );
        for(uint stride = size / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos +      0], &l_key[pos + stride], ddd);
        }
    }

    //ddd == dir for the last bitonic merge step
    {
        for(uint stride = arrayLength / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], dir);
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}

////////////////////////////////////////////////////////////////////////////////
// Bitonic sort kernel for large arrays (not fitting into local memory)
////////////////////////////////////////////////////////////////////////////////
//Bottom-level bitonic sort
//Almost the same as bitonicSortLocal with the only exception
//of even / odd subarrays (of LOCAL_SIZE_LIMIT points) being
//sorted in opposite directions
__kernel void kBitonicSortCellIdLocal1(__global int2* pKey GUID_ARG)
{
    __local int2 l_key[1024U];
    uint localSizeLimit = get_local_size(0) * 2;

    //Offset to the beginning of subarray and load data
    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

    uint comparatorI = get_global_id(0) & ((localSizeLimit / 2) - 1);

    for(uint size = 2; size < localSizeLimit; size <<= 1)
    {
        //Bitonic merge
        uint ddd = (comparatorI & (size / 2)) != 0;
        for(uint stride = size / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
        }
    }

    //Odd / even arrays of localSizeLimit elements
    //sorted in opposite directions
    {
        uint ddd = (get_group_id(0) & 1);
        for(uint stride = localSizeLimit / 2; stride > 0; stride >>= 1)
        {
            barrier(CLK_LOCAL_MEM_FENCE);
            uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
            ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}

//Bitonic merge iteration for 'stride' >= LOCAL_SIZE_LIMIT
__kernel void kBitonicSortCellIdMergeGlobal(__global int2* pKey, uint arrayLength, uint size, uint stride, uint dir GUID_ARG)
{
    uint global_comparatorI = get_global_id(0);
    uint        comparatorI = global_comparatorI & (arrayLength / 2 - 1);

    //Bitonic merge
    uint ddd = dir ^ ( (comparatorI & (size / 2)) != 0 );
    uint pos = 2 * global_comparatorI - (global_comparatorI & (stride - 1));

    int2 keyA = pKey[pos +      0];
    int2 keyB = pKey[pos + stride];

    ComparatorPrivate(&keyA, &keyB, ddd);

    pKey[pos +      0] = keyA;
    pKey[pos + stride] = keyB;
}

//Combined bitonic merge steps for
//'size' > LOCAL_SIZE_LIMIT and 'stride' = [1 .. LOCAL_SIZE_LIMIT / 2]
__kernel void kBitonicSortCellIdMergeLocal(__global int2* pKey, uint arrayLength, uint stride, uint size, uint dir GUID_ARG)
{
    __local int2 l_key[1024U];
    int localSizeLimit = get_local_size(0) * 2;

    pKey += get_group_id(0) * localSizeLimit + get_local_id(0);
    l_key[get_local_id(0) +                    0] = pKey[                   0];
    l_key[get_local_id(0) + (localSizeLimit / 2)] = pKey[(localSizeLimit / 2)];

    //Bitonic merge
    uint comparatorI = get_global_id(0) & ((arrayLength / 2) - 1);
    uint         ddd = dir ^ ( (comparatorI & (size / 2)) != 0 );
    for(; stride > 0; stride >>= 1)
    {
        barrier(CLK_LOCAL_MEM_FENCE);
        uint pos = 2 * get_local_id(0) - (get_local_id(0) & (stride - 1));
        ComparatorLocal(&l_key[pos + 0], &l_key[pos + stride], ddd);
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    pKey[                   0] = l_key[get_local_id(0) +                    0];
    pKey[(localSizeLimit / 2)] = l_key[get_local_id(0) + (localSizeLimit / 2)];
}
);