/*
 * Copyright 1993-2006 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO USER:
 *
 * This source code is subject to NVIDIA ownership rights under U.S. and
 * international Copyright laws.
 *
 * NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE
 * CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR
 * IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
 * IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE
 * OR PERFORMANCE OF THIS SOURCE CODE.
 *
 * U.S. Government End Users.  This source code is a "commercial item" as
 * that term is defined at 48 C.F.R. 2.101 (OCT 1995), consisting  of
 * "commercial computer software" and "commercial computer software
 * documentation" as such terms are used in 48 C.F.R. 12.212 (SEPT 1995)
 * and is provided to the U.S. Government only as a commercial end item.
 * Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through
 * 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the
 * source code with only those rights set forth herein.
 */

/* Radixsort project with key/value and arbitrary datset size support
 * which demonstrates the use of CUDA in a multi phase sorting 
 * computation.
 * Host code.
 */

#include "radixsort.cuh"
#include "radixsort_kernel.cu"

extern "C"
{

////////////////////////////////////////////////////////////////////////////////
//! Perform a radix sort
//! Sorting performed in place on passed arrays.
//!
//! @param pData0       input and output array - data will be sorted
//! @param pData1       additional array to allow ping pong computation
//! @param elements     number of elements to sort
////////////////////////////////////////////////////////////////////////////////
void RadixSort(KeyValuePair *pData0, KeyValuePair *pData1, uint elements, uint bits)
{
    // Round element count to total number of threads for efficiency
    uint elements_rounded_to_3072;
    int modval = elements % 3072;
    if( modval == 0 )
        elements_rounded_to_3072 = elements;
    else
        elements_rounded_to_3072 = elements + (3072 - (modval));

    // Iterate over n bytes of y bit word, using each byte to sort the list in turn
    for (uint shift = 0; shift < bits; shift += RADIX)
    {
        // Perform one round of radix sorting

        // Generate per radix group sums radix counts across a radix group
        RadixSum<<<NUM_BLOCKS, NUM_THREADS_PER_BLOCK, GRFSIZE>>>(pData0, elements, elements_rounded_to_3072, shift);
        // Prefix sum in radix groups, and then between groups throughout a block
        RadixPrefixSum<<<PREFIX_NUM_BLOCKS, PREFIX_NUM_THREADS_PER_BLOCK, PREFIX_GRFSIZE>>>();
        // Sum the block offsets and then shuffle data into bins
        RadixAddOffsetsAndShuffle<<<NUM_BLOCKS, NUM_THREADS_PER_BLOCK, SHUFFLE_GRFSIZE>>>(pData0, pData1, elements, elements_rounded_to_3072, shift); 

        // Exchange data pointers
        KeyValuePair* pTemp = pData0;
        pData0 = pData1;
        pData1 = pTemp;
   }
}

}
