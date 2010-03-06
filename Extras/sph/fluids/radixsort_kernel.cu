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
 * Device code.
 */

#ifndef _RADIXSORT_KERNEL_H_
#define _RADIXSORT_KERNEL_H_

#include <stdio.h>
#include "radixsort.cuh"

#define SYNCIT __syncthreads()

static const int NUM_SMS = 16;
static const int NUM_THREADS_PER_SM = 192;
static const int NUM_THREADS_PER_BLOCK = 64;
//static const int NUM_THREADS = NUM_THREADS_PER_SM * NUM_SMS;
static const int NUM_BLOCKS = (NUM_THREADS_PER_SM / NUM_THREADS_PER_BLOCK) * NUM_SMS;
static const int RADIX = 8;                                                        // Number of bits per radix sort pass
static const int RADICES = 1 << RADIX;                                             // Number of radices
static const int RADIXMASK = RADICES - 1;                                          // Mask for each radix sort pass
#if SIXTEEN
static const int RADIXBITS = 16;                                                   // Number of bits to sort over
#else
static const int RADIXBITS = 32;                                                   // Number of bits to sort over
#endif
static const int RADIXTHREADS = 16;                                                // Number of threads sharing each radix counter
static const int RADIXGROUPS = NUM_THREADS_PER_BLOCK / RADIXTHREADS;               // Number of radix groups per CTA
static const int TOTALRADIXGROUPS = NUM_BLOCKS * RADIXGROUPS;                      // Number of radix groups for each radix
static const int SORTRADIXGROUPS = TOTALRADIXGROUPS * RADICES;                     // Total radix count
static const int GRFELEMENTS = (NUM_THREADS_PER_BLOCK / RADIXTHREADS) * RADICES; 
static const int GRFSIZE = GRFELEMENTS * sizeof(uint); 

// Prefix sum variables
static const int PREFIX_NUM_THREADS_PER_SM = NUM_THREADS_PER_SM;
static const int PREFIX_NUM_THREADS_PER_BLOCK = PREFIX_NUM_THREADS_PER_SM;
static const int PREFIX_NUM_BLOCKS = (PREFIX_NUM_THREADS_PER_SM / PREFIX_NUM_THREADS_PER_BLOCK) * NUM_SMS;
static const int PREFIX_BLOCKSIZE = SORTRADIXGROUPS / PREFIX_NUM_BLOCKS;
static const int PREFIX_GRFELEMENTS = PREFIX_BLOCKSIZE + 2 * PREFIX_NUM_THREADS_PER_BLOCK;
static const int PREFIX_GRFSIZE = PREFIX_GRFELEMENTS * sizeof(uint);

// Shuffle variables
static const int SHUFFLE_GRFOFFSET = RADIXGROUPS * RADICES;
static const int SHUFFLE_GRFELEMENTS = SHUFFLE_GRFOFFSET + PREFIX_NUM_BLOCKS; 
static const int SHUFFLE_GRFSIZE = SHUFFLE_GRFELEMENTS * sizeof(uint); 


#define SDATA( index)      CUT_BANK_CHECKER(sdata, index)

// Prefix sum data
uint gRadixSum[TOTALRADIXGROUPS * RADICES];
__device__ uint dRadixSum[TOTALRADIXGROUPS * RADICES];
uint gRadixBlockSum[PREFIX_NUM_BLOCKS];
__device__ uint dRadixBlockSum[PREFIX_NUM_BLOCKS];

extern __shared__ uint sRadixSum[];



////////////////////////////////////////////////////////////////////////////////
//! Perform a radix sum on the list to be sorted.  Each SM holds a set of
//! radix counters for each group of RADIXGROUPS thread in the GRF. 
//!
//! @param pData     input data
//! @param elements  total number of elements
//! @param elements_rounded_to_3072  total number of elements rounded up to the 
//!                                  nearest multiple of 3072
//! @param shift     the shift (0 to 24) that we are using to obtain the correct 
//!                  byte
////////////////////////////////////////////////////////////////////////////////
__global__ void RadixSum(KeyValuePair *pData, uint elements, uint elements_rounded_to_3072, uint shift)
{
    uint pos    = threadIdx.x;

    // Zero radix counts
    while (pos < GRFELEMENTS)
    {
        sRadixSum[pos] = 0;
        pos += NUM_THREADS_PER_BLOCK;
    }

    // Sum up data
    // Source addresses computed so that each thread is reading from a block of 
    // consecutive addresses so there are no conflicts between threads
    // They then loop over their combined region and the next batch works elsewhere.
    // So threads 0 to 16 work on memory 0 to 320.
    // First reading 0,1,2,3...15 then 16,17,18,19...31 and so on
    // optimising parallel access to shared memory by a thread accessing 16*threadID
    // The next radix group runs from 320 to 640 and the same applies in that region
    uint tmod   =   threadIdx.x % RADIXTHREADS;
    uint tpos   =   threadIdx.x / RADIXTHREADS;

    // Take the rounded element list size so that all threads have a certain size dataset to work with
    // and no zero size datasets confusing the issue
    // By using a multiple of 3072 we ensure that all threads have elements
    // to work with until the last phase, at which point we individually test
    uint element_fraction  =   elements_rounded_to_3072 / TOTALRADIXGROUPS;

    // Generate range 
    // Note that it is possible for both pos and end to be past the end of the element set
    // which will be caught later.
    pos       = (blockIdx.x * RADIXGROUPS + tpos) * element_fraction;
    uint end  = pos + element_fraction;
    pos      += tmod; 
    //printf("pos: %d\n", pos);
    __syncthreads();

    while (pos < end )
    {
        uint key = 0;

        // Read first data element if we are in the set of elements
        //if( pos < elements )
            //key = pData[pos].key;
        KeyValuePair kvp;
        // Read first data element, both items at once as the memory will want to coalesce like that anyway
        if (pos < elements)
            kvp = pData[pos];
        else
            kvp.key = 0;
        key = kvp.key;


        // Calculate position of radix counter to increment
        // There are RADICES radices in each pass (256)
        // and hence this many counters for bin grouping
        // Multiply by RADIXGROUPS (4) to spread through memory
        // and into 4 radix groups
        uint p = ((key >> shift) & RADIXMASK) * RADIXGROUPS;
       
        // Increment radix counters
        // Each radix group has its own set of counters
        // so we add the thread position [0-3], ie the group index.
        // We slow down here and take at least 16 cycles to write to the summation boxes
        // but other groups will only conflict with themselves and so can also be writing
        // 16 cycles here at least avoids retries.
        uint ppos = p + tpos;

        // If we are past the last element we don't want to do anything
        // We do have to check each time, however, to ensure that all
        // threads sync on each sync here.
        if (tmod == 0 && pos < elements)
            sRadixSum[ppos]++;
            SYNCIT;
        if (tmod == 1 && pos < elements)
            sRadixSum[ppos]++;  
            SYNCIT;          
        if (tmod == 2 && pos < elements)
            sRadixSum[ppos]++;   
            SYNCIT; 
        if (tmod == 3 && pos < elements)
            sRadixSum[ppos]++; 
            SYNCIT;  
        if (tmod == 4 && pos < elements)
            sRadixSum[ppos]++;
            SYNCIT;
        if (tmod == 5 && pos < elements)
            sRadixSum[ppos]++; 
            SYNCIT;           
        if (tmod == 6 && pos < elements)
            sRadixSum[ppos]++;
            SYNCIT;   
        if (tmod == 7 && pos < elements)
            sRadixSum[ppos]++;  
            SYNCIT;   
        if (tmod == 8 && pos < elements)
            sRadixSum[ppos]++;
            SYNCIT;
        if (tmod == 9 && pos < elements)
            sRadixSum[ppos]++; 
            SYNCIT;           
        if (tmod == 10 && pos < elements)
            sRadixSum[ppos]++;
            SYNCIT;    
        if (tmod == 11 && pos < elements)
            sRadixSum[ppos]++; 
            SYNCIT;     
        if (tmod == 12 && pos < elements)
            sRadixSum[ppos]++;
            SYNCIT;
        if (tmod == 13 && pos < elements)
            sRadixSum[ppos]++; 
            SYNCIT;           
        if (tmod == 14 && pos < elements)
            sRadixSum[ppos]++; 
            SYNCIT;   
        if (tmod == 15 && pos < elements)
            sRadixSum[ppos]++; 
            SYNCIT;   
           
        pos += RADIXTHREADS;
                
    }

    __syncthreads();

    __syncthreads();

    // Output radix sums into separate memory regions for each radix group
    // So this memory then is layed out:
    // 0...... 192..... 384 ................ 192*256
    // ie all 256 bins for each radix group
    // in there:
    // 0.............192
    // 0  4  8  12...     - block idx * 4
    // And in the block boxes we see the 4 radix groups for that block
    // So 0-192 should contain bin 0 for each radix group, and so on
    uint offset = blockIdx.x * RADIXGROUPS;
    uint row    = threadIdx.x / RADIXGROUPS;
    uint column = threadIdx.x % RADIXGROUPS;
    while (row < RADICES)
    {
        dRadixSum[offset + row * TOTALRADIXGROUPS + column] = sRadixSum[row * RADIXGROUPS + column];
        row += NUM_THREADS_PER_BLOCK / RADIXGROUPS;
    }
}

////////////////////////////////////////////////////////////////////////////////
//! Performs first part of parallel prefix sum - individual sums of each radix
//! count. By the end of this we have prefix sums on a block level in dRadixSum
//! and totals for blocks in dRadixBlockSum.
////////////////////////////////////////////////////////////////////////////////
__global__ void RadixPrefixSum()
{
    // Read radix groups in offset by one in the GRF so a zero can be inserted at the beginning
    // and the final sum of all radix counts summed here is tacked onto the end for reading by
    // the next stage
    // Each block in this case is the full number of threads per SM (and hence the total number 
    // of radix groups), 192. We should then have the total set of offsets for an entire radix 
    // group by the end of this stage
    // Device mem addressing
    
    uint brow       = blockIdx.x * (RADICES / PREFIX_NUM_BLOCKS);
    uint drow       = threadIdx.x / TOTALRADIXGROUPS; // In default parameterisation this is always 0
    uint dcolumn    = threadIdx.x % TOTALRADIXGROUPS; // And similarly this is always the same as threadIdx.x   
    uint dpos       = (brow + drow) * TOTALRADIXGROUPS + dcolumn;
    uint end        = ((blockIdx.x + 1) * (RADICES / PREFIX_NUM_BLOCKS)) * TOTALRADIXGROUPS;
    // Shared mem addressing
    uint srow       = threadIdx.x / (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK);
    uint scolumn    = threadIdx.x % (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK);
    uint spos       = srow * (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1) + scolumn;

    // Read (RADICES / PREFIX_NUM_BLOCKS) radix counts into the GRF alongside each other
    while (dpos < end)
    {
        sRadixSum[spos] = dRadixSum[dpos];
        spos += (PREFIX_NUM_THREADS_PER_BLOCK / (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK)) * 
                (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1);
        dpos += (TOTALRADIXGROUPS / PREFIX_NUM_THREADS_PER_BLOCK) * TOTALRADIXGROUPS;
    }
    __syncthreads();
       
    // Perform preliminary sum on each thread's stretch of data
    // Each thread having a block of 16, with spacers between 0...16 18...33 and so on
    int pos     = threadIdx.x * (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1);
    end         = pos + (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK);
    uint sum    = 0;
    while (pos < end)
    {
        sum += sRadixSum[pos];
        sRadixSum[pos] = sum;
        pos++;  
    }
    __syncthreads();   
  
 
    // Calculate internal offsets by performing a more traditional parallel
    // prefix sum of the topmost member of each thread's work data.  Right now,
    // these are stored between the work data for each thread, allowing us to 
    // eliminate GRF conflicts as well as hold the offsets needed to complete the sum
    // In other words we have:
    // 0....15 16 17....32 33 34....
    // Where this first stage updates the intermediate values (so 16=15, 33=32 etc)
    int m           = (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1);
    pos             = threadIdx.x  * (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1) +
                      (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK);
    sRadixSum[pos]  = sRadixSum[pos - 1];
    __syncthreads();
    // This stage then performs a parallel prefix sum (ie use powers of 2 to propagate in log n stages)
    // to update 17, 34 etc with the totals to that point (so 34 becomes [34] + [17]) and so on.
    while (m < PREFIX_NUM_THREADS_PER_BLOCK * (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1))
    {
        int p  = pos - m;
        uint t = ((p > 0) ? sRadixSum[p] : 0);
        __syncthreads();
        sRadixSum[pos] += t;
        __syncthreads();
        m *= 2;
    } 
    __syncthreads();

  
  
    // Add internal offsets to each thread's work data.
    // So now we take 17 and add it to all values 18 to 33 so all offsets for that block 
    // are updated.
    pos     = threadIdx.x * (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1);
    end     = pos + (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK);
    int p   = pos - 1;
    sum     = ((p > 0) ? sRadixSum[p] : 0);
    while (pos < end)
    {
        sRadixSum[pos] += sum;
        pos++; 
    }
    __syncthreads();        
               
    // Write summed data back out to global memory in the same way as we read it in
    // We now have prefix sum values internal to groups
    brow       = blockIdx.x * (RADICES / PREFIX_NUM_BLOCKS);
    drow       = threadIdx.x / TOTALRADIXGROUPS;
    dcolumn    = threadIdx.x % TOTALRADIXGROUPS;    
    srow       = threadIdx.x / (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK);
    scolumn    = threadIdx.x % (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK);
    dpos       = (brow + drow) * TOTALRADIXGROUPS + dcolumn + 1;
    spos       = srow * (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1) + scolumn;
    end        = ((blockIdx.x + 1) * RADICES / PREFIX_NUM_BLOCKS) * TOTALRADIXGROUPS;
    while (dpos < end)
    {
        dRadixSum[dpos] = sRadixSum[spos];
        dpos += (TOTALRADIXGROUPS / PREFIX_NUM_THREADS_PER_BLOCK) * TOTALRADIXGROUPS;        
        spos += (PREFIX_NUM_THREADS_PER_BLOCK / (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK)) * 
                (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1);
    }

    // Write last element to summation
    // Storing block sums in a separate array
    if (threadIdx.x == 0) {
        dRadixBlockSum[blockIdx.x] = sRadixSum[PREFIX_NUM_THREADS_PER_BLOCK * (PREFIX_BLOCKSIZE / PREFIX_NUM_THREADS_PER_BLOCK + 1) - 1];
        dRadixSum[blockIdx.x * PREFIX_BLOCKSIZE] = 0;
    }
}


////////////////////////////////////////////////////////////////////////////////
//! Initially perform prefix sum of block totals to obtain final set of offsets.
//! Then make use of radix sums to perform a shuffling of the data into the 
//! correct bins.
//!
//! @param pSrc      input data
//! @param pDst      output data
//! @param elements  total number of elements
//! @param shift     the shift (0 to 24) that we are using to obtain the correct 
//!                  byte
////////////////////////////////////////////////////////////////////////////////
__global__ void RadixAddOffsetsAndShuffle(KeyValuePair* pSrc, KeyValuePair* pDst, uint elements, uint elements_rounded_to_3072, int shift)
{
    // Read offsets from previous blocks
    if (threadIdx.x == 0)
        sRadixSum[SHUFFLE_GRFOFFSET] = 0;
        
    if (threadIdx.x < PREFIX_NUM_BLOCKS - 1)
        sRadixSum[SHUFFLE_GRFOFFSET + threadIdx.x + 1] = dRadixBlockSum[threadIdx.x];
    __syncthreads();
    
    // Parallel prefix sum over block sums
    int pos = threadIdx.x;
    int n = 1;
    while (n < PREFIX_NUM_BLOCKS)
    {
        int ppos = pos - n;
        uint t0 = ((pos < PREFIX_NUM_BLOCKS) && (ppos >= 0)) ? sRadixSum[SHUFFLE_GRFOFFSET + ppos] : 0;
        __syncthreads();
        if (pos < PREFIX_NUM_BLOCKS)
            sRadixSum[SHUFFLE_GRFOFFSET + pos] += t0;
        __syncthreads(); 
        n *= 2;
    }

    // Read radix count data and add appropriate block offset
    // for each radix at the memory location for this thread
    // (where the other threads in the block will be reading
    // as well, hence the large stride).
    // There is one counter box per radix group per radix 
    // per block (4*256*3)
    // We use 64 threads to read the 4 radix groups set of radices 
    // for the block. 
    int row    = threadIdx.x / RADIXGROUPS;
    int column = threadIdx.x % RADIXGROUPS;
    int spos   = row * RADIXGROUPS + column;
    int dpos   = row * TOTALRADIXGROUPS + column + blockIdx.x * RADIXGROUPS;
    while (spos < SHUFFLE_GRFOFFSET)
    {
        sRadixSum[spos] = dRadixSum[dpos] + sRadixSum[SHUFFLE_GRFOFFSET + dpos / (TOTALRADIXGROUPS * RADICES / PREFIX_NUM_BLOCKS)];
        spos += NUM_THREADS_PER_BLOCK;
        dpos += (NUM_THREADS_PER_BLOCK / RADIXGROUPS) * TOTALRADIXGROUPS;
    }
    __syncthreads();

    //int pos;
    // Shuffle data
    // Each of the subbins for a block should be filled via the counters, properly interleaved
    // Then, as we now iterate over each data value, we increment the subbins (each thread in the 
    // radix group in turn to avoid miss writes due to conflicts) and set locations correctly.
    uint element_fraction  =   elements_rounded_to_3072 / TOTALRADIXGROUPS;
    int tmod   =   threadIdx.x % RADIXTHREADS;
    int tpos   =   threadIdx.x / RADIXTHREADS;

    pos       = (blockIdx.x * RADIXGROUPS + tpos) * element_fraction;
    uint end  = pos + element_fraction; //(blockIdx.x * RADIXGROUPS + tpos + 1) * element_fraction;
    pos      += tmod; 

    __syncthreads();

    while (pos < end )
    {
        KeyValuePair kvp;
#if 1 // old load
        // Read first data element, both items at once as the memory will want to coalesce like that anyway
        if (pos < elements)
        {
            kvp = pSrc[pos];
        }
        else
            kvp.key = 0;

#else // casting to float2 to get it to combine loads
        int2 kvpf2;

        // Read first data element, both items at once as the memory will want to coalesce like that anyway
        if (pos < elements)
        {
  //          kvp = pSrc[pos];
            kvpf2 = ((int2*)pSrc)[pos];
           // printf("kvp: %f %f  kvpf2: %f %f\n", kvp.key, kvp.value, kvpf2.x, kvpf2.y);
        }
        else
            //kvp.key = 0;
            kvpf2.x = 0;
 
        kvp.key = kvpf2.x;
        kvp.value = kvpf2.y;
#endif  

        uint index;
                
        // Calculate position of radix counter to increment
        uint p = ((kvp.key >> shift) & RADIXMASK) * RADIXGROUPS;
                
        // Move data, keeping counts updated.
        // Increment radix counters, relying on hexadecathread
        // warp to prevent this code from stepping all over itself.
        uint ppos = p + tpos;
        if (tmod == 0 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 1 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;     
        if (tmod == 2 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 3 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 4 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 5 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT; 
        if (tmod == 6 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 7 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT; 
        if (tmod == 8 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 9 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;          
        if (tmod == 10 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 11 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 12 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 13 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;           
        if (tmod == 14 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;
        if (tmod == 15 && pos < elements)
        {
            index = sRadixSum[ppos]++;
            pDst[index] = kvp;
        }
            SYNCIT;                         

        pos += RADIXTHREADS;
    }

    __syncthreads();
}

#endif // #ifndef _RADIXSORT_KERNEL_H_
