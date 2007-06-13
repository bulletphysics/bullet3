
#ifndef FAKE_DMA_H
#define FAKE_DMA_H


#include "PlatformDefinitions.h"

#include <LinearMath/btScalar.h> //for definition of uint64_t,uint32_t
#define DMA_TAG(a) (a)
#define DMA_MASK(a) (a)

/// cellDmaLargeGet Win32 replacements for Cell DMA to allow simulating most of the SPU code (just memcpy)
int	cellDmaLargeGet(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid);
int	cellDmaGet(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid);
/// cellDmaLargePut Win32 replacements for Cell DMA to allow simulating most of the SPU code (just memcpy)
int cellDmaLargePut(const void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid);
/// cellDmaWaitTagStatusAll Win32 replacements for Cell DMA to allow simulating most of the SPU code (just memcpy)
void	cellDmaWaitTagStatusAll(int ignore);


#endif //FAKE_DMA_H