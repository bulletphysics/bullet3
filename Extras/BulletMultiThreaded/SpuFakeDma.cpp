
#include "SpuFakeDma.h"
#include <LinearMath/btScalar.h> //for btAssert
//Disabling memcpy sometimes helps debugging DMA

#define USE_MEMCPY 1

void*	cellDmaLargeGetReadOnly(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid)
{
#if defined (WIN32) || defined (__PPU__)
	return (void*)(uint32_t)ea;
#else
	cellDmaLargeGet(ls,ea,size,tag,tid,rid);
	return ls;
#endif //WIN32
}


void*	cellDmaGetReadOnly(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid)
{
#if defined (WIN32) || defined (__PPU__)
	return (void*)(uint32_t)ea;
#else
	cellDmaGet(ls,ea,size,tag,tid,rid);
	return ls;
#endif //WIN32
}


///this unalignedDma should not be frequently used, only for small data. It handles alignment and performs check on size (<16 bytes)
int stallingUnalignedDmaSmallGet(void *ls, uint64_t ea, uint32_t size)
{
	
	btAssert(size<32);
	
	ATTRIBUTE_ALIGNED16(char	tmpBuffer[32]);

	char* mainMem = (char*)ea;
	char* localStore = (char*)ls;
	uint32_t i;
	

	///make sure last 4 bits are the same, for cellDmaSmallGet
	uint32_t last4BitsOffset = ea & 0x0f;
	char* tmpTarget = tmpBuffer + last4BitsOffset;
#ifdef WIN32

#ifdef USE_MEMCPY
		memcpy(tmpTarget,mainMem,size);
#else
		for ( i=0;i<size;i++)
		{
			tmpTarget[i] = mainMem[i];
		}
#endif //USE_MEMCPY
#else
	mfc_get(tmpTarget,ea,size,DMA_TAG(1),0,0);
	//copy into final destination
#endif //WIN32

	cellDmaWaitTagStatusAll(DMA_MASK(1));

	//this is slowish, perhaps memcpy on SPU is smarter?
	for (i=0; btLikely( i<size );i++)
	{
		localStore[i] = tmpTarget[i];
	}

	return 0;
}


#ifdef WIN32

int	cellDmaLargeGet(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid)
{
	char* mainMem = (char*)ea;
	char* localStore = (char*)ls;

#ifdef USE_MEMCPY
	memcpy(localStore,mainMem,size);
#else
	for (uint32_t i=0;i<size;i++)
	{
		localStore[i] = mainMem[i];
	}
#endif
	return 0;
}

int	cellDmaGet(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid)
{
	char* mainMem = (char*)ea;
	char* localStore = (char*)ls;
#ifdef USE_MEMCPY
	memcpy(localStore,mainMem,size);
#else
	for (uint32_t i=0;i<size;i++)
	{
		localStore[i] = mainMem[i];
	}	
#endif //#ifdef USE_MEMCPY
	return 0;
}

int cellDmaLargePut(const void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid)
{
	char* mainMem = (char*)ea;
	const char* localStore = (const char*)ls;
#ifdef USE_MEMCPY
	memcpy(mainMem,localStore,size);
#else
	for (uint32_t i=0;i<size;i++)
	{
		mainMem[i] = localStore[i];
	}	
#endif //#ifdef USE_MEMCPY

	return 0;
}



void	cellDmaWaitTagStatusAll(int ignore)
{

}

#endif //WIN32
