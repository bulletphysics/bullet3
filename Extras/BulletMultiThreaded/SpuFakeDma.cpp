
#include "SpuFakeDma.h"

int	cellDmaLargeGet(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid)
{
	void* targetMainMem = (void*)ea;
	memcpy(ls,targetMainMem,size);
	return 0;
}

int	cellDmaGet(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid)
{
	void* targetMainMem = (void*)ea;
	memcpy(ls,targetMainMem,size);
	return 0;
}

int cellDmaLargePut(const void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid)
{
	void* targetMainMem = (void*)ea;
	memcpy(targetMainMem,ls,size);
	return 0;
}



void	cellDmaWaitTagStatusAll(int ignore)
{

}
