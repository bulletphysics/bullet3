
#ifndef FAKE_DMA_H
#define FAKE_DMA_H


#include "PlatformDefinitions.h"



#ifdef __CELLOS_LV2__

#include <cell/dma.h>
#include <stdint.h>

#define DMA_TAG(xfer) (xfer + 1)
#define DMA_MASK(xfer) (1 << DMA_TAG(xfer))


#else
#ifdef WIN32

#define DMA_TAG(a) (a)
#define DMA_MASK(a) (a)


		/// cellDmaLargeGet Win32 replacements for Cell DMA to allow simulating most of the SPU code (just memcpy)
		int	cellDmaLargeGet(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid);
		int	cellDmaGet(void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid);
		/// cellDmaLargePut Win32 replacements for Cell DMA to allow simulating most of the SPU code (just memcpy)
		int cellDmaLargePut(const void *ls, uint64_t ea, uint32_t size, uint32_t tag, uint32_t tid, uint32_t rid);
		/// cellDmaWaitTagStatusAll Win32 replacements for Cell DMA to allow simulating most of the SPU code (just memcpy)
		void	cellDmaWaitTagStatusAll(int ignore);


#elif defined(USE_LIBSPE2)

#define DMA_TAG(xfer) (xfer + 1)
#define DMA_MASK(xfer) (1 << DMA_TAG(xfer))
		
#include <spu_mfcio.h>		
		
#define DEBUG_DMA		
#ifdef DEBUG_DMA
#define dUASSERT(a,b) if (!(a)) { printf(b);}
#ifdef USE_ADDR64
#define uintsize unsigned long long
#else
#define uintsize unsigned int
#endif
		
#define cellDmaLargeGet(ls, ea, size, tag, tid, rid) if (  (((uintsize)ls%16) != ((uintsize)ea%16)) || ((((uintsize)ea%16) || ((uintsize)ls%16)) && (( ((uintsize)ls%16) != ((uintsize)size%16) ) || ( ((uintsize)ea%16) != ((uintsize)size%16) ) ) ) || ( ((uintsize)size%16) && ((uintsize)size!=1) && ((uintsize)size!=2) && ((uintsize)size!=4) && ((uintsize)size!=8) ) || (size >= 16384) || !(uintsize)ls || !(uintsize)ea) { \
															dUASSERT( (((uintsize)ea % 16) == 0) || (size < 16), "XDR Address not aligned: "); \
															dUASSERT( (((uintsize)ls % 16) == 0) || (size < 16), "LS Address not aligned: "); \
															dUASSERT( ((((uintsize)ls % size) == 0) && (((uintsize)ea % size) == 0))  || (size > 16), "Not naturally aligned: "); \
															dUASSERT((size == 1) || (size == 2) || (size == 4) || (size == 8) || ((size % 16) == 0), "size not a multiple of 16byte: "); \
															dUASSERT(size < 16384, "size too big: "); \
															dUASSERT( ((uintsize)ea%16)==((uintsize)ls%16), "wrong Quadword alignment of LS and EA: "); \
	    													dUASSERT(ea != 0, "Nullpointer EA: "); dUASSERT(ls != 0, "Nullpointer LS: ");\
															printf("GET %s:%d from: 0x%x, to: 0x%x - %d bytes\n", __FILE__, __LINE__, (unsigned int)ea,(unsigned int)ls,(unsigned int)size);\
															} \
															mfc_get(ls, ea, size, tag, tid, rid)
#define cellDmaGet(ls, ea, size, tag, tid, rid) if (  (((uintsize)ls%16) != ((uintsize)ea%16)) || ((((uintsize)ea%16) || ((uintsize)ls%16)) && (( ((uintsize)ls%16) != ((uintsize)size%16) ) || ( ((uintsize)ea%16) != ((uintsize)size%16) ) ) ) || ( ((uintsize)size%16) && ((uintsize)size!=1) && ((uintsize)size!=2) && ((uintsize)size!=4) && ((uintsize)size!=8) ) || (size >= 16384) || !(uintsize)ls || !(uintsize)ea) { \
														dUASSERT( (((uintsize)ea % 16) == 0) || (size < 16), "XDR Address not aligned: "); \
														dUASSERT( (((uintsize)ls % 16) == 0) || (size < 16), "LS Address not aligned: "); \
														dUASSERT( ((((uintsize)ls % size) == 0) && (((uintsize)ea % size) == 0))  || (size > 16), "Not naturally aligned: "); \
														dUASSERT((size == 1) || (size == 2) || (size == 4) || (size == 8) || ((size % 16) == 0), "size not a multiple of 16byte: "); \
    													dUASSERT(size < 16384, "size too big: "); \
														dUASSERT( ((uintsize)ea%16)==((uintsize)ls%16), "wrong Quadword alignment of LS and EA: "); \
    													dUASSERT(ea != 0, "Nullpointer EA: "); dUASSERT(ls != 0, "Nullpointer LS: ");\
    													printf("GET %s:%d from: 0x%x, to: 0x%x - %d bytes\n", __FILE__, __LINE__, (unsigned int)ea,(unsigned int)ls,(unsigned int)size);\
														} \
														mfc_get(ls, ea, size, tag, tid, rid)
#define cellDmaLargePut(ls, ea, size, tag, tid, rid) if (  (((uintsize)ls%16) != ((uintsize)ea%16)) || ((((uintsize)ea%16) || ((uintsize)ls%16)) && (( ((uintsize)ls%16) != ((uintsize)size%16) ) || ( ((uintsize)ea%16) != ((uintsize)size%16) ) ) ) || ( ((uintsize)size%16) && ((uintsize)size!=1) && ((uintsize)size!=2) && ((uintsize)size!=4) && ((uintsize)size!=8) ) || (size >= 16384) || !(uintsize)ls || !(uintsize)ea) { \
															dUASSERT( (((uintsize)ea % 16) == 0) || (size < 16), "XDR Address not aligned: "); \
															dUASSERT( (((uintsize)ls % 16) == 0) || (size < 16), "LS Address not aligned: "); \
															dUASSERT( ((((uintsize)ls % size) == 0) && (((uintsize)ea % size) == 0))  || (size > 16), "Not naturally aligned: "); \
															dUASSERT((size == 1) || (size == 2) || (size == 4) || (size == 8) || ((size % 16) == 0), "size not a multiple of 16byte: "); \
        													dUASSERT(size < 16384, "size too big: "); \
															dUASSERT( ((uintsize)ea%16)==((uintsize)ls%16), "wrong Quadword alignment of LS and EA: "); \
        													dUASSERT(ea != 0, "Nullpointer EA: "); dUASSERT(ls != 0, "Nullpointer LS: ");\
    														printf("PUT %s:%d from: 0x%x, to: 0x%x - %d bytes\n", __FILE__, __LINE__, (unsigned int)ls,(unsigned int)ea,(unsigned int)size); \
															} \
															mfc_put(ls, ea, size, tag, tid, rid)
#define cellDmaSmallGet(ls, ea, size, tag, tid, rid) if (  (((uintsize)ls%16) != ((uintsize)ea%16)) || ((((uintsize)ea%16) || ((uintsize)ls%16)) && (( ((uintsize)ls%16) != ((uintsize)size%16) ) || ( ((uintsize)ea%16) != ((uintsize)size%16) ) ) ) || ( ((uintsize)size%16) && ((uintsize)size!=1) && ((uintsize)size!=2) && ((uintsize)size!=4) && ((uintsize)size!=8) ) || (size >= 16384) || !(uintsize)ls || !(uintsize)ea) { \
																dUASSERT( (((uintsize)ea % 16) == 0) || (size < 16), "XDR Address not aligned: "); \
																dUASSERT( (((uintsize)ls % 16) == 0) || (size < 16), "LS Address not aligned: "); \
																dUASSERT( ((((uintsize)ls % size) == 0) && (((uintsize)ea % size) == 0))  || (size > 16), "Not naturally aligned: "); \
    															dUASSERT((size == 1) || (size == 2) || (size == 4) || (size == 8) || ((size % 16) == 0), "size not a multiple of 16byte: "); \
    															dUASSERT(size < 16384, "size too big: "); \
    															dUASSERT( ((uintsize)ea%16)==((uintsize)ls%16), "wrong Quadword alignment of LS and EA: "); \
    	    													dUASSERT(ea != 0, "Nullpointer EA: "); dUASSERT(ls != 0, "Nullpointer LS: ");\
    															printf("GET %s:%d from: 0x%x, to: 0x%x - %d bytes\n", __FILE__, __LINE__, (unsigned int)ea,(unsigned int)ls,(unsigned int)size);\
																} \
																mfc_get(ls, ea, size, tag, tid, rid)
#define cellDmaWaitTagStatusAll(ignore) mfc_write_tag_mask(ignore) ; mfc_read_tag_status_all()

#else
#define cellDmaLargeGet(ls, ea, size, tag, tid, rid) mfc_get(ls, ea, size, tag, tid, rid)
#define cellDmaGet(ls, ea, size, tag, tid, rid) mfc_get(ls, ea, size, tag, tid, rid)
#define cellDmaLargePut(ls, ea, size, tag, tid, rid) mfc_put(ls, ea, size, tag, tid, rid)
#define cellDmaSmallGet(ls, ea, size, tag, tid, rid) mfc_get(ls, ea, size, tag, tid, rid)
#define cellDmaWaitTagStatusAll(ignore) mfc_write_tag_mask(ignore) ; mfc_read_tag_status_all()
#endif // DEBUG_DMA

		
		
		
		
		
		
		
		
		
#endif // WIN32

#endif //__CELLOS_LV2__

///stallingUnalignedDmaSmallGet internally uses DMA_TAG(1)
int	stallingUnalignedDmaSmallGet(void *ls, uint64_t ea, uint32_t size);

#endif //FAKE_DMA_H
