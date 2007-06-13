#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

#ifdef __CELLOS_LV2__

#include <cell/dma.h>
#include <stdint.h>

#else
#include "SpuFakeDma.h"
#endif //


///DoubleBuffer
template<class T, int size>
class DoubleBuffer
{
#ifdef __CELLOS_LV2__
	T m_buffer0[size] __attribute__ ((aligned (128)));
	T m_buffer1[size] __attribute__ ((aligned (128)));
#else
	T m_buffer0[size];
	T m_buffer1[size];
#endif
	
	T *m_frontBuffer;
	T *m_backBuffer;

	unsigned int m_dmaTag;
	bool m_dmaPending;
public:
	bool	isPending() const { return m_dmaPending;}
	DoubleBuffer();

	// dma get and put commands
	void backBufferDmaGet(uint64_t ea, unsigned int numBytes, unsigned int tag);
	void backBufferDmaPut(uint64_t ea, unsigned int numBytes, unsigned int tag);

	// gets pointer to a buffer
	T *getFront();
	T *getBack();

	// if back buffer dma was started, wait for it to complete
	// then move back to front and vice versa
	T *swapBuffers();
};

template<class T, int size>
DoubleBuffer<T,size>::DoubleBuffer()
{
	m_dmaPending = false;
	m_frontBuffer = &m_buffer0[0];
	m_backBuffer = &m_buffer1[0];
}

template<class T, int size>
void
DoubleBuffer<T,size>::backBufferDmaGet(uint64_t ea, unsigned int numBytes, unsigned int tag)
{
	m_dmaPending = true;
	m_dmaTag = tag;
	cellDmaLargeGet(m_backBuffer, ea, numBytes, tag, 0, 0);
}

template<class T, int size>
void
DoubleBuffer<T,size>::backBufferDmaPut(uint64_t ea, unsigned int numBytes, unsigned int tag)
{
	m_dmaPending = true;
	m_dmaTag = tag;
	cellDmaLargePut(m_backBuffer, ea, numBytes, tag, 0, 0);
}

template<class T, int size>
T *
DoubleBuffer<T,size>::getFront()
{
	return m_frontBuffer;
}

template<class T, int size>
T *
DoubleBuffer<T,size>::getBack()
{
	return m_backBuffer;
}

template<class T, int size>
T *
DoubleBuffer<T,size>::swapBuffers()
{
	if (m_dmaPending)
	{
		cellDmaWaitTagStatusAll(1<<m_dmaTag);
		m_dmaPending = false;
	}

	T *tmp = m_backBuffer;
	m_backBuffer = m_frontBuffer;
	m_frontBuffer = tmp;

	return m_frontBuffer;
}

#endif
