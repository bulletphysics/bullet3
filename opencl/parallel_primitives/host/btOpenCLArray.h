#ifndef BT_OPENCL_ARRAY_H
#define BT_OPENCL_ARRAY_H

#include "BulletCommon/btAlignedObjectArray.h"
#include "../../basic_initialize/b3OpenCLInclude.h"

template <typename T> 
class btOpenCLArray
{
	int	m_size;
	int	m_capacity;
	cl_mem	m_clBuffer;

	cl_context		 m_clContext;
	cl_command_queue m_commandQueue;

	bool	m_ownsMemory;

	bool	m_allowGrowingCapacity;

	void deallocate()
	{
		if (m_clBuffer && m_ownsMemory)
		{
			clReleaseMemObject(m_clBuffer);
		}
		m_clBuffer = 0;
		m_capacity=0;
	}

	btOpenCLArray<T>& operator=(const btOpenCLArray<T>& src);

	SIMD_FORCE_INLINE	int	allocSize(int size)
		{
			return (size ? size*2 : 1);
		}

public:

	btOpenCLArray(cl_context ctx, cl_command_queue queue, int initialCapacity=0, bool allowGrowingCapacity=true)
	:m_size(0),  m_capacity(0),m_clBuffer(0),
	m_clContext(ctx),m_commandQueue(queue),
	m_ownsMemory(true),m_allowGrowingCapacity(true)
	{
		if (initialCapacity)
		{
			reserve(initialCapacity);
		}
		m_allowGrowingCapacity = allowGrowingCapacity;
	}

	///this is an error-prone method with no error checking, be careful!
	void setFromOpenCLBuffer(cl_mem buffer, int sizeInElements)
	{
		deallocate();
		m_ownsMemory = false;
		m_allowGrowingCapacity = false;
		m_clBuffer = buffer;
		m_size = sizeInElements;
		m_capacity = sizeInElements;
	}
	
// we could enable this assignment, but need to make sure to avoid accidental deep copies
//	btOpenCLArray<T>& operator=(const btAlignedObjectArray<T>& src) 
//	{
//		copyFromArray(src);
//		return *this;
//	}


	cl_mem	getBufferCL() const
	{
		return m_clBuffer;
	}

	
	virtual ~btOpenCLArray()
	{
		deallocate();
		m_size=0;
		m_capacity=0;
	}
	
	SIMD_FORCE_INLINE	void push_back(const T& _Val,bool waitForCompletion=true)
	{	
		int sz = size();
		if( sz == capacity() )
		{
			reserve( allocSize(size()) );
		}
		copyFromHostPointer(&_Val, 1, sz, waitForCompletion);
		m_size++;
	}

	SIMD_FORCE_INLINE T forcedAt(int n) const
	{
		btAssert(n>=0);
		btAssert(n<capacity());
		T elem;
		copyToHostPointer(&elem,1,n,true);
		return elem;
	}

	SIMD_FORCE_INLINE T at(int n) const
	{
		btAssert(n>=0);
		btAssert(n<size());
		T elem;
		copyToHostPointer(&elem,1,n,true);
		return elem;
	}

	SIMD_FORCE_INLINE	void	resize(int newsize, bool copyOldContents=true)
	{
		int curSize = size();

		if (newsize < curSize)
		{
			//leave the OpenCL memory for now
		} else
		{
			if (newsize > size())
			{
				reserve(newsize,copyOldContents);
			}

			//leave new data uninitialized (init in debug mode?)
			//for (int i=curSize;i<newsize;i++) ...
		}

		m_size = newsize;
	}

	SIMD_FORCE_INLINE int size() const
	{
		return m_size;
	}

	SIMD_FORCE_INLINE	int capacity() const
	{	
		return m_capacity;
	}

	SIMD_FORCE_INLINE	void reserve(int _Count, bool copyOldContents=true)
	{	// determine new minimum length of allocated storage
		if (capacity() < _Count)
		{	// not enough room, reallocate

			if (m_allowGrowingCapacity)
			{
				cl_int ciErrNum;
				//create a new OpenCL buffer
				int memSizeInBytes = sizeof(T)*_Count;
				cl_mem buf = clCreateBuffer(m_clContext, CL_MEM_READ_WRITE, memSizeInBytes, NULL, &ciErrNum);
				btAssert(ciErrNum==CL_SUCCESS);

//#define BT_ALWAYS_INITIALIZE_OPENCL_BUFFERS
#ifdef BT_ALWAYS_INITIALIZE_OPENCL_BUFFERS
				unsigned char* src = (unsigned char*)malloc(memSizeInBytes);
				for (int i=0;i<memSizeInBytes;i++)
					src[i] = 0xbb;
				ciErrNum = clEnqueueWriteBuffer( m_commandQueue, buf, CL_TRUE, 0, memSizeInBytes, src, 0,0,0 );
				btAssert(ciErrNum==CL_SUCCESS);
				clFinish(m_commandQueue);
				free(src);
#endif //BT_ALWAYS_INITIALIZE_OPENCL_BUFFERS

				if (copyOldContents)
					copyToCL(buf, size());

				//deallocate the old buffer
				deallocate();

				m_clBuffer = buf;
			
				m_capacity = _Count;
			} else
			{
				//fail: assert and
				btAssert(0);
				deallocate();
			}
		}
	}


	void copyToCL(cl_mem destination, int numElements, int firstElem=0, int dstOffsetInElems=0) const
	{
		if (numElements<=0)
			return;

		btAssert(m_clBuffer);
		btAssert(destination);
		
		//likely some error, destination is same as source
		btAssert(m_clBuffer != destination);

		btAssert((firstElem+numElements)<=m_size);
		
		cl_int status = 0;
		

		btAssert(numElements>0);
		btAssert(numElements<=m_size);

		int srcOffsetBytes = sizeof(T)*firstElem;
		int dstOffsetInBytes = sizeof(T)*dstOffsetInElems;

		status = clEnqueueCopyBuffer( m_commandQueue, m_clBuffer, destination, 
			srcOffsetBytes, dstOffsetInBytes, sizeof(T)*numElements, 0, 0, 0 );

		btAssert( status == CL_SUCCESS );
	}

	void copyFromHost(const btAlignedObjectArray<T>& srcArray, bool waitForCompletion=true)
	{
		int newSize = srcArray.size();
		
		bool copyOldContents = false;
		resize (newSize,copyOldContents);
		if (newSize)
			copyFromHostPointer(&srcArray[0],newSize,0,waitForCompletion);

	}

	void copyFromHostPointer(const T* src, int numElems, int destFirstElem= 0, bool waitForCompletion=true)
	{
		btAssert(numElems+destFirstElem <= capacity());

		cl_int status = 0;
		int sizeInBytes=sizeof(T)*numElems;
		status = clEnqueueWriteBuffer( m_commandQueue, m_clBuffer, 0, sizeof(T)*destFirstElem, sizeInBytes,
		src, 0,0,0 );
		btAssert(status == CL_SUCCESS );
		if (waitForCompletion)
			clFinish(m_commandQueue);

	}
	

	void copyToHost(btAlignedObjectArray<T>& destArray, bool waitForCompletion=true) const
	{
		destArray.resize(this->size());
		if (size())
			copyToHostPointer(&destArray[0], size(),0,waitForCompletion);
	}

	void copyToHostPointer(T* destPtr, int numElem, int srcFirstElem=0, bool waitForCompletion=true) const
	{
		btAssert(numElem+srcFirstElem <= capacity());

		cl_int status = 0;
		status = clEnqueueReadBuffer( m_commandQueue, m_clBuffer, 0, sizeof(T)*srcFirstElem, sizeof(T)*numElem,
		destPtr, 0,0,0 );
		btAssert( status==CL_SUCCESS );

		if (waitForCompletion)
			clFinish(m_commandQueue);
	}
	
	void copyFromOpenCLArray(const btOpenCLArray& src)
	{
		int newSize = src.size();
		resize(newSize);
		if (size())
		{
			src.copyToCL(m_clBuffer,size());
		}
	}

};


#endif //BT_OPENCL_ARRAY_H
