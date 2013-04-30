#ifndef B3_OPENCL_ARRAY_H
#define B3_OPENCL_ARRAY_H

#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"

template <typename T> 
class b3OpenCLArray
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

	b3OpenCLArray<T>& operator=(const b3OpenCLArray<T>& src);

	B3_FORCE_INLINE	int	allocSize(int size)
		{
			return (size ? size*2 : 1);
		}

public:

	b3OpenCLArray(cl_context ctx, cl_command_queue queue, int initialCapacity=0, bool allowGrowingCapacity=true)
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
//	b3OpenCLArray<T>& operator=(const b3AlignedObjectArray<T>& src) 
//	{
//		copyFromArray(src);
//		return *this;
//	}


	cl_mem	getBufferCL() const
	{
		return m_clBuffer;
	}

	
	virtual ~b3OpenCLArray()
	{
		deallocate();
		m_size=0;
		m_capacity=0;
	}
	
	B3_FORCE_INLINE	void push_back(const T& _Val,bool waitForCompletion=true)
	{	
		int sz = size();
		if( sz == capacity() )
		{
			reserve( allocSize(size()) );
		}
		copyFromHostPointer(&_Val, 1, sz, waitForCompletion);
		m_size++;
	}

	B3_FORCE_INLINE T forcedAt(int n) const
	{
		b3Assert(n>=0);
		b3Assert(n<capacity());
		T elem;
		copyToHostPointer(&elem,1,n,true);
		return elem;
	}

	B3_FORCE_INLINE T at(int n) const
	{
		b3Assert(n>=0);
		b3Assert(n<size());
		T elem;
		copyToHostPointer(&elem,1,n,true);
		return elem;
	}

	B3_FORCE_INLINE	void	resize(int newsize, bool copyOldContents=true)
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

	B3_FORCE_INLINE int size() const
	{
		return m_size;
	}

	B3_FORCE_INLINE	int capacity() const
	{	
		return m_capacity;
	}

	B3_FORCE_INLINE	void reserve(int _Count, bool copyOldContents=true)
	{	// determine new minimum length of allocated storage
		if (capacity() < _Count)
		{	// not enough room, reallocate

			if (m_allowGrowingCapacity)
			{
				cl_int ciErrNum;
				//create a new OpenCL buffer
				int memSizeInBytes = sizeof(T)*_Count;
				cl_mem buf = clCreateBuffer(m_clContext, CL_MEM_READ_WRITE, memSizeInBytes, NULL, &ciErrNum);
				b3Assert(ciErrNum==CL_SUCCESS);

//#define B3_ALWAYS_INITIALIZE_OPENCL_BUFFERS
#ifdef B3_ALWAYS_INITIALIZE_OPENCL_BUFFERS
				unsigned char* src = (unsigned char*)malloc(memSizeInBytes);
				for (int i=0;i<memSizeInBytes;i++)
					src[i] = 0xbb;
				ciErrNum = clEnqueueWriteBuffer( m_commandQueue, buf, CL_TRUE, 0, memSizeInBytes, src, 0,0,0 );
				b3Assert(ciErrNum==CL_SUCCESS);
				clFinish(m_commandQueue);
				free(src);
#endif //B3_ALWAYS_INITIALIZE_OPENCL_BUFFERS

				if (copyOldContents)
					copyToCL(buf, size());

				//deallocate the old buffer
				deallocate();

				m_clBuffer = buf;
			
				m_capacity = _Count;
			} else
			{
				//fail: assert and
				b3Assert(0);
				deallocate();
			}
		}
	}


	void copyToCL(cl_mem destination, int numElements, int firstElem=0, int dstOffsetInElems=0) const
	{
		if (numElements<=0)
			return;

		b3Assert(m_clBuffer);
		b3Assert(destination);
		
		//likely some error, destination is same as source
		b3Assert(m_clBuffer != destination);

		b3Assert((firstElem+numElements)<=m_size);
		
		cl_int status = 0;
		

		b3Assert(numElements>0);
		b3Assert(numElements<=m_size);

		int srcOffsetBytes = sizeof(T)*firstElem;
		int dstOffsetInBytes = sizeof(T)*dstOffsetInElems;

		status = clEnqueueCopyBuffer( m_commandQueue, m_clBuffer, destination, 
			srcOffsetBytes, dstOffsetInBytes, sizeof(T)*numElements, 0, 0, 0 );

		b3Assert( status == CL_SUCCESS );
	}

	void copyFromHost(const b3AlignedObjectArray<T>& srcArray, bool waitForCompletion=true)
	{
		int newSize = srcArray.size();
		
		bool copyOldContents = false;
		resize (newSize,copyOldContents);
		if (newSize)
			copyFromHostPointer(&srcArray[0],newSize,0,waitForCompletion);

	}

	void copyFromHostPointer(const T* src, int numElems, int destFirstElem= 0, bool waitForCompletion=true)
	{
		b3Assert(numElems+destFirstElem <= capacity());

		cl_int status = 0;
		int sizeInBytes=sizeof(T)*numElems;
		status = clEnqueueWriteBuffer( m_commandQueue, m_clBuffer, 0, sizeof(T)*destFirstElem, sizeInBytes,
		src, 0,0,0 );
		b3Assert(status == CL_SUCCESS );
		if (waitForCompletion)
			clFinish(m_commandQueue);

	}
	

	void copyToHost(b3AlignedObjectArray<T>& destArray, bool waitForCompletion=true) const
	{
		destArray.resize(this->size());
		if (size())
			copyToHostPointer(&destArray[0], size(),0,waitForCompletion);
	}

	void copyToHostPointer(T* destPtr, int numElem, int srcFirstElem=0, bool waitForCompletion=true) const
	{
		b3Assert(numElem+srcFirstElem <= capacity());

		cl_int status = 0;
		status = clEnqueueReadBuffer( m_commandQueue, m_clBuffer, 0, sizeof(T)*srcFirstElem, sizeof(T)*numElem,
		destPtr, 0,0,0 );
		b3Assert( status==CL_SUCCESS );

		if (waitForCompletion)
			clFinish(m_commandQueue);
	}
	
	void copyFromOpenCLArray(const b3OpenCLArray& src)
	{
		int newSize = src.size();
		resize(newSize);
		if (size())
		{
			src.copyToCL(m_clBuffer,size());
		}
	}

};


#endif //B3_OPENCL_ARRAY_H
