/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_OBJECT_ARRAY__
#define BT_OBJECT_ARRAY__

#include "btScalar.h" // has definitions like SIMD_FORCE_INLINE
#include "btAlignedAllocator.h"

///btAlignedObjectArray uses a subset of the stl::vector interface for its methods
///It is developed to replace stl::vector to avoid STL alignment issues to add SIMD/SSE data
template <typename T> 
//template <class T> 
class btAlignedObjectArray
{
	btAlignedAllocator<T , 16>	m_allocator;

	int					m_size;
	int					m_capacity;
	T*					m_data;

	protected:
		SIMD_FORCE_INLINE	int	allocSize(int size)
		{
			return (size ? size*2 : 1);
		}
		SIMD_FORCE_INLINE	void	copy(int start,int end, T* dest)
		{
			int i;
			for (i=start;i<end;++i)
				dest[i] = m_data[i];
		}

		SIMD_FORCE_INLINE	void	init()
		{
			m_data = 0;
			m_size = 0;
			m_capacity = 0;
		}
		SIMD_FORCE_INLINE	void	destroy(int first,int last)
		{
			int i;
			for (i=first; i<last;i++)
			{
				m_data[i].~T();
			}
		}

		SIMD_FORCE_INLINE	void* allocate(int size)
		{
			if (size)
				return m_allocator.allocate(size);
			return 0;
		}

		SIMD_FORCE_INLINE	void	deallocate()
		{
			if(m_data)	{
				m_allocator.deallocate(m_data);
				m_data = 0;
			}
		}

	


	public:
		
		btAlignedObjectArray()
		{
			init();
		}

		~btAlignedObjectArray()
		{
			clear();
		}

		SIMD_FORCE_INLINE	int capacity() const
		{	// return current length of allocated storage
			return m_capacity;
		}
		
		SIMD_FORCE_INLINE	int size() const
		{	// return length of sequence
			return m_size;
		}
		
		SIMD_FORCE_INLINE const T& operator[](int n) const
		{
			return m_data[n];
		}

		SIMD_FORCE_INLINE T& operator[](int n)
		{
			return m_data[n];
		}
		

		SIMD_FORCE_INLINE	void	clear()
		{
			destroy(0,size());
			
			deallocate();
			
			init();
		}

		SIMD_FORCE_INLINE	void	pop_back()
		{
			m_size--;
			m_data[m_size].~T();
		}

		SIMD_FORCE_INLINE	void	resize(int newsize)
		{
			if (newsize > size())
			{
				reserve(newsize);
			}

			m_size = newsize;
		}
	

		SIMD_FORCE_INLINE	T&  expand()
		{	
			int sz = size();
			if( sz == capacity() )
			{
				reserve( allocSize(size()) );
			}
			m_size++;

			return m_data[sz];		
		}

		SIMD_FORCE_INLINE	void push_back(const T& _Val)
		{	
			int sz = size();
			if( sz == capacity() )
			{
				reserve( allocSize(size()) );
			}
			
			m_data[size()] = _Val;
			//::new ( m_data[m_size] ) T(_Val);
			m_size++;
		}

	
		
		SIMD_FORCE_INLINE	void reserve(int _Count)
		{	// determine new minimum length of allocated storage
			if (capacity() < _Count)
			{	// not enough room, reallocate
				if (capacity() < _Count)
				{
					T*	s = (T*)allocate(_Count);

					copy(0, size(), s);

					destroy(0,size());

					deallocate();

					m_data = s;
					
					m_capacity = _Count;

				}
			}
		}


		class less
		{
			public:

				bool operator() ( const T& a, const T& b )
				{
					return ( a < b );
				}
		};
	

		///heap sort from http://www.csse.monash.edu.au/~lloyd/tildeAlgDS/Sort/Heap/
		template <typename L>
		void downHeap(T *pArr, int k, int n,L CompareFunc)
		{
			/*  PRE: a[k+1..N] is a heap */
			/* POST:  a[k..N]  is a heap */
			
			T temp = pArr[k - 1];
			/* k has child(s) */
			while (k <= n/2) 
			{
				int child = 2*k;
				
				if ((child < n) && CompareFunc(pArr[child - 1] , pArr[child]))
				{
					child++;
				}
				/* pick larger child */
				if (CompareFunc(temp , pArr[child - 1]))
				{
					/* move child up */
					pArr[k - 1] = pArr[child - 1];
					k = child;
				}
				else
				{
					break;
				}
			}
			pArr[k - 1] = temp;
		} /*downHeap*/

		void	swap(int index0,int index1)
		{
			T temp = m_data[index0];
			m_data[index0] = m_data[index1];
			m_data[index1] = temp;
		}

	template <typename L>
	void heapSort(L CompareFunc)
	{
		/* sort a[0..N-1],  N.B. 0 to N-1 */
		int k;
		int n = m_size;
		for (k = n/2; k > 0; k--) 
		{
			downHeap(m_data, k, n, CompareFunc);
		}

		/* a[1..N] is now a heap */
		while ( n>=1 ) 
		{
			swap(0,n-1); /* largest of a[0..n-1] */


			n = n - 1;
			/* restore a[1..i-1] heap */
			downHeap(m_data, 1, n, CompareFunc);
		} 
	}

	///non-recursive binary search, assumes sorted array
	int	findBinarySearch(const T& key) const
	{
		int first = 0;
		int last = size();

		//assume sorted array
		while (first <= last) {
			int mid = (first + last) / 2;  // compute mid point.
			if (key > m_data[mid]) 
				first = mid + 1;  // repeat search in top half.
			else if (key < m_data[mid]) 
				last = mid - 1; // repeat search in bottom half.
			else
				return mid;     // found it. return position /////
		}
		return size();    // failed to find key
	}


	int	findLinearSearch(const T& key) const
	{
		int index=size();
		int i;

		for (i=0;i<size();i++)
		{
			if (m_data[i] == key)
			{
				index = i;
				break;
			}
		}
		return index;
	}

	void	remove(const T& key)
	{

		int findIndex = findLinearSearch(key);
		if (findIndex<size())
		{
			swap( findIndex,size()-1);
			pop_back();
		}
	}

};

#endif //BT_OBJECT_ARRAY__


