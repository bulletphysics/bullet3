/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada


#ifndef ARRAY_H
#define ARRAY_H

#include <string.h>
#include <malloc.h>
#include <Common/Base/Error.h>
#include <new.h>


template <class T>
class Array
{
	public:
		__inline
		Array();
		__inline
		Array(int size);
		__inline
		~Array();
		__inline
		T& operator[] (int idx);
		__inline
		const T& operator[] (int idx) const;
		__inline
		void pushBack(const T& elem);
		__inline
		void popBack();
		__inline
		void clear();
		__inline
		void setSize(int size);
		__inline
		int getSize() const;
		__inline
		T* begin();
		__inline
		const T* begin() const;
		__inline
		int indexOf(const T& data) const;
		__inline
		void removeAt(int idx);
		__inline
		T& expandOne();

	private:
		Array(const Array& a){}

	private:
		enum
		{
			DEFAULT_SIZE = 128,
			INCREASE_SIZE = 128,
		};

		T* m_data;
		int m_size;
		int m_capacity;
};

template<class T>
Array<T>::Array()
{
	m_size = 0;
	m_capacity = DEFAULT_SIZE;
//	m_data = new T[ m_capacity ];
	m_data = (T*)_aligned_malloc(sizeof(T)*m_capacity, 16);
	for(int i=0; i<m_capacity; i++) new(&m_data[i])T;
}

template<class T>
Array<T>::Array(int size)
{
	m_size = size;
	m_capacity = size;
//	m_data = new T[ m_capacity ];
	m_data = (T*)_aligned_malloc(sizeof(T)*m_capacity, 16);
	for(int i=0; i<m_capacity; i++) new(&m_data[i])T;
}

template<class T>
Array<T>::~Array()
{
	if( m_data )
	{
//		delete [] m_data;
		_aligned_free( m_data );
		m_data = NULL;
	}
}

template<class T>
T& Array<T>::operator[](int idx)
{
	CLASSERT(idx<m_size);
	return m_data[idx];
}

template<class T>
const T& Array<T>::operator[](int idx) const
{
	CLASSERT(idx<m_size);
	return m_data[idx];
}

template<class T>
void Array<T>::pushBack(const T& elem)
{
	if( m_size == m_capacity )
	{
		int oldCap = m_capacity;
		m_capacity += INCREASE_SIZE;
//		T* s = new T[m_capacity];
		T* s = (T*)_aligned_malloc(sizeof(T)*m_capacity, 16);
		memcpy( s, m_data, sizeof(T)*oldCap );
//		delete [] m_data;
		_aligned_free( m_data );
		m_data = s;
	}
	m_data[ m_size++ ] = elem;
}

template<class T>
void Array<T>::popBack()
{
	CLASSERT( m_size>0 );
	m_size--;
}

template<class T>
void Array<T>::clear()
{
	m_size = 0;
}

template<class T>
void Array<T>::setSize(int size)
{
	if( size > m_capacity )
	{
		int oldCap = m_capacity;
		m_capacity = size;
//		T* s = new T[m_capacity];
		T* s = (T*)_aligned_malloc(sizeof(T)*m_capacity, 16);
		for(int i=0; i<m_capacity; i++) new(&s[i])T;
		memcpy( s, m_data, sizeof(T)*oldCap );
//		delete [] m_data;
		_aligned_free( m_data );
		m_data = s;
	}
	m_size = size;
}

template<class T>
int Array<T>::getSize() const
{
	return m_size;
}

template<class T>
const T* Array<T>::begin() const
{
	return m_data;
}

template<class T>
T* Array<T>::begin()
{
	return m_data;
}

template<class T>
int Array<T>::indexOf(const T& data) const
{
	for(int i=0; i<m_size; i++)
	{
		if( data == m_data[i] ) return i;
	}
	return -1;
}

template<class T>
void Array<T>::removeAt(int idx)
{
	CLASSERT(idx<m_size);
	m_data[idx] = m_data[--m_size];
}

template<class T>
T& Array<T>::expandOne()
{
	setSize( m_size+1 );
	return m_data[ m_size-1 ];
}

#endif

