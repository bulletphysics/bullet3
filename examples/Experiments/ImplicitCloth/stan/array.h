//
//  Typical template dynamic array container class.
//  By S Melax 1998
//
// anyone is free to use, inspect, learn from, or ignore
// the code here as they see fit.
//
// A very simple template array class.
// Its easiest to understand this array
// class by seeing how it is used in code.
//
// For example:
//     for(i=0;i<myarray.count;i++)
//         myarray[i] = somefunction(i);
//
// When the array runs out of room, it
// reallocates memory and doubles the size of its
// storage buffer.  The reason for *doubleing* the amount of
// memory is so the order of any algorithm using this class
// is the same as it would be had you used a regular C array.
// The penalty for reallocating and copying
// For example consider adding n elements to a list.
// Lets sum the number of times elements are "copied".
// The worst case occurs when n=2^k+1 where k is integer.
// In this case we do a big reallocation when we add the last element.
// n elements are copied once, n/2 elements are copied twice,
// n/4 elements are copied 3 times, and so on ...
//    total == n* (1+1/2 + 1/4 + 1/8 + ...) == n * 2
// So we do n*2 copies.  Therefore adding n
// elements to an Array is still O(n).
// The memory usage is also of the same order as if a C array was used.
// An Array uses less than double the minimum needed space.  Again, we
// see that we are within a small constant multiple.
//
// Why no "realloc" to avoid the copy when reallocating memory?
// You have a choice to either use malloc/free and friends
// or to use new/delete.  Its bad mojo to mix these.  new/delete was
// chosen to be C++ish and have the array elements constructors/destructors
// invoked as expected.
//
//

#ifndef SM_ARRAY_H
#define SM_ARRAY_H

#include <assert.h>
#include <stdio.h>

template <class Type>
class Array
{
public:
	Array(int s = 0);
	Array(Array<Type> &array);
	~Array();
	void allocate(int s);
	void SetSize(int s);
	void Pack();
	Type &Add(Type);
	void AddUnique(Type);
	int Contains(Type);
	void Insert(Type, int);
	int IndexOf(Type);
	void Remove(Type);
	void DelIndex(int i);
	Type &DelIndexWithLast(int i);
	Type *element;
	int count;
	int array_size;
	const Type &operator[](int i) const
	{
		assert(i >= 0 && i < count);
		return element[i];
	}
	Type &operator[](int i)
	{
		assert(i >= 0 && i < count);
		return element[i];
	}
	Type &Pop()
	{
		assert(count);
		count--;
		return element[count];
	}
	Array<Type> &copy(const Array<Type> &array);
	Array<Type> &operator=(Array<Type> &array);
};

template <class Type>
Array<Type>::Array(int s)
{
	if (s == -1) return;
	count = 0;
	array_size = 0;
	element = NULL;
	if (s)
	{
		allocate(s);
	}
}

template <class Type>
Array<Type>::Array(Array<Type> &array)
{
	count = 0;
	array_size = 0;
	element = NULL;
	*this = array;
}

template <class Type>
Array<Type> &Array<Type>::copy(const Array<Type> &array)
{
	assert(array.array_size >= 0);
	count = 0;
	for (int i = 0; i < array.count; i++)
	{
		Add(array[i]);
	}
	return *this;
}
template <class Type>
Array<Type> &Array<Type>::operator=(Array<Type> &array)
{
	if (array.array_size < 0)  //  negative number means steal the data buffer instead of copying
	{
		delete[] element;
		element = array.element;
		array_size = -array.array_size;
		count = array.count;
		array.count = array.array_size = 0;
		array.element = NULL;
		return *this;
	}
	count = 0;
	for (int i = 0; i < array.count; i++)
	{
		Add(array[i]);
	}
	return *this;
}

template <class Type>
Array<Type>::~Array()
{
	if (element != NULL && array_size != 0)
	{
		delete[] element;
	}
	count = 0;
	array_size = 0;
	element = NULL;
}

template <class Type>
void Array<Type>::allocate(int s)
{
	assert(s > 0);
	assert(s >= count);
	if (s == array_size) return;
	Type *old = element;
	array_size = s;
	element = new Type[array_size];
	assert(element);
	for (int i = 0; i < count; i++)
	{
		element[i] = old[i];
	}
	if (old) delete[] old;
}

template <class Type>
void Array<Type>::SetSize(int s)
{
	if (s == 0)
	{
		if (element)
		{
			delete[] element;
			element = NULL;
		}
		array_size = s;
	}
	else
	{
		allocate(s);
	}
	count = s;
}

template <class Type>
void Array<Type>::Pack()
{
	allocate(count);
}

template <class Type>
Type &Array<Type>::Add(Type t)
{
	assert(count <= array_size);
	if (count == array_size)
	{
		allocate((array_size) ? array_size * 2 : 16);
	}
	//int i;
	//for(i=0;i<count;i++) {
	// dissallow duplicates
	//	assert(element[i] != t);
	//}
	element[count++] = t;
	return element[count - 1];
}

template <class Type>
int Array<Type>::Contains(Type t)
{
	int i;
	int found = 0;
	for (i = 0; i < count; i++)
	{
		if (element[i] == t) found++;
	}
	return found;
}

template <class Type>
void Array<Type>::AddUnique(Type t)
{
	if (!Contains(t)) Add(t);
}

template <class Type>
void Array<Type>::DelIndex(int i)
{
	assert(i < count);
	count--;
	while (i < count)
	{
		element[i] = element[i + 1];
		i++;
	}
}

template <class Type>
Type &Array<Type>::DelIndexWithLast(int i)
{
	assert(i < count);
	count--;
	if (i < count)
	{
		Type r = element[i];
		element[i] = element[count];
		element[count] = r;
	}
	return element[count];
}

template <class Type>
void Array<Type>::Remove(Type t)
{
	int i;
	for (i = 0; i < count; i++)
	{
		if (element[i] == t)
		{
			break;
		}
	}
	assert(i < count);  // assert object t is in the array.
	DelIndex(i);
	for (i = 0; i < count; i++)
	{
		assert(element[i] != t);
	}
}

template <class Type>
void Array<Type>::Insert(Type t, int k)
{
	int i = count;
	Add(t);  // to allocate space
	while (i > k)
	{
		element[i] = element[i - 1];
		i--;
	}
	assert(i == k);
	element[k] = t;
}

template <class Type>
int Array<Type>::IndexOf(Type t)
{
	int i;
	for (i = 0; i < count; i++)
	{
		if (element[i] == t)
		{
			return i;
		}
	}
	assert(0);
	return -1;
}

#endif
