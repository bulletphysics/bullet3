/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the SCEA Shared Source License, Version 1.0 (the "License"); you may not use this 
 * file except in compliance with the License. You may obtain a copy of the License at:
 * http://research.scea.com/scea_shared_source_license.html
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License 
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 
 * implied. See the License for the specific language governing permissions and limitations under the 
 * License. 
 */

#ifndef __DAE_ARRAY_H__
#define __DAE_ARRAY_H__
#include <dae/daeMemorySystem.h>

class daeAtomicType;

/**
 * COLLADA C++ class that implements storage for resizable array containers.
 */
class daeArray
{
protected:
	size_t			_count;
	size_t			_capacity;
	daeMemoryRef	_data;
	size_t			_elementSize;
	daeAtomicType*	_type;
public:
	/**
	 * Constructor
	 */
	daeArray();
	/**
	 * Copy Constructor
	 */
	daeArray( const daeArray &cpy ): _count(cpy._count), _capacity(cpy._capacity), _data(0),
								 	 _elementSize( cpy._elementSize), _type( cpy._type ) {
		grow(_capacity);
		memcpy( _data, cpy._data, _elementSize * _count );
	}

	/**
	 * Destructor
	 */
	virtual ~daeArray();
	/**
	 * Clears the contents of the array. Do not use this function if the array contains @c daeSmartRef objects and the
	 * @c dom* class the array belongs to has a @c _contents member.
	 *
	 * Many @c dom* objects have a @c _contents member that stores the original creation order of the @c daeElements
	 * that are their children.  If you use @c clear() on a @c daeArray of @c daeSmartRef derived objects, these
	 * objects will not be removed from @c _contents, which can cause problems when you
	 * save the data.  We recommended that @c clear() not be used on arrays that are part of a @c dom* object.
	 */
	virtual void clear();
	/**
	 * Sets the size of an element in the array when creating a @c daeArray of a specific type.
	 * @param elementSize Size of an element in the array.
	 */
	void setElementSize(size_t elementSize) {_elementSize = elementSize;}
	/**
	 * Grows the array to the specified size and sets the @c daeArray to that size.
	 * @param cnt Size to grow the array to.
	 */
	inline void setRawCount(size_t cnt) {grow(cnt);_count = cnt;}
	/**
	 * Gets the current capacity of the array, the biggest it can get without incurring a realloc.
	 * @return Returns the capacity of the array.
	 */
	inline size_t getCapacity() const {return _capacity;}
	/**
	 * Gets the number of items stored in this @c daeArray.
	 * @return Returns the number of items stored in this @c daeArray.
	 */
	inline size_t getCount() const {return _count;}
	/**
	 * Gets the size of an element in this array.
	 * @return Returns the size of an element in this array.
	 */
	inline size_t getElementSize() const {return _elementSize;}
	/**
	 * Gets a pointer to the memory where the raw data for this @c daeArray is stored.
	 * @return Returns a pointer to the memory for the raw data.
	 */
	inline daeMemoryRef getRawData() const {return _data;}
	
	/**
	 * Increases the size of the @c daeArray.
	 * @param sz  Size to grow the array to.
	 */
	void grow(size_t sz);
	/**
	 * Removes an item at a specific index in the @c daeArray. 
	 * @param index  Index number of the item to delete.
	 * @return Returns DAE_OK if success, a negative value defined in daeError.h otherwise.
	 * @note The @c daeElement objects sometimes list
	 * objects in two places, the class member and the <i> @c _contents </i> array, when you remove something from the
	 * dom, you must remove it from both places.
	 */
	virtual daeInt removeIndex(size_t index);
};

/**
 * COLLADA C++ templated version of @c daeArray for storing items of various types.
 */
template <class T>
class daeTArray : public daeArray
{
public:
	/**
	*  Constructor.
	 */
	daeTArray() {
//		_type = daeAtomicType::getType("" T "");
		_elementSize = sizeof( T );
	}
	/**
	 * Copy Constructor
	 */
	daeTArray( const daeTArray<T> &cpy ) : daeArray() {
		_count = cpy._count;
		_capacity = cpy._capacity;
		_data = NULL;
		_elementSize = cpy._elementSize;
		_type = cpy._type;
		grow(_capacity);
		for(size_t i=0;i<_count;i++)
			set( i, cpy[i] ); 
	}
	/**
	 * Constructor that takes one element and turns into an array
	 */
	daeTArray( const T &el ) : daeArray() {
		_elementSize = sizeof(T);
		append( el );
	}
	/**
	*  Destructor.
	 */
	virtual ~daeTArray() {
		clear();
	}
	/**
	 * Frees the memory in this array and resets it to it's initial state.
	 */
	virtual void clear()
	{
		size_t i;
		for(i=0;i<_count;i++)
			((T*)_data + i)->~T();
		daeArray::clear();
	}
	
	/**
	 * Removes an item at a specific index in the @c daeArray. 
	 * @param index  Index number of the item to delete.
	 * @return Returns DAE_OK if success, a negative value defined in daeError.h otherwise.
	 * @note The @c daeElement objects sometimes list
	 * objects in two places, the class member and the <i> @c _contents </i> array, when you remove something from the
	 * dom, you must remove it from both places.
	 */
	virtual daeInt removeIndex(size_t index)
	{
		if ((index >= _count)||(_count < 1))
			return(DAE_ERR_INVALID_CALL);
		((T*)_data + index)->~T();
		return(daeArray::removeIndex(index));

	}

	/**
	 * Resets the count of items in a daeArray to an absolute value, if necessary the array
	 * storage will grow to the requested size and new elements will be initialized to zero  
	 * @param nElements The new size of the array.
	 * @note Shrinking the array does NOT free up memory.
	 */
	inline void setCount(size_t nElements) 
	{
		grow(nElements);
		if(nElements < _count)
		{
			// If the array shrank, destruct the elements
			size_t i;
			for(i=_count; i>nElements; i--)
			{
				((T*)_data + (i-1))->~T();
				memset(_data+(i-1)*_elementSize,0,_elementSize);
			}
		}
		_count = nElements;
	}
	
	/**
	 * Sets a specific index in the @c daeArray, growing the array if necessary.
	 * @param index Index of the object to set, asserts if the index is out of bounds.
	 * @param value Value to store at index in the array.
	 */
	inline void set(size_t index, const T& value) {
		if (index >= _capacity)
			grow(index);
		((T*)_data)[index] = value; }
	
	/**
	 * Gets the object at a specific index in the @c daeArray.
	 * @param index Index of the object to get, asserts if the index is out of bounds.
	 * @return Returns the object at index.
	 */
	inline T& get(size_t index) const {
		assert(index < _count);
		return ((T*)_data)[index]; }
	
	/**
	 * Appends a new object to the end of the @c daeArray.
	 * @param value Value of the object to append.
	 * @return Returns the index of the new object.
	 */
	inline size_t append(const T& value) {
		set(_count, value);
		_count++;
		return _count-1;
	}

	/**
	 * Appends a unique object to the end of the @c daeArray.
	 * Functions the same as @c append(), but does nothing if the value is already in the @c daeArray.
	 * @param value Value of the object to append.
	 * @return Returns the index where this value was appended. If the value already exists in the array, 
	 * returns the index in this array where the value was found.
	 */
	inline size_t appendUnique(const T& value) {
		size_t ret;
		if (find(value,ret) != DAE_OK)
			return append(value);
		else 
			return ret;
	}
	
	/**
	 * Removes an item from the @c daeArray.
	 * @param value A reference to the item to delete.
	 * @return Returns DAE_OK if success, a negative value defined in daeError.h otherwise.
	 * @note The @c daeElement objects sometimes list
	 * objects in two places, the class member and the <i> @c _contents </i> array, when you remove something from the
	 * do, you must remove it from both places.
	 */
	inline daeInt remove(const T& value, size_t *idx = NULL ) 
	{
		size_t index;
		if(find(value,index) == DAE_OK)
		{
			if ( idx != NULL ) {
				*idx = index;
			}
			return(removeIndex( index ));
		}
		else
		{
			return(DAE_ERR_INVALID_CALL);
		}
	}
	/**
	 * Finds an item from the @c daeArray.
	 * @param value A reference to the item to find.
	 * @param index If the function returns DAE_OK, this is set to the index where the value appears in the array.
	 * @return Returns DAE_OK if no error or DAE_ERR_QUERY_NO_MATCH if the value was not found.
	 */
	inline daeInt find(const T& value,  size_t &index) const 
	{
		size_t i;
		for(i=0;i<_count;i++)
		{
			if (((T*)_data)[i] == value)
			{
				index = i;
				return DAE_OK;
			}
		}
		return DAE_ERR_QUERY_NO_MATCH; 
	}
	/**
	 * Gets the object at a specific index in the @c daeArray.
	 * @param index Index of the object to get, asserts if the index is out of bounds.
	 * @return Returns the object at @c index.
	 */
	inline T& operator[](size_t index) const {
		assert(index < _count);
		return ((T*)_data)[index]; }
	
	/**
	 * Inserts an object at a specific index in the daeArray, growing the array if neccessary
	 * @param index Index into the array for where to place the object, asserts if the index is out of bounds
	 * @param value of the object to append
	 */
	inline void insertAt(size_t index, const T& value) {
		assert(index <= _capacity);
		if ( _count == _capacity ) {
			grow( _count +1 );
		}
		//memmove( &(((T*)_data)[index+1]), &(((T*)_data)[index]), (_count - index)*_elementSize );
		for (size_t i = _count; i > index; i-- ) {
			set( i, ((T*)_data)[i-1] );
		}
		set( index, value );
		_count++;
	}

	/**
	 * Overloaded assignment operator.
	 * @param other A reference to the array to copy
	 * @return A reference to this object.
	 */
	inline daeTArray<T> &operator=( const daeTArray<T> &other ) {
		clear();
		_count = other._count;
		_capacity = other._capacity;
		grow(_capacity);
		for(size_t i=0;i<_count;i++)
			set( i, other[i] ); 	

		return *this;
	}
};


#endif //__DAE_ARRAY_H__


