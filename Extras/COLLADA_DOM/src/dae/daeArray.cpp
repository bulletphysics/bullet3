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

#include <dae/daeArrayTypes.h>
#include <dae/daeArray.h>

daeArray::daeArray():_count(0),_capacity(0),_data(NULL),_elementSize(4),_type(NULL)
{
}

daeArray::~daeArray()
{
	clear();
}

void 
daeArray::clear()
{
	if (_data)
	{
		daeMemorySystem::free("array",_data);
		_capacity = 0;
		_data = NULL;
		_count = 0;
	}
}

daeInt
daeArray::removeIndex(size_t index)
{
	if ((index >= _count)||(_count < 1)||(_data == NULL))
		return(DAE_ERR_INVALID_CALL);
	if (_count-1-index)
		memmove(_data+index*_elementSize, _data+(index+1)*_elementSize, _elementSize*(_count-1-index));
	memset(_data+(_count-1)*_elementSize,0,_elementSize);
	_count--;
	return(DAE_OK);
}

void
daeArray::grow(size_t sz)
{
	
	if (sz < _capacity)
		return;

	size_t newSize = 4*_elementSize;
	size_t neccesarySize = (sz+1)*_elementSize;
	
	while(newSize < neccesarySize) {
		if (newSize < 16384)
			newSize *= 2;
		else
			newSize += 16384;
	}
	
	size_t newCapacity = newSize/_elementSize;
	daeChar* newData =
		(daeChar*)daeMemorySystem::malloc("array", newCapacity*_elementSize);
	
	if (_data != NULL)
		memcpy(newData,_data,_capacity*_elementSize);
	//else
	//	memset(newData,0,_capacity*_elementSize);
	
	memset(newData+_capacity*_elementSize,0,
		   (newCapacity-_capacity)*_elementSize);
	
	if (_data != NULL)
		daeMemorySystem::free("array",_data);
	
	_data = newData;
	_capacity = newCapacity;
}

