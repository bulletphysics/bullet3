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

#include <dae/daeStringTable.h>

daeStringTable::daeStringTable(int stringBufferSize):_stringBufferSize(stringBufferSize)
{
	//allocate initial buffer
	allocateBuffer();
}

daeString daeStringTable::allocateBuffer()
{
	daeString buf = new daeChar[_stringBufferSize];
	_stringBuffersList.append(buf);
	_stringBufferIndex = 0;
	return buf;
}

daeString daeStringTable::allocString(daeString string)
{
	size_t stringSize = strlen(string) + 1;
	size_t sizeLeft = _stringBufferSize - _stringBufferIndex;
	daeString buf;
	if (sizeLeft < stringSize)
	{
		buf = allocateBuffer();
	}
	else
	{
		buf = _stringBuffersList.get((daeInt)_stringBuffersList.getCount()-1);
	}
	daeChar *str = (char*)buf + _stringBufferIndex;
	memcpy(str,string,stringSize);
	_stringBufferIndex += stringSize;

	int align = sizeof(void*);
	_stringBufferIndex = (_stringBufferIndex+(align-1)) & (~(align-1));

	//assert
#if defined(_DEBUG) && defined(WIN32)
	if (_stringBufferIndex>_stringBufferSize)
	{
		//error the size of the buffer is not aligned,
		//or there is an internal error
		assert(0);
		return NULL;
	}
#endif
	
	return str;
}

void daeStringTable::clear()
{
	unsigned int i;
	for (i=0;i<_stringBuffersList.getCount();i++)
#if _MSC_VER <= 1200
		delete [] (char *) _stringBuffersList[i];
#else
		delete [] _stringBuffersList[i];
#endif

	_stringBuffersList.clear();
	_stringBufferIndex = 0;
}
