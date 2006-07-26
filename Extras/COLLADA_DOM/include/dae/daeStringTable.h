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

#ifndef __DAE_STRING_TABLE_H__
#define __DAE_STRING_TABLE_H__
#include <dae/daeTypes.h>
#include <dae/daeMemorySystem.h>

/**
 * The @c daeStringTable is a simple string table class to hold a float list of strings
 * without a lot of allocations.
 */
class daeStringTable
{
public: // allocate/construct/destruct/deallocate
	/**
	 * Macro that defines new and delete overrides for this class
	 */
	DAE_ALLOC;
	/**
	 * Constructor which specifies fixed buffer size.
	 * @param stringBufferSize The size of the buffer to create for string allocation.
	 */
	daeStringTable(int stringBufferSize = 1024*1024);

	/**
	 * Destructor.
	 */
	~daeStringTable() { clear(); }

public: // INTERFACE
	/**
	 * Allocates a string from the table.
	 * @param string <tt> const char * </tt> to copy into the table.
	 * @return Returns an allocated string.
	 */
	daeString allocString(daeString string);

	/**
	 * Clears the storage.
	 */
	void clear();

private: // MEMBERS
	size_t _stringBufferSize;
	size_t _stringBufferIndex;
	daeStringArray _stringBuffersList;

	daeString allocateBuffer();

	daeString _empty;
};

#endif //__DAE_STRING_TABLE_H__
