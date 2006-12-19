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

#ifndef __DAE_STRING_REF_H__
#define __DAE_STRING_REF_H__

#include <dae/daeMemorySystem.h>
#include <dae/daeStringTable.h>

/**
  *Defines the @c daeStringRef class.
 */
class daeStringRef
{
public:
	/**
	 * Macro that defines new and delete overrides for this class
	 */
	DAE_ALLOC;
private:
	daeString _string;
	static daeStringTable &_stringTable();
public:

	/**
	 * Destructor
	 */
	inline ~daeStringRef() { _string = NULL; }

	/**
	 * Constructor
	 */
	inline daeStringRef() { _string = NULL; }

	/**
	 * Constructor that copies from another @c daeStringRef.
	 * @param other Reference to copy from.
	 */
	inline daeStringRef(const daeStringRef& other) {
		_string = other._string; }

	/**
	 * Constructor that creates from a <tt>const char *.</tt>
	 * @param string External string to create from.
	 */
	DLLSPEC daeStringRef(daeString string);

	/**
	 * Assignment operator.
	 * @param other The daeStringRef to copy.
	 * @return A reference to this object.
	 */
	inline const daeStringRef& operator= (const daeStringRef& other) {
		_string = other._string;
		return *this;
	}

	/**
	 * Sets a string from an external <tt>const char *.</tt>
	 * @param string The daeString to copy.
	 * @return A reference to this object.
	 */
	DLLSPEC const daeStringRef& set(daeString string);

	/**
	 * Assignment operator from an external <tt>const char *.</tt>
	 * @param string The daeString to copy.
	 * @return A reference to this object.
	 */
	DLLSPEC const daeStringRef& operator= (daeString string);

	/**
	 * Cast operator that returns a <tt>const char *.</tt>
	 */
	inline operator daeString() const { return _string; }

	/**
	 * Comparison operator, the comparison is done via pointers as both
	 * strings will have same pointer if they are the same address
	 * @param other The daeStringRef to compare
	 * @return True if strings are equal. False otherwise.
	 */
	inline bool operator==(const daeStringRef& other) const{
		//return (other._string == _string); }
		return (!strcmp(other._string, _string)); }

//Contributed by Nus - Wed, 08 Nov 2006
	/**
	 * Release string table...
	 */
	static void releaseStringTable(void);
//--------------------
};

typedef daeTArray<daeStringRef> daeStringRefArray;
typedef daeTArray<daeStringRefArray> daeStringRefArrayArray;

#endif //__DAE_STRING_REF_H__
