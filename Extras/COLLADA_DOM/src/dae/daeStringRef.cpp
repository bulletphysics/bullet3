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

#include <dae/daeStringRef.h>

//Contributed by Nus - Wed, 08 Nov 2006
// Nus: Use global pointer instead of local static.
static daeStringTable *pST = NULL;
//---------------------------

daeStringTable &daeStringRef::_stringTable()
{
//Contributed by Nus - Wed, 08 Nov 2006
  // static daeStringTable *st = new daeStringTable();
  // return *st;
  if(!pST)
    pST = new daeStringTable();
  return *pST;
}

void daeStringRef::releaseStringTable(void)
{
  if(pST) {
    delete pST;
    pST = NULL;
  }
}
//--------------------------------

daeStringRef::daeStringRef(daeString string)
{
	daeStringTable &st = _stringTable();
	_string = st.allocString(string);
}

const daeStringRef&
daeStringRef::set(daeString string)
{
	daeStringTable &st = _stringTable();
	_string = st.allocString(string);
	return *this;
}

const daeStringRef&
daeStringRef::operator= (daeString string)
{
	return set(string);
}
