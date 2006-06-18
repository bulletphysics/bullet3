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

#ifndef __DAE_ARRAY_TYPES_H__
#define __DAE_ARRAY_TYPES_H__

#include <dae/daeTypes.h>
#include <dae/daeArray.h>
typedef daeTArray<daeInt> daeIntArray;
typedef daeTArray<daeUInt> daeUIntArray;
typedef daeTArray<daeFloat> daeFloatArray;
typedef daeTArray<daeEnum> daeEnumArray;
typedef daeTArray<daeString> daeStringArray;
typedef daeTArray<daeChar> daeCharArray;
typedef daeTArray<daeBool> daeBoolArray;
typedef daeTArray<daeDouble> daeDoubleArray;
typedef daeTArray<daeLong> daeLongArray;
typedef daeTArray<daeShort> daeShortArray;

#endif //__DAE_ARRAY_TYPES_H__
