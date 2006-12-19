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

#ifndef __DAE_TYPES_H__
#define __DAE_TYPES_H__

#ifdef WIN32
#include <dae/daeWin32Platform.h>
#elif defined( __GCC__ )
#include <dae/daeGCCPlatform.h>
#else
#include <dae/daeGenericPlatform.h>
#endif
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <wchar.h>
#include <string.h>

#include <dae/daeError.h>

#define daeOffsetOf(class, member) \
 ((size_t)&(((class*)0x0100)->member) - (size_t)0x0100)
 
typedef PLATFORM_INT8		daeChar;
typedef PLATFORM_INT16		daeShort;
typedef PLATFORM_INT32		daeInt;
typedef PLATFORM_INT64		daeLong;
typedef PLATFORM_UINT8		daeUChar;
typedef PLATFORM_UINT16		daeUShort;
typedef PLATFORM_UINT32		daeUInt;
typedef PLATFORM_UINT64		daeULong;
typedef PLATFORM_FLOAT32	daeFloat;
typedef PLATFORM_FLOAT64	daeDouble;

// base types

typedef const char*			daeString;
typedef bool				daeBool;
typedef const void*			daeConstRawRef;
typedef void*				daeRawRef;
typedef daeInt				daeEnum;
typedef daeChar*			daeMemoryRef;

typedef daeChar				daeFixedName[512];

#include <dae/daeArray.h>
#include <dae/daeArrayTypes.h>

#endif //__DAE_TYPES_H__
