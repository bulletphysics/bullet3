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

#ifndef __DAE_DOM__
#define __DAE_DOM__

class daeMetaElement;

#ifdef WIN32
	#pragma warning(disable : 4324) // disable padding warning
	#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
	#pragma warning(disable:4996) //Turn off warnings about deprecated C routines
	#pragma warning(disable:4786) // Disable the "debug name too long" warning
	#pragma warning(disable:4244) // Disable the "possible loss of data" warning
	#pragma warning(disable:4018) // signed/unsigned int mismatch
	#pragma warning(disable:4267) // possible loss of data
	#pragma warning(disable:4530) // C++ exception handler used
#endif

daeMetaElement* initializeDomMeta();

#endif //__DAE_DOM__
