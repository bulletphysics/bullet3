/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
 * @mainpage FCollada Documentation
 *
 * @section intro_sec Introduction
 * The FCollada classes are designed to read and write Collada files.
 *
 * @section install_sec Installation
 *
 * @subsection step1 Step 1: Download
 * You can download the FCollada libraries from our website: http://www.feelingsoftware.com
 *
 * @section copyright Copyright
 * Copyright (C) 2005-2006 Feeling Software Inc.
 * MIT License: http://www.opensource.org/licenses/mit-license.php
 */

#ifndef _FCOLLADA_H_
#define _FCOLLADA_H_

#ifdef FCOLLADA_DLL
#error DO NOT USE THE DLL VERSION: IT IS STILL IN PROGRESS.
#endif

/**
	FCollada exception handling.
	Force this #define to 0 to disallow exception handling within the FCollada library.
	By default, a debug library will no handle exceptions so that your debugger can.
	In release, all exceptions should be handled so that your users receive a meaningful message,
	rather than crash your application. Force this #define to 0 only if your platform does not
	support exception handling.
*/
#ifdef _DEBUG
#define	FCOLLADA_EXCEPTION 0
#else
#define FCOLLADA_EXCEPTION 1
#endif

#include "FUtils/FUtils.h"

/**
	FCollada version number.
	You should verify that you have the correct version, if you use the FCollada library as a DLLs.
	For a history of version, check the Changes.txt file.
*/
#define FCOLLADA_VERSION 0x00010008 /* MMMM.NNNN */

/** 
	This namespace contains FCollada global functions and member variables
*/
namespace FCollada
{
	/** Retrieves the FCollada version number.
		Used for DLL-versions of the FCollada library: verify that you have a compatible version
		of the FCollada library using this function.
		@return The FCollada version number. */
	FCOLLADA_EXPORT unsigned long GetVersion();
}

#endif // _FCOLLADA_H_
