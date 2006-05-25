/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FUCrc32.h
	This file contains the CRC-32 hashing functions.
*/

#ifndef _FU_CRC32_H_
#define _FU_CRC32_H_

/**
	CRC-32 hashing functions.
	CRC-32 is a commonly used hashing mechanism for strings.

	@ingroup FUtils
*/
namespace FUCrc32
{
	/** A CRC32 hash value. */
	typedef uint32 crc32;

	/** Hashes a string.
		@param text The string to hash.
		@return The 32-bit hash value. */
	crc32 FCOLLADA_EXPORT CRC32(const fstring& text);
	crc32 FCOLLADA_EXPORT CRC32(const fchar* text); /**< See above. */
	crc32 FCOLLADA_EXPORT CRC32(const string& text); /**< See above. */
	crc32 FCOLLADA_EXPORT CRC32(const char* text); /**< See above. */
};

#endif // _FU_CRC32_H_

