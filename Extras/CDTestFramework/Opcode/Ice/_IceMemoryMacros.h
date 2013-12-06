/*
 *	ICE / OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains all memory macros.
 *	\file		IceMemoryMacros.h
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICEMEMORYMACROS_H__
#define __ICEMEMORYMACROS_H__

#undef ZeroMemory
#undef CopyMemory
#undef MoveMemory
#undef FillMemory

	//!	Clears a buffer.
	//!	\param		addr	[in] buffer address
	//!	\param		size	[in] buffer length
	//!	\see		FillMemory
	//!	\see		StoreDwords
	//!	\see		CopyMemory
	//!	\see		MoveMemory
	inline_ void ZeroMemory(void* addr, udword size)					{ memset(addr, 0, size);		}

	//!	Fills a buffer with a given byte.
	//!	\param		addr	[in] buffer address
	//!	\param		size	[in] buffer length
	//!	\param		val		[in] the byte value
	//!	\see		StoreDwords
	//!	\see		ZeroMemory
	//!	\see		CopyMemory
	//!	\see		MoveMemory
	inline_ void FillMemory(void* dest, udword size, ubyte val)			{ memset(dest, val, size);		}

	//!	Fills a buffer with a given dword.
	//!	\param		addr	[in] buffer address
	//!	\param		nb		[in] number of dwords to write
	//!	\param		value	[in] the dword value
	//!	\see		FillMemory
	//!	\see		ZeroMemory
	//!	\see		CopyMemory
	//!	\see		MoveMemory
	//!	\warning	writes nb*4 bytes !
	inline_ void StoreDwords(udword* dest, udword nb, udword value)
	{
		// The asm code below **SHOULD** be equivalent to one of those C versions
		// or the other if your compiled is good: (checked on VC++ 6.0)
		//
		//	1) while(nb--)	*dest++ = value;
		//
		//	2) for(udword i=0;i<nb;i++)	dest[i] = value;
		//
		_asm push eax
		_asm push ecx
		_asm push edi
		_asm mov edi, dest
		_asm mov ecx, nb
		_asm mov eax, value
		_asm rep stosd
		_asm pop edi
		_asm pop ecx
		_asm pop eax
	}

	//!	Copies a buffer.
	//!	\param		addr	[in] destination buffer address
	//!	\param		addr	[in] source buffer address
	//!	\param		size	[in] buffer length
	//!	\see		ZeroMemory
	//!	\see		FillMemory
	//!	\see		StoreDwords
	//!	\see		MoveMemory
	inline_ void CopyMemory(void* dest, const void* src, udword size)	{ memcpy(dest, src, size);		}

	//!	Moves a buffer.
	//!	\param		addr	[in] destination buffer address
	//!	\param		addr	[in] source buffer address
	//!	\param		size	[in] buffer length
	//!	\see		ZeroMemory
	//!	\see		FillMemory
	//!	\see		StoreDwords
	//!	\see		CopyMemory
	inline_ void MoveMemory(void* dest, const void* src, udword size)	{ memmove(dest, src, size);		}

	#define SIZEOFOBJECT		sizeof(*this)									//!< Gives the size of current object. Avoid some mistakes (e.g. "sizeof(this)").
	//#define CLEAROBJECT		{ memset(this, 0, SIZEOFOBJECT);	}			//!< Clears current object. Laziness is my business. HANDLE WITH CARE.
	#define DELETESINGLE(x)		if (x) { delete x;				x = null; }		//!< Deletes an instance of a class.
	#define DELETEARRAY(x)		if (x) { delete []x;			x = null; }		//!< Deletes an array.
	#define SAFE_RELEASE(x)		if (x) { (x)->Release();		(x) = null; }	//!< Safe D3D-style release
	#define SAFE_DESTRUCT(x)	if (x) { (x)->SelfDestruct();	(x) = null; }	//!< Safe ICE-style release

#ifdef __ICEERROR_H__
	#define CHECKALLOC(x)		if(!x) return SetIceError("Out of memory.", EC_OUT_OF_MEMORY);	//!< Standard alloc checking. HANDLE WITH CARE.
#else
	#define CHECKALLOC(x)		if(!x) return false;
#endif

	//! Standard allocation cycle
	#define SAFE_ALLOC(ptr, type, count)	DELETEARRAY(ptr);	ptr = new type[count];	CHECKALLOC(ptr);

#endif // __ICEMEMORYMACROS_H__
