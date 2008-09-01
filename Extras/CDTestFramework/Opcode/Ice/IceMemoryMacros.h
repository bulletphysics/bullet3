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
#ifndef ICEMEMORYMACROS_H
#define ICEMEMORYMACROS_H

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
	inline_ void ZeroMemory(void* addr, regsize size)					{ memset(addr, 0, size);		}

	//!	Fills a buffer with a given byte.
	//!	\param		addr	[in] buffer address
	//!	\param		size	[in] buffer length
	//!	\param		val		[in] the byte value
	//!	\see		StoreDwords
	//!	\see		ZeroMemory
	//!	\see		CopyMemory
	//!	\see		MoveMemory
	inline_ void FillMemory(void* dest, regsize size, ubyte val)			{ memset(dest, val, size);		}

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
	inline_ void CopyMemory(void* dest, const void* src, regsize size)	{ memcpy(dest, src, size);		}

	//!	Moves a buffer.
	//!	\param		addr	[in] destination buffer address
	//!	\param		addr	[in] source buffer address
	//!	\param		size	[in] buffer length
	//!	\see		ZeroMemory
	//!	\see		FillMemory
	//!	\see		StoreDwords
	//!	\see		CopyMemory
	inline_ void MoveMemory(void* dest, const void* src, regsize size)	{ memmove(dest, src, size);		}

	//!	Flexible buffer copy
	//!	\param		src			[in] source buffer address
	//!	\param		dst			[in] destination buffer address
	//!	\param		nb_elem		[in] number of elements to copy
	//!	\param		elem_size	[in] size of an element
	//!	\param		stride		[in] stride in bytes, including size of element
	inline_ void FlexiCopy(const void* src, void* dst, udword nb_elem, regsize elem_size, udword stride)
	{
		ubyte* d = (ubyte*)dst;
		const ubyte* s = (const ubyte*)src;
		const ubyte* Last = s + stride*nb_elem;
		while(s!=Last)
		{
			CopyMemory(d, s, elem_size);
			d += elem_size;
			s += stride;
		}
	}

	//! Gives the size of current object. This avoids some mistakes (e.g. "sizeof(this)").
	#define SIZEOFOBJECT		sizeof(*this)

	//! Clears current object. Laziness is my business! HANDLE WITH CARE. ### Removed, too dangerous, cleared too many v-tables
	//#define CLEAROBJECT		{ memset(this, 0, SIZEOFOBJECT);	}

	// The two macros below are here for several reasons:
	// - sometimes we write "delete x" instead of "delete []x" just because we don't pay attention. Using the macro forces you
	//   to think about what you're deleting, just because you have to write the macro's name (SINGLE or ARRAY).
	// - always clearing the pointer afterwards prevents some double-deletion in various situations.
	// - deleting null is a valid operation according to the standard, yet some lame memory managers don't like it. In sake of
	//   robustness, we avoid trying.

	//! Deletes an instance of a class.
	#define DELETESINGLE(x)		if (x) { delete x;				x = null; }
	//! Deletes an array.
	#define DELETEARRAY(x)		if (x) { delete []x;			x = null; }

	//! Safe D3D-style release
	#define SAFE_RELEASE(x)		if (x) { (x)->Release();		(x) = null; }

	//! Safe ICE-style release
	#define SAFE_DESTRUCT(x)	if (x) { (x)->SelfDestruct();	(x) = null; }

#ifdef ICEERROR_H
	//! Standard alloc checking. HANDLE WITH CARE. Relies on strict coding rules. Probably shouldn't be used outside of ICE.
	#define CHECKALLOC(x)		if(!x) return SetIceError("Out of memory.", EC_OUT_OF_MEMORY);
#else
	#define CHECKALLOC(x)		if(!x) return false;
#endif

	//! Standard allocation cycle
	#define SAFE_ALLOC(ptr, type, count)		DELETEARRAY(ptr);	ptr = new type[count];		CHECKALLOC(ptr);
	#define SAFE_ICE_ALLOC(ptr, type, count)	DELETEARRAY(ptr);	ptr = ICE_NEW(type)[count];	CHECKALLOC(ptr);

	//! Don't use inline for alloca !!!
#ifdef WIN32
	#define StackAlloc(x)	_alloca(x)
#elif LINUX
	#define StackAlloc(x)	alloca(x)
#elif defined(__APPLE__)
	#define StackAlloc(x)	alloca(x)
#elif defined(_XBOX)
	#define StackAlloc(x)	_alloca(x)
#endif

#ifdef _DEBUG
//	#define ICE_ALLOC_TMP(x)		GetAllocator()->mallocDebug(x, __FILE__, __LINE__, #x, MEMORY_TEMP)
//	#define ICE_ALLOC(x)			GetAllocator()->mallocDebug(x, __FILE__, __LINE__, #x, MEMORY_PERSISTENT)
	#define ICE_ALLOC_TMP(x)		GetAllocator()->mallocDebug(x, __FILE__, __LINE__, "(undefined)", MEMORY_TEMP)
	#define ICE_ALLOC(x)			GetAllocator()->mallocDebug(x, __FILE__, __LINE__, "(undefined)", MEMORY_PERSISTENT)
	#define ICE_ALLOC_TMP2(x, y)	GetAllocator()->mallocDebug(x, __FILE__, __LINE__, #y, MEMORY_TEMP)
	#define ICE_ALLOC2(x, y)		GetAllocator()->mallocDebug(x, __FILE__, __LINE__, #y, MEMORY_PERSISTENT)
#else
	#define ICE_ALLOC_TMP(x)		GetAllocator()->malloc(x, MEMORY_TEMP)
	#define ICE_ALLOC(x)			GetAllocator()->malloc(x, MEMORY_PERSISTENT)
#endif
	#define	ICE_FREE(x)				if(x)	{ GetAllocator()->free(x); x = null;	}

#ifdef DONT_TRACK_MEMORY_LEAKS
	#ifdef _DEBUG
		#define ICE_NEW_TMP(x)		new(__FILE__, __LINE__, #x, MEMORY_TEMP) x
		#define ICE_NEW(x)			new(__FILE__, __LINE__, #x, MEMORY_PERSISTENT) x
	#else
		#define ICE_NEW_TMP(x)		new(MEMORY_TEMP) x
		#define ICE_NEW(x)			new(MEMORY_PERSISTENT) x
	#endif
#else
	#define ICE_NEW_TMP(x)			new x
	#define ICE_NEW(x)				new	x
#endif

#endif // ICEMEMORYMACROS_H
