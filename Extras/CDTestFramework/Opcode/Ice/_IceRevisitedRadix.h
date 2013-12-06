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
 *	Contains source code from the article "Radix Sort Revisited".
 *	\file		IceRevisitedRadix.h
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICERADIXSORT_H__
#define __ICERADIXSORT_H__

	//! Allocate histograms & offsets locally
	#define RADIX_LOCAL_RAM

	enum RadixHint
	{
		RADIX_SIGNED,		//!< Input values are signed
		RADIX_UNSIGNED,		//!< Input values are unsigned

		RADIX_FORCE_DWORD = 0x7fffffff
	};

	class ICECORE_API RadixSort
	{
		public:
		// Constructor/Destructor
								RadixSort();
								~RadixSort();
		// Sorting methods
				RadixSort&		Sort(const udword* input, udword nb, RadixHint hint=RADIX_SIGNED);
				RadixSort&		Sort(const float* input, udword nb);

		//! Access to results. mRanks is a list of indices in sorted order, i.e. in the order you may further process your data
		inline_	const udword*	GetRanks()			const	{ return mRanks;		}

		//! mIndices2 gets trashed on calling the sort routine, but otherwise you can recycle it the way you want.
		inline_	udword*			GetRecyclable()		const	{ return mRanks2;		}

		// Stats
				udword			GetUsedRam()		const;
		//! Returns the total number of calls to the radix sorter.
		inline_	udword			GetNbTotalCalls()	const	{ return mTotalCalls;	}
		//! Returns the number of eraly exits due to temporal coherence.
		inline_	udword			GetNbHits()			const	{ return mNbHits;		}

		private:
#ifndef RADIX_LOCAL_RAM
				udword*			mHistogram;			//!< Counters for each byte
				udword*			mOffset;			//!< Offsets (nearly a cumulative distribution function)
#endif
				udword			mCurrentSize;		//!< Current size of the indices list
				udword*			mRanks;				//!< Two lists, swapped each pass
				udword*			mRanks2;
		// Stats
				udword			mTotalCalls;		//!< Total number of calls to the sort routine
				udword			mNbHits;			//!< Number of early exits due to coherence
		// Internal methods
				void			CheckResize(udword nb);
				bool			Resize(udword nb);
	};

#endif // __ICERADIXSORT_H__
