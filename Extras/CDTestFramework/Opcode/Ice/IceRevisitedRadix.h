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
#ifndef ICERADIXSORT_H
#define ICERADIXSORT_H

	//! Allocate histograms & offsets locally
	#define RADIX_LOCAL_RAM

	enum RadixHint
	{
		RADIX_SIGNED,		//!< Input values are signed
		RADIX_UNSIGNED,		//!< Input values are unsigned

		RADIX_FORCE_DWORD = 0x7fffffff
	};

	class ICECORE_API RadixSort : public Allocateable
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

				bool			SetRankBuffers(udword* ranks0, udword* ranks1);

								PREVENT_COPY(RadixSort)
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
		// Stack-radix
				bool			mDeleteRanks;		//!<
		// Internal methods
				void			CheckResize(udword nb);
				bool			Resize(udword nb);
	};

	#define StackRadixSort(name, ranks0, ranks1)	\
		RadixSort name;								\
		name.SetRankBuffers(ranks0, ranks1);

#endif // ICERADIXSORT_H
