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
 *	Contains code for random generators.
 *	\file		IceRandom.h
 *	\author		Pierre Terdiman
 *	\date		August, 9, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICERANDOM_H__
#define __ICERANDOM_H__

	FUNCTION ICECORE_API	void	SRand(udword seed);
	FUNCTION ICECORE_API	udword	Rand();

	//! Returns a unit random floating-point value
	inline_ float UnitRandomFloat()	{ return float(Rand()) * ONE_OVER_RAND_MAX;	}

	//! Returns a random index so that 0<= index < max_index
	ICECORE_API	udword GetRandomIndex(udword max_index);

	class ICECORE_API BasicRandom
	{
		public:

		//! Constructor
		inline_				BasicRandom(udword seed=0)	: mRnd(seed)	{}
		//! Destructor
		inline_				~BasicRandom()								{}

		inline_	void		SetSeed(udword seed)		{ mRnd = seed;											}
		inline_	udword		GetCurrentValue()	const	{ return mRnd;											}
		inline_	udword		Randomize()					{ mRnd = mRnd * 2147001325 + 715136305; return mRnd;	}

		private:
				udword		mRnd;
	};

#endif // __ICERANDOM_H__

