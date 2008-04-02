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
 *	Contains misc. useful macros & defines.
 *	\file		IceUtils.h
 *	\author		Pierre Terdiman (collected from various sources)
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICEUTILS_H__
#define __ICEUTILS_H__

	#define START_RUNONCE	{ static bool __RunOnce__ = false;	if(!__RunOnce__){
	#define END_RUNONCE		__RunOnce__ = true;}}

	//! Reverse all the bits in a 32 bit word (from Steve Baker's Cute Code Collection)
	//! (each line can be done in any order.
	inline_ void	ReverseBits(udword& n)
	{
		n = ((n >>  1) & 0x55555555) | ((n <<  1) & 0xaaaaaaaa);
		n = ((n >>  2) & 0x33333333) | ((n <<  2) & 0xcccccccc);
		n = ((n >>  4) & 0x0f0f0f0f) | ((n <<  4) & 0xf0f0f0f0);
		n = ((n >>  8) & 0x00ff00ff) | ((n <<  8) & 0xff00ff00);
		n = ((n >> 16) & 0x0000ffff) | ((n << 16) & 0xffff0000);
		// Etc for larger intergers (64 bits in Java)
		// NOTE: the >> operation must be unsigned! (>>> in java)
	}

	//! Count the number of '1' bits in a 32 bit word (from Steve Baker's Cute Code Collection)
	inline_ udword	CountBits(udword n)
	{
		// This relies of the fact that the count of n bits can NOT overflow 
		// an n bit interger. EG: 1 bit count takes a 1 bit interger, 2 bit counts
		// 2 bit interger, 3 bit count requires only a 2 bit interger.
		// So we add all bit pairs, then each nible, then each byte etc...
		n = (n & 0x55555555) + ((n & 0xaaaaaaaa) >> 1);
		n = (n & 0x33333333) + ((n & 0xcccccccc) >> 2);
		n = (n & 0x0f0f0f0f) + ((n & 0xf0f0f0f0) >> 4);
		n = (n & 0x00ff00ff) + ((n & 0xff00ff00) >> 8);
		n = (n & 0x0000ffff) + ((n & 0xffff0000) >> 16);
		// Etc for larger intergers (64 bits in Java)
		// NOTE: the >> operation must be unsigned! (>>> in java)
		return n;
	}

	//! Even faster?
	inline_ udword	CountBits2(udword bits)
	{
		bits = bits - ((bits >> 1) & 0x55555555);
		bits = ((bits >> 2) & 0x33333333) + (bits & 0x33333333);
		bits = ((bits >> 4) + bits) & 0x0F0F0F0F;
		return (bits * 0x01010101) >> 24;
	}

	//! Spread out bits.	EG	00001111  ->   0101010101
	//! 						00001010  ->   0100010000
	//! This is used to interleve to intergers to produce a `Morten Key'
	//! used in Space Filling Curves (See DrDobbs Journal, July 1999)
	//! Order is important.
	inline_ void	SpreadBits(udword& n)
	{
		n = ( n & 0x0000ffff) | (( n & 0xffff0000) << 16);
		n = ( n & 0x000000ff) | (( n & 0x0000ff00) <<  8);
		n = ( n & 0x000f000f) | (( n & 0x00f000f0) <<  4);
		n = ( n & 0x03030303) | (( n & 0x0c0c0c0c) <<  2);
		n = ( n & 0x11111111) | (( n & 0x22222222) <<  1);
	}

	// Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value: 
	inline_ udword	nlpo2(udword x)
	{
		x |= (x >> 1);
		x |= (x >> 2);
		x |= (x >> 4);
		x |= (x >> 8);
		x |= (x >> 16);
		return x+1;
	}

	//! Test to see if a number is an exact power of two (from Steve Baker's Cute Code Collection)
	inline_ bool	IsPowerOfTwo(udword n)				{ return ((n&(n-1))==0);					}

	//! Zero the least significant '1' bit in a word. (from Steve Baker's Cute Code Collection)
	inline_ void	ZeroLeastSetBit(udword& n)			{ n&=(n-1);									}

	//! Set the least significant N bits in a word. (from Steve Baker's Cute Code Collection)
	inline_ void	SetLeastNBits(udword& x, udword n)	{ x|=~(~0<<n);								}

	//! Classic XOR swap (from Steve Baker's Cute Code Collection)
	//! x ^= y;		/* x' = (x^y) */
	//! y ^= x;		/* y' = (y^(x^y)) = x */
	//! x ^= y;		/* x' = (x^y)^x = y */
	inline_ void	Swap(udword& x, udword& y)			{ x ^= y; y ^= x; x ^= y;					}

	//! Little/Big endian (from Steve Baker's Cute Code Collection)
	//!
	//! Extra comments by Kenny Hoff:
	//! Determines the byte-ordering of the current machine (little or big endian)
	//! by setting an integer value to 1 (so least significant bit is now 1); take
	//! the address of the int and cast to a byte pointer (treat integer as an
	//! array of four bytes); check the value of the first byte (must be 0 or 1).
	//! If the value is 1, then the first byte least significant byte and this
	//! implies LITTLE endian. If the value is 0, the first byte is the most
	//! significant byte, BIG endian. Examples:
	//!      integer 1 on BIG endian: 00000000 00000000 00000000 00000001
	//!   integer 1 on LITTLE endian: 00000001 00000000 00000000 00000000
	//!---------------------------------------------------------------------------
	//! int IsLittleEndian()	{ int x=1;	return ( ((char*)(&x))[0] );	}
	inline_ char	LittleEndian()						{ int i = 1; return *((char*)&i);			}

	//!< Alternative abs function
	inline_ udword	abs_(sdword x)					{ sdword y= x >> 31;	return (x^y)-y;		}

	//!< Alternative min function
	inline_ sdword	min_(sdword a, sdword b)			{ sdword delta = b-a;	return a + (delta&(delta>>31));	}

	// Determine if one of the bytes in a 4 byte word is zero
	inline_	BOOL	HasNullByte(udword x)			{ return ((x + 0xfefefeff) & (~x) & 0x80808080);		}

	// To find the smallest 1 bit in a word  EG: ~~~~~~10---0    =>    0----010---0
	inline_	udword	LowestOneBit(udword w)			{ return ((w) & (~(w)+1));					}
//	inline_	udword	LowestOneBit_(udword w)			{ return ((w) & (-(w)));					}

	// Most Significant 1 Bit
	// Given a binary integer value x, the most significant 1 bit (highest numbered element of a bit set)
	// can be computed using a SWAR algorithm that recursively "folds" the upper bits into the lower bits.
	// This process yields a bit vector with the same most significant 1 as x, but all 1's below it.
	 // Bitwise AND of the original value with the complement of the "folded" value shifted down by one
	// yields the most significant bit. For a 32-bit value: 
	inline_ udword	msb32(udword x)
	{
		x |= (x >> 1);
		x |= (x >> 2);
		x |= (x >> 4);
		x |= (x >> 8);
		x |= (x >> 16);
		return (x & ~(x >> 1));
	}

	/*
	"Just call it repeatedly with various input values and always with the same variable as "memory".
	The sharpness determines the degree of filtering, where 0 completely filters out the input, and 1
	does no filtering at all.

	I seem to recall from college that this is called an IIR (Infinite Impulse Response) filter. As opposed
	to the more typical FIR (Finite Impulse Response).

	Also, I'd say that you can make more intelligent and interesting filters than this, for example filters
	that remove wrong responses from the mouse because it's being moved too fast. You'd want such a filter
	to be applied before this one, of course."

	(JCAB on Flipcode)
	*/
	inline_ float	FeedbackFilter(float val, float& memory, float sharpness)
	{
		ASSERT(sharpness>=0.0f && sharpness<=1.0f && "Invalid sharpness value in feedback filter");
				if(sharpness<0.0f)	sharpness = 0.0f;
		else	if(sharpness>1.0f)	sharpness = 1.0f;
		return memory = val * sharpness + memory * (1.0f - sharpness);
	}

	//! If you can guarantee that your input domain (i.e. value of x) is slightly
	//! limited (abs(x) must be < ((1<<31u)-32767)), then you can use the
	//! following code to clamp the resulting value into [-32768,+32767] range:
	inline_ int	ClampToInt16(int x)
	{
//		ASSERT(abs(x) < (int)((1<<31u)-32767));

		int delta = 32767 - x;
		x += (delta>>31) & delta;
		delta = x + 32768;
		x -= (delta>>31) & delta;
		return x;
	}

	// Generic functions
	template<class Type> inline_ void TSwap(Type& a, Type& b)								{ const Type c = a; a = b; b = c;			}
	template<class Type> inline_ Type TClamp(const Type& x, const Type& lo, const Type& hi)	{ return ((x<lo) ? lo : (x>hi) ? hi : x);	}

	template<class Type> inline_ void TSort(Type& a, Type& b)
	{
		if(a>b)	TSwap(a, b);
	}

	template<class Type> inline_ void TSort(Type& a, Type& b, Type& c)
	{
		if(a>b)	TSwap(a, b);
		if(b>c)	TSwap(b, c);
		if(a>b)	TSwap(a, b);
		if(b>c)	TSwap(b, c);
	}

	// Prevent nasty user-manipulations (strategy borrowed from Charles Bloom)
//	#define PREVENT_COPY(curclass)	void operator = (const curclass& object)	{	ASSERT(!"Bad use of operator =");	}
	// ... actually this is better !
	#define PREVENT_COPY(cur_class)	private: cur_class(const cur_class& object);	cur_class& operator=(const cur_class& object);

	//! TO BE DOCUMENTED
	#define OFFSET_OF(Class, Member)	(size_t)&(((Class*)0)->Member)
	//! TO BE DOCUMENTED
	#define ARRAYSIZE(p)				(sizeof(p)/sizeof(p[0]))

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Returns the alignment of the input address.
	 *	\fn			Alignment()
	 *	\param		address	[in] address to check
	 *	\return		the best alignment (e.g. 1 for odd addresses, etc)
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	FUNCTION ICECORE_API udword Alignment(udword address);

	#define IS_ALIGNED_2(x)		((x&1)==0)
	#define IS_ALIGNED_4(x)		((x&3)==0)
	#define IS_ALIGNED_8(x)		((x&7)==0)

	inline_ void _prefetch(void const* ptr)		{ (void)*(char const volatile *)ptr;	}

	// Compute implicit coords from an index:
	// The idea is to get back 2D coords from a 1D index.
	// For example:
	//
	// 0		1		2	...	nbu-1
	// nbu		nbu+1	i	...
	//
	// We have i, we're looking for the equivalent (u=2, v=1) location.
	//		i = u + v*nbu
	// <=>	i/nbu = u/nbu + v
	// Since 0 <= u < nbu, u/nbu = 0 (integer)
	// Hence: v = i/nbu
	// Then we simply put it back in the original equation to compute u = i - v*nbu
	inline_ void Compute2DCoords(udword& u, udword& v, udword i, udword nbu)
	{
		v = i / nbu;
		u = i - (v * nbu);
	}

	// In 3D:	i = u + v*nbu + w*nbu*nbv
	// <=>		i/(nbu*nbv) = u/(nbu*nbv) + v/nbv + w
	// u/(nbu*nbv) is null since u/nbu was null already.
	// v/nbv is null as well for the same reason.
	// Hence w = i/(nbu*nbv)
	// Then we're left with a 2D problem: i' = i - w*nbu*nbv = u + v*nbu
	inline_ void Compute3DCoords(udword& u, udword& v, udword& w, udword i, udword nbu, udword nbu_nbv)
	{
		w = i / (nbu_nbv);
		Compute2DCoords(u, v, i - (w * nbu_nbv), nbu);
	}

#endif // __ICEUTILS_H__
