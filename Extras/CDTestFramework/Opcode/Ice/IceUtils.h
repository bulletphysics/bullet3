///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains misc. useful macros & defines.
 *	\file		IceUtils.h
 *	\author		Pierre Terdiman (personal code + collected from various sources)
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEUTILS_H
#define ICEUTILS_H

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
		// Etc for larger integers (64 bits in Java)
		// NOTE: the >> operation must be unsigned! (>>> in java)
	}

	//! Count the number of '1' bits in a 32 bit word (from Steve Baker's Cute Code Collection)
	inline_ udword	CountBits(udword n)
	{
		// This relies of the fact that the count of n bits can NOT overflow 
		// an n bit integer. EG: 1 bit count takes a 1 bit integer, 2 bit counts
		// 2 bit integer, 3 bit count requires only a 2 bit integer.
		// So we add all bit pairs, then each nible, then each byte etc...
		n = (n & 0x55555555) + ((n & 0xaaaaaaaa) >> 1);
		n = (n & 0x33333333) + ((n & 0xcccccccc) >> 2);
		n = (n & 0x0f0f0f0f) + ((n & 0xf0f0f0f0) >> 4);
		n = (n & 0x00ff00ff) + ((n & 0xff00ff00) >> 8);
		n = (n & 0x0000ffff) + ((n & 0xffff0000) >> 16);
		// Etc for larger integers (64 bits in Java)
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

	// "Population Count (Ones Count)
	// The population count of a binary integer value x is the number of one bits in the value. Although many machines have
	// single instructions for this, the single instructions are usually microcoded loops that test a bit per cycle; a log-time
	// algorithm coded in C is often faster. The following code uses a variable-precision SWAR algorithm to perform a tree
	// reduction adding the bits in a 32-bit value:"
	inline_ udword	ones32(udword x)
	{
		/* 32-bit recursive reduction using SWAR...
		but first step is mapping 2-bit values
		into sum of 2 1-bit values in sneaky way
		*/
		x -= ((x >> 1) & 0x55555555);
		x = (((x >> 2) & 0x33333333) + (x & 0x33333333));
		x = (((x >> 4) + x) & 0x0f0f0f0f);
		x += (x >> 8);
		x += (x >> 16);
		return (x & 0x0000003f);
		// "It is worthwhile noting that the SWAR population count algorithm given above can be improved upon for the case of
		// counting the population of multi-word bit sets. How? The last few steps in the reduction are using only a portion
		// of the SWAR width to produce their results; thus, it would be possible to combine these steps across multiple words
		// being reduced. One additional note: the AMD Athlon optimization guidelines suggest a very similar algorithm that
		// replaces the last three lines with return((x * 0x01010101) >> 24);. For the Athlon (which has a very fast integer
		// multiply), I would have expected AMD's code to be faster... but it is actually 6% slower according to my benchmarks
		// using a 1.2GHz Athlon (a Thunderbird). Why? Well, it so happens that GCC doesn't use a multiply instruction - it
		// writes out the equivalent shift and add sequence!"
	}

	// "Trailing Zero Count
	// Given the Least Significant 1 Bit and Population Count (Ones Count) algorithms, it is trivial to combine them to
	// construct a trailing zero count (as pointed-out by Joe Bowbeer):"
	inline_ udword	tzc(sdword x)
	{
		return(ones32((x & -x) - 1));
	}

	//! Spread out bits.	EG	00001111  ->   0101010101
	//! 						00001010  ->   0100010000
	//! This is used to interleave two integers to produce a `Morton Key'
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

	// "Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value:"
	inline_ udword	NextPowerOfTwo(udword x)
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
	inline_ udword	abs_(sdword x)						{ sdword y= x >> 31;	return (x^y)-y;		}

	// "Integer Minimum or Maximum
	// Given 2's complement integer values x and y, the minimum can be computed without any branches as
	// x+(((y-x)>>(WORDBITS-1))&(y-x)).
	// Logically, this works because the shift by (WORDBITS-1) replicates the sign bit to create a mask
	// -- be aware, however, that the C language does not require that shifts are signed even if their
	// operands are signed, so there is a potential portability problem. Additionally, one might think
	// that a shift by any number greater than or equal to WORDBITS would have the same effect, but many
	// instruction sets have shifts that behave strangely when such shift distances are specified. 
	// Of course, maximum can be computed using the same trick:
	// x-(((x-y)>>(WORDBITS-1))&(x-y))."

	//!< Alternative min function
	inline_ sdword	min_(sdword a, sdword b)			{ sdword delta = b-a;	return a + (delta&(delta>>31));	}
	//!< Alternative max function
	inline_ sdword	max_(sdword a, sdword b)			{ sdword delta = a-b;	return a - (delta&(delta>>31));	}

	// "Integer Selection
	// A branchless, lookup-free, alternative to code like if (a<b) x=c; else x=d; is ((((a-b) >> (WORDBITS-1)) & (c^d)) ^ d).
	// This code assumes that the shift is signed, which, of course, C does not promise."
	inline_ sdword IntegerSelection(sdword a, sdword b, sdword c, sdword d)
	{
		return ((((a-b)>>31) & (c^d)) ^ d);
	}

	// Determine if one of the bytes in a 4 byte word is zero
	inline_	BOOL	HasNullByte(udword x)				{ return ((x + 0xfefefeff) & (~x) & 0x80808080);		}

	// To find the smallest 1 bit in a word  EG: ~~~~~~10---0    =>    0----010---0
	inline_	udword	LowestOneBit(udword w)				{ return ((w) & (~(w)+1));								}
//	inline_	udword	LowestOneBit_(udword w)				{ return ((w) & (-(w)));								}

	// "Most Significant 1 Bit
	// Given a binary integer value x, the most significant 1 bit (highest numbered element of a bit set)
	// can be computed using a SWAR algorithm that recursively "folds" the upper bits into the lower bits.
	// This process yields a bit vector with the same most significant 1 as x, but all 1's below it.
	 // Bitwise AND of the original value with the complement of the "folded" value shifted down by one
	// yields the most significant bit. For a 32-bit value:"
	inline_ udword	msb32(udword x)
	{
		x |= (x >> 1);
		x |= (x >> 2);
		x |= (x >> 4);
		x |= (x >> 8);
		x |= (x >> 16);
		return (x & ~(x >> 1));
	}

	// "Gray Code Conversion
	// A Gray code is any binary coding sequence in which only a single bit position changes as we move from one value to the next.
	// There are many such codes, but the traditional one is computed such that the Kth Gray code is K^(K>>1).
	//
	// The well-known algorithm for conversion from Gray to binary is a linear sequence of XORs that makes it seem each bit must be
	// dealt with separately. Fortunately, that is equivalent to a parallel prefix XOR that can be computed using SWAR techniques
	// in log time. For 32-bit Gray code values produced as described above, the conversion from Gray code back to unsigned binary is:"
	inline_ udword g2b(udword gray)
	{
		gray ^= (gray >> 16);
		gray ^= (gray >> 8);
		gray ^= (gray >> 4);
		gray ^= (gray >> 2);
		gray ^= (gray >> 1);
		return gray;
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

	//! "If you can guarantee that your input domain (i.e. value of x) is slightly
	//! limited (abs(x) must be < ((1<<31u)-32767)), then you can use the
	//! following code to clamp the resulting value into [-32768,+32767] range:"
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
	#if !defined(_XBOX)
		// Already defined on Xbox.
		#define ARRAYSIZE(p)			(sizeof(p)/sizeof(p[0]))
	#endif

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

	// Updates a pointer with "stride" bytes
	inline_	void UpdatePtr(void*& ptr, udword stride)	{ ptr = ((ubyte*)ptr) + stride;			}

	// From Jon Watte IIRC
	inline_ void _prefetch(void const* ptr)				{ (void)*(char const volatile *)ptr;	}

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

	// Calling fsincos instead of fsin+fcos. Twice faster.
	inline_	void FSinCos(float& c, float& s, float f)
	{
		float LocalCos, LocalSin;
		float Local = f;
#ifdef WIN32
		_asm	fld		Local
		_asm	fsincos
		_asm	fstp	LocalCos
		_asm	fstp	LocalSin
#elif LINUX
		asm("fld	Local\n\t"
			"fsincos\n\t"
			"fstp	LocalCos\n\t"
			"fstp	LocalSin\n\t"
		);
#endif
		c = LocalCos;
		s = LocalSin;
	}

	// Modulo3 macros. See http://www.codercorner.com/Modulo3.htm
	#define GET_NEXT_INDICES(i, j, k)	\
		k = 0x01000201;					\
		k>>=(i<<3);						\
		j = k & 0xff;					\
		k>>=8;							\
		k&=0xff;

	#define GET_NEXT_INDICES2(i, j, k)	\
		j = ( 9 >> (i<<1)) & 3;			\
		k = (18 >> (i<<1)) & 3;

	// 0=>1, 1=>2, 2=>0
	inline_	udword Modulo3(udword i)
	{
		ASSERT(i==0 || i==1 || i==2);
		return (9 >> (i << 1)) & 3;
	}

#endif // ICEUTILS_H
