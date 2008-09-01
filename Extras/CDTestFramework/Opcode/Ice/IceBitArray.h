///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for bit arrays.
 *	\file		IceBitArray.h
 *	\author		Pierre Terdiman
 *	\date		February, 5, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEBITARRAY_H
#define ICEBITARRAY_H

	inline_ udword BitsToBytes(udword nb_bits)
	{
		return (nb_bits>>3) + ((nb_bits&7) ? 1 : 0);
	}

	inline_ udword BitsToDwords(udword nb_bits)
	{
		return (nb_bits>>5) + ((nb_bits&31) ? 1 : 0);
	}

	// Use that one instead of an array of bools. Takes less ram, nearly as fast [no bounds checkings and so on].
	class ICECORE_API BitArray
	{
		public:
		//! Constructor
								BitArray();
								BitArray(udword nb_bits);
		//! Destructor
								~BitArray();

				bool			Init(udword nb_bits);

		// Data management
		inline_	void			SetBit(udword bit_number)			{ mBits[bit_number>>5] |= 1<<(bit_number&31);				}
		inline_	void			ClearBit(udword bit_number)			{ mBits[bit_number>>5] &= ~(1<<(bit_number&31));			}
		inline_	void			ToggleBit(udword bit_number)		{ mBits[bit_number>>5] ^= 1<<(bit_number&31);				}

		inline_	void			ClearAll()							{ ZeroMemory(mBits, mSize*4);								}
		inline_	void			SetAll()							{ FillMemory(mBits, mSize*4, 0xff);							}

		// Data access
		inline_	BOOL			IsSet(udword bit_number)	const	{ return mBits[bit_number>>5] & (1<<(bit_number&31));		}

		inline_	const udword*	GetBits()					const	{ return mBits;												}
		inline_	udword			GetSize()					const	{ return mSize;												}

		protected:
				udword*			mBits;		//!< Array of bits
				udword			mSize;		//!< Size of the array in dwords
	};

	// - We consider square symmetric N*N matrices
	// - A N*N symmetric matrix has N(N+1)/2 elements
	// - A boolean version needs N(N+1)/16 bytes
	//		N		NbBits	NbBytes
	//		4		10		-
	//		8		36		4.5
	//		16		136		17		<= the one we select
	//		32		528		66
	static ubyte BitMasks[]		= { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
	static ubyte NegBitMasks[]	= { 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F };
	class ICECORE_API BoolSquareSymmetricMatrix16
	{
		public:
		inline_	udword	Index(udword x, udword y)	const	{ if(x>y)	Swap(x,y);	return x + (y ? ((y-1)*y)>>1 : 0);				}

		inline_	void	Set(udword x, udword y)				{ udword i = Index(x, y);	mBits[i>>3] |= BitMasks[i&7];				}
		inline_	void	Clear(udword x, udword y)			{ udword i = Index(x, y);	mBits[i>>3] &= NegBitMasks[i&7];			}
		inline_	void	Toggle(udword x, udword y)			{ udword i = Index(x, y);	mBits[i>>3] ^= BitMasks[i&7];				}
		inline_	bool	IsSet(udword x, udword y)	const	{ udword i = Index(x, y);	return (mBits[i>>3] & BitMasks[i&7])!=0;	}

		inline_	void	ClearAll()							{ ZeroMemory(mBits, 17);		}
		inline_	void	SetAll()							{ FillMemory(mBits, 17, 0xff);	}

				ubyte	mBits[17];
	};

#endif // ICEBITARRAY_H
