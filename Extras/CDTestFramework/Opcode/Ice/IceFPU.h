///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains FPU related code.
 *	\file		IceFPU.h
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEFPU_H
#define ICEFPU_H

	#define	SIGN_BITMASK			0x80000000

	//! Integer representation of a floating-point value.
	#define IR(x)					((udword&)(x))

	//! Signed integer representation of a floating-point value.
	#define SIR(x)					((sdword&)(x))

	//! Absolute integer representation of a floating-point value
	#define AIR(x)					(IR(x)&0x7fffffff)

	//! Floating-point representation of an integer value.
	#define FR(x)					((float&)(x))

	//! Integer-based comparison of a floating point value.
	//! Don't use it blindly, it can be faster or slower than the FPU comparison, depends on the context.
	#define IS_NEGATIVE_FLOAT(x)	(IR(x)&0x80000000)

	//! Checks 2 values have different signs
	inline_ BOOL DifferentSign(float f0, float f1)
	{
		return (IR(f0)^IR(f1))&SIGN_BITMASK;
	}

	//! Fast fabs for floating-point values. It just clears the sign bit.
	//! Don't use it blindy, it can be faster or slower than the FPU comparison, depends on the context.
	inline_ float FastFabs(float x)
	{
		udword FloatBits = IR(x)&0x7fffffff;
		return FR(FloatBits);
	}

	//! Fast square root for floating-point values.
	inline_ float FastSqrt(float square)
	{
			float retval;

			__asm {
					mov             eax, square
					sub             eax, 0x3F800000
					sar             eax, 1
					add             eax, 0x3F800000
					mov             [retval], eax
			}
			return retval;
	}

	//! Saturates positive to zero.
	inline_ float fsat(float f)
	{
		udword y = (udword&)f & ~((sdword&)f >>31);
		return (float&)y;
	}

	//! Computes 1.0f / sqrtf(x).
	inline_ float frsqrt(float f)
	{
		float x = f * 0.5f;
		udword y = 0x5f3759df - ((udword&)f >> 1);
		// Iteration...
		(float&)y  = (float&)y * ( 1.5f - ( x * (float&)y * (float&)y ) );
		// Result
		return (float&)y;
	}

	//! Computes 1.0f / sqrtf(x). Comes from NVIDIA.
	inline_ float InvSqrt(const float& x)
	{
		udword tmp = (udword(IEEE_1_0 << 1) + IEEE_1_0 - *(udword*)&x) >> 1;   
		float y = *(float*)&tmp;                                             
		return y * (1.47f - 0.47f * x * y * y);
	}

	//! Computes 1.0f / sqrtf(x). Comes from Quake3. Looks like the first one I had above.
	//! See http://www.magic-software.com/3DGEDInvSqrt.html
	inline_ float RSqrt(float number)
	{
		long i;
		float x2, y;
		const float threehalfs = 1.5f;

		x2 = number * 0.5f;
		y  = number;
		i  = * (long *) &y;
		i  = 0x5f3759df - (i >> 1);
		y  = * (float *) &i;
		y  = y * (threehalfs - (x2 * y * y));

		return y;
	}

	//! TO BE DOCUMENTED
	inline_ float fsqrt(float f)
	{
		udword y = ( ( (sdword&)f - 0x3f800000 ) >> 1 ) + 0x3f800000;
		// Iteration...?
		// (float&)y = (3.0f - ((float&)y * (float&)y) / f) * (float&)y * 0.5f;
		// Result
		return (float&)y;
	}

	//! Returns the float ranged espilon value.
	inline_ float fepsilon(float f)
	{
		udword b = (udword&)f & 0xff800000;
		udword a = b | 0x00000001;
		(float&)a -= (float&)b;
		// Result
		return (float&)a;
	}

	//! Is the float valid ?
	inline_ bool IsNAN(float value)				{ return (IR(value)&0x7f800000) == 0x7f800000;	}
	inline_ bool IsIndeterminate(float value)	{ return IR(value) == 0xffc00000;				}
	inline_ bool IsPlusInf(float value)			{ return IR(value) == 0x7f800000;				}
	inline_ bool IsMinusInf(float value)		{ return IR(value) == 0xff800000;				}

	inline_	bool IsValidFloat(float value)
	{
		if(IsNAN(value))			return false;
		if(IsIndeterminate(value))	return false;
		if(IsPlusInf(value))		return false;
		if(IsMinusInf(value))		return false;
		return true;
	}

	#define CHECK_VALID_FLOAT(x)	ASSERT(IsValidFloat(x));

/*
	//! FPU precision setting function.
	inline_ void SetFPU()
	{
		// This function evaluates whether the floating-point
		// control word is set to single precision/round to nearest/
		// exceptions disabled. If these conditions don't hold, the
		// function changes the control word to set them and returns
		// TRUE, putting the old control word value in the passback
		// location pointed to by pwOldCW.
		{
			uword wTemp, wSave;
 
			__asm fstcw wSave
			if (wSave & 0x300 ||            // Not single mode
				0x3f != (wSave & 0x3f) ||   // Exceptions enabled
				wSave & 0xC00)              // Not round to nearest mode
			{
				__asm
				{
					mov ax, wSave
					and ax, not 300h    ;; single mode
					or  ax, 3fh         ;; disable all exceptions
					and ax, not 0xC00   ;; round to nearest mode
					mov wTemp, ax
					fldcw   wTemp
				}
			}
		}
	}
*/
	//! This function computes the slowest possible floating-point value (you can also directly use FLT_EPSILON)
	inline_ float ComputeFloatEpsilon()
	{
		float f = 1.0f;
		((udword&)f)^=1;
		return f - 1.0f;	// You can check it's the same as FLT_EPSILON
	}

	inline_ bool IsFloatZero(float x, float epsilon=1e-6f)
	{
		return x*x < epsilon;
	}

	#define FCOMI_ST0	_asm	_emit	0xdb	_asm	_emit	0xf0
	#define FCOMIP_ST0	_asm	_emit	0xdf	_asm	_emit	0xf0
	#define FCMOVB_ST0	_asm	_emit	0xda	_asm	_emit	0xc0
	#define FCMOVNB_ST0	_asm	_emit	0xdb	_asm	_emit	0xc0

	#define FCOMI_ST1	_asm	_emit	0xdb	_asm	_emit	0xf1
	#define FCOMIP_ST1	_asm	_emit	0xdf	_asm	_emit	0xf1
	#define FCMOVB_ST1	_asm	_emit	0xda	_asm	_emit	0xc1
	#define FCMOVNB_ST1	_asm	_emit	0xdb	_asm	_emit	0xc1

	#define FCOMI_ST2	_asm	_emit	0xdb	_asm	_emit	0xf2
	#define FCOMIP_ST2	_asm	_emit	0xdf	_asm	_emit	0xf2
	#define FCMOVB_ST2	_asm	_emit	0xda	_asm	_emit	0xc2
	#define FCMOVNB_ST2	_asm	_emit	0xdb	_asm	_emit	0xc2

	#define FCOMI_ST3	_asm	_emit	0xdb	_asm	_emit	0xf3
	#define FCOMIP_ST3	_asm	_emit	0xdf	_asm	_emit	0xf3
	#define FCMOVB_ST3	_asm	_emit	0xda	_asm	_emit	0xc3
	#define FCMOVNB_ST3	_asm	_emit	0xdb	_asm	_emit	0xc3

	#define FCOMI_ST4	_asm	_emit	0xdb	_asm	_emit	0xf4
	#define FCOMIP_ST4	_asm	_emit	0xdf	_asm	_emit	0xf4
	#define FCMOVB_ST4	_asm	_emit	0xda	_asm	_emit	0xc4
	#define FCMOVNB_ST4	_asm	_emit	0xdb	_asm	_emit	0xc4

	#define FCOMI_ST5	_asm	_emit	0xdb	_asm	_emit	0xf5
	#define FCOMIP_ST5	_asm	_emit	0xdf	_asm	_emit	0xf5
	#define FCMOVB_ST5	_asm	_emit	0xda	_asm	_emit	0xc5
	#define FCMOVNB_ST5	_asm	_emit	0xdb	_asm	_emit	0xc5

	#define FCOMI_ST6	_asm	_emit	0xdb	_asm	_emit	0xf6
	#define FCOMIP_ST6	_asm	_emit	0xdf	_asm	_emit	0xf6
	#define FCMOVB_ST6	_asm	_emit	0xda	_asm	_emit	0xc6
	#define FCMOVNB_ST6	_asm	_emit	0xdb	_asm	_emit	0xc6

	#define FCOMI_ST7	_asm	_emit	0xdb	_asm	_emit	0xf7
	#define FCOMIP_ST7	_asm	_emit	0xdf	_asm	_emit	0xf7
	#define FCMOVB_ST7	_asm	_emit	0xda	_asm	_emit	0xc7
	#define FCMOVNB_ST7	_asm	_emit	0xdb	_asm	_emit	0xc7

	//! A global function to find MAX(a,b) using FCOMI/FCMOV
	inline_ float FCMax2(float a, float b)
	{
		float Res;
		_asm	fld		[a]
		_asm	fld		[b]
		FCOMI_ST1
		FCMOVB_ST1
		_asm	fstp	[Res]
		_asm	fcomp
		return Res;
	}

	//! A global function to find MIN(a,b) using FCOMI/FCMOV
	inline_ float FCMin2(float a, float b)
	{
		float Res;
		_asm	fld		[a]
		_asm	fld		[b]
		FCOMI_ST1
		FCMOVNB_ST1
		_asm	fstp	[Res]
		_asm	fcomp
		return Res;
	}

	//! A global function to find MAX(a,b,c) using FCOMI/FCMOV
	inline_ float FCMax3(float a, float b, float c)
	{
		float Res;
		_asm	fld		[a]
		_asm	fld		[b]
		_asm	fld		[c]
		FCOMI_ST1
		FCMOVB_ST1
		FCOMI_ST2
		FCMOVB_ST2
		_asm	fstp	[Res]
		_asm	fcompp
		return Res;
	}

	//! A global function to find MIN(a,b,c) using FCOMI/FCMOV
	inline_ float FCMin3(float a, float b, float c)
	{
		float Res;
		_asm	fld		[a]
		_asm	fld		[b]
		_asm	fld		[c]
		FCOMI_ST1
		FCMOVNB_ST1
		FCOMI_ST2
		FCMOVNB_ST2
		_asm	fstp	[Res]
		_asm	fcompp
		return Res;
	}

	//! A global function to find MAX(a,b,c,d) using FCOMI/FCMOV
	inline_ float FCMax4(float a, float b, float c, float d)
	{
		float Res;
		_asm	fld		[a]
		_asm	fld		[b]
		_asm	fld		[c]
		_asm	fld		[d]
		FCOMI_ST1
		FCMOVB_ST1
		FCOMI_ST2
		FCMOVB_ST2
		FCOMI_ST3
		FCMOVB_ST3
		_asm	fstp	[Res]
		_asm	fcompp
		_asm	fcomp
		return Res;
	}

	//! A global function to find MIN(a,b,c,d) using FCOMI/FCMOV
	inline_ float FCMin4(float a, float b, float c, float d)
	{
		float Res;
		_asm	fld		[a]
		_asm	fld		[b]
		_asm	fld		[c]
		_asm	fld		[d]
		FCOMI_ST1
		FCMOVNB_ST1
		FCOMI_ST2
		FCMOVNB_ST2
		FCOMI_ST3
		FCMOVNB_ST3
		_asm	fstp	[Res]
		_asm	fcompp
		_asm	fcomp
		return Res;
	}

	inline_ int ConvertToSortable(float f)
	{
		int& Fi = (int&)f;
		int Fmask = (Fi>>31);
		Fi ^= Fmask;
		Fmask &= ~(1<<31);
		Fi -= Fmask;
		return Fi;
	}

	inline_ udword EncodeFloat(const float val)
	{
		// We may need to check on -0 and 0
		// But it should make no practical difference.
		udword ir = IR(val);

		if(ir & 0x80000000) //negative?
			ir = ~ir;//reverse sequence of negative numbers
		else
			ir |= 0x80000000; // flip sign

		return ir;
	}

	inline_ float DecodeFloat(udword ir)
	{
		udword rv;

		if(ir & 0x80000000) //positive?
			rv = ir & ~0x80000000; //flip sign
		else
			rv = ~ir; //undo reversal

		return FR(rv);
	}

	enum FPUMode
	{
		FPU_FLOOR		= 0,
		FPU_CEIL		= 1,
		FPU_BEST		= 2,

		FPU_FORCE_DWORD	= 0x7fffffff
	};

	FUNCTION ICECORE_API FPUMode	GetFPUMode();
	FUNCTION ICECORE_API void		SaveFPU();
	FUNCTION ICECORE_API void		RestoreFPU();
	FUNCTION ICECORE_API void		SetFPUFloorMode();
	FUNCTION ICECORE_API void		SetFPUCeilMode();
	FUNCTION ICECORE_API void		SetFPUBestMode();

	FUNCTION ICECORE_API void		SetFPUPrecision24();
	FUNCTION ICECORE_API void		SetFPUPrecision53();
	FUNCTION ICECORE_API void		SetFPUPrecision64();
	FUNCTION ICECORE_API void		SetFPURoundingChop();
	FUNCTION ICECORE_API void		SetFPURoundingUp();
	FUNCTION ICECORE_API void		SetFPURoundingDown();
	FUNCTION ICECORE_API void		SetFPURoundingNear();

	FUNCTION ICECORE_API int		intChop(const float& f);
	FUNCTION ICECORE_API int		intFloor(const float& f);
	FUNCTION ICECORE_API int		intCeil(const float& f);

	inline_ sdword MyFloor(float f)
	{
		return (sdword)f - (IR(f)>>31);
	}

	class ICECORE_API FPUGuard
	{
		public:
				FPUGuard();
				~FPUGuard();
		private:
		uword	mControlWord;
	};

#endif // ICEFPU_H
