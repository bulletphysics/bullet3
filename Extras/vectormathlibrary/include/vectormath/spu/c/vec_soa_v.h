/*
   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
   All rights reserved.

   Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Sony Computer Entertainment Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _VECTORMATH_VEC_SOA_V_C_H
#define _VECTORMATH_VEC_SOA_V_C_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Constants
 * for shuffles, words are labeled [x,y,z,w] [a,b,c,d]
 */
#define _VECTORMATH_SHUF_X 0x00010203
#define _VECTORMATH_SHUF_Y 0x04050607
#define _VECTORMATH_SHUF_Z 0x08090a0b
#define _VECTORMATH_SHUF_W 0x0c0d0e0f
#define _VECTORMATH_SHUF_A 0x10111213
#define _VECTORMATH_SHUF_B 0x14151617
#define _VECTORMATH_SHUF_C 0x18191a1b
#define _VECTORMATH_SHUF_D 0x1c1d1e1f
#define _VECTORMATH_SHUF_0 0x80808080
#define _VECTORMATH_SHUF_XAYB ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_B })
#define _VECTORMATH_SHUF_ZCWD ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_D })
#define _VECTORMATH_SHUF_ZBW0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_XCY0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_ZDW0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_D, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_XAZC ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_C })
#define _VECTORMATH_SHUF_ZDXB ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_D, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_B })
#define _VECTORMATH_SHUF_YBWD ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_D })
#define _VECTORMATH_SHUF_XDZB ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_D, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_B })
#define _VECTORMATH_SHUF_YAWC ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_C })
#define _VECTORMATH_SHUF_ZBXD ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_D })
#define _VECTORMATH_SHUF_XYCD ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_D })
#define _VECTORMATH_SLERP_TOL 0.999f

/*-----------------------------------------------------------------------------
 * Definitions
 */
#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

static inline VmathSoaVector3 vmathSoaV3MakeFromElems_V( vec_float4 _x, vec_float4 _y, vec_float4 _z )
{
    VmathSoaVector3 result;
    vmathSoaV3MakeFromElems(&result, _x, _y, _z);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MakeFromP3_V( VmathSoaPoint3 pnt )
{
    VmathSoaVector3 result;
    vmathSoaV3MakeFromP3(&result, &pnt);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MakeFromScalar_V( vec_float4 scalar )
{
    VmathSoaVector3 result;
    vmathSoaV3MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MakeFromAos_V( VmathVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaV3MakeFromAos(&result, &vec);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MakeFrom4Aos_V( VmathVector3 vec0, VmathVector3 vec1, VmathVector3 vec2, VmathVector3 vec3 )
{
    VmathSoaVector3 result;
    vmathSoaV3MakeFrom4Aos(&result, &vec0, &vec1, &vec2, &vec3);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MakeXAxis_V( )
{
    VmathSoaVector3 result;
    vmathSoaV3MakeXAxis(&result);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MakeYAxis_V( )
{
    VmathSoaVector3 result;
    vmathSoaV3MakeYAxis(&result);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MakeZAxis_V( )
{
    VmathSoaVector3 result;
    vmathSoaV3MakeZAxis(&result);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3Lerp_V( vec_float4 t, VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3Lerp(&result, t, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3Slerp_V( vec_float4 t, VmathSoaVector3 unitVec0, VmathSoaVector3 unitVec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3Slerp(&result, t, &unitVec0, &unitVec1);
    return result;
}

static inline void vmathSoaV3Get4Aos_V( VmathSoaVector3 vec, VmathVector3 *result0, VmathVector3 *result1, VmathVector3 *result2, VmathVector3 *result3 )
{
    vmathSoaV3Get4Aos(&vec, result0, result1, result2, result3);
}

static inline void vmathSoaV3LoadXYZArray_V( VmathSoaVector3 *vec, const vec_float4 *threeQuads )
{
    vmathSoaV3LoadXYZArray(vec, threeQuads);
}

static inline void vmathSoaV3StoreXYZArray_V( VmathSoaVector3 vec, vec_float4 *threeQuads )
{
    vmathSoaV3StoreXYZArray(&vec, threeQuads);
}

static inline void vmathSoaV3StoreHalfFloats_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1, vec_ushort8 *threeQuads )
{
    vmathSoaV3StoreHalfFloats(&vec0, &vec1, threeQuads);
}

static inline void vmathSoaV3SetX_V( VmathSoaVector3 *result, vec_float4 _x )
{
    vmathSoaV3SetX(result, _x);
}

static inline vec_float4 vmathSoaV3GetX_V( VmathSoaVector3 vec )
{
    return vmathSoaV3GetX(&vec);
}

static inline void vmathSoaV3SetY_V( VmathSoaVector3 *result, vec_float4 _y )
{
    vmathSoaV3SetY(result, _y);
}

static inline vec_float4 vmathSoaV3GetY_V( VmathSoaVector3 vec )
{
    return vmathSoaV3GetY(&vec);
}

static inline void vmathSoaV3SetZ_V( VmathSoaVector3 *result, vec_float4 _z )
{
    vmathSoaV3SetZ(result, _z);
}

static inline vec_float4 vmathSoaV3GetZ_V( VmathSoaVector3 vec )
{
    return vmathSoaV3GetZ(&vec);
}

static inline void vmathSoaV3SetElem_V( VmathSoaVector3 *result, int idx, vec_float4 value )
{
    vmathSoaV3SetElem(result, idx, value);
}

static inline vec_float4 vmathSoaV3GetElem_V( VmathSoaVector3 vec, int idx )
{
    return vmathSoaV3GetElem(&vec, idx);
}

static inline VmathSoaVector3 vmathSoaV3Add_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3Add(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3Sub_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3Sub(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaPoint3 vmathSoaV3AddP3_V( VmathSoaVector3 vec, VmathSoaPoint3 pnt1 )
{
    VmathSoaPoint3 result;
    vmathSoaV3AddP3(&result, &vec, &pnt1);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3ScalarMul_V( VmathSoaVector3 vec, vec_float4 scalar )
{
    VmathSoaVector3 result;
    vmathSoaV3ScalarMul(&result, &vec, scalar);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3ScalarDiv_V( VmathSoaVector3 vec, vec_float4 scalar )
{
    VmathSoaVector3 result;
    vmathSoaV3ScalarDiv(&result, &vec, scalar);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3Neg_V( VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaV3Neg(&result, &vec);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MulPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3MulPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3DivPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3DivPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3RecipPerElem_V( VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaV3RecipPerElem(&result, &vec);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3SqrtPerElem_V( VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaV3SqrtPerElem(&result, &vec);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3RsqrtPerElem_V( VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaV3RsqrtPerElem(&result, &vec);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3AbsPerElem_V( VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaV3AbsPerElem(&result, &vec);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3CopySignPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3CopySignPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3MaxPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3MaxPerElem(&result, &vec0, &vec1);
    return result;
}

static inline vec_float4 vmathSoaV3MaxElem_V( VmathSoaVector3 vec )
{
    return vmathSoaV3MaxElem(&vec);
}

static inline VmathSoaVector3 vmathSoaV3MinPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3MinPerElem(&result, &vec0, &vec1);
    return result;
}

static inline vec_float4 vmathSoaV3MinElem_V( VmathSoaVector3 vec )
{
    return vmathSoaV3MinElem(&vec);
}

static inline vec_float4 vmathSoaV3Sum_V( VmathSoaVector3 vec )
{
    return vmathSoaV3Sum(&vec);
}

static inline vec_float4 vmathSoaV3Dot_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    return vmathSoaV3Dot(&vec0, &vec1);
}

static inline vec_float4 vmathSoaV3LengthSqr_V( VmathSoaVector3 vec )
{
    return vmathSoaV3LengthSqr(&vec);
}

static inline vec_float4 vmathSoaV3Length_V( VmathSoaVector3 vec )
{
    return vmathSoaV3Length(&vec);
}

static inline VmathSoaVector3 vmathSoaV3Normalize_V( VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaV3Normalize(&result, &vec);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3Cross_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 )
{
    VmathSoaVector3 result;
    vmathSoaV3Cross(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3Select_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1, vec_uint4 select1 )
{
    VmathSoaVector3 result;
    vmathSoaV3Select(&result, &vec0, &vec1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaV3Print_V( VmathSoaVector3 vec )
{
    vmathSoaV3Print(&vec);
}

static inline void vmathSoaV3Prints_V( VmathSoaVector3 vec, const char *name )
{
    vmathSoaV3Prints(&vec, name);
}

#endif

static inline VmathSoaVector4 vmathSoaV4MakeFromElems_V( vec_float4 _x, vec_float4 _y, vec_float4 _z, vec_float4 _w )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeFromElems(&result, _x, _y, _z, _w);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeFromV3Scalar_V( VmathSoaVector3 xyz, vec_float4 _w )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeFromV3Scalar(&result, &xyz, _w);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeFromV3_V( VmathSoaVector3 vec )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeFromV3(&result, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeFromP3_V( VmathSoaPoint3 pnt )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeFromP3(&result, &pnt);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeFromQ_V( VmathSoaQuat quat )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeFromQ(&result, &quat);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeFromScalar_V( vec_float4 scalar )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeFromAos_V( VmathVector4 vec )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeFromAos(&result, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeFrom4Aos_V( VmathVector4 vec0, VmathVector4 vec1, VmathVector4 vec2, VmathVector4 vec3 )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeFrom4Aos(&result, &vec0, &vec1, &vec2, &vec3);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeXAxis_V( )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeXAxis(&result);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeYAxis_V( )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeYAxis(&result);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeZAxis_V( )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeZAxis(&result);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MakeWAxis_V( )
{
    VmathSoaVector4 result;
    vmathSoaV4MakeWAxis(&result);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4Lerp_V( vec_float4 t, VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4Lerp(&result, t, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4Slerp_V( vec_float4 t, VmathSoaVector4 unitVec0, VmathSoaVector4 unitVec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4Slerp(&result, t, &unitVec0, &unitVec1);
    return result;
}

static inline void vmathSoaV4Get4Aos_V( VmathSoaVector4 vec, VmathVector4 *result0, VmathVector4 *result1, VmathVector4 *result2, VmathVector4 *result3 )
{
    vmathSoaV4Get4Aos(&vec, result0, result1, result2, result3);
}

static inline void vmathSoaV4StoreHalfFloats_V( VmathSoaVector4 vec, vec_ushort8 *twoQuads )
{
    vmathSoaV4StoreHalfFloats(&vec, twoQuads);
}

static inline void vmathSoaV4SetXYZ_V( VmathSoaVector4 *result, VmathSoaVector3 vec )
{
    vmathSoaV4SetXYZ(result, &vec);
}

static inline VmathSoaVector3 vmathSoaV4GetXYZ_V( VmathSoaVector4 vec )
{
    VmathSoaVector3 result;
    vmathSoaV4GetXYZ(&result, &vec);
    return result;
}

static inline void vmathSoaV4SetX_V( VmathSoaVector4 *result, vec_float4 _x )
{
    vmathSoaV4SetX(result, _x);
}

static inline vec_float4 vmathSoaV4GetX_V( VmathSoaVector4 vec )
{
    return vmathSoaV4GetX(&vec);
}

static inline void vmathSoaV4SetY_V( VmathSoaVector4 *result, vec_float4 _y )
{
    vmathSoaV4SetY(result, _y);
}

static inline vec_float4 vmathSoaV4GetY_V( VmathSoaVector4 vec )
{
    return vmathSoaV4GetY(&vec);
}

static inline void vmathSoaV4SetZ_V( VmathSoaVector4 *result, vec_float4 _z )
{
    vmathSoaV4SetZ(result, _z);
}

static inline vec_float4 vmathSoaV4GetZ_V( VmathSoaVector4 vec )
{
    return vmathSoaV4GetZ(&vec);
}

static inline void vmathSoaV4SetW_V( VmathSoaVector4 *result, vec_float4 _w )
{
    vmathSoaV4SetW(result, _w);
}

static inline vec_float4 vmathSoaV4GetW_V( VmathSoaVector4 vec )
{
    return vmathSoaV4GetW(&vec);
}

static inline void vmathSoaV4SetElem_V( VmathSoaVector4 *result, int idx, vec_float4 value )
{
    vmathSoaV4SetElem(result, idx, value);
}

static inline vec_float4 vmathSoaV4GetElem_V( VmathSoaVector4 vec, int idx )
{
    return vmathSoaV4GetElem(&vec, idx);
}

static inline VmathSoaVector4 vmathSoaV4Add_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4Add(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4Sub_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4Sub(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4ScalarMul_V( VmathSoaVector4 vec, vec_float4 scalar )
{
    VmathSoaVector4 result;
    vmathSoaV4ScalarMul(&result, &vec, scalar);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4ScalarDiv_V( VmathSoaVector4 vec, vec_float4 scalar )
{
    VmathSoaVector4 result;
    vmathSoaV4ScalarDiv(&result, &vec, scalar);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4Neg_V( VmathSoaVector4 vec )
{
    VmathSoaVector4 result;
    vmathSoaV4Neg(&result, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MulPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4MulPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4DivPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4DivPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4RecipPerElem_V( VmathSoaVector4 vec )
{
    VmathSoaVector4 result;
    vmathSoaV4RecipPerElem(&result, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4SqrtPerElem_V( VmathSoaVector4 vec )
{
    VmathSoaVector4 result;
    vmathSoaV4SqrtPerElem(&result, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4RsqrtPerElem_V( VmathSoaVector4 vec )
{
    VmathSoaVector4 result;
    vmathSoaV4RsqrtPerElem(&result, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4AbsPerElem_V( VmathSoaVector4 vec )
{
    VmathSoaVector4 result;
    vmathSoaV4AbsPerElem(&result, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4CopySignPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4CopySignPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4MaxPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4MaxPerElem(&result, &vec0, &vec1);
    return result;
}

static inline vec_float4 vmathSoaV4MaxElem_V( VmathSoaVector4 vec )
{
    return vmathSoaV4MaxElem(&vec);
}

static inline VmathSoaVector4 vmathSoaV4MinPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    VmathSoaVector4 result;
    vmathSoaV4MinPerElem(&result, &vec0, &vec1);
    return result;
}

static inline vec_float4 vmathSoaV4MinElem_V( VmathSoaVector4 vec )
{
    return vmathSoaV4MinElem(&vec);
}

static inline vec_float4 vmathSoaV4Sum_V( VmathSoaVector4 vec )
{
    return vmathSoaV4Sum(&vec);
}

static inline vec_float4 vmathSoaV4Dot_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 )
{
    return vmathSoaV4Dot(&vec0, &vec1);
}

static inline vec_float4 vmathSoaV4LengthSqr_V( VmathSoaVector4 vec )
{
    return vmathSoaV4LengthSqr(&vec);
}

static inline vec_float4 vmathSoaV4Length_V( VmathSoaVector4 vec )
{
    return vmathSoaV4Length(&vec);
}

static inline VmathSoaVector4 vmathSoaV4Normalize_V( VmathSoaVector4 vec )
{
    VmathSoaVector4 result;
    vmathSoaV4Normalize(&result, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaV4Select_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1, vec_uint4 select1 )
{
    VmathSoaVector4 result;
    vmathSoaV4Select(&result, &vec0, &vec1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaV4Print_V( VmathSoaVector4 vec )
{
    vmathSoaV4Print(&vec);
}

static inline void vmathSoaV4Prints_V( VmathSoaVector4 vec, const char *name )
{
    vmathSoaV4Prints(&vec, name);
}

#endif

static inline VmathSoaPoint3 vmathSoaP3MakeFromElems_V( vec_float4 _x, vec_float4 _y, vec_float4 _z )
{
    VmathSoaPoint3 result;
    vmathSoaP3MakeFromElems(&result, _x, _y, _z);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3MakeFromV3_V( VmathSoaVector3 vec )
{
    VmathSoaPoint3 result;
    vmathSoaP3MakeFromV3(&result, &vec);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3MakeFromScalar_V( vec_float4 scalar )
{
    VmathSoaPoint3 result;
    vmathSoaP3MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3MakeFromAos_V( VmathPoint3 pnt )
{
    VmathSoaPoint3 result;
    vmathSoaP3MakeFromAos(&result, &pnt);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3MakeFrom4Aos_V( VmathPoint3 pnt0, VmathPoint3 pnt1, VmathPoint3 pnt2, VmathPoint3 pnt3 )
{
    VmathSoaPoint3 result;
    vmathSoaP3MakeFrom4Aos(&result, &pnt0, &pnt1, &pnt2, &pnt3);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3Lerp_V( vec_float4 t, VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3Lerp(&result, t, &pnt0, &pnt1);
    return result;
}

static inline void vmathSoaP3Get4Aos_V( VmathSoaPoint3 pnt, VmathPoint3 *result0, VmathPoint3 *result1, VmathPoint3 *result2, VmathPoint3 *result3 )
{
    vmathSoaP3Get4Aos(&pnt, result0, result1, result2, result3);
}

static inline void vmathSoaP3LoadXYZArray_V( VmathSoaPoint3 *vec, const vec_float4 *threeQuads )
{
    vmathSoaP3LoadXYZArray(vec, threeQuads);
}

static inline void vmathSoaP3StoreXYZArray_V( VmathSoaPoint3 vec, vec_float4 *threeQuads )
{
    vmathSoaP3StoreXYZArray(&vec, threeQuads);
}

static inline void vmathSoaP3StoreHalfFloats_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1, vec_ushort8 *threeQuads )
{
    vmathSoaP3StoreHalfFloats(&pnt0, &pnt1, threeQuads);
}

static inline void vmathSoaP3SetX_V( VmathSoaPoint3 *result, vec_float4 _x )
{
    vmathSoaP3SetX(result, _x);
}

static inline vec_float4 vmathSoaP3GetX_V( VmathSoaPoint3 pnt )
{
    return vmathSoaP3GetX(&pnt);
}

static inline void vmathSoaP3SetY_V( VmathSoaPoint3 *result, vec_float4 _y )
{
    vmathSoaP3SetY(result, _y);
}

static inline vec_float4 vmathSoaP3GetY_V( VmathSoaPoint3 pnt )
{
    return vmathSoaP3GetY(&pnt);
}

static inline void vmathSoaP3SetZ_V( VmathSoaPoint3 *result, vec_float4 _z )
{
    vmathSoaP3SetZ(result, _z);
}

static inline vec_float4 vmathSoaP3GetZ_V( VmathSoaPoint3 pnt )
{
    return vmathSoaP3GetZ(&pnt);
}

static inline void vmathSoaP3SetElem_V( VmathSoaPoint3 *result, int idx, vec_float4 value )
{
    vmathSoaP3SetElem(result, idx, value);
}

static inline vec_float4 vmathSoaP3GetElem_V( VmathSoaPoint3 pnt, int idx )
{
    return vmathSoaP3GetElem(&pnt, idx);
}

static inline VmathSoaVector3 vmathSoaP3Sub_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    VmathSoaVector3 result;
    vmathSoaP3Sub(&result, &pnt0, &pnt1);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3AddV3_V( VmathSoaPoint3 pnt, VmathSoaVector3 vec1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3AddV3(&result, &pnt, &vec1);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3SubV3_V( VmathSoaPoint3 pnt, VmathSoaVector3 vec1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3SubV3(&result, &pnt, &vec1);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3MulPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3MulPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3DivPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3DivPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3RecipPerElem_V( VmathSoaPoint3 pnt )
{
    VmathSoaPoint3 result;
    vmathSoaP3RecipPerElem(&result, &pnt);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3SqrtPerElem_V( VmathSoaPoint3 pnt )
{
    VmathSoaPoint3 result;
    vmathSoaP3SqrtPerElem(&result, &pnt);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3RsqrtPerElem_V( VmathSoaPoint3 pnt )
{
    VmathSoaPoint3 result;
    vmathSoaP3RsqrtPerElem(&result, &pnt);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3AbsPerElem_V( VmathSoaPoint3 pnt )
{
    VmathSoaPoint3 result;
    vmathSoaP3AbsPerElem(&result, &pnt);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3CopySignPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3CopySignPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3MaxPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3MaxPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline vec_float4 vmathSoaP3MaxElem_V( VmathSoaPoint3 pnt )
{
    return vmathSoaP3MaxElem(&pnt);
}

static inline VmathSoaPoint3 vmathSoaP3MinPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3MinPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline vec_float4 vmathSoaP3MinElem_V( VmathSoaPoint3 pnt )
{
    return vmathSoaP3MinElem(&pnt);
}

static inline vec_float4 vmathSoaP3Sum_V( VmathSoaPoint3 pnt )
{
    return vmathSoaP3Sum(&pnt);
}

static inline VmathSoaPoint3 vmathSoaP3Scale_V( VmathSoaPoint3 pnt, vec_float4 scaleVal )
{
    VmathSoaPoint3 result;
    vmathSoaP3Scale(&result, &pnt, scaleVal);
    return result;
}

static inline VmathSoaPoint3 vmathSoaP3NonUniformScale_V( VmathSoaPoint3 pnt, VmathSoaVector3 scaleVec )
{
    VmathSoaPoint3 result;
    vmathSoaP3NonUniformScale(&result, &pnt, &scaleVec);
    return result;
}

static inline vec_float4 vmathSoaP3Projection_V( VmathSoaPoint3 pnt, VmathSoaVector3 unitVec )
{
    return vmathSoaP3Projection(&pnt, &unitVec);
}

static inline vec_float4 vmathSoaP3DistSqrFromOrigin_V( VmathSoaPoint3 pnt )
{
    return vmathSoaP3DistSqrFromOrigin(&pnt);
}

static inline vec_float4 vmathSoaP3DistFromOrigin_V( VmathSoaPoint3 pnt )
{
    return vmathSoaP3DistFromOrigin(&pnt);
}

static inline vec_float4 vmathSoaP3DistSqr_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    return vmathSoaP3DistSqr(&pnt0, &pnt1);
}

static inline vec_float4 vmathSoaP3Dist_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 )
{
    return vmathSoaP3Dist(&pnt0, &pnt1);
}

static inline VmathSoaPoint3 vmathSoaP3Select_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1, vec_uint4 select1 )
{
    VmathSoaPoint3 result;
    vmathSoaP3Select(&result, &pnt0, &pnt1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaP3Print_V( VmathSoaPoint3 pnt )
{
    vmathSoaP3Print(&pnt);
}

static inline void vmathSoaP3Prints_V( VmathSoaPoint3 pnt, const char *name )
{
    vmathSoaP3Prints(&pnt, name);
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
