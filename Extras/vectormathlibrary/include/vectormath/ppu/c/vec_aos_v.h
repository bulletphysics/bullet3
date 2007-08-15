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

#ifndef _VECTORMATH_VEC_AOS_V_C_H
#define _VECTORMATH_VEC_AOS_V_C_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Constants
 * for permutes words are labeled [x,y,z,w] [a,b,c,d]
 */
#define _VECTORMATH_PERM_X 0x00010203
#define _VECTORMATH_PERM_Y 0x04050607
#define _VECTORMATH_PERM_Z 0x08090a0b
#define _VECTORMATH_PERM_W 0x0c0d0e0f
#define _VECTORMATH_PERM_A 0x10111213
#define _VECTORMATH_PERM_B 0x14151617
#define _VECTORMATH_PERM_C 0x18191a1b
#define _VECTORMATH_PERM_D 0x1c1d1e1f
#define _VECTORMATH_PERM_XYZA (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_A }
#define _VECTORMATH_PERM_ZXYW (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_W }
#define _VECTORMATH_PERM_YZXW (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_X, _VECTORMATH_PERM_W }
#define _VECTORMATH_PERM_YZAB (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_A, _VECTORMATH_PERM_B }
#define _VECTORMATH_PERM_ZABC (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_A, _VECTORMATH_PERM_B, _VECTORMATH_PERM_C }
#define _VECTORMATH_PERM_XYAW (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_A, _VECTORMATH_PERM_W }
#define _VECTORMATH_PERM_XAZW (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_A, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_W }
#define _VECTORMATH_MASK_0xF000 (vec_uint4){ 0xffffffff, 0, 0, 0 }
#define _VECTORMATH_MASK_0x0F00 (vec_uint4){ 0, 0xffffffff, 0, 0 }
#define _VECTORMATH_MASK_0x00F0 (vec_uint4){ 0, 0, 0xffffffff, 0 }
#define _VECTORMATH_MASK_0x000F (vec_uint4){ 0, 0, 0, 0xffffffff }
#define _VECTORMATH_UNIT_1000 (vec_float4){ 1.0f, 0.0f, 0.0f, 0.0f }
#define _VECTORMATH_UNIT_0100 (vec_float4){ 0.0f, 1.0f, 0.0f, 0.0f }
#define _VECTORMATH_UNIT_0010 (vec_float4){ 0.0f, 0.0f, 1.0f, 0.0f }
#define _VECTORMATH_UNIT_0001 (vec_float4){ 0.0f, 0.0f, 0.0f, 1.0f }
#define _VECTORMATH_SLERP_TOL 0.999f

/*-----------------------------------------------------------------------------
 * Definitions
 */
#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

static inline VmathVector3 vmathV3MakeFromElems_V( float _x, float _y, float _z )
{
    VmathVector3 result;
    vmathV3MakeFromElems(&result, _x, _y, _z);
    return result;
}

static inline VmathVector3 vmathV3MakeFromP3_V( VmathPoint3 pnt )
{
    VmathVector3 result;
    vmathV3MakeFromP3(&result, &pnt);
    return result;
}

static inline VmathVector3 vmathV3MakeFromScalar_V( float scalar )
{
    VmathVector3 result;
    vmathV3MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathVector3 vmathV3MakeFrom128_V( vec_float4 vf4 )
{
    VmathVector3 result;
    vmathV3MakeFrom128(&result, vf4);
    return result;
}

static inline VmathVector3 vmathV3MakeXAxis_V( )
{
    VmathVector3 result;
    vmathV3MakeXAxis(&result);
    return result;
}

static inline VmathVector3 vmathV3MakeYAxis_V( )
{
    VmathVector3 result;
    vmathV3MakeYAxis(&result);
    return result;
}

static inline VmathVector3 vmathV3MakeZAxis_V( )
{
    VmathVector3 result;
    vmathV3MakeZAxis(&result);
    return result;
}

static inline VmathVector3 vmathV3Lerp_V( float t, VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3Lerp(&result, t, &vec0, &vec1);
    return result;
}

static inline VmathVector3 vmathV3Slerp_V( float t, VmathVector3 unitVec0, VmathVector3 unitVec1 )
{
    VmathVector3 result;
    vmathV3Slerp(&result, t, &unitVec0, &unitVec1);
    return result;
}

static inline vec_float4 vmathV3Get128_V( VmathVector3 vec )
{
    return vmathV3Get128(&vec);
}

static inline void vmathV3StoreXYZ_V( VmathVector3 vec, vec_float4 *quad )
{
    vmathV3StoreXYZ(&vec, quad);
}

static inline void vmathV3LoadXYZArray_V( VmathVector3 *vec0, VmathVector3 *vec1, VmathVector3 *vec2, VmathVector3 *vec3, const vec_float4 *threeQuads )
{
    vmathV3LoadXYZArray(vec0, vec1, vec2, vec3, threeQuads);
}

static inline void vmathV3StoreXYZArray_V( VmathVector3 vec0, VmathVector3 vec1, VmathVector3 vec2, VmathVector3 vec3, vec_float4 *threeQuads )
{
    vmathV3StoreXYZArray(&vec0, &vec1, &vec2, &vec3, threeQuads);
}

static inline void vmathV3StoreHalfFloats_V( VmathVector3 vec0, VmathVector3 vec1, VmathVector3 vec2, VmathVector3 vec3, VmathVector3 vec4, VmathVector3 vec5, VmathVector3 vec6, VmathVector3 vec7, vec_ushort8 *threeQuads )
{
    vmathV3StoreHalfFloats(&vec0, &vec1, &vec2, &vec3, &vec4, &vec5, &vec6, &vec7, threeQuads);
}

static inline void vmathV3SetX_V( VmathVector3 *result, float _x )
{
    vmathV3SetX(result, _x);
}

static inline float vmathV3GetX_V( VmathVector3 vec )
{
    return vmathV3GetX(&vec);
}

static inline void vmathV3SetY_V( VmathVector3 *result, float _y )
{
    vmathV3SetY(result, _y);
}

static inline float vmathV3GetY_V( VmathVector3 vec )
{
    return vmathV3GetY(&vec);
}

static inline void vmathV3SetZ_V( VmathVector3 *result, float _z )
{
    vmathV3SetZ(result, _z);
}

static inline float vmathV3GetZ_V( VmathVector3 vec )
{
    return vmathV3GetZ(&vec);
}

static inline void vmathV3SetElem_V( VmathVector3 *result, int idx, float value )
{
    vmathV3SetElem(result, idx, value);
}

static inline float vmathV3GetElem_V( VmathVector3 vec, int idx )
{
    return vmathV3GetElem(&vec, idx);
}

static inline VmathVector3 vmathV3Add_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3Add(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector3 vmathV3Sub_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3Sub(&result, &vec0, &vec1);
    return result;
}

static inline VmathPoint3 vmathV3AddP3_V( VmathVector3 vec, VmathPoint3 pnt1 )
{
    VmathPoint3 result;
    vmathV3AddP3(&result, &vec, &pnt1);
    return result;
}

static inline VmathVector3 vmathV3ScalarMul_V( VmathVector3 vec, float scalar )
{
    VmathVector3 result;
    vmathV3ScalarMul(&result, &vec, scalar);
    return result;
}

static inline VmathVector3 vmathV3ScalarDiv_V( VmathVector3 vec, float scalar )
{
    VmathVector3 result;
    vmathV3ScalarDiv(&result, &vec, scalar);
    return result;
}

static inline VmathVector3 vmathV3Neg_V( VmathVector3 vec )
{
    VmathVector3 result;
    vmathV3Neg(&result, &vec);
    return result;
}

static inline VmathVector3 vmathV3MulPerElem_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3MulPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector3 vmathV3DivPerElem_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3DivPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector3 vmathV3RecipPerElem_V( VmathVector3 vec )
{
    VmathVector3 result;
    vmathV3RecipPerElem(&result, &vec);
    return result;
}

static inline VmathVector3 vmathV3SqrtPerElem_V( VmathVector3 vec )
{
    VmathVector3 result;
    vmathV3SqrtPerElem(&result, &vec);
    return result;
}

static inline VmathVector3 vmathV3RsqrtPerElem_V( VmathVector3 vec )
{
    VmathVector3 result;
    vmathV3RsqrtPerElem(&result, &vec);
    return result;
}

static inline VmathVector3 vmathV3AbsPerElem_V( VmathVector3 vec )
{
    VmathVector3 result;
    vmathV3AbsPerElem(&result, &vec);
    return result;
}

static inline VmathVector3 vmathV3CopySignPerElem_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3CopySignPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector3 vmathV3MaxPerElem_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3MaxPerElem(&result, &vec0, &vec1);
    return result;
}

static inline float vmathV3MaxElem_V( VmathVector3 vec )
{
    return vmathV3MaxElem(&vec);
}

static inline VmathVector3 vmathV3MinPerElem_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3MinPerElem(&result, &vec0, &vec1);
    return result;
}

static inline float vmathV3MinElem_V( VmathVector3 vec )
{
    return vmathV3MinElem(&vec);
}

static inline float vmathV3Sum_V( VmathVector3 vec )
{
    return vmathV3Sum(&vec);
}

static inline float vmathV3Dot_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    return vmathV3Dot(&vec0, &vec1);
}

static inline float vmathV3LengthSqr_V( VmathVector3 vec )
{
    return vmathV3LengthSqr(&vec);
}

static inline float vmathV3Length_V( VmathVector3 vec )
{
    return vmathV3Length(&vec);
}

static inline VmathVector3 vmathV3Normalize_V( VmathVector3 vec )
{
    VmathVector3 result;
    vmathV3Normalize(&result, &vec);
    return result;
}

static inline VmathVector3 vmathV3Cross_V( VmathVector3 vec0, VmathVector3 vec1 )
{
    VmathVector3 result;
    vmathV3Cross(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector3 vmathV3Select_V( VmathVector3 vec0, VmathVector3 vec1, unsigned int select1 )
{
    VmathVector3 result;
    vmathV3Select(&result, &vec0, &vec1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathV3Print_V( VmathVector3 vec )
{
    vmathV3Print(&vec);
}

static inline void vmathV3Prints_V( VmathVector3 vec, const char *name )
{
    vmathV3Prints(&vec, name);
}

#endif

static inline VmathVector4 vmathV4MakeFromElems_V( float _x, float _y, float _z, float _w )
{
    VmathVector4 result;
    vmathV4MakeFromElems(&result, _x, _y, _z, _w);
    return result;
}

static inline VmathVector4 vmathV4MakeFromV3Scalar_V( VmathVector3 xyz, float _w )
{
    VmathVector4 result;
    vmathV4MakeFromV3Scalar(&result, &xyz, _w);
    return result;
}

static inline VmathVector4 vmathV4MakeFromV3_V( VmathVector3 vec )
{
    VmathVector4 result;
    vmathV4MakeFromV3(&result, &vec);
    return result;
}

static inline VmathVector4 vmathV4MakeFromP3_V( VmathPoint3 pnt )
{
    VmathVector4 result;
    vmathV4MakeFromP3(&result, &pnt);
    return result;
}

static inline VmathVector4 vmathV4MakeFromQ_V( VmathQuat quat )
{
    VmathVector4 result;
    vmathV4MakeFromQ(&result, &quat);
    return result;
}

static inline VmathVector4 vmathV4MakeFromScalar_V( float scalar )
{
    VmathVector4 result;
    vmathV4MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathVector4 vmathV4MakeFrom128_V( vec_float4 vf4 )
{
    VmathVector4 result;
    vmathV4MakeFrom128(&result, vf4);
    return result;
}

static inline VmathVector4 vmathV4MakeXAxis_V( )
{
    VmathVector4 result;
    vmathV4MakeXAxis(&result);
    return result;
}

static inline VmathVector4 vmathV4MakeYAxis_V( )
{
    VmathVector4 result;
    vmathV4MakeYAxis(&result);
    return result;
}

static inline VmathVector4 vmathV4MakeZAxis_V( )
{
    VmathVector4 result;
    vmathV4MakeZAxis(&result);
    return result;
}

static inline VmathVector4 vmathV4MakeWAxis_V( )
{
    VmathVector4 result;
    vmathV4MakeWAxis(&result);
    return result;
}

static inline VmathVector4 vmathV4Lerp_V( float t, VmathVector4 vec0, VmathVector4 vec1 )
{
    VmathVector4 result;
    vmathV4Lerp(&result, t, &vec0, &vec1);
    return result;
}

static inline VmathVector4 vmathV4Slerp_V( float t, VmathVector4 unitVec0, VmathVector4 unitVec1 )
{
    VmathVector4 result;
    vmathV4Slerp(&result, t, &unitVec0, &unitVec1);
    return result;
}

static inline vec_float4 vmathV4Get128_V( VmathVector4 vec )
{
    return vmathV4Get128(&vec);
}

static inline void vmathV4StoreHalfFloats_V( VmathVector4 vec0, VmathVector4 vec1, VmathVector4 vec2, VmathVector4 vec3, vec_ushort8 *twoQuads )
{
    vmathV4StoreHalfFloats(&vec0, &vec1, &vec2, &vec3, twoQuads);
}

static inline void vmathV4SetXYZ_V( VmathVector4 *result, VmathVector3 vec )
{
    vmathV4SetXYZ(result, &vec);
}

static inline VmathVector3 vmathV4GetXYZ_V( VmathVector4 vec )
{
    VmathVector3 result;
    vmathV4GetXYZ(&result, &vec);
    return result;
}

static inline void vmathV4SetX_V( VmathVector4 *result, float _x )
{
    vmathV4SetX(result, _x);
}

static inline float vmathV4GetX_V( VmathVector4 vec )
{
    return vmathV4GetX(&vec);
}

static inline void vmathV4SetY_V( VmathVector4 *result, float _y )
{
    vmathV4SetY(result, _y);
}

static inline float vmathV4GetY_V( VmathVector4 vec )
{
    return vmathV4GetY(&vec);
}

static inline void vmathV4SetZ_V( VmathVector4 *result, float _z )
{
    vmathV4SetZ(result, _z);
}

static inline float vmathV4GetZ_V( VmathVector4 vec )
{
    return vmathV4GetZ(&vec);
}

static inline void vmathV4SetW_V( VmathVector4 *result, float _w )
{
    vmathV4SetW(result, _w);
}

static inline float vmathV4GetW_V( VmathVector4 vec )
{
    return vmathV4GetW(&vec);
}

static inline void vmathV4SetElem_V( VmathVector4 *result, int idx, float value )
{
    vmathV4SetElem(result, idx, value);
}

static inline float vmathV4GetElem_V( VmathVector4 vec, int idx )
{
    return vmathV4GetElem(&vec, idx);
}

static inline VmathVector4 vmathV4Add_V( VmathVector4 vec0, VmathVector4 vec1 )
{
    VmathVector4 result;
    vmathV4Add(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector4 vmathV4Sub_V( VmathVector4 vec0, VmathVector4 vec1 )
{
    VmathVector4 result;
    vmathV4Sub(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector4 vmathV4ScalarMul_V( VmathVector4 vec, float scalar )
{
    VmathVector4 result;
    vmathV4ScalarMul(&result, &vec, scalar);
    return result;
}

static inline VmathVector4 vmathV4ScalarDiv_V( VmathVector4 vec, float scalar )
{
    VmathVector4 result;
    vmathV4ScalarDiv(&result, &vec, scalar);
    return result;
}

static inline VmathVector4 vmathV4Neg_V( VmathVector4 vec )
{
    VmathVector4 result;
    vmathV4Neg(&result, &vec);
    return result;
}

static inline VmathVector4 vmathV4MulPerElem_V( VmathVector4 vec0, VmathVector4 vec1 )
{
    VmathVector4 result;
    vmathV4MulPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector4 vmathV4DivPerElem_V( VmathVector4 vec0, VmathVector4 vec1 )
{
    VmathVector4 result;
    vmathV4DivPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector4 vmathV4RecipPerElem_V( VmathVector4 vec )
{
    VmathVector4 result;
    vmathV4RecipPerElem(&result, &vec);
    return result;
}

static inline VmathVector4 vmathV4SqrtPerElem_V( VmathVector4 vec )
{
    VmathVector4 result;
    vmathV4SqrtPerElem(&result, &vec);
    return result;
}

static inline VmathVector4 vmathV4RsqrtPerElem_V( VmathVector4 vec )
{
    VmathVector4 result;
    vmathV4RsqrtPerElem(&result, &vec);
    return result;
}

static inline VmathVector4 vmathV4AbsPerElem_V( VmathVector4 vec )
{
    VmathVector4 result;
    vmathV4AbsPerElem(&result, &vec);
    return result;
}

static inline VmathVector4 vmathV4CopySignPerElem_V( VmathVector4 vec0, VmathVector4 vec1 )
{
    VmathVector4 result;
    vmathV4CopySignPerElem(&result, &vec0, &vec1);
    return result;
}

static inline VmathVector4 vmathV4MaxPerElem_V( VmathVector4 vec0, VmathVector4 vec1 )
{
    VmathVector4 result;
    vmathV4MaxPerElem(&result, &vec0, &vec1);
    return result;
}

static inline float vmathV4MaxElem_V( VmathVector4 vec )
{
    return vmathV4MaxElem(&vec);
}

static inline VmathVector4 vmathV4MinPerElem_V( VmathVector4 vec0, VmathVector4 vec1 )
{
    VmathVector4 result;
    vmathV4MinPerElem(&result, &vec0, &vec1);
    return result;
}

static inline float vmathV4MinElem_V( VmathVector4 vec )
{
    return vmathV4MinElem(&vec);
}

static inline float vmathV4Sum_V( VmathVector4 vec )
{
    return vmathV4Sum(&vec);
}

static inline float vmathV4Dot_V( VmathVector4 vec0, VmathVector4 vec1 )
{
    return vmathV4Dot(&vec0, &vec1);
}

static inline float vmathV4LengthSqr_V( VmathVector4 vec )
{
    return vmathV4LengthSqr(&vec);
}

static inline float vmathV4Length_V( VmathVector4 vec )
{
    return vmathV4Length(&vec);
}

static inline VmathVector4 vmathV4Normalize_V( VmathVector4 vec )
{
    VmathVector4 result;
    vmathV4Normalize(&result, &vec);
    return result;
}

static inline VmathVector4 vmathV4Select_V( VmathVector4 vec0, VmathVector4 vec1, unsigned int select1 )
{
    VmathVector4 result;
    vmathV4Select(&result, &vec0, &vec1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathV4Print_V( VmathVector4 vec )
{
    vmathV4Print(&vec);
}

static inline void vmathV4Prints_V( VmathVector4 vec, const char *name )
{
    vmathV4Prints(&vec, name);
}

#endif

static inline VmathPoint3 vmathP3MakeFromElems_V( float _x, float _y, float _z )
{
    VmathPoint3 result;
    vmathP3MakeFromElems(&result, _x, _y, _z);
    return result;
}

static inline VmathPoint3 vmathP3MakeFromV3_V( VmathVector3 vec )
{
    VmathPoint3 result;
    vmathP3MakeFromV3(&result, &vec);
    return result;
}

static inline VmathPoint3 vmathP3MakeFromScalar_V( float scalar )
{
    VmathPoint3 result;
    vmathP3MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathPoint3 vmathP3MakeFrom128_V( vec_float4 vf4 )
{
    VmathPoint3 result;
    vmathP3MakeFrom128(&result, vf4);
    return result;
}

static inline VmathPoint3 vmathP3Lerp_V( float t, VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    VmathPoint3 result;
    vmathP3Lerp(&result, t, &pnt0, &pnt1);
    return result;
}

static inline vec_float4 vmathP3Get128_V( VmathPoint3 pnt )
{
    return vmathP3Get128(&pnt);
}

static inline void vmathP3StoreXYZ_V( VmathPoint3 pnt, vec_float4 *quad )
{
    vmathP3StoreXYZ(&pnt, quad);
}

static inline void vmathP3LoadXYZArray_V( VmathPoint3 *pnt0, VmathPoint3 *pnt1, VmathPoint3 *pnt2, VmathPoint3 *pnt3, const vec_float4 *threeQuads )
{
    vmathP3LoadXYZArray(pnt0, pnt1, pnt2, pnt3, threeQuads);
}

static inline void vmathP3StoreXYZArray_V( VmathPoint3 pnt0, VmathPoint3 pnt1, VmathPoint3 pnt2, VmathPoint3 pnt3, vec_float4 *threeQuads )
{
    vmathP3StoreXYZArray(&pnt0, &pnt1, &pnt2, &pnt3, threeQuads);
}

static inline void vmathP3StoreHalfFloats_V( VmathPoint3 pnt0, VmathPoint3 pnt1, VmathPoint3 pnt2, VmathPoint3 pnt3, VmathPoint3 pnt4, VmathPoint3 pnt5, VmathPoint3 pnt6, VmathPoint3 pnt7, vec_ushort8 *threeQuads )
{
    vmathP3StoreHalfFloats(&pnt0, &pnt1, &pnt2, &pnt3, &pnt4, &pnt5, &pnt6, &pnt7, threeQuads);
}

static inline void vmathP3SetX_V( VmathPoint3 *result, float _x )
{
    vmathP3SetX(result, _x);
}

static inline float vmathP3GetX_V( VmathPoint3 pnt )
{
    return vmathP3GetX(&pnt);
}

static inline void vmathP3SetY_V( VmathPoint3 *result, float _y )
{
    vmathP3SetY(result, _y);
}

static inline float vmathP3GetY_V( VmathPoint3 pnt )
{
    return vmathP3GetY(&pnt);
}

static inline void vmathP3SetZ_V( VmathPoint3 *result, float _z )
{
    vmathP3SetZ(result, _z);
}

static inline float vmathP3GetZ_V( VmathPoint3 pnt )
{
    return vmathP3GetZ(&pnt);
}

static inline void vmathP3SetElem_V( VmathPoint3 *result, int idx, float value )
{
    vmathP3SetElem(result, idx, value);
}

static inline float vmathP3GetElem_V( VmathPoint3 pnt, int idx )
{
    return vmathP3GetElem(&pnt, idx);
}

static inline VmathVector3 vmathP3Sub_V( VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    VmathVector3 result;
    vmathP3Sub(&result, &pnt0, &pnt1);
    return result;
}

static inline VmathPoint3 vmathP3AddV3_V( VmathPoint3 pnt, VmathVector3 vec1 )
{
    VmathPoint3 result;
    vmathP3AddV3(&result, &pnt, &vec1);
    return result;
}

static inline VmathPoint3 vmathP3SubV3_V( VmathPoint3 pnt, VmathVector3 vec1 )
{
    VmathPoint3 result;
    vmathP3SubV3(&result, &pnt, &vec1);
    return result;
}

static inline VmathPoint3 vmathP3MulPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    VmathPoint3 result;
    vmathP3MulPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline VmathPoint3 vmathP3DivPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    VmathPoint3 result;
    vmathP3DivPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline VmathPoint3 vmathP3RecipPerElem_V( VmathPoint3 pnt )
{
    VmathPoint3 result;
    vmathP3RecipPerElem(&result, &pnt);
    return result;
}

static inline VmathPoint3 vmathP3SqrtPerElem_V( VmathPoint3 pnt )
{
    VmathPoint3 result;
    vmathP3SqrtPerElem(&result, &pnt);
    return result;
}

static inline VmathPoint3 vmathP3RsqrtPerElem_V( VmathPoint3 pnt )
{
    VmathPoint3 result;
    vmathP3RsqrtPerElem(&result, &pnt);
    return result;
}

static inline VmathPoint3 vmathP3AbsPerElem_V( VmathPoint3 pnt )
{
    VmathPoint3 result;
    vmathP3AbsPerElem(&result, &pnt);
    return result;
}

static inline VmathPoint3 vmathP3CopySignPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    VmathPoint3 result;
    vmathP3CopySignPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline VmathPoint3 vmathP3MaxPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    VmathPoint3 result;
    vmathP3MaxPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline float vmathP3MaxElem_V( VmathPoint3 pnt )
{
    return vmathP3MaxElem(&pnt);
}

static inline VmathPoint3 vmathP3MinPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    VmathPoint3 result;
    vmathP3MinPerElem(&result, &pnt0, &pnt1);
    return result;
}

static inline float vmathP3MinElem_V( VmathPoint3 pnt )
{
    return vmathP3MinElem(&pnt);
}

static inline float vmathP3Sum_V( VmathPoint3 pnt )
{
    return vmathP3Sum(&pnt);
}

static inline VmathPoint3 vmathP3Scale_V( VmathPoint3 pnt, float scaleVal )
{
    VmathPoint3 result;
    vmathP3Scale(&result, &pnt, scaleVal);
    return result;
}

static inline VmathPoint3 vmathP3NonUniformScale_V( VmathPoint3 pnt, VmathVector3 scaleVec )
{
    VmathPoint3 result;
    vmathP3NonUniformScale(&result, &pnt, &scaleVec);
    return result;
}

static inline float vmathP3Projection_V( VmathPoint3 pnt, VmathVector3 unitVec )
{
    return vmathP3Projection(&pnt, &unitVec);
}

static inline float vmathP3DistSqrFromOrigin_V( VmathPoint3 pnt )
{
    return vmathP3DistSqrFromOrigin(&pnt);
}

static inline float vmathP3DistFromOrigin_V( VmathPoint3 pnt )
{
    return vmathP3DistFromOrigin(&pnt);
}

static inline float vmathP3DistSqr_V( VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    return vmathP3DistSqr(&pnt0, &pnt1);
}

static inline float vmathP3Dist_V( VmathPoint3 pnt0, VmathPoint3 pnt1 )
{
    return vmathP3Dist(&pnt0, &pnt1);
}

static inline VmathPoint3 vmathP3Select_V( VmathPoint3 pnt0, VmathPoint3 pnt1, unsigned int select1 )
{
    VmathPoint3 result;
    vmathP3Select(&result, &pnt0, &pnt1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathP3Print_V( VmathPoint3 pnt )
{
    vmathP3Print(&pnt);
}

static inline void vmathP3Prints_V( VmathPoint3 pnt, const char *name )
{
    vmathP3Prints(&pnt, name);
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
