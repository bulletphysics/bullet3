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

#ifndef _VECTORMATH_VEC_AOS_C_H
#define _VECTORMATH_VEC_AOS_C_H
#include <altivec.h>
#include <simdmath.h>
#include <stddef.h>
#include "vec_types.h"
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

static inline vec_float4 _vmathVfDot3( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 result;
    result = vec_madd( vec0, vec1, (vec_float4){0.0f,0.0f,0.0f,0.0f} );
    result = vec_madd( vec_sld( vec0, vec0, 4 ), vec_sld( vec1, vec1, 4 ), result );
    return vec_madd( vec_sld( vec0, vec0, 8 ), vec_sld( vec1, vec1, 8 ), result );
}

static inline vec_float4 _vmathVfDot4( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 result;
    result = vec_madd( vec0, vec1, (vec_float4){0.0f,0.0f,0.0f,0.0f} );
    result = vec_madd( vec_sld( vec0, vec0, 4 ), vec_sld( vec1, vec1, 4 ), result );
    return vec_add( vec_sld( result, result, 8 ), result );
}

static inline vec_float4 _vmathVfCross( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3, result;
    tmp0 = vec_perm( vec0, vec0, _VECTORMATH_PERM_YZXW );
    tmp1 = vec_perm( vec1, vec1, _VECTORMATH_PERM_ZXYW );
    tmp2 = vec_perm( vec0, vec0, _VECTORMATH_PERM_ZXYW );
    tmp3 = vec_perm( vec1, vec1, _VECTORMATH_PERM_YZXW );
    result = vec_madd( tmp0, tmp1, (vec_float4){0.0f,0.0f,0.0f,0.0f} );
    result = vec_nmsub( tmp2, tmp3, result );
    return result;
}

static inline vec_uint4 _vmathVfToHalfFloatsUnpacked(vec_float4 v)
{
    vec_int4 bexp;
    vec_uint4 mant, sign, hfloat;
    vec_uint4 notZero, isInf;
    const vec_uint4 hfloatInf = (vec_uint4){0x00007c00u,0x00007c00u,0x00007c00u,0x00007c00u};
    const vec_uint4 mergeMant = (vec_uint4){0x000003ffu,0x000003ffu,0x000003ffu,0x000003ffu};
    const vec_uint4 mergeSign = (vec_uint4){0x00008000u,0x00008000u,0x00008000u,0x00008000u};

    sign = vec_sr((vec_uint4)v, (vec_uint4){16,16,16,16});
    mant = vec_sr((vec_uint4)v, (vec_uint4){13,13,13,13});
    bexp = vec_and(vec_sr((vec_int4)v, (vec_uint4){23,23,23,23}), (vec_int4){0xff,0xff,0xff,0xff});

    notZero = (vec_uint4)vec_cmpgt(bexp, (vec_int4){112,112,112,112});
    isInf = (vec_uint4)vec_cmpgt(bexp, (vec_int4){142,142,142,142});

    bexp = vec_add(bexp, (vec_int4){-112,-112,-112,-112});
    bexp = vec_sl(bexp, (vec_uint4){10,10,10,10});

    hfloat = vec_sel((vec_uint4)bexp, mant, mergeMant);
    hfloat = vec_sel((vec_uint4){0,0,0,0}, hfloat, notZero);
    hfloat = vec_sel(hfloat, hfloatInf, isInf);
    hfloat = vec_sel(hfloat, sign, mergeSign);

    return hfloat;
}

static inline vec_ushort8 _vmath2VfToHalfFloats(vec_float4 u, vec_float4 v)
{
    vec_uint4 hfloat_u, hfloat_v;
    const vec_uchar16 pack = (vec_uchar16){2,3,6,7,10,11,14,15,18,19,22,23,26,27,30,31};
    hfloat_u = _vmathVfToHalfFloatsUnpacked(u);
    hfloat_v = _vmathVfToHalfFloatsUnpacked(v);
    return (vec_ushort8)vec_perm(hfloat_u, hfloat_v, pack);
}

#ifndef __GNUC__
#define __builtin_constant_p(x) 0
#endif

static inline vec_float4 _vmathVfInsert(vec_float4 dst, vec_float4 src, int slot)
{
#ifdef __GNUC__
    if (__builtin_constant_p(slot)) {
        dst = vec_sld(dst, dst, slot<<2);
        dst = vec_sld(dst, src, 4);
        if (slot != 3) dst = vec_sld(dst, dst, (3-slot)<<2);
        return dst;
    } else
#endif
    {
        vec_uchar16 shiftpattern = vec_lvsr( 0, (float *)(size_t)(slot<<2) );
        vec_uint4 selectmask = (vec_uint4)vec_perm( (vec_uint4){0,0,0,0}, _VECTORMATH_MASK_0xF000, shiftpattern );
        return vec_sel( dst, src, selectmask );
    }
}

#define _vmathVfGetElement(vec, slot) ((float *)&(vec))[slot]
#ifdef _VECTORMATH_SET_CONSTS_IN_MEM
#define _vmathVfSetElement(vec, scalar, slot) ((float *)&(vec))[slot] = scalar
#else
#define _vmathVfSetElement(vec, scalar, slot)                                            \
{                                                                                        \
    if (__builtin_constant_p(scalar)) {                                                  \
        (vec) = _vmathVfInsert(vec, (vec_float4){scalar, scalar, scalar, scalar}, slot); \
    } else {                                                                             \
        ((float *)&(vec))[slot] = scalar;                                                \
    }                                                                                    \
}
#endif

static inline vec_float4 _vmathVfSplatScalar(float scalar)
{
    vec_float4 result;
    if (__builtin_constant_p(scalar)) {
        result = (vec_float4){scalar, scalar, scalar, scalar};
    } else {
        result = vec_ld(0, &scalar);
        result = vec_splat(vec_perm(result, result, vec_lvsl(0, &scalar)), 0);
    } 
    return result;
}

static inline vec_uint4 _vmathVuiSplatScalar(unsigned int scalar)
{
    vec_uint4 result;
    if (__builtin_constant_p(scalar)) {
        result = (vec_uint4){scalar, scalar, scalar, scalar};
    } else {
        result = vec_ld(0, &scalar);
        result = vec_splat(vec_perm(result, result, vec_lvsl(0, &scalar)), 0);
    } 
    return result;
}

#endif

static inline void vmathV3Copy( VmathVector3 *result, const VmathVector3 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathV3MakeFromElems( VmathVector3 *result, float _x, float _y, float _z )
{
    if (__builtin_constant_p(_x) & __builtin_constant_p(_y) & __builtin_constant_p(_z)) {
        result->vec128 = (vec_float4){_x, _y, _z, 0.0f};
    } else {
        float *pf = (float *)&result->vec128;
        pf[0] = _x;
        pf[1] = _y;
        pf[2] = _z;
    }
}

static inline void vmathV3MakeFromP3( VmathVector3 *result, const VmathPoint3 *pnt )
{
    result->vec128 = pnt->vec128;
}

static inline void vmathV3MakeFromScalar( VmathVector3 *result, float scalar )
{
    result->vec128 = _vmathVfSplatScalar(scalar);
}

static inline void vmathV3MakeFrom128( VmathVector3 *result, vec_float4 vf4 )
{
    result->vec128 = vf4;
}

static inline void vmathV3MakeXAxis( VmathVector3 *result )
{
    result->vec128 = _VECTORMATH_UNIT_1000;
}

static inline void vmathV3MakeYAxis( VmathVector3 *result )
{
    result->vec128 = _VECTORMATH_UNIT_0100;
}

static inline void vmathV3MakeZAxis( VmathVector3 *result )
{
    result->vec128 = _VECTORMATH_UNIT_0010;
}

static inline void vmathV3Lerp( VmathVector3 *result, float t, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    VmathVector3 tmpV3_0, tmpV3_1;
    vmathV3Sub( &tmpV3_0, vec1, vec0 );
    vmathV3ScalarMul( &tmpV3_1, &tmpV3_0, t );
    vmathV3Add( result, vec0, &tmpV3_1 );
}

static inline void vmathV3Slerp( VmathVector3 *result, float t, const VmathVector3 *unitVec0, const VmathVector3 *unitVec1 )
{
    vec_float4 scales, scale0, scale1, cosAngle, angle, tttt, oneMinusT, angles, sines;
    vec_uint4 selectMask;
    cosAngle = _vmathVfDot3( unitVec0->vec128, unitVec1->vec128 );
    cosAngle = vec_splat( cosAngle, 0 );
    selectMask = (vec_uint4)vec_cmpgt( ((vec_float4){_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL}), cosAngle );
    angle = acosf4( cosAngle );
    tttt = _vmathVfSplatScalar(t);
    oneMinusT = vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), tttt );
    angles = vec_mergeh( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), tttt );
    angles = vec_mergeh( angles, oneMinusT );
    angles = vec_madd( angles, angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sines = sinf4( angles );
    scales = divf4( sines, vec_splat( sines, 0 ) );
    scale0 = vec_sel( oneMinusT, vec_splat( scales, 1 ), selectMask );
    scale1 = vec_sel( tttt, vec_splat( scales, 2 ), selectMask );
    result->vec128 = vec_madd( unitVec0->vec128, scale0, vec_madd( unitVec1->vec128, scale1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

static inline vec_float4 vmathV3Get128( const VmathVector3 *vec )
{
    return vec->vec128;
}

static inline void vmathV3StoreXYZ( const VmathVector3 *vec, vec_float4 *quad )
{
    vec_float4 dstVec = *quad;
    vec_uint4 mask = _VECTORMATH_MASK_0x000F;
    dstVec = vec_sel(vec->vec128, dstVec, mask);
    *quad = dstVec;
}

static inline void vmathV3LoadXYZArray( VmathVector3 *vec0, VmathVector3 *vec1, VmathVector3 *vec2, VmathVector3 *vec3, const vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz, xyz1, xyz2, xyz3;
    xyzx = threeQuads[0];
    yzxy = threeQuads[1];
    zxyz = threeQuads[2];
    xyz1 = vec_sld( xyzx, yzxy, 12 );
    xyz2 = vec_sld( yzxy, zxyz, 8 );
    xyz3 = vec_sld( zxyz, zxyz, 4 );
    vec0->vec128 = xyzx;
    vec1->vec128 = xyz1;
    vec2->vec128 = xyz2;
    vec3->vec128 = xyz3;
}

static inline void vmathV3StoreXYZArray( const VmathVector3 *vec0, const VmathVector3 *vec1, const VmathVector3 *vec2, const VmathVector3 *vec3, vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz;
    xyzx = vec_perm( vec0->vec128, vec1->vec128, _VECTORMATH_PERM_XYZA );
    yzxy = vec_perm( vec1->vec128, vec2->vec128, _VECTORMATH_PERM_YZAB );
    zxyz = vec_perm( vec2->vec128, vec3->vec128, _VECTORMATH_PERM_ZABC );
    threeQuads[0] = xyzx;
    threeQuads[1] = yzxy;
    threeQuads[2] = zxyz;
}

static inline void vmathV3StoreHalfFloats( const VmathVector3 *vec0, const VmathVector3 *vec1, const VmathVector3 *vec2, const VmathVector3 *vec3, const VmathVector3 *vec4, const VmathVector3 *vec5, const VmathVector3 *vec6, const VmathVector3 *vec7, vec_ushort8 *threeQuads )
{
    vec_float4 xyz0[3];
    vec_float4 xyz1[3];
    vmathV3StoreXYZArray( vec0, vec1, vec2, vec3, xyz0 );
    vmathV3StoreXYZArray( vec4, vec5, vec6, vec7, xyz1 );
    threeQuads[0] = _vmath2VfToHalfFloats(xyz0[0], xyz0[1]);
    threeQuads[1] = _vmath2VfToHalfFloats(xyz0[2], xyz1[0]);
    threeQuads[2] = _vmath2VfToHalfFloats(xyz1[1], xyz1[2]);
}

static inline void vmathV3SetX( VmathVector3 *result, float _x )
{
    _vmathVfSetElement(result->vec128, _x, 0);
}

static inline float vmathV3GetX( const VmathVector3 *vec )
{
    return _vmathVfGetElement(vec->vec128, 0);
}

static inline void vmathV3SetY( VmathVector3 *result, float _y )
{
    _vmathVfSetElement(result->vec128, _y, 1);
}

static inline float vmathV3GetY( const VmathVector3 *vec )
{
    return _vmathVfGetElement(vec->vec128, 1);
}

static inline void vmathV3SetZ( VmathVector3 *result, float _z )
{
    _vmathVfSetElement(result->vec128, _z, 2);
}

static inline float vmathV3GetZ( const VmathVector3 *vec )
{
    return _vmathVfGetElement(vec->vec128, 2);
}

static inline void vmathV3SetElem( VmathVector3 *result, int idx, float value )
{
    _vmathVfSetElement(result->vec128, value, idx);
}

static inline float vmathV3GetElem( const VmathVector3 *vec, int idx )
{
    return _vmathVfGetElement(vec->vec128, idx);
}

static inline void vmathV3Add( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = vec_add( vec0->vec128, vec1->vec128 );
}

static inline void vmathV3Sub( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = vec_sub( vec0->vec128, vec1->vec128 );
}

static inline void vmathV3AddP3( VmathPoint3 *result, const VmathVector3 *vec, const VmathPoint3 *pnt1 )
{
    result->vec128 = vec_add( vec->vec128, pnt1->vec128 );
}

static inline void vmathV3ScalarMul( VmathVector3 *result, const VmathVector3 *vec, float scalar )
{
    result->vec128 = vec_madd( vec->vec128, _vmathVfSplatScalar(scalar), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathV3ScalarDiv( VmathVector3 *result, const VmathVector3 *vec, float scalar )
{
    result->vec128 = divf4( vec->vec128, _vmathVfSplatScalar(scalar) );
}

static inline void vmathV3Neg( VmathVector3 *result, const VmathVector3 *vec )
{
    result->vec128 = negatef4( vec->vec128 );
}

static inline void vmathV3MulPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = vec_madd( vec0->vec128, vec1->vec128, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathV3DivPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = divf4( vec0->vec128, vec1->vec128 );
}

static inline void vmathV3RecipPerElem( VmathVector3 *result, const VmathVector3 *vec )
{
    result->vec128 = recipf4( vec->vec128 );
}

static inline void vmathV3SqrtPerElem( VmathVector3 *result, const VmathVector3 *vec )
{
    result->vec128 = sqrtf4( vec->vec128 );
}

static inline void vmathV3RsqrtPerElem( VmathVector3 *result, const VmathVector3 *vec )
{
    result->vec128 = rsqrtf4( vec->vec128 );
}

static inline void vmathV3AbsPerElem( VmathVector3 *result, const VmathVector3 *vec )
{
    result->vec128 = fabsf4( vec->vec128 );
}

static inline void vmathV3CopySignPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = copysignf4( vec0->vec128, vec1->vec128 );
}

static inline void vmathV3MaxPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = fmaxf4( vec0->vec128, vec1->vec128 );
}

static inline float vmathV3MaxElem( const VmathVector3 *vec )
{
    vec_float4 result;
    result = fmaxf4( vec_splat( vec->vec128, 1 ), vec->vec128 );
    result = fmaxf4( vec_splat( vec->vec128, 2 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline void vmathV3MinPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = fminf4( vec0->vec128, vec1->vec128 );
}

static inline float vmathV3MinElem( const VmathVector3 *vec )
{
    vec_float4 result;
    result = fminf4( vec_splat( vec->vec128, 1 ), vec->vec128 );
    result = fminf4( vec_splat( vec->vec128, 2 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathV3Sum( const VmathVector3 *vec )
{
    vec_float4 result;
    result = vec_add( vec_splat( vec->vec128, 1 ), vec->vec128 );
    result = vec_add( vec_splat( vec->vec128, 2 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathV3Dot( const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    vec_float4 result = _vmathVfDot3( vec0->vec128, vec1->vec128 );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathV3LengthSqr( const VmathVector3 *vec )
{
    vec_float4 result = _vmathVfDot3( vec->vec128, vec->vec128 );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathV3Length( const VmathVector3 *vec )
{
    return sqrtf( vmathV3LengthSqr( vec ) );
}

static inline void vmathV3Normalize( VmathVector3 *result, const VmathVector3 *vec )
{
    vec_float4 dot = _vmathVfDot3( vec->vec128, vec->vec128 );
    dot = vec_splat( dot, 0 );
    result->vec128 = vec_madd( vec->vec128, rsqrtf4( dot ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathV3Cross( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = _vmathVfCross( vec0->vec128, vec1->vec128 );
}

static inline void vmathV3Select( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1, unsigned int select1 )
{
    unsigned int tmp;
    tmp = (unsigned int)-(select1 > 0);
    result->vec128 = vec_sel( vec0->vec128, vec1->vec128, _vmathVuiSplatScalar(tmp) );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathV3Print( const VmathVector3 *vec )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = vec->vec128;
    printf( "( %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2] );
}

static inline void vmathV3Prints( const VmathVector3 *vec, const char *name )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = vec->vec128;
    printf( "%s: ( %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2] );
}

#endif

static inline void vmathV4Copy( VmathVector4 *result, const VmathVector4 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathV4MakeFromElems( VmathVector4 *result, float _x, float _y, float _z, float _w )
{
    if (__builtin_constant_p(_x) & __builtin_constant_p(_y) &
        __builtin_constant_p(_z) & __builtin_constant_p(_w)) {
        result->vec128 = (vec_float4){_x, _y, _z, _w};
    } else {
        float *pf = (float *)&result->vec128;
        pf[0] = _x;
        pf[1] = _y;
        pf[2] = _z;
        pf[3] = _w;
    }
}

static inline void vmathV4MakeFromV3Scalar( VmathVector4 *result, const VmathVector3 *xyz, float _w )
{
    result->vec128 = xyz->vec128;
    _vmathVfSetElement(result->vec128, _w, 3);
}

static inline void vmathV4MakeFromV3( VmathVector4 *result, const VmathVector3 *vec )
{
    result->vec128 = vec->vec128;
    result->vec128 = _vmathVfInsert(result->vec128, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), 3);
}

static inline void vmathV4MakeFromP3( VmathVector4 *result, const VmathPoint3 *pnt )
{
    result->vec128 = pnt->vec128;
    result->vec128 = _vmathVfInsert(result->vec128, ((vec_float4){1.0f,1.0f,1.0f,1.0f}), 3);
}

static inline void vmathV4MakeFromQ( VmathVector4 *result, const VmathQuat *quat )
{
    result->vec128 = quat->vec128;
}

static inline void vmathV4MakeFromScalar( VmathVector4 *result, float scalar )
{
    result->vec128 = _vmathVfSplatScalar(scalar);
}

static inline void vmathV4MakeFrom128( VmathVector4 *result, vec_float4 vf4 )
{
    result->vec128 = vf4;
}

static inline void vmathV4MakeXAxis( VmathVector4 *result )
{
    result->vec128 = _VECTORMATH_UNIT_1000;
}

static inline void vmathV4MakeYAxis( VmathVector4 *result )
{
    result->vec128 = _VECTORMATH_UNIT_0100;
}

static inline void vmathV4MakeZAxis( VmathVector4 *result )
{
    result->vec128 = _VECTORMATH_UNIT_0010;
}

static inline void vmathV4MakeWAxis( VmathVector4 *result )
{
    result->vec128 = _VECTORMATH_UNIT_0001;
}

static inline void vmathV4Lerp( VmathVector4 *result, float t, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    VmathVector4 tmpV4_0, tmpV4_1;
    vmathV4Sub( &tmpV4_0, vec1, vec0 );
    vmathV4ScalarMul( &tmpV4_1, &tmpV4_0, t );
    vmathV4Add( result, vec0, &tmpV4_1 );
}

static inline void vmathV4Slerp( VmathVector4 *result, float t, const VmathVector4 *unitVec0, const VmathVector4 *unitVec1 )
{
    vec_float4 scales, scale0, scale1, cosAngle, angle, tttt, oneMinusT, angles, sines;
    vec_uint4 selectMask;
    cosAngle = _vmathVfDot4( unitVec0->vec128, unitVec1->vec128 );
    cosAngle = vec_splat( cosAngle, 0 );
    selectMask = (vec_uint4)vec_cmpgt( ((vec_float4){_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL}), cosAngle );
    angle = acosf4( cosAngle );
    tttt = _vmathVfSplatScalar(t);
    oneMinusT = vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), tttt );
    angles = vec_mergeh( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), tttt );
    angles = vec_mergeh( angles, oneMinusT );
    angles = vec_madd( angles, angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sines = sinf4( angles );
    scales = divf4( sines, vec_splat( sines, 0 ) );
    scale0 = vec_sel( oneMinusT, vec_splat( scales, 1 ), selectMask );
    scale1 = vec_sel( tttt, vec_splat( scales, 2 ), selectMask );
    result->vec128 = vec_madd( unitVec0->vec128, scale0, vec_madd( unitVec1->vec128, scale1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

static inline vec_float4 vmathV4Get128( const VmathVector4 *vec )
{
    return vec->vec128;
}

static inline void vmathV4StoreHalfFloats( const VmathVector4 *vec0, const VmathVector4 *vec1, const VmathVector4 *vec2, const VmathVector4 *vec3, vec_ushort8 *twoQuads )
{
    twoQuads[0] = _vmath2VfToHalfFloats(vec0->vec128, vec1->vec128);
    twoQuads[1] = _vmath2VfToHalfFloats(vec2->vec128, vec3->vec128);
}

static inline void vmathV4SetXYZ( VmathVector4 *result, const VmathVector3 *vec )
{
    result->vec128 = vec_sel( vec->vec128, result->vec128, _VECTORMATH_MASK_0x000F );
}

static inline void vmathV4GetXYZ( VmathVector3 *result, const VmathVector4 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathV4SetX( VmathVector4 *result, float _x )
{
    _vmathVfSetElement(result->vec128, _x, 0);
}

static inline float vmathV4GetX( const VmathVector4 *vec )
{
    return _vmathVfGetElement(vec->vec128, 0);
}

static inline void vmathV4SetY( VmathVector4 *result, float _y )
{
    _vmathVfSetElement(result->vec128, _y, 1);
}

static inline float vmathV4GetY( const VmathVector4 *vec )
{
    return _vmathVfGetElement(vec->vec128, 1);
}

static inline void vmathV4SetZ( VmathVector4 *result, float _z )
{
    _vmathVfSetElement(result->vec128, _z, 2);
}

static inline float vmathV4GetZ( const VmathVector4 *vec )
{
    return _vmathVfGetElement(vec->vec128, 2);
}

static inline void vmathV4SetW( VmathVector4 *result, float _w )
{
    _vmathVfSetElement(result->vec128, _w, 3);
}

static inline float vmathV4GetW( const VmathVector4 *vec )
{
    return _vmathVfGetElement(vec->vec128, 3);
}

static inline void vmathV4SetElem( VmathVector4 *result, int idx, float value )
{
    _vmathVfSetElement(result->vec128, value, idx);
}

static inline float vmathV4GetElem( const VmathVector4 *vec, int idx )
{
    return _vmathVfGetElement(vec->vec128, idx);
}

static inline void vmathV4Add( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = vec_add( vec0->vec128, vec1->vec128 );
}

static inline void vmathV4Sub( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = vec_sub( vec0->vec128, vec1->vec128 );
}

static inline void vmathV4ScalarMul( VmathVector4 *result, const VmathVector4 *vec, float scalar )
{
    result->vec128 = vec_madd( vec->vec128, _vmathVfSplatScalar(scalar), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathV4ScalarDiv( VmathVector4 *result, const VmathVector4 *vec, float scalar )
{
    result->vec128 = divf4( vec->vec128, _vmathVfSplatScalar(scalar) );
}

static inline void vmathV4Neg( VmathVector4 *result, const VmathVector4 *vec )
{
    result->vec128 = negatef4( vec->vec128 );
}

static inline void vmathV4MulPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = vec_madd( vec0->vec128, vec1->vec128, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathV4DivPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = divf4( vec0->vec128, vec1->vec128 );
}

static inline void vmathV4RecipPerElem( VmathVector4 *result, const VmathVector4 *vec )
{
    result->vec128 = recipf4( vec->vec128 );
}

static inline void vmathV4SqrtPerElem( VmathVector4 *result, const VmathVector4 *vec )
{
    result->vec128 = sqrtf4( vec->vec128 );
}

static inline void vmathV4RsqrtPerElem( VmathVector4 *result, const VmathVector4 *vec )
{
    result->vec128 = rsqrtf4( vec->vec128 );
}

static inline void vmathV4AbsPerElem( VmathVector4 *result, const VmathVector4 *vec )
{
    result->vec128 = fabsf4( vec->vec128 );
}

static inline void vmathV4CopySignPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = copysignf4( vec0->vec128, vec1->vec128 );
}

static inline void vmathV4MaxPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = fmaxf4( vec0->vec128, vec1->vec128 );
}

static inline float vmathV4MaxElem( const VmathVector4 *vec )
{
    vec_float4 result;
    result = fmaxf4( vec_splat( vec->vec128, 1 ), vec->vec128 );
    result = fmaxf4( vec_splat( vec->vec128, 2 ), result );
    result = fmaxf4( vec_splat( vec->vec128, 3 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline void vmathV4MinPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = fminf4( vec0->vec128, vec1->vec128 );
}

static inline float vmathV4MinElem( const VmathVector4 *vec )
{
    vec_float4 result;
    result = fminf4( vec_splat( vec->vec128, 1 ), vec->vec128 );
    result = fminf4( vec_splat( vec->vec128, 2 ), result );
    result = fminf4( vec_splat( vec->vec128, 3 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathV4Sum( const VmathVector4 *vec )
{
    vec_float4 result;
    result = vec_add( vec_splat( vec->vec128, 1 ), vec->vec128 );
    result = vec_add( vec_splat( vec->vec128, 2 ), result );
    result = vec_add( vec_splat( vec->vec128, 3 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathV4Dot( const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    vec_float4 result = _vmathVfDot4( vec0->vec128, vec1->vec128 );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathV4LengthSqr( const VmathVector4 *vec )
{
    vec_float4 result = _vmathVfDot4( vec->vec128, vec->vec128 );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathV4Length( const VmathVector4 *vec )
{
    return sqrtf( vmathV4LengthSqr( vec ) );
}

static inline void vmathV4Normalize( VmathVector4 *result, const VmathVector4 *vec )
{
    vec_float4 dot = _vmathVfDot4( vec->vec128, vec->vec128 );
    result->vec128 = vec_madd( vec->vec128, rsqrtf4( dot ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathV4Select( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1, unsigned int select1 )
{
    unsigned int tmp;
    tmp = (unsigned int)-(select1 > 0);
    result->vec128 = vec_sel( vec0->vec128, vec1->vec128, _vmathVuiSplatScalar(tmp) );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathV4Print( const VmathVector4 *vec )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = vec->vec128;
    printf( "( %f %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

static inline void vmathV4Prints( const VmathVector4 *vec, const char *name )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = vec->vec128;
    printf( "%s: ( %f %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

#endif

static inline void vmathP3Copy( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->vec128 = pnt->vec128;
}

static inline void vmathP3MakeFromElems( VmathPoint3 *result, float _x, float _y, float _z )
{
    if (__builtin_constant_p(_x) & __builtin_constant_p(_y) & __builtin_constant_p(_z)) {
        result->vec128 = (vec_float4){_x, _y, _z, 0.0f};
    } else {
        float *pf = (float *)&result->vec128;
        pf[0] = _x;
        pf[1] = _y;
        pf[2] = _z;
    }
}

static inline void vmathP3MakeFromV3( VmathPoint3 *result, const VmathVector3 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathP3MakeFromScalar( VmathPoint3 *result, float scalar )
{
    result->vec128 = _vmathVfSplatScalar(scalar);
}

static inline void vmathP3MakeFrom128( VmathPoint3 *result, vec_float4 vf4 )
{
    result->vec128 = vf4;
}

static inline void vmathP3Lerp( VmathPoint3 *result, float t, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    VmathVector3 tmpV3_0, tmpV3_1;
    vmathP3Sub( &tmpV3_0, pnt1, pnt0 );
    vmathV3ScalarMul( &tmpV3_1, &tmpV3_0, t );
    vmathP3AddV3( result, pnt0, &tmpV3_1 );
}

static inline vec_float4 vmathP3Get128( const VmathPoint3 *pnt )
{
    return pnt->vec128;
}

static inline void vmathP3StoreXYZ( const VmathPoint3 *pnt, vec_float4 *quad )
{
    vec_float4 dstVec = *quad;
    vec_uint4 mask = _VECTORMATH_MASK_0x000F;
    dstVec = vec_sel(pnt->vec128, dstVec, mask);
    *quad = dstVec;
}

static inline void vmathP3LoadXYZArray( VmathPoint3 *pnt0, VmathPoint3 *pnt1, VmathPoint3 *pnt2, VmathPoint3 *pnt3, const vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz, xyz1, xyz2, xyz3;
    xyzx = threeQuads[0];
    yzxy = threeQuads[1];
    zxyz = threeQuads[2];
    xyz1 = vec_sld( xyzx, yzxy, 12 );
    xyz2 = vec_sld( yzxy, zxyz, 8 );
    xyz3 = vec_sld( zxyz, zxyz, 4 );
    pnt0->vec128 = xyzx;
    pnt1->vec128 = xyz1;
    pnt2->vec128 = xyz2;
    pnt3->vec128 = xyz3;
}

static inline void vmathP3StoreXYZArray( const VmathPoint3 *pnt0, const VmathPoint3 *pnt1, const VmathPoint3 *pnt2, const VmathPoint3 *pnt3, vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz;
    xyzx = vec_perm( pnt0->vec128, pnt1->vec128, _VECTORMATH_PERM_XYZA );
    yzxy = vec_perm( pnt1->vec128, pnt2->vec128, _VECTORMATH_PERM_YZAB );
    zxyz = vec_perm( pnt2->vec128, pnt3->vec128, _VECTORMATH_PERM_ZABC );
    threeQuads[0] = xyzx;
    threeQuads[1] = yzxy;
    threeQuads[2] = zxyz;
}

static inline void vmathP3StoreHalfFloats( const VmathPoint3 *pnt0, const VmathPoint3 *pnt1, const VmathPoint3 *pnt2, const VmathPoint3 *pnt3, const VmathPoint3 *pnt4, const VmathPoint3 *pnt5, const VmathPoint3 *pnt6, const VmathPoint3 *pnt7, vec_ushort8 *threeQuads )
{
    vec_float4 xyz0[3];
    vec_float4 xyz1[3];
    vmathP3StoreXYZArray( pnt0, pnt1, pnt2, pnt3, xyz0 );
    vmathP3StoreXYZArray( pnt4, pnt5, pnt6, pnt7, xyz1 );
    threeQuads[0] = _vmath2VfToHalfFloats(xyz0[0], xyz0[1]);
    threeQuads[1] = _vmath2VfToHalfFloats(xyz0[2], xyz1[0]);
    threeQuads[2] = _vmath2VfToHalfFloats(xyz1[1], xyz1[2]);
}

static inline void vmathP3SetX( VmathPoint3 *result, float _x )
{
    _vmathVfSetElement(result->vec128, _x, 0);
}

static inline float vmathP3GetX( const VmathPoint3 *pnt )
{
    return _vmathVfGetElement(pnt->vec128, 0);
}

static inline void vmathP3SetY( VmathPoint3 *result, float _y )
{
    _vmathVfSetElement(result->vec128, _y, 1);
}

static inline float vmathP3GetY( const VmathPoint3 *pnt )
{
    return _vmathVfGetElement(pnt->vec128, 1);
}

static inline void vmathP3SetZ( VmathPoint3 *result, float _z )
{
    _vmathVfSetElement(result->vec128, _z, 2);
}

static inline float vmathP3GetZ( const VmathPoint3 *pnt )
{
    return _vmathVfGetElement(pnt->vec128, 2);
}

static inline void vmathP3SetElem( VmathPoint3 *result, int idx, float value )
{
    _vmathVfSetElement(result->vec128, value, idx);
}

static inline float vmathP3GetElem( const VmathPoint3 *pnt, int idx )
{
    return _vmathVfGetElement(pnt->vec128, idx);
}

static inline void vmathP3Sub( VmathVector3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = vec_sub( pnt0->vec128, pnt1->vec128 );
}

static inline void vmathP3AddV3( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *vec1 )
{
    result->vec128 = vec_add( pnt->vec128, vec1->vec128 );
}

static inline void vmathP3SubV3( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *vec1 )
{
    result->vec128 = vec_sub( pnt->vec128, vec1->vec128 );
}

static inline void vmathP3MulPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = vec_madd( pnt0->vec128, pnt1->vec128, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathP3DivPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = divf4( pnt0->vec128, pnt1->vec128 );
}

static inline void vmathP3RecipPerElem( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->vec128 = recipf4( pnt->vec128 );
}

static inline void vmathP3SqrtPerElem( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->vec128 = sqrtf4( pnt->vec128 );
}

static inline void vmathP3RsqrtPerElem( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->vec128 = rsqrtf4( pnt->vec128 );
}

static inline void vmathP3AbsPerElem( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->vec128 = fabsf4( pnt->vec128 );
}

static inline void vmathP3CopySignPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = copysignf4( pnt0->vec128, pnt1->vec128 );
}

static inline void vmathP3MaxPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = fmaxf4( pnt0->vec128, pnt1->vec128 );
}

static inline float vmathP3MaxElem( const VmathPoint3 *pnt )
{
    vec_float4 result;
    result = fmaxf4( vec_splat( pnt->vec128, 1 ), pnt->vec128 );
    result = fmaxf4( vec_splat( pnt->vec128, 2 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline void vmathP3MinPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = fminf4( pnt0->vec128, pnt1->vec128 );
}

static inline float vmathP3MinElem( const VmathPoint3 *pnt )
{
    vec_float4 result;
    result = fminf4( vec_splat( pnt->vec128, 1 ), pnt->vec128 );
    result = fminf4( vec_splat( pnt->vec128, 2 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathP3Sum( const VmathPoint3 *pnt )
{
    vec_float4 result;
    result = vec_add( vec_splat( pnt->vec128, 1 ), pnt->vec128 );
    result = vec_add( vec_splat( pnt->vec128, 2 ), result );
    return _vmathVfGetElement(result, 0);
}

static inline void vmathP3Scale( VmathPoint3 *result, const VmathPoint3 *pnt, float scaleVal )
{
    VmathPoint3 tmpP3_0;
    vmathP3MakeFromScalar( &tmpP3_0, scaleVal );
    vmathP3MulPerElem( result, pnt, &tmpP3_0 );
}

static inline void vmathP3NonUniformScale( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *scaleVec )
{
    VmathPoint3 tmpP3_0;
    vmathP3MakeFromV3( &tmpP3_0, scaleVec );
    vmathP3MulPerElem( result, pnt, &tmpP3_0 );
}

static inline float vmathP3Projection( const VmathPoint3 *pnt, const VmathVector3 *unitVec )
{
    vec_float4 result = _vmathVfDot3( pnt->vec128, unitVec->vec128 );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathP3DistSqrFromOrigin( const VmathPoint3 *pnt )
{
    VmathVector3 tmpV3_0;
    vmathV3MakeFromP3( &tmpV3_0, pnt );
    return vmathV3LengthSqr( &tmpV3_0 );
}

static inline float vmathP3DistFromOrigin( const VmathPoint3 *pnt )
{
    VmathVector3 tmpV3_0;
    vmathV3MakeFromP3( &tmpV3_0, pnt );
    return vmathV3Length( &tmpV3_0 );
}

static inline float vmathP3DistSqr( const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    VmathVector3 tmpV3_0;
    vmathP3Sub( &tmpV3_0, pnt1, pnt0 );
    return vmathV3LengthSqr( &tmpV3_0 );
}

static inline float vmathP3Dist( const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    VmathVector3 tmpV3_0;
    vmathP3Sub( &tmpV3_0, pnt1, pnt0 );
    return vmathV3Length( &tmpV3_0 );
}

static inline void vmathP3Select( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1, unsigned int select1 )
{
    unsigned int tmp;
    tmp = (unsigned int)-(select1 > 0);
    result->vec128 = vec_sel( pnt0->vec128, pnt1->vec128, _vmathVuiSplatScalar(tmp) );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathP3Print( const VmathPoint3 *pnt )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = pnt->vec128;
    printf( "( %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2] );
}

static inline void vmathP3Prints( const VmathPoint3 *pnt, const char *name )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = pnt->vec128;
    printf( "%s: ( %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2] );
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
