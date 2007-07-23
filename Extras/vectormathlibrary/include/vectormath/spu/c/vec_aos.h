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
#define _VECTORMATH_SHUF_XYZA (vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_A }
#define _VECTORMATH_SHUF_ZXYW (vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_W }
#define _VECTORMATH_SHUF_YZXW (vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_W }
#define _VECTORMATH_SHUF_WABC (vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_C }
#define _VECTORMATH_SHUF_ZWAB (vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_B }
#define _VECTORMATH_SHUF_XYZA (vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_A }
#define _VECTORMATH_SHUF_YZAB (vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_B }
#define _VECTORMATH_SHUF_ZABC (vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_C }
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
    result = spu_mul( vec0, vec1 );
    result = spu_madd( spu_rlqwbyte( vec0, 4 ), spu_rlqwbyte( vec1, 4 ), result );
    return spu_madd( spu_rlqwbyte( vec0, 8 ), spu_rlqwbyte( vec1, 8 ), result );
}

static inline vec_float4 _vmathVfDot4( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 result;
    result = spu_mul( vec0, vec1 );
    result = spu_madd( spu_rlqwbyte( vec0, 4 ), spu_rlqwbyte( vec1, 4 ), result );
    return spu_add( spu_rlqwbyte( result, 8 ), result );
}

static inline vec_float4 _vmathVfCross( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3, result;
    tmp0 = spu_shuffle( vec0, vec0, _VECTORMATH_SHUF_YZXW );
    tmp1 = spu_shuffle( vec1, vec1, _VECTORMATH_SHUF_ZXYW );
    tmp2 = spu_shuffle( vec0, vec0, _VECTORMATH_SHUF_ZXYW );
    tmp3 = spu_shuffle( vec1, vec1, _VECTORMATH_SHUF_YZXW );
    result = spu_mul( tmp0, tmp1 );
    result = spu_nmsub( tmp2, tmp3, result );
    return result;
}

static inline vec_uint4 _vmathVfToHalfFloatsUnpacked(vec_float4 v)
{
    vec_int4 bexp;
    vec_uint4 mant, sign, hfloat;
    vec_uint4 notZero, isInf;
    const vec_uint4 hfloatInf = spu_splats(0x00007c00u);
    const vec_uint4 mergeMant = spu_splats(0x000003ffu);
    const vec_uint4 mergeSign = spu_splats(0x00008000u);

    sign = spu_rlmask((vec_uint4)v, -16);
    mant = spu_rlmask((vec_uint4)v, -13);
    bexp = spu_and(spu_rlmask((vec_int4)v, -23), 0xff);

    notZero = spu_cmpgt(bexp, 112);
    isInf = spu_cmpgt(bexp, 142);

    bexp = spu_add(bexp, -112);
    bexp = spu_sl(bexp, 10);

    hfloat = spu_sel((vec_uint4)bexp, mant, mergeMant);
    hfloat = spu_sel(spu_splats(0u), hfloat, notZero);
    hfloat = spu_sel(hfloat, hfloatInf, isInf);
    hfloat = spu_sel(hfloat, sign, mergeSign);

    return hfloat;
}

static inline vec_ushort8 _vmath2VfToHalfFloats(vec_float4 u, vec_float4 v)
{
    vec_uint4 hfloat_u, hfloat_v;
    const vec_uchar16 pack = (vec_uchar16){2,3,6,7,10,11,14,15,18,19,22,23,26,27,30,31};
    hfloat_u = _vmathVfToHalfFloatsUnpacked(u);
    hfloat_v = _vmathVfToHalfFloatsUnpacked(v);
    return (vec_ushort8)spu_shuffle(hfloat_u, hfloat_v, pack);
}

#endif

static inline void vmathV3Copy( VmathVector3 *result, const VmathVector3 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathV3MakeFromElems( VmathVector3 *result, float _x, float _y, float _z )
{
    result->vec128 = (vec_float4){ _x, _y, _z, 0.0f  };
}

static inline void vmathV3MakeFromP3( VmathVector3 *result, const VmathPoint3 *pnt )
{
    result->vec128 = pnt->vec128;
}

static inline void vmathV3MakeFromScalar( VmathVector3 *result, float scalar )
{
    result->vec128 = spu_splats( scalar );
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
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    cosAngle = _vmathVfDot3( unitVec0->vec128, unitVec1->vec128 );
    cosAngle = spu_shuffle( cosAngle, cosAngle, shuffle_xxxx );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(_VECTORMATH_SLERP_TOL), cosAngle );
    angle = acosf4( cosAngle );
    tttt = spu_splats(t);
    oneMinusT = spu_sub( spu_splats(1.0f), tttt );
    angles = spu_sel( spu_splats(1.0f), oneMinusT, (vec_uint4)spu_maskb(0x0f00) );
    angles = spu_sel( angles, tttt, (vec_uint4)spu_maskb(0x00f0) );
    angles = spu_mul( angles, angle );
    sines = sinf4( angles );
    scales = divf4( sines, spu_shuffle( sines, sines, shuffle_xxxx ) );
    scale0 = spu_sel( oneMinusT, spu_shuffle( scales, scales, shuffle_yyyy ), selectMask );
    scale1 = spu_sel( tttt, spu_shuffle( scales, scales, shuffle_zzzz ), selectMask );
    result->vec128 = spu_madd( unitVec0->vec128, scale0, spu_mul( unitVec1->vec128, scale1 ) );
}

static inline vec_float4 vmathV3Get128( const VmathVector3 *vec )
{
    return vec->vec128;
}

static inline void vmathV3StoreXYZ( const VmathVector3 *vec, vec_float4 *quad )
{
    vec_float4 dstVec = *quad;
    vec_uint4 mask = (vec_uint4)spu_maskb(0x000f);
    dstVec = spu_sel(vec->vec128, dstVec, mask);
    *quad = dstVec;
}

static inline void vmathV3LoadXYZArray( VmathVector3 *vec0, VmathVector3 *vec1, VmathVector3 *vec2, VmathVector3 *vec3, const vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz, xyz1, xyz2, xyz3;
    xyzx = threeQuads[0];
    yzxy = threeQuads[1];
    zxyz = threeQuads[2];
    xyz1 = spu_shuffle( xyzx, yzxy, _VECTORMATH_SHUF_WABC );
    xyz2 = spu_shuffle( yzxy, zxyz, _VECTORMATH_SHUF_ZWAB );
    xyz3 = spu_rlqwbyte( zxyz, 4 );
    vec0->vec128 = xyzx;
    vec1->vec128 = xyz1;
    vec2->vec128 = xyz2;
    vec3->vec128 = xyz3;
}

static inline void vmathV3StoreXYZArray( const VmathVector3 *vec0, const VmathVector3 *vec1, const VmathVector3 *vec2, const VmathVector3 *vec3, vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz;
    xyzx = spu_shuffle( vec0->vec128, vec1->vec128, _VECTORMATH_SHUF_XYZA );
    yzxy = spu_shuffle( vec1->vec128, vec2->vec128, _VECTORMATH_SHUF_YZAB );
    zxyz = spu_shuffle( vec2->vec128, vec3->vec128, _VECTORMATH_SHUF_ZABC );
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
    result->vec128 = spu_insert( _x, result->vec128, 0 );
}

static inline float vmathV3GetX( const VmathVector3 *vec )
{
    return spu_extract( vec->vec128, 0 );
}

static inline void vmathV3SetY( VmathVector3 *result, float _y )
{
    result->vec128 = spu_insert( _y, result->vec128, 1 );
}

static inline float vmathV3GetY( const VmathVector3 *vec )
{
    return spu_extract( vec->vec128, 1 );
}

static inline void vmathV3SetZ( VmathVector3 *result, float _z )
{
    result->vec128 = spu_insert( _z, result->vec128, 2 );
}

static inline float vmathV3GetZ( const VmathVector3 *vec )
{
    return spu_extract( vec->vec128, 2 );
}

static inline void vmathV3SetElem( VmathVector3 *result, int idx, float value )
{
    result->vec128 = spu_insert( value, result->vec128, idx );
}

static inline float vmathV3GetElem( const VmathVector3 *vec, int idx )
{
    return spu_extract( vec->vec128, idx );
}

static inline void vmathV3Add( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = spu_add( vec0->vec128, vec1->vec128 );
}

static inline void vmathV3Sub( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = spu_sub( vec0->vec128, vec1->vec128 );
}

static inline void vmathV3AddP3( VmathPoint3 *result, const VmathVector3 *vec, const VmathPoint3 *pnt1 )
{
    result->vec128 = spu_add( vec->vec128, pnt1->vec128 );
}

static inline void vmathV3ScalarMul( VmathVector3 *result, const VmathVector3 *vec, float scalar )
{
    result->vec128 = spu_mul( vec->vec128, spu_splats(scalar) );
}

static inline void vmathV3ScalarDiv( VmathVector3 *result, const VmathVector3 *vec, float scalar )
{
    result->vec128 = divf4( vec->vec128, spu_splats(scalar) );
}

static inline void vmathV3Neg( VmathVector3 *result, const VmathVector3 *vec )
{
    result->vec128 = negatef4( vec->vec128 );
}

static inline void vmathV3MulPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = spu_mul( vec0->vec128, vec1->vec128 );
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
    result = fmaxf4( spu_promote( spu_extract( vec->vec128, 1 ), 0 ), vec->vec128 );
    result = fmaxf4( spu_promote( spu_extract( vec->vec128, 2 ), 0 ), result );
    return spu_extract( result, 0 );
}

static inline void vmathV3MinPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = fminf4( vec0->vec128, vec1->vec128 );
}

static inline float vmathV3MinElem( const VmathVector3 *vec )
{
    vec_float4 result;
    result = fminf4( spu_promote( spu_extract( vec->vec128, 1 ), 0 ), vec->vec128 );
    result = fminf4( spu_promote( spu_extract( vec->vec128, 2 ), 0 ), result );
    return spu_extract( result, 0 );
}

static inline float vmathV3Sum( const VmathVector3 *vec )
{
    return
        spu_extract( vec->vec128, 0 ) +
        spu_extract( vec->vec128, 1 ) +
        spu_extract( vec->vec128, 2 );
}

static inline float vmathV3Dot( const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    return spu_extract( _vmathVfDot3( vec0->vec128, vec1->vec128 ), 0 );
}

static inline float vmathV3LengthSqr( const VmathVector3 *vec )
{
    return spu_extract( _vmathVfDot3( vec->vec128, vec->vec128 ), 0 );
}

static inline float vmathV3Length( const VmathVector3 *vec )
{
    return sqrtf( vmathV3LengthSqr( vec ) );
}

static inline void vmathV3Normalize( VmathVector3 *result, const VmathVector3 *vec )
{
    vec_float4 dot = _vmathVfDot3( vec->vec128, vec->vec128 );
    dot = spu_shuffle( dot, dot, (vec_uchar16)spu_splats(0x00010203) );
    result->vec128 = spu_mul( vec->vec128, rsqrtf4( dot ) );
}

static inline void vmathV3Cross( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->vec128 = _vmathVfCross( vec0->vec128, vec1->vec128 );
}

static inline void vmathV3Select( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1, unsigned int select1 )
{
    result->vec128 = spu_sel( vec0->vec128, vec1->vec128, spu_splats( (unsigned int)-(select1 > 0) ) );
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
    result->vec128 = (vec_float4){ _x, _y, _z, _w };
}

static inline void vmathV4MakeFromV3Scalar( VmathVector4 *result, const VmathVector3 *xyz, float _w )
{
    result->vec128 = spu_shuffle( xyz->vec128, spu_promote( _w, 0 ), _VECTORMATH_SHUF_XYZA );
}

static inline void vmathV4MakeFromV3( VmathVector4 *result, const VmathVector3 *vec )
{
    result->vec128 = spu_sel( vec->vec128, spu_splats(0.0f), (vec_uint4)spu_maskb(0x000f) );
}

static inline void vmathV4MakeFromP3( VmathVector4 *result, const VmathPoint3 *pnt )
{
    result->vec128 = spu_sel( pnt->vec128, spu_splats(1.0f), (vec_uint4)spu_maskb(0x000f) );
}

static inline void vmathV4MakeFromQ( VmathVector4 *result, const VmathQuat *quat )
{
    result->vec128 = quat->vec128;
}

static inline void vmathV4MakeFromScalar( VmathVector4 *result, float scalar )
{
    result->vec128 = spu_splats( scalar );
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
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    cosAngle = _vmathVfDot4( unitVec0->vec128, unitVec1->vec128 );
    cosAngle = spu_shuffle( cosAngle, cosAngle, shuffle_xxxx );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(_VECTORMATH_SLERP_TOL), cosAngle );
    angle = acosf4( cosAngle );
    tttt = spu_splats(t);
    oneMinusT = spu_sub( spu_splats(1.0f), tttt );
    angles = spu_sel( spu_splats(1.0f), oneMinusT, (vec_uint4)spu_maskb(0x0f00) );
    angles = spu_sel( angles, tttt, (vec_uint4)spu_maskb(0x00f0) );
    angles = spu_mul( angles, angle );
    sines = sinf4( angles );
    scales = divf4( sines, spu_shuffle( sines, sines, shuffle_xxxx ) );
    scale0 = spu_sel( oneMinusT, spu_shuffle( scales, scales, shuffle_yyyy ), selectMask );
    scale1 = spu_sel( tttt, spu_shuffle( scales, scales, shuffle_zzzz ), selectMask );
    result->vec128 = spu_madd( unitVec0->vec128, scale0, spu_mul( unitVec1->vec128, scale1 ) );
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
    result->vec128 = spu_sel( vec->vec128, result->vec128, (vec_uint4)spu_maskb(0x000f) );
}

static inline void vmathV4GetXYZ( VmathVector3 *result, const VmathVector4 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathV4SetX( VmathVector4 *result, float _x )
{
    result->vec128 = spu_insert( _x, result->vec128, 0 );
}

static inline float vmathV4GetX( const VmathVector4 *vec )
{
    return spu_extract( vec->vec128, 0 );
}

static inline void vmathV4SetY( VmathVector4 *result, float _y )
{
    result->vec128 = spu_insert( _y, result->vec128, 1 );
}

static inline float vmathV4GetY( const VmathVector4 *vec )
{
    return spu_extract( vec->vec128, 1 );
}

static inline void vmathV4SetZ( VmathVector4 *result, float _z )
{
    result->vec128 = spu_insert( _z, result->vec128, 2 );
}

static inline float vmathV4GetZ( const VmathVector4 *vec )
{
    return spu_extract( vec->vec128, 2 );
}

static inline void vmathV4SetW( VmathVector4 *result, float _w )
{
    result->vec128 = spu_insert( _w, result->vec128, 3 );
}

static inline float vmathV4GetW( const VmathVector4 *vec )
{
    return spu_extract( vec->vec128, 3 );
}

static inline void vmathV4SetElem( VmathVector4 *result, int idx, float value )
{
    result->vec128 = spu_insert( value, result->vec128, idx );
}

static inline float vmathV4GetElem( const VmathVector4 *vec, int idx )
{
    return spu_extract( vec->vec128, idx );
}

static inline void vmathV4Add( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = spu_add( vec0->vec128, vec1->vec128 );
}

static inline void vmathV4Sub( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = spu_sub( vec0->vec128, vec1->vec128 );
}

static inline void vmathV4ScalarMul( VmathVector4 *result, const VmathVector4 *vec, float scalar )
{
    result->vec128 = spu_mul( vec->vec128, spu_splats(scalar) );
}

static inline void vmathV4ScalarDiv( VmathVector4 *result, const VmathVector4 *vec, float scalar )
{
    result->vec128 = divf4( vec->vec128, spu_splats(scalar) );
}

static inline void vmathV4Neg( VmathVector4 *result, const VmathVector4 *vec )
{
    result->vec128 = negatef4( vec->vec128 );
}

static inline void vmathV4MulPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = spu_mul( vec0->vec128, vec1->vec128 );
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
    result = fmaxf4( spu_promote( spu_extract( vec->vec128, 1 ), 0 ), vec->vec128 );
    result = fmaxf4( spu_promote( spu_extract( vec->vec128, 2 ), 0 ), result );
    result = fmaxf4( spu_promote( spu_extract( vec->vec128, 3 ), 0 ), result );
    return spu_extract( result, 0 );
}

static inline void vmathV4MinPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->vec128 = fminf4( vec0->vec128, vec1->vec128 );
}

static inline float vmathV4MinElem( const VmathVector4 *vec )
{
    vec_float4 result;
    result = fminf4( spu_promote( spu_extract( vec->vec128, 1 ), 0 ), vec->vec128 );
    result = fminf4( spu_promote( spu_extract( vec->vec128, 2 ), 0 ), result );
    result = fminf4( spu_promote( spu_extract( vec->vec128, 3 ), 0 ), result );
    return spu_extract( result, 0 );
}

static inline float vmathV4Sum( const VmathVector4 *vec )
{
    return
        spu_extract( vec->vec128, 0 ) +
        spu_extract( vec->vec128, 1 ) +
        spu_extract( vec->vec128, 2 ) +
        spu_extract( vec->vec128, 3 );
}

static inline float vmathV4Dot( const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    return spu_extract( _vmathVfDot4( vec0->vec128, vec1->vec128 ), 0 );
}

static inline float vmathV4LengthSqr( const VmathVector4 *vec )
{
    return spu_extract( _vmathVfDot4( vec->vec128, vec->vec128 ), 0 );
}

static inline float vmathV4Length( const VmathVector4 *vec )
{
    return sqrtf( vmathV4LengthSqr( vec ) );
}

static inline void vmathV4Normalize( VmathVector4 *result, const VmathVector4 *vec )
{
    vec_float4 dot = _vmathVfDot4( vec->vec128, vec->vec128 );
    result->vec128 = spu_mul( vec->vec128, rsqrtf4( dot ) );
}

static inline void vmathV4Select( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1, unsigned int select1 )
{
    result->vec128 = spu_sel( vec0->vec128, vec1->vec128, spu_splats( (unsigned int)-(select1 > 0) ) );
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
    result->vec128 = (vec_float4){ _x, _y, _z, 0.0f  };
}

static inline void vmathP3MakeFromV3( VmathPoint3 *result, const VmathVector3 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathP3MakeFromScalar( VmathPoint3 *result, float scalar )
{
    result->vec128 = spu_splats( scalar );
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
    vec_uint4 mask = (vec_uint4)spu_maskb(0x000f);
    dstVec = spu_sel(pnt->vec128, dstVec, mask);
    *quad = dstVec;
}

static inline void vmathP3LoadXYZArray( VmathPoint3 *pnt0, VmathPoint3 *pnt1, VmathPoint3 *pnt2, VmathPoint3 *pnt3, const vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz, xyz1, xyz2, xyz3;
    xyzx = threeQuads[0];
    yzxy = threeQuads[1];
    zxyz = threeQuads[2];
    xyz1 = spu_shuffle( xyzx, yzxy, _VECTORMATH_SHUF_WABC );
    xyz2 = spu_shuffle( yzxy, zxyz, _VECTORMATH_SHUF_ZWAB );
    xyz3 = spu_rlqwbyte( zxyz, 4 );
    pnt0->vec128 = xyzx;
    pnt1->vec128 = xyz1;
    pnt2->vec128 = xyz2;
    pnt3->vec128 = xyz3;
}

static inline void vmathP3StoreXYZArray( const VmathPoint3 *pnt0, const VmathPoint3 *pnt1, const VmathPoint3 *pnt2, const VmathPoint3 *pnt3, vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz;
    xyzx = spu_shuffle( pnt0->vec128, pnt1->vec128, _VECTORMATH_SHUF_XYZA );
    yzxy = spu_shuffle( pnt1->vec128, pnt2->vec128, _VECTORMATH_SHUF_YZAB );
    zxyz = spu_shuffle( pnt2->vec128, pnt3->vec128, _VECTORMATH_SHUF_ZABC );
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
    result->vec128 = spu_insert( _x, result->vec128, 0 );
}

static inline float vmathP3GetX( const VmathPoint3 *pnt )
{
    return spu_extract( pnt->vec128, 0 );
}

static inline void vmathP3SetY( VmathPoint3 *result, float _y )
{
    result->vec128 = spu_insert( _y, result->vec128, 1 );
}

static inline float vmathP3GetY( const VmathPoint3 *pnt )
{
    return spu_extract( pnt->vec128, 1 );
}

static inline void vmathP3SetZ( VmathPoint3 *result, float _z )
{
    result->vec128 = spu_insert( _z, result->vec128, 2 );
}

static inline float vmathP3GetZ( const VmathPoint3 *pnt )
{
    return spu_extract( pnt->vec128, 2 );
}

static inline void vmathP3SetElem( VmathPoint3 *result, int idx, float value )
{
    result->vec128 = spu_insert( value, result->vec128, idx );
}

static inline float vmathP3GetElem( const VmathPoint3 *pnt, int idx )
{
    return spu_extract( pnt->vec128, idx );
}

static inline void vmathP3Sub( VmathVector3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = spu_sub( pnt0->vec128, pnt1->vec128 );
}

static inline void vmathP3AddV3( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *vec1 )
{
    result->vec128 = spu_add( pnt->vec128, vec1->vec128 );
}

static inline void vmathP3SubV3( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *vec1 )
{
    result->vec128 = spu_sub( pnt->vec128, vec1->vec128 );
}

static inline void vmathP3MulPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = spu_mul( pnt0->vec128, pnt1->vec128 );
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
    result = fmaxf4( spu_promote( spu_extract( pnt->vec128, 1 ), 0 ), pnt->vec128 );
    result = fmaxf4( spu_promote( spu_extract( pnt->vec128, 2 ), 0 ), result );
    return spu_extract( result, 0 );
}

static inline void vmathP3MinPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->vec128 = fminf4( pnt0->vec128, pnt1->vec128 );
}

static inline float vmathP3MinElem( const VmathPoint3 *pnt )
{
    vec_float4 result;
    result = fminf4( spu_promote( spu_extract( pnt->vec128, 1 ), 0 ), pnt->vec128 );
    result = fminf4( spu_promote( spu_extract( pnt->vec128, 2 ), 0 ), result );
    return spu_extract( result, 0 );
}

static inline float vmathP3Sum( const VmathPoint3 *pnt )
{
    return
        spu_extract( pnt->vec128, 0 ) +
        spu_extract( pnt->vec128, 1 ) +
        spu_extract( pnt->vec128, 2 );
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
    return spu_extract( _vmathVfDot3( pnt->vec128, unitVec->vec128 ), 0 );
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
    result->vec128 = spu_sel( pnt0->vec128, pnt1->vec128, spu_splats( (unsigned int)-(select1 > 0) ) );
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
