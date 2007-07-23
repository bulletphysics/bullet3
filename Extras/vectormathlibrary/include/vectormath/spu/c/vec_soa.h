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

#ifndef _VECTORMATH_VEC_SOA_C_H
#define _VECTORMATH_VEC_SOA_C_H
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

static inline void vmathSoaV3Copy( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
}

static inline void vmathSoaV3MakeFromElems( VmathSoaVector3 *result, vec_float4 _x, vec_float4 _y, vec_float4 _z )
{
    result->x = _x;
    result->y = _y;
    result->z = _z;
}

static inline void vmathSoaV3MakeFromP3( VmathSoaVector3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = pnt->x;
    result->y = pnt->y;
    result->z = pnt->z;
}

static inline void vmathSoaV3MakeFromScalar( VmathSoaVector3 *result, vec_float4 scalar )
{
    result->x = scalar;
    result->y = scalar;
    result->z = scalar;
}

static inline void vmathSoaV3MakeFromAos( VmathSoaVector3 *result, const VmathVector3 *vec )
{
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    vec_float4 vec128 = vec->vec128;
    result->x = spu_shuffle( vec128, vec128, shuffle_xxxx );
    result->y = spu_shuffle( vec128, vec128, shuffle_yyyy );
    result->z = spu_shuffle( vec128, vec128, shuffle_zzzz );
}

static inline void vmathSoaV3MakeFrom4Aos( VmathSoaVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1, const VmathVector3 *vec2, const VmathVector3 *vec3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = spu_shuffle( vec0->vec128, vec2->vec128, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( vec1->vec128, vec3->vec128, _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( vec0->vec128, vec2->vec128, _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( vec1->vec128, vec3->vec128, _VECTORMATH_SHUF_ZCWD );
    result->x = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB );
    result->y = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD );
    result->z = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB );
}

static inline void vmathSoaV3MakeXAxis( VmathSoaVector3 *result )
{
    vmathSoaV3MakeFromElems( result, spu_splats(1.0f), spu_splats(0.0f), spu_splats(0.0f) );
}

static inline void vmathSoaV3MakeYAxis( VmathSoaVector3 *result )
{
    vmathSoaV3MakeFromElems( result, spu_splats(0.0f), spu_splats(1.0f), spu_splats(0.0f) );
}

static inline void vmathSoaV3MakeZAxis( VmathSoaVector3 *result )
{
    vmathSoaV3MakeFromElems( result, spu_splats(0.0f), spu_splats(0.0f), spu_splats(1.0f) );
}

static inline void vmathSoaV3Lerp( VmathSoaVector3 *result, vec_float4 t, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    VmathSoaVector3 tmpV3_0, tmpV3_1;
    vmathSoaV3Sub( &tmpV3_0, vec1, vec0 );
    vmathSoaV3ScalarMul( &tmpV3_1, &tmpV3_0, t );
    vmathSoaV3Add( result, vec0, &tmpV3_1 );
}

static inline void vmathSoaV3Slerp( VmathSoaVector3 *result, vec_float4 t, const VmathSoaVector3 *unitVec0, const VmathSoaVector3 *unitVec1 )
{
    VmathSoaVector3 tmpV3_0, tmpV3_1;
    vec_float4 recipSinAngle, scale0, scale1, cosAngle, angle;
    vec_uint4 selectMask;
    cosAngle = vmathSoaV3Dot( unitVec0, unitVec1 );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(_VECTORMATH_SLERP_TOL), cosAngle );
    angle = acosf4( cosAngle );
    recipSinAngle = recipf4( sinf4( angle ) );
    scale0 = spu_sel( spu_sub( spu_splats(1.0f), t ), spu_mul( sinf4( spu_mul( spu_sub( spu_splats(1.0f), t ), angle ) ), recipSinAngle ), selectMask );
    scale1 = spu_sel( t, spu_mul( sinf4( spu_mul( t, angle ) ), recipSinAngle ), selectMask );
    vmathSoaV3ScalarMul( &tmpV3_0, unitVec0, scale0 );
    vmathSoaV3ScalarMul( &tmpV3_1, unitVec1, scale1 );
    vmathSoaV3Add( result, &tmpV3_0, &tmpV3_1 );
}

static inline void vmathSoaV3Get4Aos( const VmathSoaVector3 *vec, VmathVector3 *result0, VmathVector3 *result1, VmathVector3 *result2, VmathVector3 *result3 )
{
    vec_float4 tmp0, tmp1;
    tmp0 = spu_shuffle( vec->x, vec->z, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( vec->x, vec->z, _VECTORMATH_SHUF_ZCWD );
    vmathV3MakeFrom128( result0, spu_shuffle( tmp0, vec->y, _VECTORMATH_SHUF_XAYB ) );
    vmathV3MakeFrom128( result1, spu_shuffle( tmp0, vec->y, _VECTORMATH_SHUF_ZBW0 ) );
    vmathV3MakeFrom128( result2, spu_shuffle( tmp1, vec->y, _VECTORMATH_SHUF_XCY0 ) );
    vmathV3MakeFrom128( result3, spu_shuffle( tmp1, vec->y, _VECTORMATH_SHUF_ZDW0 ) );
}

static inline void vmathSoaV3LoadXYZArray( VmathSoaVector3 *vec, const vec_float4 *threeQuads )
{
    vec_float4 xyxy, yzyz, zxzx, xyzx, yzxy, zxyz;
    xyzx = threeQuads[0];
    yzxy = threeQuads[1];
    zxyz = threeQuads[2];
    xyxy = spu_shuffle( xyzx, yzxy, _VECTORMATH_SHUF_XYCD );
    zxzx = spu_shuffle( zxyz, xyzx, _VECTORMATH_SHUF_XYCD );
    yzyz = spu_shuffle( yzxy, zxyz, _VECTORMATH_SHUF_XYCD );
    vmathSoaV3SetX( vec, spu_shuffle( xyxy, zxzx, _VECTORMATH_SHUF_XDZB ) );
    vmathSoaV3SetY( vec, spu_shuffle( xyxy, yzyz, _VECTORMATH_SHUF_YAWC ) );
    vmathSoaV3SetZ( vec, spu_shuffle( zxzx, yzyz, _VECTORMATH_SHUF_ZBXD ) );
}

static inline void vmathSoaV3StoreXYZArray( const VmathSoaVector3 *vec, vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz, xyxy, zxzx, yzyz;
    xyxy = spu_shuffle( vec->x, vec->y, _VECTORMATH_SHUF_XAZC );
    zxzx = spu_shuffle( vec->z, vec->x, _VECTORMATH_SHUF_ZDXB );
    yzyz = spu_shuffle( vec->y, vec->z, _VECTORMATH_SHUF_YBWD );
    xyzx = spu_shuffle( xyxy, zxzx, _VECTORMATH_SHUF_XYCD );
    yzxy = spu_shuffle( yzyz, xyxy, _VECTORMATH_SHUF_XYCD );
    zxyz = spu_shuffle( zxzx, yzyz, _VECTORMATH_SHUF_XYCD );
    threeQuads[0] = xyzx;
    threeQuads[1] = yzxy;
    threeQuads[2] = zxyz;
}

static inline void vmathSoaV3StoreHalfFloats( const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1, vec_ushort8 *threeQuads )
{
    vec_float4 xyz0[3];
    vec_float4 xyz1[3];
    vmathSoaV3StoreXYZArray( vec0, xyz0 );
    vmathSoaV3StoreXYZArray( vec1, xyz1 );
    threeQuads[0] = _vmath2VfToHalfFloats(xyz0[0], xyz0[1]);
    threeQuads[1] = _vmath2VfToHalfFloats(xyz0[2], xyz1[0]);
    threeQuads[2] = _vmath2VfToHalfFloats(xyz1[1], xyz1[2]);
}

static inline void vmathSoaV3SetX( VmathSoaVector3 *result, vec_float4 _x )
{
    result->x = _x;
}

static inline vec_float4 vmathSoaV3GetX( const VmathSoaVector3 *vec )
{
    return vec->x;
}

static inline void vmathSoaV3SetY( VmathSoaVector3 *result, vec_float4 _y )
{
    result->y = _y;
}

static inline vec_float4 vmathSoaV3GetY( const VmathSoaVector3 *vec )
{
    return vec->y;
}

static inline void vmathSoaV3SetZ( VmathSoaVector3 *result, vec_float4 _z )
{
    result->z = _z;
}

static inline vec_float4 vmathSoaV3GetZ( const VmathSoaVector3 *vec )
{
    return vec->z;
}

static inline void vmathSoaV3SetElem( VmathSoaVector3 *result, int idx, vec_float4 value )
{
    *(&result->x + idx) = value;
}

static inline vec_float4 vmathSoaV3GetElem( const VmathSoaVector3 *vec, int idx )
{
    return *(&vec->x + idx);
}

static inline void vmathSoaV3Add( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = spu_add( vec0->x, vec1->x );
    result->y = spu_add( vec0->y, vec1->y );
    result->z = spu_add( vec0->z, vec1->z );
}

static inline void vmathSoaV3Sub( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = spu_sub( vec0->x, vec1->x );
    result->y = spu_sub( vec0->y, vec1->y );
    result->z = spu_sub( vec0->z, vec1->z );
}

static inline void vmathSoaV3AddP3( VmathSoaPoint3 *result, const VmathSoaVector3 *vec, const VmathSoaPoint3 *pnt1 )
{
    result->x = spu_add( vec->x, pnt1->x );
    result->y = spu_add( vec->y, pnt1->y );
    result->z = spu_add( vec->z, pnt1->z );
}

static inline void vmathSoaV3ScalarMul( VmathSoaVector3 *result, const VmathSoaVector3 *vec, vec_float4 scalar )
{
    result->x = spu_mul( vec->x, scalar );
    result->y = spu_mul( vec->y, scalar );
    result->z = spu_mul( vec->z, scalar );
}

static inline void vmathSoaV3ScalarDiv( VmathSoaVector3 *result, const VmathSoaVector3 *vec, vec_float4 scalar )
{
    result->x = divf4( vec->x, scalar );
    result->y = divf4( vec->y, scalar );
    result->z = divf4( vec->z, scalar );
}

static inline void vmathSoaV3Neg( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = negatef4( vec->x );
    result->y = negatef4( vec->y );
    result->z = negatef4( vec->z );
}

static inline void vmathSoaV3MulPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = spu_mul( vec0->x, vec1->x );
    result->y = spu_mul( vec0->y, vec1->y );
    result->z = spu_mul( vec0->z, vec1->z );
}

static inline void vmathSoaV3DivPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = divf4( vec0->x, vec1->x );
    result->y = divf4( vec0->y, vec1->y );
    result->z = divf4( vec0->z, vec1->z );
}

static inline void vmathSoaV3RecipPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = recipf4( vec->x );
    result->y = recipf4( vec->y );
    result->z = recipf4( vec->z );
}

static inline void vmathSoaV3SqrtPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = sqrtf4( vec->x );
    result->y = sqrtf4( vec->y );
    result->z = sqrtf4( vec->z );
}

static inline void vmathSoaV3RsqrtPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = rsqrtf4( vec->x );
    result->y = rsqrtf4( vec->y );
    result->z = rsqrtf4( vec->z );
}

static inline void vmathSoaV3AbsPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = fabsf4( vec->x );
    result->y = fabsf4( vec->y );
    result->z = fabsf4( vec->z );
}

static inline void vmathSoaV3CopySignPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = copysignf4( vec0->x, vec1->x );
    result->y = copysignf4( vec0->y, vec1->y );
    result->z = copysignf4( vec0->z, vec1->z );
}

static inline void vmathSoaV3MaxPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = fmaxf4( vec0->x, vec1->x );
    result->y = fmaxf4( vec0->y, vec1->y );
    result->z = fmaxf4( vec0->z, vec1->z );
}

static inline vec_float4 vmathSoaV3MaxElem( const VmathSoaVector3 *vec )
{
    vec_float4 result;
    result = fmaxf4( vec->x, vec->y );
    result = fmaxf4( vec->z, result );
    return result;
}

static inline void vmathSoaV3MinPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = fminf4( vec0->x, vec1->x );
    result->y = fminf4( vec0->y, vec1->y );
    result->z = fminf4( vec0->z, vec1->z );
}

static inline vec_float4 vmathSoaV3MinElem( const VmathSoaVector3 *vec )
{
    vec_float4 result;
    result = fminf4( vec->x, vec->y );
    result = fminf4( vec->z, result );
    return result;
}

static inline vec_float4 vmathSoaV3Sum( const VmathSoaVector3 *vec )
{
    vec_float4 result;
    result = spu_add( vec->x, vec->y );
    result = spu_add( result, vec->z );
    return result;
}

static inline vec_float4 vmathSoaV3Dot( const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    vec_float4 result;
    result = spu_mul( vec0->x, vec1->x );
    result = spu_add( result, spu_mul( vec0->y, vec1->y ) );
    result = spu_add( result, spu_mul( vec0->z, vec1->z ) );
    return result;
}

static inline vec_float4 vmathSoaV3LengthSqr( const VmathSoaVector3 *vec )
{
    vec_float4 result;
    result = spu_mul( vec->x, vec->x );
    result = spu_add( result, spu_mul( vec->y, vec->y ) );
    result = spu_add( result, spu_mul( vec->z, vec->z ) );
    return result;
}

static inline vec_float4 vmathSoaV3Length( const VmathSoaVector3 *vec )
{
    return sqrtf4( vmathSoaV3LengthSqr( vec ) );
}

static inline void vmathSoaV3Normalize( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    vec_float4 lenSqr, lenInv;
    lenSqr = vmathSoaV3LengthSqr( vec );
    lenInv = rsqrtf4( lenSqr );
    result->x = spu_mul( vec->x, lenInv );
    result->y = spu_mul( vec->y, lenInv );
    result->z = spu_mul( vec->z, lenInv );
}

static inline void vmathSoaV3Cross( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    vec_float4 tmpX, tmpY, tmpZ;
    tmpX = spu_sub( spu_mul( vec0->y, vec1->z ), spu_mul( vec0->z, vec1->y ) );
    tmpY = spu_sub( spu_mul( vec0->z, vec1->x ), spu_mul( vec0->x, vec1->z ) );
    tmpZ = spu_sub( spu_mul( vec0->x, vec1->y ), spu_mul( vec0->y, vec1->x ) );
    vmathSoaV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathSoaV3Select( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1, vec_uint4 select1 )
{
    result->x = spu_sel( vec0->x, vec1->x, select1 );
    result->y = spu_sel( vec0->y, vec1->y, select1 );
    result->z = spu_sel( vec0->z, vec1->z, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaV3Print( const VmathSoaVector3 *vec )
{
    VmathVector3 vec0, vec1, vec2, vec3;
    vmathSoaV3Get4Aos( vec, &vec0, &vec1, &vec2, &vec3 );
    printf("slot 0:\n");
    vmathV3Print( &vec0 );
    printf("slot 1:\n");
    vmathV3Print( &vec1 );
    printf("slot 2:\n");
    vmathV3Print( &vec2 );
    printf("slot 3:\n");
    vmathV3Print( &vec3 );
}

static inline void vmathSoaV3Prints( const VmathSoaVector3 *vec, const char *name )
{
    VmathVector3 vec0, vec1, vec2, vec3;
    printf( "%s:\n", name );
    vmathSoaV3Get4Aos( vec, &vec0, &vec1, &vec2, &vec3 );
    printf("slot 0:\n");
    vmathV3Print( &vec0 );
    printf("slot 1:\n");
    vmathV3Print( &vec1 );
    printf("slot 2:\n");
    vmathV3Print( &vec2 );
    printf("slot 3:\n");
    vmathV3Print( &vec3 );
}

#endif

static inline void vmathSoaV4Copy( VmathSoaVector4 *result, const VmathSoaVector4 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
    result->w = vec->w;
}

static inline void vmathSoaV4MakeFromElems( VmathSoaVector4 *result, vec_float4 _x, vec_float4 _y, vec_float4 _z, vec_float4 _w )
{
    result->x = _x;
    result->y = _y;
    result->z = _z;
    result->w = _w;
}

static inline void vmathSoaV4MakeFromV3Scalar( VmathSoaVector4 *result, const VmathSoaVector3 *xyz, vec_float4 _w )
{
    vmathSoaV4SetXYZ( result, xyz );
    vmathSoaV4SetW( result, _w );
}

static inline void vmathSoaV4MakeFromV3( VmathSoaVector4 *result, const VmathSoaVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
    result->w = spu_splats(0.0f);
}

static inline void vmathSoaV4MakeFromP3( VmathSoaVector4 *result, const VmathSoaPoint3 *pnt )
{
    result->x = pnt->x;
    result->y = pnt->y;
    result->z = pnt->z;
    result->w = spu_splats(1.0f);
}

static inline void vmathSoaV4MakeFromQ( VmathSoaVector4 *result, const VmathSoaQuat *quat )
{
    result->x = quat->x;
    result->y = quat->y;
    result->z = quat->z;
    result->w = quat->w;
}

static inline void vmathSoaV4MakeFromScalar( VmathSoaVector4 *result, vec_float4 scalar )
{
    result->x = scalar;
    result->y = scalar;
    result->z = scalar;
    result->w = scalar;
}

static inline void vmathSoaV4MakeFromAos( VmathSoaVector4 *result, const VmathVector4 *vec )
{
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    vec_uchar16 shuffle_wwww = (vec_uchar16)spu_splats((int)0x0c0d0e0f);
    vec_float4 vec128 = vec->vec128;
    result->x = spu_shuffle( vec128, vec128, shuffle_xxxx );
    result->y = spu_shuffle( vec128, vec128, shuffle_yyyy );
    result->z = spu_shuffle( vec128, vec128, shuffle_zzzz );
    result->w = spu_shuffle( vec128, vec128, shuffle_wwww );
}

static inline void vmathSoaV4MakeFrom4Aos( VmathSoaVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1, const VmathVector4 *vec2, const VmathVector4 *vec3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = spu_shuffle( vec0->vec128, vec2->vec128, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( vec1->vec128, vec3->vec128, _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( vec0->vec128, vec2->vec128, _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( vec1->vec128, vec3->vec128, _VECTORMATH_SHUF_ZCWD );
    result->x = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB );
    result->y = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD );
    result->z = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB );
    result->w = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_ZCWD );
}

static inline void vmathSoaV4MakeXAxis( VmathSoaVector4 *result )
{
    vmathSoaV4MakeFromElems( result, spu_splats(1.0f), spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) );
}

static inline void vmathSoaV4MakeYAxis( VmathSoaVector4 *result )
{
    vmathSoaV4MakeFromElems( result, spu_splats(0.0f), spu_splats(1.0f), spu_splats(0.0f), spu_splats(0.0f) );
}

static inline void vmathSoaV4MakeZAxis( VmathSoaVector4 *result )
{
    vmathSoaV4MakeFromElems( result, spu_splats(0.0f), spu_splats(0.0f), spu_splats(1.0f), spu_splats(0.0f) );
}

static inline void vmathSoaV4MakeWAxis( VmathSoaVector4 *result )
{
    vmathSoaV4MakeFromElems( result, spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f), spu_splats(1.0f) );
}

static inline void vmathSoaV4Lerp( VmathSoaVector4 *result, vec_float4 t, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    VmathSoaVector4 tmpV4_0, tmpV4_1;
    vmathSoaV4Sub( &tmpV4_0, vec1, vec0 );
    vmathSoaV4ScalarMul( &tmpV4_1, &tmpV4_0, t );
    vmathSoaV4Add( result, vec0, &tmpV4_1 );
}

static inline void vmathSoaV4Slerp( VmathSoaVector4 *result, vec_float4 t, const VmathSoaVector4 *unitVec0, const VmathSoaVector4 *unitVec1 )
{
    VmathSoaVector4 tmpV4_0, tmpV4_1;
    vec_float4 recipSinAngle, scale0, scale1, cosAngle, angle;
    vec_uint4 selectMask;
    cosAngle = vmathSoaV4Dot( unitVec0, unitVec1 );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(_VECTORMATH_SLERP_TOL), cosAngle );
    angle = acosf4( cosAngle );
    recipSinAngle = recipf4( sinf4( angle ) );
    scale0 = spu_sel( spu_sub( spu_splats(1.0f), t ), spu_mul( sinf4( spu_mul( spu_sub( spu_splats(1.0f), t ), angle ) ), recipSinAngle ), selectMask );
    scale1 = spu_sel( t, spu_mul( sinf4( spu_mul( t, angle ) ), recipSinAngle ), selectMask );
    vmathSoaV4ScalarMul( &tmpV4_0, unitVec0, scale0 );
    vmathSoaV4ScalarMul( &tmpV4_1, unitVec1, scale1 );
    vmathSoaV4Add( result, &tmpV4_0, &tmpV4_1 );
}

static inline void vmathSoaV4Get4Aos( const VmathSoaVector4 *vec, VmathVector4 *result0, VmathVector4 *result1, VmathVector4 *result2, VmathVector4 *result3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = spu_shuffle( vec->x, vec->z, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( vec->y, vec->w, _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( vec->x, vec->z, _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( vec->y, vec->w, _VECTORMATH_SHUF_ZCWD );
    vmathV4MakeFrom128( result0, spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB ) );
    vmathV4MakeFrom128( result1, spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD ) );
    vmathV4MakeFrom128( result2, spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB ) );
    vmathV4MakeFrom128( result3, spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_ZCWD ) );
}

static inline void vmathSoaV4StoreHalfFloats( const VmathSoaVector4 *vec, vec_ushort8 *twoQuads )
{
    VmathVector4 v0, v1, v2, v3;
    vmathSoaV4Get4Aos( vec, &v0, &v1, &v2, &v3 );
    twoQuads[0] = _vmath2VfToHalfFloats(v0.vec128, v1.vec128);
    twoQuads[1] = _vmath2VfToHalfFloats(v2.vec128, v3.vec128);
}

static inline void vmathSoaV4SetXYZ( VmathSoaVector4 *result, const VmathSoaVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
}

static inline void vmathSoaV4GetXYZ( VmathSoaVector3 *result, const VmathSoaVector4 *vec )
{
    vmathSoaV3MakeFromElems( result, vec->x, vec->y, vec->z );
}

static inline void vmathSoaV4SetX( VmathSoaVector4 *result, vec_float4 _x )
{
    result->x = _x;
}

static inline vec_float4 vmathSoaV4GetX( const VmathSoaVector4 *vec )
{
    return vec->x;
}

static inline void vmathSoaV4SetY( VmathSoaVector4 *result, vec_float4 _y )
{
    result->y = _y;
}

static inline vec_float4 vmathSoaV4GetY( const VmathSoaVector4 *vec )
{
    return vec->y;
}

static inline void vmathSoaV4SetZ( VmathSoaVector4 *result, vec_float4 _z )
{
    result->z = _z;
}

static inline vec_float4 vmathSoaV4GetZ( const VmathSoaVector4 *vec )
{
    return vec->z;
}

static inline void vmathSoaV4SetW( VmathSoaVector4 *result, vec_float4 _w )
{
    result->w = _w;
}

static inline vec_float4 vmathSoaV4GetW( const VmathSoaVector4 *vec )
{
    return vec->w;
}

static inline void vmathSoaV4SetElem( VmathSoaVector4 *result, int idx, vec_float4 value )
{
    *(&result->x + idx) = value;
}

static inline vec_float4 vmathSoaV4GetElem( const VmathSoaVector4 *vec, int idx )
{
    return *(&vec->x + idx);
}

static inline void vmathSoaV4Add( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    result->x = spu_add( vec0->x, vec1->x );
    result->y = spu_add( vec0->y, vec1->y );
    result->z = spu_add( vec0->z, vec1->z );
    result->w = spu_add( vec0->w, vec1->w );
}

static inline void vmathSoaV4Sub( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    result->x = spu_sub( vec0->x, vec1->x );
    result->y = spu_sub( vec0->y, vec1->y );
    result->z = spu_sub( vec0->z, vec1->z );
    result->w = spu_sub( vec0->w, vec1->w );
}

static inline void vmathSoaV4ScalarMul( VmathSoaVector4 *result, const VmathSoaVector4 *vec, vec_float4 scalar )
{
    result->x = spu_mul( vec->x, scalar );
    result->y = spu_mul( vec->y, scalar );
    result->z = spu_mul( vec->z, scalar );
    result->w = spu_mul( vec->w, scalar );
}

static inline void vmathSoaV4ScalarDiv( VmathSoaVector4 *result, const VmathSoaVector4 *vec, vec_float4 scalar )
{
    result->x = divf4( vec->x, scalar );
    result->y = divf4( vec->y, scalar );
    result->z = divf4( vec->z, scalar );
    result->w = divf4( vec->w, scalar );
}

static inline void vmathSoaV4Neg( VmathSoaVector4 *result, const VmathSoaVector4 *vec )
{
    result->x = negatef4( vec->x );
    result->y = negatef4( vec->y );
    result->z = negatef4( vec->z );
    result->w = negatef4( vec->w );
}

static inline void vmathSoaV4MulPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    result->x = spu_mul( vec0->x, vec1->x );
    result->y = spu_mul( vec0->y, vec1->y );
    result->z = spu_mul( vec0->z, vec1->z );
    result->w = spu_mul( vec0->w, vec1->w );
}

static inline void vmathSoaV4DivPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    result->x = divf4( vec0->x, vec1->x );
    result->y = divf4( vec0->y, vec1->y );
    result->z = divf4( vec0->z, vec1->z );
    result->w = divf4( vec0->w, vec1->w );
}

static inline void vmathSoaV4RecipPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec )
{
    result->x = recipf4( vec->x );
    result->y = recipf4( vec->y );
    result->z = recipf4( vec->z );
    result->w = recipf4( vec->w );
}

static inline void vmathSoaV4SqrtPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec )
{
    result->x = sqrtf4( vec->x );
    result->y = sqrtf4( vec->y );
    result->z = sqrtf4( vec->z );
    result->w = sqrtf4( vec->w );
}

static inline void vmathSoaV4RsqrtPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec )
{
    result->x = rsqrtf4( vec->x );
    result->y = rsqrtf4( vec->y );
    result->z = rsqrtf4( vec->z );
    result->w = rsqrtf4( vec->w );
}

static inline void vmathSoaV4AbsPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec )
{
    result->x = fabsf4( vec->x );
    result->y = fabsf4( vec->y );
    result->z = fabsf4( vec->z );
    result->w = fabsf4( vec->w );
}

static inline void vmathSoaV4CopySignPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    result->x = copysignf4( vec0->x, vec1->x );
    result->y = copysignf4( vec0->y, vec1->y );
    result->z = copysignf4( vec0->z, vec1->z );
    result->w = copysignf4( vec0->w, vec1->w );
}

static inline void vmathSoaV4MaxPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    result->x = fmaxf4( vec0->x, vec1->x );
    result->y = fmaxf4( vec0->y, vec1->y );
    result->z = fmaxf4( vec0->z, vec1->z );
    result->w = fmaxf4( vec0->w, vec1->w );
}

static inline vec_float4 vmathSoaV4MaxElem( const VmathSoaVector4 *vec )
{
    vec_float4 result;
    result = fmaxf4( vec->x, vec->y );
    result = fmaxf4( vec->z, result );
    result = fmaxf4( vec->w, result );
    return result;
}

static inline void vmathSoaV4MinPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    result->x = fminf4( vec0->x, vec1->x );
    result->y = fminf4( vec0->y, vec1->y );
    result->z = fminf4( vec0->z, vec1->z );
    result->w = fminf4( vec0->w, vec1->w );
}

static inline vec_float4 vmathSoaV4MinElem( const VmathSoaVector4 *vec )
{
    vec_float4 result;
    result = fminf4( vec->x, vec->y );
    result = fminf4( vec->z, result );
    result = fminf4( vec->w, result );
    return result;
}

static inline vec_float4 vmathSoaV4Sum( const VmathSoaVector4 *vec )
{
    vec_float4 result;
    result = spu_add( vec->x, vec->y );
    result = spu_add( result, vec->z );
    result = spu_add( result, vec->w );
    return result;
}

static inline vec_float4 vmathSoaV4Dot( const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    vec_float4 result;
    result = spu_mul( vec0->x, vec1->x );
    result = spu_add( result, spu_mul( vec0->y, vec1->y ) );
    result = spu_add( result, spu_mul( vec0->z, vec1->z ) );
    result = spu_add( result, spu_mul( vec0->w, vec1->w ) );
    return result;
}

static inline vec_float4 vmathSoaV4LengthSqr( const VmathSoaVector4 *vec )
{
    vec_float4 result;
    result = spu_mul( vec->x, vec->x );
    result = spu_add( result, spu_mul( vec->y, vec->y ) );
    result = spu_add( result, spu_mul( vec->z, vec->z ) );
    result = spu_add( result, spu_mul( vec->w, vec->w ) );
    return result;
}

static inline vec_float4 vmathSoaV4Length( const VmathSoaVector4 *vec )
{
    return sqrtf4( vmathSoaV4LengthSqr( vec ) );
}

static inline void vmathSoaV4Normalize( VmathSoaVector4 *result, const VmathSoaVector4 *vec )
{
    vec_float4 lenSqr, lenInv;
    lenSqr = vmathSoaV4LengthSqr( vec );
    lenInv = rsqrtf4( lenSqr );
    result->x = spu_mul( vec->x, lenInv );
    result->y = spu_mul( vec->y, lenInv );
    result->z = spu_mul( vec->z, lenInv );
    result->w = spu_mul( vec->w, lenInv );
}

static inline void vmathSoaV4Select( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1, vec_uint4 select1 )
{
    result->x = spu_sel( vec0->x, vec1->x, select1 );
    result->y = spu_sel( vec0->y, vec1->y, select1 );
    result->z = spu_sel( vec0->z, vec1->z, select1 );
    result->w = spu_sel( vec0->w, vec1->w, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaV4Print( const VmathSoaVector4 *vec )
{
    VmathVector4 vec0, vec1, vec2, vec3;
    vmathSoaV4Get4Aos( vec, &vec0, &vec1, &vec2, &vec3 );
    printf("slot 0:\n");
    vmathV4Print( &vec0 );
    printf("slot 1:\n");
    vmathV4Print( &vec1 );
    printf("slot 2:\n");
    vmathV4Print( &vec2 );
    printf("slot 3:\n");
    vmathV4Print( &vec3 );
}

static inline void vmathSoaV4Prints( const VmathSoaVector4 *vec, const char *name )
{
    VmathVector4 vec0, vec1, vec2, vec3;
    printf( "%s:\n", name );
    vmathSoaV4Get4Aos( vec, &vec0, &vec1, &vec2, &vec3 );
    printf("slot 0:\n");
    vmathV4Print( &vec0 );
    printf("slot 1:\n");
    vmathV4Print( &vec1 );
    printf("slot 2:\n");
    vmathV4Print( &vec2 );
    printf("slot 3:\n");
    vmathV4Print( &vec3 );
}

#endif

static inline void vmathSoaP3Copy( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = pnt->x;
    result->y = pnt->y;
    result->z = pnt->z;
}

static inline void vmathSoaP3MakeFromElems( VmathSoaPoint3 *result, vec_float4 _x, vec_float4 _y, vec_float4 _z )
{
    result->x = _x;
    result->y = _y;
    result->z = _z;
}

static inline void vmathSoaP3MakeFromV3( VmathSoaPoint3 *result, const VmathSoaVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
}

static inline void vmathSoaP3MakeFromScalar( VmathSoaPoint3 *result, vec_float4 scalar )
{
    result->x = scalar;
    result->y = scalar;
    result->z = scalar;
}

static inline void vmathSoaP3MakeFromAos( VmathSoaPoint3 *result, const VmathPoint3 *pnt )
{
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    vec_float4 vec128 = pnt->vec128;
    result->x = spu_shuffle( vec128, vec128, shuffle_xxxx );
    result->y = spu_shuffle( vec128, vec128, shuffle_yyyy );
    result->z = spu_shuffle( vec128, vec128, shuffle_zzzz );
}

static inline void vmathSoaP3MakeFrom4Aos( VmathSoaPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1, const VmathPoint3 *pnt2, const VmathPoint3 *pnt3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = spu_shuffle( pnt0->vec128, pnt2->vec128, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( pnt1->vec128, pnt3->vec128, _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( pnt0->vec128, pnt2->vec128, _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( pnt1->vec128, pnt3->vec128, _VECTORMATH_SHUF_ZCWD );
    result->x = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB );
    result->y = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD );
    result->z = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB );
}

static inline void vmathSoaP3Lerp( VmathSoaPoint3 *result, vec_float4 t, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    VmathSoaVector3 tmpV3_0, tmpV3_1;
    vmathSoaP3Sub( &tmpV3_0, pnt1, pnt0 );
    vmathSoaV3ScalarMul( &tmpV3_1, &tmpV3_0, t );
    vmathSoaP3AddV3( result, pnt0, &tmpV3_1 );
}

static inline void vmathSoaP3Get4Aos( const VmathSoaPoint3 *pnt, VmathPoint3 *result0, VmathPoint3 *result1, VmathPoint3 *result2, VmathPoint3 *result3 )
{
    vec_float4 tmp0, tmp1;
    tmp0 = spu_shuffle( pnt->x, pnt->z, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( pnt->x, pnt->z, _VECTORMATH_SHUF_ZCWD );
    vmathP3MakeFrom128( result0, spu_shuffle( tmp0, pnt->y, _VECTORMATH_SHUF_XAYB ) );
    vmathP3MakeFrom128( result1, spu_shuffle( tmp0, pnt->y, _VECTORMATH_SHUF_ZBW0 ) );
    vmathP3MakeFrom128( result2, spu_shuffle( tmp1, pnt->y, _VECTORMATH_SHUF_XCY0 ) );
    vmathP3MakeFrom128( result3, spu_shuffle( tmp1, pnt->y, _VECTORMATH_SHUF_ZDW0 ) );
}

static inline void vmathSoaP3LoadXYZArray( VmathSoaPoint3 *vec, const vec_float4 *threeQuads )
{
    vec_float4 xyxy, yzyz, zxzx, xyzx, yzxy, zxyz;
    xyzx = threeQuads[0];
    yzxy = threeQuads[1];
    zxyz = threeQuads[2];
    xyxy = spu_shuffle( xyzx, yzxy, _VECTORMATH_SHUF_XYCD );
    zxzx = spu_shuffle( zxyz, xyzx, _VECTORMATH_SHUF_XYCD );
    yzyz = spu_shuffle( yzxy, zxyz, _VECTORMATH_SHUF_XYCD );
    vmathSoaP3SetX( vec, spu_shuffle( xyxy, zxzx, _VECTORMATH_SHUF_XDZB ) );
    vmathSoaP3SetY( vec, spu_shuffle( xyxy, yzyz, _VECTORMATH_SHUF_YAWC ) );
    vmathSoaP3SetZ( vec, spu_shuffle( zxzx, yzyz, _VECTORMATH_SHUF_ZBXD ) );
}

static inline void vmathSoaP3StoreXYZArray( const VmathSoaPoint3 *vec, vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz, xyxy, zxzx, yzyz;
    xyxy = spu_shuffle( vec->x, vec->y, _VECTORMATH_SHUF_XAZC );
    zxzx = spu_shuffle( vec->z, vec->x, _VECTORMATH_SHUF_ZDXB );
    yzyz = spu_shuffle( vec->y, vec->z, _VECTORMATH_SHUF_YBWD );
    xyzx = spu_shuffle( xyxy, zxzx, _VECTORMATH_SHUF_XYCD );
    yzxy = spu_shuffle( yzyz, xyxy, _VECTORMATH_SHUF_XYCD );
    zxyz = spu_shuffle( zxzx, yzyz, _VECTORMATH_SHUF_XYCD );
    threeQuads[0] = xyzx;
    threeQuads[1] = yzxy;
    threeQuads[2] = zxyz;
}

static inline void vmathSoaP3StoreHalfFloats( const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1, vec_ushort8 *threeQuads )
{
    vec_float4 xyz0[3];
    vec_float4 xyz1[3];
    vmathSoaP3StoreXYZArray( pnt0, xyz0 );
    vmathSoaP3StoreXYZArray( pnt1, xyz1 );
    threeQuads[0] = _vmath2VfToHalfFloats(xyz0[0], xyz0[1]);
    threeQuads[1] = _vmath2VfToHalfFloats(xyz0[2], xyz1[0]);
    threeQuads[2] = _vmath2VfToHalfFloats(xyz1[1], xyz1[2]);
}

static inline void vmathSoaP3SetX( VmathSoaPoint3 *result, vec_float4 _x )
{
    result->x = _x;
}

static inline vec_float4 vmathSoaP3GetX( const VmathSoaPoint3 *pnt )
{
    return pnt->x;
}

static inline void vmathSoaP3SetY( VmathSoaPoint3 *result, vec_float4 _y )
{
    result->y = _y;
}

static inline vec_float4 vmathSoaP3GetY( const VmathSoaPoint3 *pnt )
{
    return pnt->y;
}

static inline void vmathSoaP3SetZ( VmathSoaPoint3 *result, vec_float4 _z )
{
    result->z = _z;
}

static inline vec_float4 vmathSoaP3GetZ( const VmathSoaPoint3 *pnt )
{
    return pnt->z;
}

static inline void vmathSoaP3SetElem( VmathSoaPoint3 *result, int idx, vec_float4 value )
{
    *(&result->x + idx) = value;
}

static inline vec_float4 vmathSoaP3GetElem( const VmathSoaPoint3 *pnt, int idx )
{
    return *(&pnt->x + idx);
}

static inline void vmathSoaP3Sub( VmathSoaVector3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    result->x = spu_sub( pnt0->x, pnt1->x );
    result->y = spu_sub( pnt0->y, pnt1->y );
    result->z = spu_sub( pnt0->z, pnt1->z );
}

static inline void vmathSoaP3AddV3( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, const VmathSoaVector3 *vec1 )
{
    result->x = spu_add( pnt->x, vec1->x );
    result->y = spu_add( pnt->y, vec1->y );
    result->z = spu_add( pnt->z, vec1->z );
}

static inline void vmathSoaP3SubV3( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, const VmathSoaVector3 *vec1 )
{
    result->x = spu_sub( pnt->x, vec1->x );
    result->y = spu_sub( pnt->y, vec1->y );
    result->z = spu_sub( pnt->z, vec1->z );
}

static inline void vmathSoaP3MulPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    result->x = spu_mul( pnt0->x, pnt1->x );
    result->y = spu_mul( pnt0->y, pnt1->y );
    result->z = spu_mul( pnt0->z, pnt1->z );
}

static inline void vmathSoaP3DivPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    result->x = divf4( pnt0->x, pnt1->x );
    result->y = divf4( pnt0->y, pnt1->y );
    result->z = divf4( pnt0->z, pnt1->z );
}

static inline void vmathSoaP3RecipPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = recipf4( pnt->x );
    result->y = recipf4( pnt->y );
    result->z = recipf4( pnt->z );
}

static inline void vmathSoaP3SqrtPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = sqrtf4( pnt->x );
    result->y = sqrtf4( pnt->y );
    result->z = sqrtf4( pnt->z );
}

static inline void vmathSoaP3RsqrtPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = rsqrtf4( pnt->x );
    result->y = rsqrtf4( pnt->y );
    result->z = rsqrtf4( pnt->z );
}

static inline void vmathSoaP3AbsPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = fabsf4( pnt->x );
    result->y = fabsf4( pnt->y );
    result->z = fabsf4( pnt->z );
}

static inline void vmathSoaP3CopySignPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    result->x = copysignf4( pnt0->x, pnt1->x );
    result->y = copysignf4( pnt0->y, pnt1->y );
    result->z = copysignf4( pnt0->z, pnt1->z );
}

static inline void vmathSoaP3MaxPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    result->x = fmaxf4( pnt0->x, pnt1->x );
    result->y = fmaxf4( pnt0->y, pnt1->y );
    result->z = fmaxf4( pnt0->z, pnt1->z );
}

static inline vec_float4 vmathSoaP3MaxElem( const VmathSoaPoint3 *pnt )
{
    vec_float4 result;
    result = fmaxf4( pnt->x, pnt->y );
    result = fmaxf4( pnt->z, result );
    return result;
}

static inline void vmathSoaP3MinPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    result->x = fminf4( pnt0->x, pnt1->x );
    result->y = fminf4( pnt0->y, pnt1->y );
    result->z = fminf4( pnt0->z, pnt1->z );
}

static inline vec_float4 vmathSoaP3MinElem( const VmathSoaPoint3 *pnt )
{
    vec_float4 result;
    result = fminf4( pnt->x, pnt->y );
    result = fminf4( pnt->z, result );
    return result;
}

static inline vec_float4 vmathSoaP3Sum( const VmathSoaPoint3 *pnt )
{
    vec_float4 result;
    result = spu_add( pnt->x, pnt->y );
    result = spu_add( result, pnt->z );
    return result;
}

static inline void vmathSoaP3Scale( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, vec_float4 scaleVal )
{
    VmathSoaPoint3 tmpP3_0;
    vmathSoaP3MakeFromScalar( &tmpP3_0, scaleVal );
    vmathSoaP3MulPerElem( result, pnt, &tmpP3_0 );
}

static inline void vmathSoaP3NonUniformScale( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, const VmathSoaVector3 *scaleVec )
{
    VmathSoaPoint3 tmpP3_0;
    vmathSoaP3MakeFromV3( &tmpP3_0, scaleVec );
    vmathSoaP3MulPerElem( result, pnt, &tmpP3_0 );
}

static inline vec_float4 vmathSoaP3Projection( const VmathSoaPoint3 *pnt, const VmathSoaVector3 *unitVec )
{
    vec_float4 result;
    result = spu_mul( pnt->x, unitVec->x );
    result = spu_add( result, spu_mul( pnt->y, unitVec->y ) );
    result = spu_add( result, spu_mul( pnt->z, unitVec->z ) );
    return result;
}

static inline vec_float4 vmathSoaP3DistSqrFromOrigin( const VmathSoaPoint3 *pnt )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaV3MakeFromP3( &tmpV3_0, pnt );
    return vmathSoaV3LengthSqr( &tmpV3_0 );
}

static inline vec_float4 vmathSoaP3DistFromOrigin( const VmathSoaPoint3 *pnt )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaV3MakeFromP3( &tmpV3_0, pnt );
    return vmathSoaV3Length( &tmpV3_0 );
}

static inline vec_float4 vmathSoaP3DistSqr( const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaP3Sub( &tmpV3_0, pnt1, pnt0 );
    return vmathSoaV3LengthSqr( &tmpV3_0 );
}

static inline vec_float4 vmathSoaP3Dist( const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaP3Sub( &tmpV3_0, pnt1, pnt0 );
    return vmathSoaV3Length( &tmpV3_0 );
}

static inline void vmathSoaP3Select( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1, vec_uint4 select1 )
{
    result->x = spu_sel( pnt0->x, pnt1->x, select1 );
    result->y = spu_sel( pnt0->y, pnt1->y, select1 );
    result->z = spu_sel( pnt0->z, pnt1->z, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaP3Print( const VmathSoaPoint3 *pnt )
{
    VmathPoint3 vec0, vec1, vec2, vec3;
    vmathSoaP3Get4Aos( pnt, &vec0, &vec1, &vec2, &vec3 );
    printf("slot 0:\n");
    vmathP3Print( &vec0 );
    printf("slot 1:\n");
    vmathP3Print( &vec1 );
    printf("slot 2:\n");
    vmathP3Print( &vec2 );
    printf("slot 3:\n");
    vmathP3Print( &vec3 );
}

static inline void vmathSoaP3Prints( const VmathSoaPoint3 *pnt, const char *name )
{
    VmathPoint3 vec0, vec1, vec2, vec3;
    printf( "%s:\n", name );
    vmathSoaP3Get4Aos( pnt, &vec0, &vec1, &vec2, &vec3 );
    printf("slot 0:\n");
    vmathP3Print( &vec0 );
    printf("slot 1:\n");
    vmathP3Print( &vec1 );
    printf("slot 2:\n");
    vmathP3Print( &vec2 );
    printf("slot 3:\n");
    vmathP3Print( &vec3 );
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
