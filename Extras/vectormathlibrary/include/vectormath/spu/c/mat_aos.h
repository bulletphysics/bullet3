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

#ifndef _VECTORMATH_MAT_AOS_C_H
#define _VECTORMATH_MAT_AOS_C_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Constants
 * for shuffles, words are labeled [x,y,z,w] [a,b,c,d]
 */
#define _VECTORMATH_SHUF_XAYB ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_B })
#define _VECTORMATH_SHUF_ZCWD ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_D })
#define _VECTORMATH_SHUF_ZBW0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_XCY0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_XYAB ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_B })
#define _VECTORMATH_SHUF_ZWCD ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_D })
#define _VECTORMATH_SHUF_0ZB0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_0, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_0 })     
#define _VECTORMATH_SHUF_C0X0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_0, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_YA00 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_0, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_XAZC ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_C })
#define _VECTORMATH_SHUF_YXWZ ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_Z })
#define _VECTORMATH_SHUF_YBWD ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_W, _VECTORMATH_SHUF_D })
#define _VECTORMATH_SHUF_XYCX ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_X })
#define _VECTORMATH_SHUF_YCXY ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_Y })
#define _VECTORMATH_SHUF_CXYC ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_C })
#define _VECTORMATH_SHUF_ZAY0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_A, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_BZX0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_0ZYA ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_0, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_A })
#define _VECTORMATH_SHUF_Z0XB ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_0, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_B })
#define _VECTORMATH_SHUF_YX0C ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_X, _VECTORMATH_SHUF_0, _VECTORMATH_SHUF_C })
#define _VECTORMATH_SHUF_CZD0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_C, _VECTORMATH_SHUF_Z, _VECTORMATH_SHUF_D, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_SHUF_BBY0 ((vec_uchar16)(vec_uint4){ _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_B, _VECTORMATH_SHUF_Y, _VECTORMATH_SHUF_0 })
#define _VECTORMATH_PI_OVER_2 1.570796327f

/*-----------------------------------------------------------------------------
 * Definitions
 */
static inline void vmathM3Copy( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    vmathV3Copy( &result->col0, &mat->col0 );
    vmathV3Copy( &result->col1, &mat->col1 );
    vmathV3Copy( &result->col2, &mat->col2 );
}

static inline void vmathM3MakeFromScalar( VmathMatrix3 *result, float scalar )
{
    vmathV3MakeFromScalar( &result->col0, scalar );
    vmathV3MakeFromScalar( &result->col1, scalar );
    vmathV3MakeFromScalar( &result->col2, scalar );
}

static inline void vmathM3MakeFromQ( VmathMatrix3 *result, const VmathQuat *unitQuat )
{
    vec_float4 xyzw_2, wwww, yzxw, zxyw, yzxw_2, zxyw_2;
    vec_float4 tmp0, tmp1, tmp2, tmp3, tmp4, tmp5;
    vec_uchar16 shuffle_wwww = (vec_uchar16)spu_splats((int)0x0c0d0e0f);
    vec_uint4 select_x = (vec_uint4)spu_maskb(0xf000);
    vec_uint4 select_z = (vec_uint4)spu_maskb(0x00f0);
    xyzw_2 = spu_add( unitQuat->vec128, unitQuat->vec128 );
    wwww = spu_shuffle( unitQuat->vec128, unitQuat->vec128, shuffle_wwww );
    yzxw = spu_shuffle( unitQuat->vec128, unitQuat->vec128, _VECTORMATH_SHUF_YZXW );
    zxyw = spu_shuffle( unitQuat->vec128, unitQuat->vec128, _VECTORMATH_SHUF_ZXYW );
    yzxw_2 = spu_shuffle( xyzw_2, xyzw_2, _VECTORMATH_SHUF_YZXW );
    zxyw_2 = spu_shuffle( xyzw_2, xyzw_2, _VECTORMATH_SHUF_ZXYW );
    tmp0 = spu_mul( yzxw_2, wwww );
    tmp1 = spu_nmsub( yzxw, yzxw_2, spu_splats(1.0f) );
    tmp2 = spu_mul( yzxw, xyzw_2 );
    tmp0 = spu_madd( zxyw, xyzw_2, tmp0 );
    tmp1 = spu_nmsub( zxyw, zxyw_2, tmp1 );
    tmp2 = spu_nmsub( zxyw_2, wwww, tmp2 );
    tmp3 = spu_sel( tmp0, tmp1, select_x );
    tmp4 = spu_sel( tmp1, tmp2, select_x );
    tmp5 = spu_sel( tmp2, tmp0, select_x );
    result->col0.vec128 = spu_sel( tmp3, tmp2, select_z );
    result->col1.vec128 = spu_sel( tmp4, tmp0, select_z );
    result->col2.vec128 = spu_sel( tmp5, tmp1, select_z );
}

static inline void vmathM3MakeFromCols( VmathMatrix3 *result, const VmathVector3 *_col0, const VmathVector3 *_col1, const VmathVector3 *_col2 )
{
    vmathV3Copy( &result->col0, _col0 );
    vmathV3Copy( &result->col1, _col1 );
    vmathV3Copy( &result->col2, _col2 );
}

static inline void vmathM3SetCol0( VmathMatrix3 *result, const VmathVector3 *_col0 )
{
    vmathV3Copy( &result->col0, _col0 );
}

static inline void vmathM3SetCol1( VmathMatrix3 *result, const VmathVector3 *_col1 )
{
    vmathV3Copy( &result->col1, _col1 );
}

static inline void vmathM3SetCol2( VmathMatrix3 *result, const VmathVector3 *_col2 )
{
    vmathV3Copy( &result->col2, _col2 );
}

static inline void vmathM3SetCol( VmathMatrix3 *result, int col, const VmathVector3 *vec )
{
    vmathV3Copy( (&result->col0 + col), vec );
}

static inline void vmathM3SetRow( VmathMatrix3 *result, int row, const VmathVector3 *vec )
{
    vmathV3SetElem( &result->col0, row, vmathV3GetElem( vec, 0 ) );
    vmathV3SetElem( &result->col1, row, vmathV3GetElem( vec, 1 ) );
    vmathV3SetElem( &result->col2, row, vmathV3GetElem( vec, 2 ) );
}

static inline void vmathM3SetElem( VmathMatrix3 *result, int col, int row, float val )
{
    VmathVector3 tmpV3_0;
    vmathM3GetCol( &tmpV3_0, result, col );
    vmathV3SetElem( &tmpV3_0, row, val );
    vmathM3SetCol( result, col, &tmpV3_0 );
}

static inline float vmathM3GetElem( const VmathMatrix3 *mat, int col, int row )
{
    VmathVector3 tmpV3_0;
    vmathM3GetCol( &tmpV3_0, mat, col );
    return vmathV3GetElem( &tmpV3_0, row );
}

static inline void vmathM3GetCol0( VmathVector3 *result, const VmathMatrix3 *mat )
{
    vmathV3Copy( result, &mat->col0 );
}

static inline void vmathM3GetCol1( VmathVector3 *result, const VmathMatrix3 *mat )
{
    vmathV3Copy( result, &mat->col1 );
}

static inline void vmathM3GetCol2( VmathVector3 *result, const VmathMatrix3 *mat )
{
    vmathV3Copy( result, &mat->col2 );
}

static inline void vmathM3GetCol( VmathVector3 *result, const VmathMatrix3 *mat, int col )
{
    vmathV3Copy( result, (&mat->col0 + col) );
}

static inline void vmathM3GetRow( VmathVector3 *result, const VmathMatrix3 *mat, int row )
{
    vmathV3MakeFromElems( result, vmathV3GetElem( &mat->col0, row ), vmathV3GetElem( &mat->col1, row ), vmathV3GetElem( &mat->col2, row ) );
}

static inline void vmathM3Transpose( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    vec_float4 tmp0, tmp1, res0, res1, res2;
    tmp0 = spu_shuffle( mat->col0.vec128, mat->col2.vec128, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( mat->col0.vec128, mat->col2.vec128, _VECTORMATH_SHUF_ZCWD );
    res0 = spu_shuffle( tmp0, mat->col1.vec128, _VECTORMATH_SHUF_XAYB );
    res1 = spu_shuffle( tmp0, mat->col1.vec128, _VECTORMATH_SHUF_ZBW0 );
    res2 = spu_shuffle( tmp1, mat->col1.vec128, _VECTORMATH_SHUF_XCY0 );
    result->col0.vec128 = res0;
    result->col1.vec128 = res1;
    result->col2.vec128 = res2;
}

static inline void vmathM3Inverse( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3, tmp4, dot, invdet, inv0, inv1, inv2;
    tmp2 = _vmathVfCross( mat->col0.vec128, mat->col1.vec128 );
    tmp0 = _vmathVfCross( mat->col1.vec128, mat->col2.vec128 );
    tmp1 = _vmathVfCross( mat->col2.vec128, mat->col0.vec128 );
    dot = _vmathVfDot3( tmp2, mat->col2.vec128 );
    dot = spu_shuffle( dot, dot, (vec_uchar16)spu_splats(0x00010203) );
    invdet = recipf4( dot );
    tmp3 = spu_shuffle( tmp0, tmp2, _VECTORMATH_SHUF_XAYB );
    tmp4 = spu_shuffle( tmp0, tmp2, _VECTORMATH_SHUF_ZCWD );
    inv0 = spu_shuffle( tmp3, tmp1, _VECTORMATH_SHUF_XAYB );
    inv1 = spu_shuffle( tmp3, tmp1, _VECTORMATH_SHUF_ZBW0 );
    inv2 = spu_shuffle( tmp4, tmp1, _VECTORMATH_SHUF_XCY0 );
    inv0 = spu_mul( inv0, invdet );
    inv1 = spu_mul( inv1, invdet );
    inv2 = spu_mul( inv2, invdet );
    result->col0.vec128 = inv0;
    result->col1.vec128 = inv1;
    result->col2.vec128 = inv2;
}

static inline float vmathM3Determinant( const VmathMatrix3 *mat )
{
    VmathVector3 tmpV3_0;
    vmathV3Cross( &tmpV3_0, &mat->col0, &mat->col1 );
    return vmathV3Dot( &mat->col2, &tmpV3_0 );
}

static inline void vmathM3Add( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 )
{
    vmathV3Add( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV3Add( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV3Add( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathM3Sub( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 )
{
    vmathV3Sub( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV3Sub( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV3Sub( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathM3Neg( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    vmathV3Neg( &result->col0, &mat->col0 );
    vmathV3Neg( &result->col1, &mat->col1 );
    vmathV3Neg( &result->col2, &mat->col2 );
}

static inline void vmathM3AbsPerElem( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    vmathV3AbsPerElem( &result->col0, &mat->col0 );
    vmathV3AbsPerElem( &result->col1, &mat->col1 );
    vmathV3AbsPerElem( &result->col2, &mat->col2 );
}

static inline void vmathM3ScalarMul( VmathMatrix3 *result, const VmathMatrix3 *mat, float scalar )
{
    vmathV3ScalarMul( &result->col0, &mat->col0, scalar );
    vmathV3ScalarMul( &result->col1, &mat->col1, scalar );
    vmathV3ScalarMul( &result->col2, &mat->col2, scalar );
}

static inline void vmathM3MulV3( VmathVector3 *result, const VmathMatrix3 *mat, const VmathVector3 *vec )
{
    vec_float4 res;
    vec_float4 xxxx, yyyy, zzzz;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    xxxx = spu_shuffle( vec->vec128, vec->vec128, shuffle_xxxx );
    yyyy = spu_shuffle( vec->vec128, vec->vec128, shuffle_yyyy );
    zzzz = spu_shuffle( vec->vec128, vec->vec128, shuffle_zzzz );
    res = spu_mul( mat->col0.vec128, xxxx );
    res = spu_madd( mat->col1.vec128, yyyy, res );
    res = spu_madd( mat->col2.vec128, zzzz, res );
    result->vec128 = res;
}

static inline void vmathM3Mul( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 )
{
    VmathMatrix3 tmpResult;
    vmathM3MulV3( &tmpResult.col0, mat0, &mat1->col0 );
    vmathM3MulV3( &tmpResult.col1, mat0, &mat1->col1 );
    vmathM3MulV3( &tmpResult.col2, mat0, &mat1->col2 );
    vmathM3Copy( result, &tmpResult );
}

static inline void vmathM3MulPerElem( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 )
{
    vmathV3MulPerElem( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV3MulPerElem( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV3MulPerElem( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathM3MakeIdentity( VmathMatrix3 *result )
{
    vmathV3MakeXAxis( &result->col0 );
    vmathV3MakeYAxis( &result->col1 );
    vmathV3MakeZAxis( &result->col2 );
}

static inline void vmathM3MakeRotationX( VmathMatrix3 *result, float radians )
{
    vec_float4 s, c, res1, res2;
    vec_uint4 select_y, select_z;
    vec_float4 zero;
    select_y = (vec_uint4)spu_maskb(0x0f00);
    select_z = (vec_uint4)spu_maskb(0x00f0);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res1 = spu_sel( zero, c, select_y );
    res1 = spu_sel( res1, s, select_z );
    res2 = spu_sel( zero, negatef4(s), select_y );
    res2 = spu_sel( res2, c, select_z );
    vmathV3MakeXAxis( &result->col0 );
    result->col1.vec128 = res1;
    result->col2.vec128 = res2;
}

static inline void vmathM3MakeRotationY( VmathMatrix3 *result, float radians )
{
    vec_float4 s, c, res0, res2;
    vec_uint4 select_x, select_z;
    vec_float4 zero;
    select_x = (vec_uint4)spu_maskb(0xf000);
    select_z = (vec_uint4)spu_maskb(0x00f0);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res0 = spu_sel( zero, c, select_x );
    res0 = spu_sel( res0, negatef4(s), select_z );
    res2 = spu_sel( zero, s, select_x );
    res2 = spu_sel( res2, c, select_z );
    result->col0.vec128 = res0;
    vmathV3MakeYAxis( &result->col1 );
    result->col2.vec128 = res2;
}

static inline void vmathM3MakeRotationZ( VmathMatrix3 *result, float radians )
{
    vec_float4 s, c, res0, res1;
    vec_uint4 select_x, select_y;
    vec_float4 zero;
    select_x = (vec_uint4)spu_maskb(0xf000);
    select_y = (vec_uint4)spu_maskb(0x0f00);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res0 = spu_sel( zero, c, select_x );
    res0 = spu_sel( res0, s, select_y );
    res1 = spu_sel( zero, negatef4(s), select_x );
    res1 = spu_sel( res1, c, select_y );
    result->col0.vec128 = res0;
    result->col1.vec128 = res1;
    vmathV3MakeZAxis( &result->col2 );
}

static inline void vmathM3MakeRotationZYX( VmathMatrix3 *result, const VmathVector3 *radiansXYZ )
{
    vec_float4 angles, s, negS, c, X0, X1, Y0, Y1, Z0, Z1, tmp;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    angles = radiansXYZ->vec128;
    angles = spu_insert( 0.0f, angles, 3 );
    sincosf4( angles, &s, &c );
    negS = negatef4( s );
    Z0 = spu_shuffle( s, c, _VECTORMATH_SHUF_CZD0 );
    Z1 = spu_shuffle( c, negS, _VECTORMATH_SHUF_CZD0 );
    Y0 = spu_shuffle( negS, c, _VECTORMATH_SHUF_BBY0 );
    Y1 = spu_shuffle( c, s, _VECTORMATH_SHUF_BBY0 );
    X0 = spu_shuffle( s, s, shuffle_xxxx );
    X1 = spu_shuffle( c, c, shuffle_xxxx );
    tmp = spu_mul( Z0, Y1 );
    result->col0.vec128 = spu_mul( Z0, Y0 );
    result->col1.vec128 = spu_madd( Z1, X1, spu_mul( tmp, X0 ) );
    result->col2.vec128 = spu_nmsub( Z1, X0, spu_mul( tmp, X1 ) );
}

static inline void vmathM3MakeRotationAxis( VmathMatrix3 *result, float radians, const VmathVector3 *unitVec )
{
    vec_float4 axis, s, c, oneMinusC, axisS, negAxisS, xxxx, yyyy, zzzz, tmp0, tmp1, tmp2;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    axis = unitVec->vec128;
    sincosf4( spu_splats( radians ), &s, &c );
    xxxx = spu_shuffle( axis, axis, shuffle_xxxx );
    yyyy = spu_shuffle( axis, axis, shuffle_yyyy );
    zzzz = spu_shuffle( axis, axis, shuffle_zzzz );
    oneMinusC = spu_sub( spu_splats(1.0f), c );
    axisS = spu_mul( axis, s );
    negAxisS = negatef4( axisS );
    tmp0 = spu_shuffle( axisS, negAxisS, _VECTORMATH_SHUF_0ZB0 );
    tmp1 = spu_shuffle( axisS, negAxisS, _VECTORMATH_SHUF_C0X0 );
    tmp2 = spu_shuffle( axisS, negAxisS, _VECTORMATH_SHUF_YA00 );
    tmp0 = spu_sel( tmp0, c, (vec_uint4)spu_maskb(0xf000) );
    tmp1 = spu_sel( tmp1, c, (vec_uint4)spu_maskb(0x0f00) );
    tmp2 = spu_sel( tmp2, c, (vec_uint4)spu_maskb(0x00f0) );
    result->col0.vec128 = spu_madd( spu_mul( axis, xxxx ), oneMinusC, tmp0 );
    result->col1.vec128 = spu_madd( spu_mul( axis, yyyy ), oneMinusC, tmp1 );
    result->col2.vec128 = spu_madd( spu_mul( axis, zzzz ), oneMinusC, tmp2 );
}

static inline void vmathM3MakeRotationQ( VmathMatrix3 *result, const VmathQuat *unitQuat )
{
    vmathM3MakeFromQ( result, unitQuat );
}

static inline void vmathM3MakeScale( VmathMatrix3 *result, const VmathVector3 *scaleVec )
{
    vec_float4 zero = spu_splats(0.0f);
    result->col0.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0xf000) );
    result->col1.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0x0f00) );
    result->col2.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0x00f0) );
}

static inline void vmathM3AppendScale( VmathMatrix3 *result, const VmathMatrix3 *mat, const VmathVector3 *scaleVec )
{
    vmathV3ScalarMul( &result->col0, &mat->col0, vmathV3GetX( scaleVec ) );
    vmathV3ScalarMul( &result->col1, &mat->col1, vmathV3GetY( scaleVec ) );
    vmathV3ScalarMul( &result->col2, &mat->col2, vmathV3GetZ( scaleVec ) );
}

static inline void vmathM3PrependScale( VmathMatrix3 *result, const VmathVector3 *scaleVec, const VmathMatrix3 *mat )
{
    vmathV3MulPerElem( &result->col0, &mat->col0, scaleVec );
    vmathV3MulPerElem( &result->col1, &mat->col1, scaleVec );
    vmathV3MulPerElem( &result->col2, &mat->col2, scaleVec );
}

static inline void vmathM3Select( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1, unsigned int select1 )
{
    vmathV3Select( &result->col0, &mat0->col0, &mat1->col0, select1 );
    vmathV3Select( &result->col1, &mat0->col1, &mat1->col1, select1 );
    vmathV3Select( &result->col2, &mat0->col2, &mat1->col2, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathM3Print( const VmathMatrix3 *mat )
{
    VmathVector3 tmpV3_0, tmpV3_1, tmpV3_2;
    vmathM3GetRow( &tmpV3_0, mat, 0 );
    vmathV3Print( &tmpV3_0 );
    vmathM3GetRow( &tmpV3_1, mat, 1 );
    vmathV3Print( &tmpV3_1 );
    vmathM3GetRow( &tmpV3_2, mat, 2 );
    vmathV3Print( &tmpV3_2 );
}

static inline void vmathM3Prints( const VmathMatrix3 *mat, const char *name )
{
    printf("%s:\n", name);
    vmathM3Print( mat );
}

#endif

static inline void vmathM4Copy( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( &result->col0, &mat->col0 );
    vmathV4Copy( &result->col1, &mat->col1 );
    vmathV4Copy( &result->col2, &mat->col2 );
    vmathV4Copy( &result->col3, &mat->col3 );
}

static inline void vmathM4MakeFromScalar( VmathMatrix4 *result, float scalar )
{
    vmathV4MakeFromScalar( &result->col0, scalar );
    vmathV4MakeFromScalar( &result->col1, scalar );
    vmathV4MakeFromScalar( &result->col2, scalar );
    vmathV4MakeFromScalar( &result->col3, scalar );
}

static inline void vmathM4MakeFromT3( VmathMatrix4 *result, const VmathTransform3 *mat )
{
    vmathV4MakeFromV3Scalar( &result->col0, &mat->col0, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col1, &mat->col1, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col2, &mat->col2, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col3, &mat->col3, 1.0f );
}

static inline void vmathM4MakeFromCols( VmathMatrix4 *result, const VmathVector4 *_col0, const VmathVector4 *_col1, const VmathVector4 *_col2, const VmathVector4 *_col3 )
{
    vmathV4Copy( &result->col0, _col0 );
    vmathV4Copy( &result->col1, _col1 );
    vmathV4Copy( &result->col2, _col2 );
    vmathV4Copy( &result->col3, _col3 );
}

static inline void vmathM4MakeFromM3V3( VmathMatrix4 *result, const VmathMatrix3 *mat, const VmathVector3 *translateVec )
{
    vmathV4MakeFromV3Scalar( &result->col0, &mat->col0, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col1, &mat->col1, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col2, &mat->col2, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col3, translateVec, 1.0f );
}

static inline void vmathM4MakeFromQV3( VmathMatrix4 *result, const VmathQuat *unitQuat, const VmathVector3 *translateVec )
{
    VmathMatrix3 mat;
    vmathM3MakeFromQ( &mat, unitQuat );
    vmathV4MakeFromV3Scalar( &result->col0, &mat.col0, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col1, &mat.col1, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col2, &mat.col2, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col3, translateVec, 1.0f );
}

static inline void vmathM4SetCol0( VmathMatrix4 *result, const VmathVector4 *_col0 )
{
    vmathV4Copy( &result->col0, _col0 );
}

static inline void vmathM4SetCol1( VmathMatrix4 *result, const VmathVector4 *_col1 )
{
    vmathV4Copy( &result->col1, _col1 );
}

static inline void vmathM4SetCol2( VmathMatrix4 *result, const VmathVector4 *_col2 )
{
    vmathV4Copy( &result->col2, _col2 );
}

static inline void vmathM4SetCol3( VmathMatrix4 *result, const VmathVector4 *_col3 )
{
    vmathV4Copy( &result->col3, _col3 );
}

static inline void vmathM4SetCol( VmathMatrix4 *result, int col, const VmathVector4 *vec )
{
    vmathV4Copy( (&result->col0 + col), vec );
}

static inline void vmathM4SetRow( VmathMatrix4 *result, int row, const VmathVector4 *vec )
{
    vmathV4SetElem( &result->col0, row, vmathV4GetElem( vec, 0 ) );
    vmathV4SetElem( &result->col1, row, vmathV4GetElem( vec, 1 ) );
    vmathV4SetElem( &result->col2, row, vmathV4GetElem( vec, 2 ) );
    vmathV4SetElem( &result->col3, row, vmathV4GetElem( vec, 3 ) );
}

static inline void vmathM4SetElem( VmathMatrix4 *result, int col, int row, float val )
{
    VmathVector4 tmpV3_0;
    vmathM4GetCol( &tmpV3_0, result, col );
    vmathV4SetElem( &tmpV3_0, row, val );
    vmathM4SetCol( result, col, &tmpV3_0 );
}

static inline float vmathM4GetElem( const VmathMatrix4 *mat, int col, int row )
{
    VmathVector4 tmpV4_0;
    vmathM4GetCol( &tmpV4_0, mat, col );
    return vmathV4GetElem( &tmpV4_0, row );
}

static inline void vmathM4GetCol0( VmathVector4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( result, &mat->col0 );
}

static inline void vmathM4GetCol1( VmathVector4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( result, &mat->col1 );
}

static inline void vmathM4GetCol2( VmathVector4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( result, &mat->col2 );
}

static inline void vmathM4GetCol3( VmathVector4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( result, &mat->col3 );
}

static inline void vmathM4GetCol( VmathVector4 *result, const VmathMatrix4 *mat, int col )
{
    vmathV4Copy( result, (&mat->col0 + col) );
}

static inline void vmathM4GetRow( VmathVector4 *result, const VmathMatrix4 *mat, int row )
{
    vmathV4MakeFromElems( result, vmathV4GetElem( &mat->col0, row ), vmathV4GetElem( &mat->col1, row ), vmathV4GetElem( &mat->col2, row ), vmathV4GetElem( &mat->col3, row ) );
}

static inline void vmathM4Transpose( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3, res0, res1, res2, res3;
    tmp0 = spu_shuffle( mat->col0.vec128, mat->col2.vec128, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( mat->col1.vec128, mat->col3.vec128, _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( mat->col0.vec128, mat->col2.vec128, _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( mat->col1.vec128, mat->col3.vec128, _VECTORMATH_SHUF_ZCWD );
    res0 = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB );
    res1 = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD );
    res2 = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB );
    res3 = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_ZCWD );
    result->col0.vec128 = res0;
    result->col1.vec128 = res1;
    result->col2.vec128 = res2;
    result->col3.vec128 = res3;
}

static inline void vmathM4Inverse( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    /* function implementation based on code from STIDC SDK:           */
    /* --------------------------------------------------------------  */
    /* PLEASE DO NOT MODIFY THIS SECTION                               */
    /* This prolog section is automatically generated.                 */
    /*                                                                 */
    /* (C)Copyright                                                    */
    /* Sony Computer Entertainment, Inc.,                              */
    /* Toshiba Corporation,                                            */
    /* International Business Machines Corporation,                    */
    /* 2001,2002.                                                      */
    /* S/T/I Confidential Information                                  */
    /* --------------------------------------------------------------  */
    vec_float4 in0, in1, in2, in3;
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    vec_float4 cof0, cof1, cof2, cof3;
    vec_float4 t0, t1, t2, t3;
    vec_float4 t01, t02, t03, t12, t23;
    vec_float4 t1r, t2r;
    vec_float4 t01r, t02r, t03r, t12r, t23r;
    vec_float4 t1r3, t1r3r;
    vec_float4 det, det1, det2, det3, invdet;
    in0 = mat->col0.vec128;
    in1 = mat->col1.vec128;
    in2 = mat->col2.vec128;
    in3 = mat->col3.vec128;
    /* Perform transform of the input matrix of the form:
     *    A B C D
     *    E F G H
     *    I J K L
     *    M N O P
     *
     * The pseudo transpose of the input matrix is trans:
     *    A E I M
     *    J N B F
     *    C G K O
     *    L P D H
     */
    tmp0 = spu_shuffle(in0, in1, _VECTORMATH_SHUF_XAZC);    /* A E C G */
    tmp1 = spu_shuffle(in2, in3, _VECTORMATH_SHUF_XAZC);    /* I M K O */
    tmp2 = spu_shuffle(in0, in1, _VECTORMATH_SHUF_YBWD);    /* B F D H */
    tmp3 = spu_shuffle(in2, in3, _VECTORMATH_SHUF_YBWD);    /* J N L P */
    t0 = spu_shuffle(tmp0, tmp1, _VECTORMATH_SHUF_XYAB);    /* A E I M */
    t1 = spu_shuffle(tmp3, tmp2, _VECTORMATH_SHUF_XYAB);    /* J N B F */
    t2 = spu_shuffle(tmp0, tmp1, _VECTORMATH_SHUF_ZWCD);    /* C G K O */
    t3 = spu_shuffle(tmp3, tmp2, _VECTORMATH_SHUF_ZWCD);    /* L P D H */
    /* Generate a cofactor matrix. The computed cofactors reside in
     * cof0, cof1, cof2, cof3.
     */
    t23 = spu_mul(t2, t3);                        /* CL GP KD OH */
    t23 = spu_shuffle(t23, t23, _VECTORMATH_SHUF_YXWZ);     /* GP CL OH KD */
    cof0 = spu_mul(t1, t23);                      /* JGP NCL BOH FKD */
    cof1 = spu_mul(t0, t23);                      /* AGP ECL IOH MKD */
    t23r = spu_rlqwbyte(t23, 8);                  /* OH KD GP CL */
    cof0 = spu_msub(t1, t23r, cof0);              /* JOH NKD BGP FCL  - cof0 */
    cof1 = spu_msub(t0, t23r, cof1);              /* AOH EKD IGP MCL  - cof1 */
    cof1 = spu_rlqwbyte(cof1, 8);                 /* IGP MCL AOH EKD - IOH MKD AGP ECL */

    t12 = spu_mul(t1, t2);                        /* JC NG BK FO */
    t12 = spu_shuffle(t12, t12, _VECTORMATH_SHUF_YXWZ);     /* NG JC FO BK */
    cof0 = spu_madd(t3, t12, cof0);               /* LNG PJC DFO HBK + cof0 */
    cof3 = spu_mul(t0, t12);                      /* ANG EJC IFO MBK */
    t12r = spu_rlqwbyte(t12, 8);                  /* FO BK NG JC */
    cof0 = spu_nmsub(t3, t12r, cof0);             /* cof0 - LFO PBK DNG HJC */
    cof3 = spu_msub(t0, t12r, cof3);              /* AFO EBK ING MJC - cof3 */
    cof3 = spu_rlqwbyte(cof3, 8);                 /* ING MJC AFO EBK - IFO MBK ANG EJC */
    t1r = spu_rlqwbyte(t1, 8);                    /* B F J N */
    t2r = spu_rlqwbyte(t2, 8);                    /* K O C G */
    t1r3 = spu_mul(t1r, t3);                      /* BL FP JD NH */
    t1r3 = spu_shuffle(t1r3, t1r3, _VECTORMATH_SHUF_YXWZ);  /* FP BL NH JD */
    cof0 = spu_madd(t2r, t1r3, cof0);             /* KFP OBL CNH GJD + cof0 */
    cof2 = spu_mul(t0, t1r3);                     /* AFP EBL INH MJD */
    t1r3r = spu_rlqwbyte(t1r3, 8);                /* NH JD FP BL */
    cof0 = spu_nmsub(t2r, t1r3r, cof0);           /* cof0 - KNH OJD CFP GBL */
    cof2 = spu_msub(t0, t1r3r, cof2);             /* ANH EJD IFP MBL - cof2 */
    cof2 = spu_rlqwbyte(cof2, 8);                 /* IFP MBL ANH EJD - INH MJD AFP EBL */
    t01 = spu_mul(t0, t1);                                /* AJ EN IB MF */
    t01 = spu_shuffle(t01, t01, _VECTORMATH_SHUF_YXWZ);     /* EN AJ MF IB */
    cof2 = spu_madd(t3, t01, cof2);               /* LEN PAJ DMF HIB + cof2 */
    cof3 = spu_msub(t2r, t01, cof3);              /* KEN OAJ CMF GIB - cof3 */
    t01r = spu_rlqwbyte(t01, 8);                  /* MF IB EN AJ */
    cof2 = spu_msub(t3, t01r, cof2);              /* LMF PIB DEN HAJ - cof2 */
    cof3 = spu_nmsub(t2r, t01r, cof3);            /* cof3 - KMF OIB CEN GAJ */
    t03 = spu_mul(t0, t3);                                /* AL EP ID MH */
    t03 = spu_shuffle(t03, t03, _VECTORMATH_SHUF_YXWZ);     /* EP AL MH ID */
    cof1 = spu_nmsub(t2r, t03, cof1);             /* cof1 - KEP OAL CMH GID */
    cof2 = spu_madd(t1, t03, cof2);               /* JEP NAL BMH FID + cof2 */
    t03r = spu_rlqwbyte(t03, 8);                  /* MH ID EP AL */
    cof1 = spu_madd(t2r, t03r, cof1);             /* KMH OID CEP GAL + cof1 */
    cof2 = spu_nmsub(t1, t03r, cof2);             /* cof2 - JMH NID BEP FAL */
    t02 = spu_mul(t0, t2r);                       /* AK EO IC MG */
    t02 = spu_shuffle(t02, t02, _VECTORMATH_SHUF_YXWZ);     /* E0 AK MG IC */
    cof1 = spu_madd(t3, t02, cof1);               /* LEO PAK DMG HIC + cof1 */
    cof3 = spu_nmsub(t1, t02, cof3);              /* cof3 - JEO NAK BMG FIC */
    t02r = spu_rlqwbyte(t02, 8);                  /* MG IC EO AK */
    cof1 = spu_nmsub(t3, t02r, cof1);             /* cof1 - LMG PIC DEO HAK */
    cof3 = spu_madd(t1, t02r, cof3);              /* JMG NIC BEO FAK + cof3 */
    /* Compute the determinant of the matrix
     *
     * det = sum_across(t0 * cof0);
     *
     * We perform a sum across the entire vector so that
     * we don't have to splat the result when multiplying the
     * cofactors by the inverse of the determinant.
     */
    det  = spu_mul(t0, cof0);
    det1 = spu_rlqwbyte(det, 4);
    det2 = spu_rlqwbyte(det, 8);
    det3 = spu_rlqwbyte(det, 12);
    det  = spu_add(det, det1);
    det2 = spu_add(det2, det3);
    det  = spu_add(det, det2);
    /* Compute the reciprocal of the determinant.
     */
    invdet = recipf4(det);
    /* Multiply the cofactors by the reciprocal of the determinant.
     */
    result->col0.vec128 = spu_mul(cof0, invdet);
    result->col1.vec128 = spu_mul(cof1, invdet);
    result->col2.vec128 = spu_mul(cof2, invdet);
    result->col3.vec128 = spu_mul(cof3, invdet);
}

static inline void vmathM4AffineInverse( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    VmathTransform3 affineMat, tmpT3_0;
    VmathVector3 tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3;
    vmathV4GetXYZ( &tmpV3_0, &mat->col0 );
    vmathT3SetCol0( &affineMat, &tmpV3_0 );
    vmathV4GetXYZ( &tmpV3_1, &mat->col1 );
    vmathT3SetCol1( &affineMat, &tmpV3_1 );
    vmathV4GetXYZ( &tmpV3_2, &mat->col2 );
    vmathT3SetCol2( &affineMat, &tmpV3_2 );
    vmathV4GetXYZ( &tmpV3_3, &mat->col3 );
    vmathT3SetCol3( &affineMat, &tmpV3_3 );
    vmathT3Inverse( &tmpT3_0, &affineMat );
    vmathM4MakeFromT3( result, &tmpT3_0 );
}

static inline void vmathM4OrthoInverse( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    VmathTransform3 affineMat, tmpT3_0;
    VmathVector3 tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3;
    vmathV4GetXYZ( &tmpV3_0, &mat->col0 );
    vmathT3SetCol0( &affineMat, &tmpV3_0 );
    vmathV4GetXYZ( &tmpV3_1, &mat->col1 );
    vmathT3SetCol1( &affineMat, &tmpV3_1 );
    vmathV4GetXYZ( &tmpV3_2, &mat->col2 );
    vmathT3SetCol2( &affineMat, &tmpV3_2 );
    vmathV4GetXYZ( &tmpV3_3, &mat->col3 );
    vmathT3SetCol3( &affineMat, &tmpV3_3 );
    vmathT3OrthoInverse( &tmpT3_0, &affineMat );
    vmathM4MakeFromT3( result, &tmpT3_0 );
}

static inline float vmathM4Determinant( const VmathMatrix4 *mat )
{
    /* function implementation based on code from STIDC SDK:           */
    /* --------------------------------------------------------------  */
    /* PLEASE DO NOT MODIFY THIS SECTION                               */
    /* This prolog section is automatically generated.                 */
    /*                                                                 */
    /* (C)Copyright                                                    */
    /* Sony Computer Entertainment, Inc.,                              */
    /* Toshiba Corporation,                                            */
    /* International Business Machines Corporation,                    */
    /* 2001,2002.                                                      */
    /* S/T/I Confidential Information                                  */
    /* --------------------------------------------------------------  */
    vec_float4 in0, in1, in2, in3;
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    vec_float4 cof0;
    vec_float4 t0, t1, t2, t3;
    vec_float4 t12, t23;
    vec_float4 t1r, t2r;
    vec_float4 t12r, t23r;
    vec_float4 t1r3, t1r3r;
    in0 = mat->col0.vec128;
    in1 = mat->col1.vec128;
    in2 = mat->col2.vec128;
    in3 = mat->col3.vec128;
    /* Perform transform of the input matrix of the form:
     *    A B C D
     *    E F G H
     *    I J K L
     *    M N O P
     *
     * The pseudo transpose of the input matrix is trans:
     *    A E I M
     *    J N B F
     *    C G K O
     *    L P D H
     */
    tmp0 = spu_shuffle(in0, in1, _VECTORMATH_SHUF_XAZC);    /* A E C G */
    tmp1 = spu_shuffle(in2, in3, _VECTORMATH_SHUF_XAZC);    /* I M K O */
    tmp2 = spu_shuffle(in0, in1, _VECTORMATH_SHUF_YBWD);    /* B F D H */
    tmp3 = spu_shuffle(in2, in3, _VECTORMATH_SHUF_YBWD);    /* J N L P */
    t0 = spu_shuffle(tmp0, tmp1, _VECTORMATH_SHUF_XYAB);    /* A E I M */
    t1 = spu_shuffle(tmp3, tmp2, _VECTORMATH_SHUF_XYAB);    /* J N B F */
    t2 = spu_shuffle(tmp0, tmp1, _VECTORMATH_SHUF_ZWCD);    /* C G K O */
    t3 = spu_shuffle(tmp3, tmp2, _VECTORMATH_SHUF_ZWCD);    /* L P D H */
    /* Generate a cofactor matrix. The computed cofactors reside in
     * cof0, cof1, cof2, cof3.
     */
    t23 = spu_mul(t2, t3);                        /* CL GP KD OH */
    t23 = spu_shuffle(t23, t23, _VECTORMATH_SHUF_YXWZ);     /* GP CL OH KD */
    cof0 = spu_mul(t1, t23);                      /* JGP NCL BOH FKD */
    t23r = spu_rlqwbyte(t23, 8);                  /* OH KD GP CL */
    cof0 = spu_msub(t1, t23r, cof0);              /* JOH NKD BGP FCL  - cof0 */

    t12 = spu_mul(t1, t2);                        /* JC NG BK FO */
    t12 = spu_shuffle(t12, t12, _VECTORMATH_SHUF_YXWZ);     /* NG JC FO BK */
    cof0 = spu_madd(t3, t12, cof0);               /* LNG PJC DFO HBK + cof0 */
    t12r = spu_rlqwbyte(t12, 8);                  /* FO BK NG JC */
    cof0 = spu_nmsub(t3, t12r, cof0);             /* cof0 - LFO PBK DNG HJC */
    t1r = spu_rlqwbyte(t1, 8);                    /* B F J N */
    t2r = spu_rlqwbyte(t2, 8);                    /* K O C G */
    t1r3 = spu_mul(t1r, t3);                      /* BL FP JD NH */
    t1r3 = spu_shuffle(t1r3, t1r3, _VECTORMATH_SHUF_YXWZ);  /* FP BL NH JD */
    cof0 = spu_madd(t2r, t1r3, cof0);             /* KFP OBL CNH GJD + cof0 */
    t1r3r = spu_rlqwbyte(t1r3, 8);                /* NH JD FP BL */
    cof0 = spu_nmsub(t2r, t1r3r, cof0);           /* cof0 - KNH OJD CFP GBL */
    return spu_extract( _vmathVfDot4(t0,cof0), 0 );
}

static inline void vmathM4Add( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 )
{
    vmathV4Add( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV4Add( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV4Add( &result->col2, &mat0->col2, &mat1->col2 );
    vmathV4Add( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathM4Sub( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 )
{
    vmathV4Sub( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV4Sub( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV4Sub( &result->col2, &mat0->col2, &mat1->col2 );
    vmathV4Sub( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathM4Neg( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    vmathV4Neg( &result->col0, &mat->col0 );
    vmathV4Neg( &result->col1, &mat->col1 );
    vmathV4Neg( &result->col2, &mat->col2 );
    vmathV4Neg( &result->col3, &mat->col3 );
}

static inline void vmathM4AbsPerElem( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    vmathV4AbsPerElem( &result->col0, &mat->col0 );
    vmathV4AbsPerElem( &result->col1, &mat->col1 );
    vmathV4AbsPerElem( &result->col2, &mat->col2 );
    vmathV4AbsPerElem( &result->col3, &mat->col3 );
}

static inline void vmathM4ScalarMul( VmathMatrix4 *result, const VmathMatrix4 *mat, float scalar )
{
    vmathV4ScalarMul( &result->col0, &mat->col0, scalar );
    vmathV4ScalarMul( &result->col1, &mat->col1, scalar );
    vmathV4ScalarMul( &result->col2, &mat->col2, scalar );
    vmathV4ScalarMul( &result->col3, &mat->col3, scalar );
}

static inline void vmathM4MulV4( VmathVector4 *result, const VmathMatrix4 *mat, const VmathVector4 *vec )
{
    vec_float4 tmp0, tmp1, res;
    vec_float4 xxxx, yyyy, zzzz, wwww;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    vec_uchar16 shuffle_wwww = (vec_uchar16)spu_splats((int)0x0c0d0e0f);
    xxxx = spu_shuffle( vec->vec128, vec->vec128, shuffle_xxxx );
    yyyy = spu_shuffle( vec->vec128, vec->vec128, shuffle_yyyy );
    zzzz = spu_shuffle( vec->vec128, vec->vec128, shuffle_zzzz );
    wwww = spu_shuffle( vec->vec128, vec->vec128, shuffle_wwww );
    tmp0 = spu_mul( mat->col0.vec128, xxxx );
    tmp1 = spu_mul( mat->col1.vec128, yyyy );
    tmp0 = spu_madd( mat->col2.vec128, zzzz, tmp0 );
    tmp1 = spu_madd( mat->col3.vec128, wwww, tmp1 );
    res = spu_add( tmp0, tmp1 );
    result->vec128 = res;
}

static inline void vmathM4MulV3( VmathVector4 *result, const VmathMatrix4 *mat, const VmathVector3 *vec )
{
    vec_float4 res;
    vec_float4 xxxx, yyyy, zzzz;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    xxxx = spu_shuffle( vec->vec128, vec->vec128, shuffle_xxxx );
    yyyy = spu_shuffle( vec->vec128, vec->vec128, shuffle_yyyy );
    zzzz = spu_shuffle( vec->vec128, vec->vec128, shuffle_zzzz );
    res = spu_mul( mat->col0.vec128, xxxx );
    res = spu_madd( mat->col1.vec128, yyyy, res );
    res = spu_madd( mat->col2.vec128, zzzz, res );
    result->vec128 = res;
}

static inline void vmathM4MulP3( VmathVector4 *result, const VmathMatrix4 *mat, const VmathPoint3 *pnt )
{
    vec_float4 tmp0, tmp1, res;
    vec_float4 xxxx, yyyy, zzzz;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    xxxx = spu_shuffle( pnt->vec128, pnt->vec128, shuffle_xxxx );
    yyyy = spu_shuffle( pnt->vec128, pnt->vec128, shuffle_yyyy );
    zzzz = spu_shuffle( pnt->vec128, pnt->vec128, shuffle_zzzz );
    tmp0 = spu_mul( mat->col0.vec128, xxxx );
    tmp1 = spu_mul( mat->col1.vec128, yyyy );
    tmp0 = spu_madd( mat->col2.vec128, zzzz, tmp0 );
    tmp1 = spu_add( mat->col3.vec128, tmp1 );
    res = spu_add( tmp0, tmp1 );
    result->vec128 = res;
}

static inline void vmathM4Mul( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 )
{
    VmathMatrix4 tmpResult;
    vmathM4MulV4( &tmpResult.col0, mat0, &mat1->col0 );
    vmathM4MulV4( &tmpResult.col1, mat0, &mat1->col1 );
    vmathM4MulV4( &tmpResult.col2, mat0, &mat1->col2 );
    vmathM4MulV4( &tmpResult.col3, mat0, &mat1->col3 );
    vmathM4Copy( result, &tmpResult );
}

static inline void vmathM4MulT3( VmathMatrix4 *result, const VmathMatrix4 *mat, const VmathTransform3 *tfrm1 )
{
    VmathMatrix4 tmpResult;
    VmathPoint3 tmpP3_0;
    vmathM4MulV3( &tmpResult.col0, mat, &tfrm1->col0 );
    vmathM4MulV3( &tmpResult.col1, mat, &tfrm1->col1 );
    vmathM4MulV3( &tmpResult.col2, mat, &tfrm1->col2 );
    vmathP3MakeFromV3( &tmpP3_0, &tfrm1->col3 );
    vmathM4MulP3( &tmpResult.col3, mat, &tmpP3_0 );
    vmathM4Copy( result, &tmpResult );
}

static inline void vmathM4MulPerElem( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 )
{
    vmathV4MulPerElem( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV4MulPerElem( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV4MulPerElem( &result->col2, &mat0->col2, &mat1->col2 );
    vmathV4MulPerElem( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathM4MakeIdentity( VmathMatrix4 *result )
{
    vmathV4MakeXAxis( &result->col0 );
    vmathV4MakeYAxis( &result->col1 );
    vmathV4MakeZAxis( &result->col2 );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4SetUpper3x3( VmathMatrix4 *result, const VmathMatrix3 *mat3 )
{
    vmathV4SetXYZ( &result->col0, &mat3->col0 );
    vmathV4SetXYZ( &result->col1, &mat3->col1 );
    vmathV4SetXYZ( &result->col2, &mat3->col2 );
}

static inline void vmathM4GetUpper3x3( VmathMatrix3 *result, const VmathMatrix4 *mat )
{
    vmathV4GetXYZ( &result->col0, &mat->col0 );
    vmathV4GetXYZ( &result->col1, &mat->col1 );
    vmathV4GetXYZ( &result->col2, &mat->col2 );
}

static inline void vmathM4SetTranslation( VmathMatrix4 *result, const VmathVector3 *translateVec )
{
    vmathV4SetXYZ( &result->col3, translateVec );
}

static inline void vmathM4GetTranslation( VmathVector3 *result, const VmathMatrix4 *mat )
{
    vmathV4GetXYZ( result, &mat->col3 );
}

static inline void vmathM4MakeRotationX( VmathMatrix4 *result, float radians )
{
    vec_float4 s, c, res1, res2;
    vec_uint4 select_y, select_z;
    vec_float4 zero;
    select_y = (vec_uint4)spu_maskb(0x0f00);
    select_z = (vec_uint4)spu_maskb(0x00f0);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res1 = spu_sel( zero, c, select_y );
    res1 = spu_sel( res1, s, select_z );
    res2 = spu_sel( zero, negatef4(s), select_y );
    res2 = spu_sel( res2, c, select_z );
    vmathV4MakeXAxis( &result->col0 );
    result->col1.vec128 = res1;
    result->col2.vec128 = res2;
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationY( VmathMatrix4 *result, float radians )
{
    vec_float4 s, c, res0, res2;
    vec_uint4 select_x, select_z;
    vec_float4 zero;
    select_x = (vec_uint4)spu_maskb(0xf000);
    select_z = (vec_uint4)spu_maskb(0x00f0);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res0 = spu_sel( zero, c, select_x );
    res0 = spu_sel( res0, negatef4(s), select_z );
    res2 = spu_sel( zero, s, select_x );
    res2 = spu_sel( res2, c, select_z );
    result->col0.vec128 = res0;
    vmathV4MakeYAxis( &result->col1 );
    result->col2.vec128 = res2;
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationZ( VmathMatrix4 *result, float radians )
{
    vec_float4 s, c, res0, res1;
    vec_uint4 select_x, select_y;
    vec_float4 zero;
    select_x = (vec_uint4)spu_maskb(0xf000);
    select_y = (vec_uint4)spu_maskb(0x0f00);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res0 = spu_sel( zero, c, select_x );
    res0 = spu_sel( res0, s, select_y );
    res1 = spu_sel( zero, negatef4(s), select_x );
    res1 = spu_sel( res1, c, select_y );
    result->col0.vec128 = res0;
    result->col1.vec128 = res1;
    vmathV4MakeZAxis( &result->col2 );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationZYX( VmathMatrix4 *result, const VmathVector3 *radiansXYZ )
{
    vec_float4 angles, s, negS, c, X0, X1, Y0, Y1, Z0, Z1, tmp;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    angles = radiansXYZ->vec128;
    angles = spu_insert( 0.0f, angles, 3 );
    sincosf4( angles, &s, &c );
    negS = negatef4( s );
    Z0 = spu_shuffle( s, c, _VECTORMATH_SHUF_CZD0 );
    Z1 = spu_shuffle( c, negS, _VECTORMATH_SHUF_CZD0 );
    Y0 = spu_shuffle( negS, c, _VECTORMATH_SHUF_BBY0 );
    Y1 = spu_shuffle( c, s, _VECTORMATH_SHUF_BBY0 );
    X0 = spu_shuffle( s, s, shuffle_xxxx );
    X1 = spu_shuffle( c, c, shuffle_xxxx );
    tmp = spu_mul( Z0, Y1 );
    result->col0.vec128 = spu_mul( Z0, Y0 );
    result->col1.vec128 = spu_madd( Z1, X1, spu_mul( tmp, X0 ) );
    result->col2.vec128 = spu_nmsub( Z1, X0, spu_mul( tmp, X1 ) );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationAxis( VmathMatrix4 *result, float radians, const VmathVector3 *unitVec )
{
    vec_float4 axis, s, c, oneMinusC, axisS, negAxisS, xxxx, yyyy, zzzz, tmp0, tmp1, tmp2, zeroW;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    axis = unitVec->vec128;
    sincosf4( spu_splats( radians ), &s, &c );
    xxxx = spu_shuffle( axis, axis, shuffle_xxxx );
    yyyy = spu_shuffle( axis, axis, shuffle_yyyy );
    zzzz = spu_shuffle( axis, axis, shuffle_zzzz );
    oneMinusC = spu_sub( spu_splats(1.0f), c );
    axisS = spu_mul( axis, s );
    negAxisS = negatef4( axisS );
    tmp0 = spu_shuffle( axisS, negAxisS, _VECTORMATH_SHUF_0ZB0 );
    tmp1 = spu_shuffle( axisS, negAxisS, _VECTORMATH_SHUF_C0X0 );
    tmp2 = spu_shuffle( axisS, negAxisS, _VECTORMATH_SHUF_YA00 );
    tmp0 = spu_sel( tmp0, c, (vec_uint4)spu_maskb(0xf000) );
    tmp1 = spu_sel( tmp1, c, (vec_uint4)spu_maskb(0x0f00) );
    tmp2 = spu_sel( tmp2, c, (vec_uint4)spu_maskb(0x00f0) );
    zeroW = (vec_float4)spu_maskb(0x000f);
    axis = spu_andc( axis, zeroW );
    result->col0.vec128 = spu_madd( spu_mul( axis, xxxx ), oneMinusC, tmp0 );
    result->col1.vec128 = spu_madd( spu_mul( axis, yyyy ), oneMinusC, tmp1 );
    result->col2.vec128 = spu_madd( spu_mul( axis, zzzz ), oneMinusC, tmp2 );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationQ( VmathMatrix4 *result, const VmathQuat *unitQuat )
{
    VmathTransform3 tmpT3_0;
    vmathT3MakeRotationQ( &tmpT3_0, unitQuat );
    vmathM4MakeFromT3( result, &tmpT3_0 );
}

static inline void vmathM4MakeScale( VmathMatrix4 *result, const VmathVector3 *scaleVec )
{
    vec_float4 zero = spu_splats(0.0f);
    result->col0.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0xf000) );
    result->col1.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0x0f00) );
    result->col2.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0x00f0) );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4AppendScale( VmathMatrix4 *result, const VmathMatrix4 *mat, const VmathVector3 *scaleVec )
{
    vmathV4ScalarMul( &result->col0, &mat->col0, vmathV3GetX( scaleVec ) );
    vmathV4ScalarMul( &result->col1, &mat->col1, vmathV3GetY( scaleVec ) );
    vmathV4ScalarMul( &result->col2, &mat->col2, vmathV3GetZ( scaleVec ) );
    vmathV4Copy( &result->col3, &mat->col3 );
}

static inline void vmathM4PrependScale( VmathMatrix4 *result, const VmathVector3 *scaleVec, const VmathMatrix4 *mat )
{
    VmathVector4 scale4;
    vmathV4MakeFromV3Scalar( &scale4, scaleVec, 1.0f );
    vmathV4MulPerElem( &result->col0, &mat->col0, &scale4 );
    vmathV4MulPerElem( &result->col1, &mat->col1, &scale4 );
    vmathV4MulPerElem( &result->col2, &mat->col2, &scale4 );
    vmathV4MulPerElem( &result->col3, &mat->col3, &scale4 );
}

static inline void vmathM4MakeTranslation( VmathMatrix4 *result, const VmathVector3 *translateVec )
{
    vmathV4MakeXAxis( &result->col0 );
    vmathV4MakeYAxis( &result->col1 );
    vmathV4MakeZAxis( &result->col2 );
    vmathV4MakeFromV3Scalar( &result->col3, translateVec, 1.0f );
}

static inline void vmathM4MakeLookAt( VmathMatrix4 *result, const VmathPoint3 *eyePos, const VmathPoint3 *lookAtPos, const VmathVector3 *upVec )
{
    VmathMatrix4 m4EyeFrame;
    VmathVector3 v3X, v3Y, v3Z, tmpV3_0, tmpV3_1;
    VmathVector4 tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3;
    vmathV3Normalize( &v3Y, upVec );
    vmathP3Sub( &tmpV3_0, eyePos, lookAtPos );
    vmathV3Normalize( &v3Z, &tmpV3_0 );
    vmathV3Cross( &tmpV3_1, &v3Y, &v3Z );
    vmathV3Normalize( &v3X, &tmpV3_1 );
    vmathV3Cross( &v3Y, &v3Z, &v3X );
    vmathV4MakeFromV3( &tmpV4_0, &v3X );
    vmathV4MakeFromV3( &tmpV4_1, &v3Y );
    vmathV4MakeFromV3( &tmpV4_2, &v3Z );
    vmathV4MakeFromP3( &tmpV4_3, eyePos );
    vmathM4MakeFromCols( &m4EyeFrame, &tmpV4_0, &tmpV4_1, &tmpV4_2, &tmpV4_3 );
    vmathM4OrthoInverse( result, &m4EyeFrame );
}

static inline void vmathM4MakePerspective( VmathMatrix4 *result, float fovyRadians, float aspect, float zNear, float zFar )
{
    float f, rangeInv;
    vec_float4 zero, col0, col1, col2, col3;
    f = tanf( _VECTORMATH_PI_OVER_2 - fovyRadians * 0.5f );
    rangeInv = 1.0f / ( zNear - zFar );
    zero = spu_splats(0.0f);
    col0 = zero;
    col1 = zero;
    col2 = zero;
    col3 = zero;
    col0 = spu_insert( f / aspect, col0, 0 );
    col1 = spu_insert( f, col1, 1 );
    col2 = spu_insert( ( zNear + zFar ) * rangeInv, col2, 2 );
    col2 = spu_insert( -1.0f, col2, 3 );
    col3 = spu_insert( zNear * zFar * rangeInv * 2.0f, col3, 2 );
    result->col0.vec128 = col0;
    result->col1.vec128 = col1;
    result->col2.vec128 = col2;
    result->col3.vec128 = col3;
}

static inline void vmathM4MakeFrustum( VmathMatrix4 *result, float left, float right, float bottom, float top, float zNear, float zFar )
{
    /* function implementation based on code from STIDC SDK:           */
    /* --------------------------------------------------------------  */
    /* PLEASE DO NOT MODIFY THIS SECTION                               */
    /* This prolog section is automatically generated.                 */
    /*                                                                 */
    /* (C)Copyright                                                    */
    /* Sony Computer Entertainment, Inc.,                              */
    /* Toshiba Corporation,                                            */
    /* International Business Machines Corporation,                    */
    /* 2001,2002.                                                      */
    /* S/T/I Confidential Information                                  */
    /* --------------------------------------------------------------  */
    vec_float4 lbf, rtn;
    vec_float4 diff, sum, inv_diff;
    vec_float4 diagonal, column, near2;
    vec_float4 zero = spu_splats(0.0f);
    lbf = spu_shuffle( spu_promote(left,0), spu_promote(zFar,0), _VECTORMATH_SHUF_XAYB );
    rtn = spu_shuffle( spu_promote(right,0), spu_promote(zNear,0), _VECTORMATH_SHUF_XAYB );
    lbf = spu_shuffle( lbf, spu_promote(bottom,0), _VECTORMATH_SHUF_XAYB );
    rtn = spu_shuffle( rtn, spu_promote(top,0), _VECTORMATH_SHUF_XAYB );
    diff = spu_sub( rtn, lbf );
    sum  = spu_add( rtn, lbf );
    inv_diff = recipf4( diff );
    near2 = spu_splats( zNear );
    near2 = spu_add( near2, near2 );
    diagonal = spu_mul( near2, inv_diff );
    column = spu_mul( sum, inv_diff );
    result->col0.vec128 = spu_sel( zero, diagonal, (vec_uint4)spu_maskb(0xf000) );
    result->col1.vec128 = spu_sel( zero, diagonal, (vec_uint4)spu_maskb(0x0f00) );
    result->col2.vec128 = spu_sel( column, spu_splats(-1.0f), (vec_uint4)spu_maskb(0x000f) );
    result->col3.vec128 = spu_sel( zero, spu_mul( diagonal, spu_splats(zFar) ), (vec_uint4)spu_maskb(0x00f0) );
}

static inline void vmathM4MakeOrthographic( VmathMatrix4 *result, float left, float right, float bottom, float top, float zNear, float zFar )
{
    /* function implementation based on code from STIDC SDK:           */
    /* --------------------------------------------------------------  */
    /* PLEASE DO NOT MODIFY THIS SECTION                               */
    /* This prolog section is automatically generated.                 */
    /*                                                                 */
    /* (C)Copyright                                                    */
    /* Sony Computer Entertainment, Inc.,                              */
    /* Toshiba Corporation,                                            */
    /* International Business Machines Corporation,                    */
    /* 2001,2002.                                                      */
    /* S/T/I Confidential Information                                  */
    /* --------------------------------------------------------------  */
    vec_float4 lbf, rtn;
    vec_float4 diff, sum, inv_diff, neg_inv_diff;
    vec_float4 diagonal, column;
    vec_float4 zero = spu_splats(0.0f);
    lbf = spu_shuffle( spu_promote(left,0), spu_promote(zFar,0), _VECTORMATH_SHUF_XAYB );
    rtn = spu_shuffle( spu_promote(right,0), spu_promote(zNear,0), _VECTORMATH_SHUF_XAYB );
    lbf = spu_shuffle( lbf, spu_promote(bottom,0), _VECTORMATH_SHUF_XAYB );
    rtn = spu_shuffle( rtn, spu_promote(top,0), _VECTORMATH_SHUF_XAYB );
    diff = spu_sub( rtn, lbf );
    sum  = spu_add( rtn, lbf );
    inv_diff = recipf4( diff );
    neg_inv_diff = negatef4( inv_diff );
    diagonal = spu_add( inv_diff, inv_diff );
    column = spu_mul( sum, spu_sel( neg_inv_diff, inv_diff, (vec_uint4)spu_maskb(0x00f0) ) );
    result->col0.vec128 = spu_sel( zero, diagonal, (vec_uint4)spu_maskb(0xf000) );
    result->col1.vec128 = spu_sel( zero, diagonal, (vec_uint4)spu_maskb(0x0f00) );
    result->col2.vec128 = spu_sel( zero, diagonal, (vec_uint4)spu_maskb(0x00f0) );
    result->col3.vec128 = spu_sel( column, spu_splats(1.0f), (vec_uint4)spu_maskb(0x000f) );
}

static inline void vmathM4Select( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1, unsigned int select1 )
{
    vmathV4Select( &result->col0, &mat0->col0, &mat1->col0, select1 );
    vmathV4Select( &result->col1, &mat0->col1, &mat1->col1, select1 );
    vmathV4Select( &result->col2, &mat0->col2, &mat1->col2, select1 );
    vmathV4Select( &result->col3, &mat0->col3, &mat1->col3, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathM4Print( const VmathMatrix4 *mat )
{
    VmathVector4 tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3;
    vmathM4GetRow( &tmpV4_0, mat, 0 );
    vmathV4Print( &tmpV4_0 );
    vmathM4GetRow( &tmpV4_1, mat, 1 );
    vmathV4Print( &tmpV4_1 );
    vmathM4GetRow( &tmpV4_2, mat, 2 );
    vmathV4Print( &tmpV4_2 );
    vmathM4GetRow( &tmpV4_3, mat, 3 );
    vmathV4Print( &tmpV4_3 );
}

static inline void vmathM4Prints( const VmathMatrix4 *mat, const char *name )
{
    printf("%s:\n", name);
    vmathM4Print( mat );
}

#endif

static inline void vmathT3Copy( VmathTransform3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( &result->col0, &tfrm->col0 );
    vmathV3Copy( &result->col1, &tfrm->col1 );
    vmathV3Copy( &result->col2, &tfrm->col2 );
    vmathV3Copy( &result->col3, &tfrm->col3 );
}

static inline void vmathT3MakeFromScalar( VmathTransform3 *result, float scalar )
{
    vmathV3MakeFromScalar( &result->col0, scalar );
    vmathV3MakeFromScalar( &result->col1, scalar );
    vmathV3MakeFromScalar( &result->col2, scalar );
    vmathV3MakeFromScalar( &result->col3, scalar );
}

static inline void vmathT3MakeFromCols( VmathTransform3 *result, const VmathVector3 *_col0, const VmathVector3 *_col1, const VmathVector3 *_col2, const VmathVector3 *_col3 )
{
    vmathV3Copy( &result->col0, _col0 );
    vmathV3Copy( &result->col1, _col1 );
    vmathV3Copy( &result->col2, _col2 );
    vmathV3Copy( &result->col3, _col3 );
}

static inline void vmathT3MakeFromM3V3( VmathTransform3 *result, const VmathMatrix3 *tfrm, const VmathVector3 *translateVec )
{
    vmathT3SetUpper3x3( result, tfrm );
    vmathT3SetTranslation( result, translateVec );
}

static inline void vmathT3MakeFromQV3( VmathTransform3 *result, const VmathQuat *unitQuat, const VmathVector3 *translateVec )
{
    VmathMatrix3 tmpM3_0;
    vmathM3MakeFromQ( &tmpM3_0, unitQuat );
    vmathT3SetUpper3x3( result, &tmpM3_0 );
    vmathT3SetTranslation( result, translateVec );
}

static inline void vmathT3SetCol0( VmathTransform3 *result, const VmathVector3 *_col0 )
{
    vmathV3Copy( &result->col0, _col0 );
}

static inline void vmathT3SetCol1( VmathTransform3 *result, const VmathVector3 *_col1 )
{
    vmathV3Copy( &result->col1, _col1 );
}

static inline void vmathT3SetCol2( VmathTransform3 *result, const VmathVector3 *_col2 )
{
    vmathV3Copy( &result->col2, _col2 );
}

static inline void vmathT3SetCol3( VmathTransform3 *result, const VmathVector3 *_col3 )
{
    vmathV3Copy( &result->col3, _col3 );
}

static inline void vmathT3SetCol( VmathTransform3 *result, int col, const VmathVector3 *vec )
{
    vmathV3Copy( (&result->col0 + col), vec );
}

static inline void vmathT3SetRow( VmathTransform3 *result, int row, const VmathVector4 *vec )
{
    vmathV3SetElem( &result->col0, row, vmathV4GetElem( vec, 0 ) );
    vmathV3SetElem( &result->col1, row, vmathV4GetElem( vec, 1 ) );
    vmathV3SetElem( &result->col2, row, vmathV4GetElem( vec, 2 ) );
    vmathV3SetElem( &result->col3, row, vmathV4GetElem( vec, 3 ) );
}

static inline void vmathT3SetElem( VmathTransform3 *result, int col, int row, float val )
{
    VmathVector3 tmpV3_0;
    vmathT3GetCol( &tmpV3_0, result, col );
    vmathV3SetElem( &tmpV3_0, row, val );
    vmathT3SetCol( result, col, &tmpV3_0 );
}

static inline float vmathT3GetElem( const VmathTransform3 *tfrm, int col, int row )
{
    VmathVector3 tmpV3_0;
    vmathT3GetCol( &tmpV3_0, tfrm, col );
    return vmathV3GetElem( &tmpV3_0, row );
}

static inline void vmathT3GetCol0( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col0 );
}

static inline void vmathT3GetCol1( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col1 );
}

static inline void vmathT3GetCol2( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col2 );
}

static inline void vmathT3GetCol3( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col3 );
}

static inline void vmathT3GetCol( VmathVector3 *result, const VmathTransform3 *tfrm, int col )
{
    vmathV3Copy( result, (&tfrm->col0 + col) );
}

static inline void vmathT3GetRow( VmathVector4 *result, const VmathTransform3 *tfrm, int row )
{
    vmathV4MakeFromElems( result, vmathV3GetElem( &tfrm->col0, row ), vmathV3GetElem( &tfrm->col1, row ), vmathV3GetElem( &tfrm->col2, row ), vmathV3GetElem( &tfrm->col3, row ) );
}

static inline void vmathT3Inverse( VmathTransform3 *result, const VmathTransform3 *tfrm )
{
    vec_float4 inv0, inv1, inv2, inv3;
    vec_float4 tmp0, tmp1, tmp2, tmp3, tmp4, dot, invdet;
    vec_float4 xxxx, yyyy, zzzz;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    tmp2 = _vmathVfCross( tfrm->col0.vec128, tfrm->col1.vec128 );
    tmp0 = _vmathVfCross( tfrm->col1.vec128, tfrm->col2.vec128 );
    tmp1 = _vmathVfCross( tfrm->col2.vec128, tfrm->col0.vec128 );
    inv3 = negatef4( tfrm->col3.vec128 );
    dot = _vmathVfDot3( tmp2, tfrm->col2.vec128 );
    dot = spu_shuffle( dot, dot, shuffle_xxxx );
    invdet = recipf4( dot );
    tmp3 = spu_shuffle( tmp0, tmp2, _VECTORMATH_SHUF_XAYB );
    tmp4 = spu_shuffle( tmp0, tmp2, _VECTORMATH_SHUF_ZCWD );
    inv0 = spu_shuffle( tmp3, tmp1, _VECTORMATH_SHUF_XAYB );
    xxxx = spu_shuffle( inv3, inv3, shuffle_xxxx );
    inv1 = spu_shuffle( tmp3, tmp1, _VECTORMATH_SHUF_ZBW0 );
    inv2 = spu_shuffle( tmp4, tmp1, _VECTORMATH_SHUF_XCY0 );
    yyyy = spu_shuffle( inv3, inv3, shuffle_yyyy );
    zzzz = spu_shuffle( inv3, inv3, shuffle_zzzz );
    inv3 = spu_mul( inv0, xxxx );
    inv3 = spu_madd( inv1, yyyy, inv3 );
    inv3 = spu_madd( inv2, zzzz, inv3 );
    inv0 = spu_mul( inv0, invdet );
    inv1 = spu_mul( inv1, invdet );
    inv2 = spu_mul( inv2, invdet );
    inv3 = spu_mul( inv3, invdet );
    result->col0.vec128 = inv0;
    result->col1.vec128 = inv1;
    result->col2.vec128 = inv2;
    result->col3.vec128 = inv3;
}

static inline void vmathT3OrthoInverse( VmathTransform3 *result, const VmathTransform3 *tfrm )
{
    vec_float4 inv0, inv1, inv2, inv3;
    vec_float4 tmp0, tmp1;
    vec_float4 xxxx, yyyy, zzzz;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    tmp0 = spu_shuffle( tfrm->col0.vec128, tfrm->col2.vec128, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( tfrm->col0.vec128, tfrm->col2.vec128, _VECTORMATH_SHUF_ZCWD );
    inv3 = negatef4( tfrm->col3.vec128 );
    inv0 = spu_shuffle( tmp0, tfrm->col1.vec128, _VECTORMATH_SHUF_XAYB );
    xxxx = spu_shuffle( inv3, inv3, shuffle_xxxx );
    inv1 = spu_shuffle( tmp0, tfrm->col1.vec128, _VECTORMATH_SHUF_ZBW0 );
    inv2 = spu_shuffle( tmp1, tfrm->col1.vec128, _VECTORMATH_SHUF_XCY0 );
    yyyy = spu_shuffle( inv3, inv3, shuffle_yyyy );
    zzzz = spu_shuffle( inv3, inv3, shuffle_zzzz );
    inv3 = spu_mul( inv0, xxxx );
    inv3 = spu_madd( inv1, yyyy, inv3 );
    inv3 = spu_madd( inv2, zzzz, inv3 );
    result->col0.vec128 = inv0;
    result->col1.vec128 = inv1;
    result->col2.vec128 = inv2;
    result->col3.vec128 = inv3;
}

static inline void vmathT3AbsPerElem( VmathTransform3 *result, const VmathTransform3 *tfrm )
{
    vmathV3AbsPerElem( &result->col0, &tfrm->col0 );
    vmathV3AbsPerElem( &result->col1, &tfrm->col1 );
    vmathV3AbsPerElem( &result->col2, &tfrm->col2 );
    vmathV3AbsPerElem( &result->col3, &tfrm->col3 );
}

static inline void vmathT3MulV3( VmathVector3 *result, const VmathTransform3 *tfrm, const VmathVector3 *vec )
{
    vec_float4 res;
    vec_float4 xxxx, yyyy, zzzz;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    xxxx = spu_shuffle( vec->vec128, vec->vec128, shuffle_xxxx );
    yyyy = spu_shuffle( vec->vec128, vec->vec128, shuffle_yyyy );
    zzzz = spu_shuffle( vec->vec128, vec->vec128, shuffle_zzzz );
    res = spu_mul( tfrm->col0.vec128, xxxx );
    res = spu_madd( tfrm->col1.vec128, yyyy, res );
    res = spu_madd( tfrm->col2.vec128, zzzz, res );
    result->vec128 = res;
}

static inline void vmathT3MulP3( VmathPoint3 *result, const VmathTransform3 *tfrm, const VmathPoint3 *pnt )
{
    vec_float4 tmp0, tmp1, res;
    vec_float4 xxxx, yyyy, zzzz;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    xxxx = spu_shuffle( pnt->vec128, pnt->vec128, shuffle_xxxx );
    yyyy = spu_shuffle( pnt->vec128, pnt->vec128, shuffle_yyyy );
    zzzz = spu_shuffle( pnt->vec128, pnt->vec128, shuffle_zzzz );
    tmp0 = spu_mul( tfrm->col0.vec128, xxxx );
    tmp1 = spu_mul( tfrm->col1.vec128, yyyy );
    tmp0 = spu_madd( tfrm->col2.vec128, zzzz, tmp0 );
    tmp1 = spu_add( tfrm->col3.vec128, tmp1 );
    res = spu_add( tmp0, tmp1 );
    result->vec128 = res;
}

static inline void vmathT3Mul( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1 )
{
    VmathTransform3 tmpResult;
    VmathPoint3 tmpP3_0, tmpP3_1;
    vmathT3MulV3( &tmpResult.col0, tfrm0, &tfrm1->col0 );
    vmathT3MulV3( &tmpResult.col1, tfrm0, &tfrm1->col1 );
    vmathT3MulV3( &tmpResult.col2, tfrm0, &tfrm1->col2 );
    vmathP3MakeFromV3( &tmpP3_0, &tfrm1->col3 );
    vmathT3MulP3( &tmpP3_1, tfrm0, &tmpP3_0 );
    vmathV3MakeFromP3( &tmpResult.col3, &tmpP3_1 );
    vmathT3Copy( result, &tmpResult );
}

static inline void vmathT3MulPerElem( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1 )
{
    vmathV3MulPerElem( &result->col0, &tfrm0->col0, &tfrm1->col0 );
    vmathV3MulPerElem( &result->col1, &tfrm0->col1, &tfrm1->col1 );
    vmathV3MulPerElem( &result->col2, &tfrm0->col2, &tfrm1->col2 );
    vmathV3MulPerElem( &result->col3, &tfrm0->col3, &tfrm1->col3 );
}

static inline void vmathT3MakeIdentity( VmathTransform3 *result )
{
    vmathV3MakeXAxis( &result->col0 );
    vmathV3MakeYAxis( &result->col1 );
    vmathV3MakeZAxis( &result->col2 );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3SetUpper3x3( VmathTransform3 *result, const VmathMatrix3 *tfrm )
{
    vmathV3Copy( &result->col0, &tfrm->col0 );
    vmathV3Copy( &result->col1, &tfrm->col1 );
    vmathV3Copy( &result->col2, &tfrm->col2 );
}

static inline void vmathT3GetUpper3x3( VmathMatrix3 *result, const VmathTransform3 *tfrm )
{
    vmathM3MakeFromCols( result, &tfrm->col0, &tfrm->col1, &tfrm->col2 );
}

static inline void vmathT3SetTranslation( VmathTransform3 *result, const VmathVector3 *translateVec )
{
    vmathV3Copy( &result->col3, translateVec );
}

static inline void vmathT3GetTranslation( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col3 );
}

static inline void vmathT3MakeRotationX( VmathTransform3 *result, float radians )
{
    vec_float4 s, c, res1, res2;
    vec_uint4 select_y, select_z;
    vec_float4 zero;
    select_y = (vec_uint4)spu_maskb(0x0f00);
    select_z = (vec_uint4)spu_maskb(0x00f0);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res1 = spu_sel( zero, c, select_y );
    res1 = spu_sel( res1, s, select_z );
    res2 = spu_sel( zero, negatef4(s), select_y );
    res2 = spu_sel( res2, c, select_z );
    vmathV3MakeXAxis( &result->col0 );
    result->col1.vec128 = res1;
    result->col2.vec128 = res2;
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3MakeRotationY( VmathTransform3 *result, float radians )
{
    vec_float4 s, c, res0, res2;
    vec_uint4 select_x, select_z;
    vec_float4 zero;
    select_x = (vec_uint4)spu_maskb(0xf000);
    select_z = (vec_uint4)spu_maskb(0x00f0);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res0 = spu_sel( zero, c, select_x );
    res0 = spu_sel( res0, negatef4(s), select_z );
    res2 = spu_sel( zero, s, select_x );
    res2 = spu_sel( res2, c, select_z );
    result->col0.vec128 = res0;
    vmathV3MakeYAxis( &result->col1 );
    result->col2.vec128 = res2;
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3MakeRotationZ( VmathTransform3 *result, float radians )
{
    vec_float4 s, c, res0, res1;
    vec_uint4 select_x, select_y;
    vec_float4 zero;
    select_x = (vec_uint4)spu_maskb(0xf000);
    select_y = (vec_uint4)spu_maskb(0x0f00);
    zero = spu_splats(0.0f);
    sincosf4( spu_splats(radians), &s, &c );
    res0 = spu_sel( zero, c, select_x );
    res0 = spu_sel( res0, s, select_y );
    res1 = spu_sel( zero, negatef4(s), select_x );
    res1 = spu_sel( res1, c, select_y );
    result->col0.vec128 = res0;
    result->col1.vec128 = res1;
    vmathV3MakeZAxis( &result->col2 );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3MakeRotationZYX( VmathTransform3 *result, const VmathVector3 *radiansXYZ )
{
    vec_float4 angles, s, negS, c, X0, X1, Y0, Y1, Z0, Z1, tmp;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    angles = radiansXYZ->vec128;
    angles = spu_insert( 0.0f, angles, 3 );
    sincosf4( angles, &s, &c );
    negS = negatef4( s );
    Z0 = spu_shuffle( s, c, _VECTORMATH_SHUF_CZD0 );
    Z1 = spu_shuffle( c, negS, _VECTORMATH_SHUF_CZD0 );
    Y0 = spu_shuffle( negS, c, _VECTORMATH_SHUF_BBY0 );
    Y1 = spu_shuffle( c, s, _VECTORMATH_SHUF_BBY0 );
    X0 = spu_shuffle( s, s, shuffle_xxxx );
    X1 = spu_shuffle( c, c, shuffle_xxxx );
    tmp = spu_mul( Z0, Y1 );
    result->col0.vec128 = spu_mul( Z0, Y0 );
    result->col1.vec128 = spu_madd( Z1, X1, spu_mul( tmp, X0 ) );
    result->col2.vec128 = spu_nmsub( Z1, X0, spu_mul( tmp, X1 ) );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3MakeRotationAxis( VmathTransform3 *result, float radians, const VmathVector3 *unitVec )
{
    VmathMatrix3 tmpM3_0;
    VmathVector3 tmpV3_0;
    vmathM3MakeRotationAxis( &tmpM3_0, radians, unitVec );
    vmathV3MakeFromScalar( &tmpV3_0, 0.0f );
    vmathT3MakeFromM3V3( result, &tmpM3_0, &tmpV3_0 );
}

static inline void vmathT3MakeRotationQ( VmathTransform3 *result, const VmathQuat *unitQuat )
{
    VmathMatrix3 tmpM3_0;
    VmathVector3 tmpV3_0;
    vmathM3MakeFromQ( &tmpM3_0, unitQuat );
    vmathV3MakeFromScalar( &tmpV3_0, 0.0f );
    vmathT3MakeFromM3V3( result, &tmpM3_0, &tmpV3_0 );
}

static inline void vmathT3MakeScale( VmathTransform3 *result, const VmathVector3 *scaleVec )
{
    vec_float4 zero = spu_splats(0.0f);
    result->col0.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0xf000) );
    result->col1.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0x0f00) );
    result->col2.vec128 = spu_sel( zero, scaleVec->vec128, (vec_uint4)spu_maskb(0x00f0) );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3AppendScale( VmathTransform3 *result, const VmathTransform3 *tfrm, const VmathVector3 *scaleVec )
{
    vmathV3ScalarMul( &result->col0, &tfrm->col0, vmathV3GetX( scaleVec ) );
    vmathV3ScalarMul( &result->col1, &tfrm->col1, vmathV3GetY( scaleVec ) );
    vmathV3ScalarMul( &result->col2, &tfrm->col2, vmathV3GetZ( scaleVec ) );
    vmathV3Copy( &result->col3, &tfrm->col3 );
}

static inline void vmathT3PrependScale( VmathTransform3 *result, const VmathVector3 *scaleVec, const VmathTransform3 *tfrm )
{
    vmathV3MulPerElem( &result->col0, &tfrm->col0, scaleVec );
    vmathV3MulPerElem( &result->col1, &tfrm->col1, scaleVec );
    vmathV3MulPerElem( &result->col2, &tfrm->col2, scaleVec );
    vmathV3MulPerElem( &result->col3, &tfrm->col3, scaleVec );
}

static inline void vmathT3MakeTranslation( VmathTransform3 *result, const VmathVector3 *translateVec )
{
    vmathV3MakeXAxis( &result->col0 );
    vmathV3MakeYAxis( &result->col1 );
    vmathV3MakeZAxis( &result->col2 );
    vmathV3Copy( &result->col3, translateVec );
}

static inline void vmathT3Select( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1, unsigned int select1 )
{
    vmathV3Select( &result->col0, &tfrm0->col0, &tfrm1->col0, select1 );
    vmathV3Select( &result->col1, &tfrm0->col1, &tfrm1->col1, select1 );
    vmathV3Select( &result->col2, &tfrm0->col2, &tfrm1->col2, select1 );
    vmathV3Select( &result->col3, &tfrm0->col3, &tfrm1->col3, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathT3Print( const VmathTransform3 *tfrm )
{
    VmathVector4 tmpV4_0, tmpV4_1, tmpV4_2;
    vmathT3GetRow( &tmpV4_0, tfrm, 0 );
    vmathV4Print( &tmpV4_0 );
    vmathT3GetRow( &tmpV4_1, tfrm, 1 );
    vmathV4Print( &tmpV4_1 );
    vmathT3GetRow( &tmpV4_2, tfrm, 2 );
    vmathV4Print( &tmpV4_2 );
}

static inline void vmathT3Prints( const VmathTransform3 *tfrm, const char *name )
{
    printf("%s:\n", name);
    vmathT3Print( tfrm );
}

#endif

static inline void vmathQMakeFromM3( VmathQuat *result, const VmathMatrix3 *tfrm )
{
    vec_float4 res;
    vec_float4 col0, col1, col2;
    vec_float4 xx_yy, xx_yy_zz_xx, yy_zz_xx_yy, zz_xx_yy_zz, diagSum, diagDiff;
    vec_float4 zy_xz_yx, yz_zx_xy, sum, diff;
    vec_float4 radicand, invSqrt, scale;
    vec_float4 res0, res1, res2, res3;
    vec_float4 xx, yy, zz;
    vec_uint4 select_x = (vec_uint4)spu_maskb( 0xf000 );
    vec_uint4 select_y = (vec_uint4)spu_maskb( 0x0f00 );
    vec_uint4 select_z = (vec_uint4)spu_maskb( 0x00f0 );
    vec_uint4 select_w = (vec_uint4)spu_maskb( 0x000f );
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((unsigned int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((unsigned int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((unsigned int)0x08090a0b);
    vec_uchar16 shuffle_wwww = (vec_uchar16)spu_splats((unsigned int)0x0c0d0e0f);

    col0 = tfrm->col0.vec128;
    col1 = tfrm->col1.vec128;
    col2 = tfrm->col2.vec128;

    /* four cases: */
    /* trace > 0 */
    /* else */
    /*    xx largest diagonal element */
    /*    yy largest diagonal element */
    /*    zz largest diagonal element */

    /* compute quaternion for each case */

    xx_yy = spu_sel( col0, col1, select_y );
    xx_yy_zz_xx = spu_shuffle( xx_yy, col2, _VECTORMATH_SHUF_XYCX );
    yy_zz_xx_yy = spu_shuffle( xx_yy, col2, _VECTORMATH_SHUF_YCXY );
    zz_xx_yy_zz = spu_shuffle( xx_yy, col2, _VECTORMATH_SHUF_CXYC );

    diagSum = spu_add( spu_add( xx_yy_zz_xx, yy_zz_xx_yy ), zz_xx_yy_zz );
    diagDiff = spu_sub( spu_sub( xx_yy_zz_xx, yy_zz_xx_yy ), zz_xx_yy_zz );
    radicand = spu_add( spu_sel( diagDiff, diagSum, select_w ), spu_splats(1.0f) );
    invSqrt = rsqrtf4( radicand );

    zy_xz_yx = spu_sel( col0, col1, select_z );
    zy_xz_yx = spu_shuffle( zy_xz_yx, col2, _VECTORMATH_SHUF_ZAY0 );
    yz_zx_xy = spu_sel( col0, col1, select_x );
    yz_zx_xy = spu_shuffle( yz_zx_xy, col2, _VECTORMATH_SHUF_BZX0 );

    sum = spu_add( zy_xz_yx, yz_zx_xy );
    diff = spu_sub( zy_xz_yx, yz_zx_xy );

    scale = spu_mul( invSqrt, spu_splats(0.5f) );
    res0 = spu_shuffle( sum, diff, _VECTORMATH_SHUF_0ZYA );
    res1 = spu_shuffle( sum, diff, _VECTORMATH_SHUF_Z0XB );
    res2 = spu_shuffle( sum, diff, _VECTORMATH_SHUF_YX0C );
    res3 = diff;
    res0 = spu_sel( res0, radicand, select_x );
    res1 = spu_sel( res1, radicand, select_y );
    res2 = spu_sel( res2, radicand, select_z );
    res3 = spu_sel( res3, radicand, select_w );
    res0 = spu_mul( res0, spu_shuffle( scale, scale, shuffle_xxxx ) );
    res1 = spu_mul( res1, spu_shuffle( scale, scale, shuffle_yyyy ) );
    res2 = spu_mul( res2, spu_shuffle( scale, scale, shuffle_zzzz ) );
    res3 = spu_mul( res3, spu_shuffle( scale, scale, shuffle_wwww ) );

    /* determine case and select answer */

    xx = spu_shuffle( col0, col0, shuffle_xxxx );
    yy = spu_shuffle( col1, col1, shuffle_yyyy );
    zz = spu_shuffle( col2, col2, shuffle_zzzz );
    res = spu_sel( res0, res1, spu_cmpgt( yy, xx ) );
    res = spu_sel( res, res2, spu_and( spu_cmpgt( zz, xx ), spu_cmpgt( zz, yy ) ) );
    res = spu_sel( res, res3, spu_cmpgt( spu_shuffle( diagSum, diagSum, shuffle_xxxx ), spu_splats(0.0f) ) );
    result->vec128 = res;
}

static inline void vmathV3Outer( VmathMatrix3 *result, const VmathVector3 *tfrm0, const VmathVector3 *tfrm1 )
{
    vmathV3ScalarMul( &result->col0, tfrm0, vmathV3GetX( tfrm1 ) );
    vmathV3ScalarMul( &result->col1, tfrm0, vmathV3GetY( tfrm1 ) );
    vmathV3ScalarMul( &result->col2, tfrm0, vmathV3GetZ( tfrm1 ) );
}

static inline void vmathV4Outer( VmathMatrix4 *result, const VmathVector4 *tfrm0, const VmathVector4 *tfrm1 )
{
    vmathV4ScalarMul( &result->col0, tfrm0, vmathV4GetX( tfrm1 ) );
    vmathV4ScalarMul( &result->col1, tfrm0, vmathV4GetY( tfrm1 ) );
    vmathV4ScalarMul( &result->col2, tfrm0, vmathV4GetZ( tfrm1 ) );
    vmathV4ScalarMul( &result->col3, tfrm0, vmathV4GetW( tfrm1 ) );
}

static inline void vmathV3RowMul( VmathVector3 *result, const VmathVector3 *vec, const VmathMatrix3 *mat )
{
    vec_float4 tmp0, tmp1, mcol0, mcol1, mcol2, res;
    vec_float4 xxxx, yyyy, zzzz;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    tmp0 = spu_shuffle( mat->col0.vec128, mat->col2.vec128, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( mat->col0.vec128, mat->col2.vec128, _VECTORMATH_SHUF_ZCWD );
    xxxx = spu_shuffle( vec->vec128, vec->vec128, shuffle_xxxx );
    mcol0 = spu_shuffle( tmp0, mat->col1.vec128, _VECTORMATH_SHUF_XAYB );
    mcol1 = spu_shuffle( tmp0, mat->col1.vec128, _VECTORMATH_SHUF_ZBW0 );
    mcol2 = spu_shuffle( tmp1, mat->col1.vec128, _VECTORMATH_SHUF_XCY0 );
    yyyy = spu_shuffle( vec->vec128, vec->vec128, shuffle_yyyy );
    res = spu_mul( mcol0, xxxx );
    zzzz = spu_shuffle( vec->vec128, vec->vec128, shuffle_zzzz );
    res = spu_madd( mcol1, yyyy, res );
    res = spu_madd( mcol2, zzzz, res );
    result->vec128 = res;
}

static inline void vmathV3CrossMatrix( VmathMatrix3 *result, const VmathVector3 *vec )
{
    vec_float4 neg, res0, res1, res2;
    neg = negatef4( vec->vec128 );
    res0 = spu_shuffle( vec->vec128, neg, _VECTORMATH_SHUF_0ZB0 );
    res1 = spu_shuffle( vec->vec128, neg, _VECTORMATH_SHUF_C0X0 );
    res2 = spu_shuffle( vec->vec128, neg, _VECTORMATH_SHUF_YA00 );
    result->col0.vec128 = res0;
    result->col1.vec128 = res1;
    result->col2.vec128 = res2;
}

static inline void vmathV3CrossMatrixMul( VmathMatrix3 *result, const VmathVector3 *vec, const VmathMatrix3 *mat )
{
    VmathVector3 tmpV3_0, tmpV3_1, tmpV3_2;
    vmathV3Cross( &tmpV3_0, vec, &mat->col0 );
    vmathV3Cross( &tmpV3_1, vec, &mat->col1 );
    vmathV3Cross( &tmpV3_2, vec, &mat->col2 );
    vmathM3MakeFromCols( result, &tmpV3_0, &tmpV3_1, &tmpV3_2 );
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
