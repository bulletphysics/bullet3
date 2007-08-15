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
 * for permutes, words are labeled [x,y,z,w] [a,b,c,d]
 */
#define _VECTORMATH_PERM_X 0x00010203
#define _VECTORMATH_PERM_Y 0x04050607
#define _VECTORMATH_PERM_Z 0x08090a0b
#define _VECTORMATH_PERM_W 0x0c0d0e0f
#define _VECTORMATH_PERM_A 0x10111213
#define _VECTORMATH_PERM_B 0x14151617
#define _VECTORMATH_PERM_C 0x18191a1b
#define _VECTORMATH_PERM_D 0x1c1d1e1f
#define _VECTORMATH_PERM_ZBWX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_B, _VECTORMATH_PERM_W, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_XCYX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_C, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_ZDWX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_D, _VECTORMATH_PERM_W, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_ZCXA ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_C, _VECTORMATH_PERM_X, _VECTORMATH_PERM_A })
#define _VECTORMATH_PERM_XBZD ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_B, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_D })
#define _VECTORMATH_PERM_WDYB ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_W, _VECTORMATH_PERM_D, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_B })
#define _VECTORMATH_PERM_ZBXD ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_B, _VECTORMATH_PERM_X, _VECTORMATH_PERM_D })
#define _VECTORMATH_PERM_WCYA ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_W, _VECTORMATH_PERM_C, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_A })
#define _VECTORMATH_PERM_XDZB ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_D, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_B })
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
    vec_float4 vec128 = vec->vec128;
    result->x = vec_splat( vec128, 0 );
    result->y = vec_splat( vec128, 1 );
    result->z = vec_splat( vec128, 2 );
}

static inline void vmathSoaV3MakeFrom4Aos( VmathSoaVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1, const VmathVector3 *vec2, const VmathVector3 *vec3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = vec_mergeh( vec0->vec128, vec2->vec128 );
    tmp1 = vec_mergeh( vec1->vec128, vec3->vec128 );
    tmp2 = vec_mergel( vec0->vec128, vec2->vec128 );
    tmp3 = vec_mergel( vec1->vec128, vec3->vec128 );
    result->x = vec_mergeh( tmp0, tmp1 );
    result->y = vec_mergel( tmp0, tmp1 );
    result->z = vec_mergeh( tmp2, tmp3 );
}

static inline void vmathSoaV3MakeXAxis( VmathSoaVector3 *result )
{
    vmathSoaV3MakeFromElems( result, ((vec_float4){1.0f,1.0f,1.0f,1.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaV3MakeYAxis( VmathSoaVector3 *result )
{
    vmathSoaV3MakeFromElems( result, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){1.0f,1.0f,1.0f,1.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaV3MakeZAxis( VmathSoaVector3 *result )
{
    vmathSoaV3MakeFromElems( result, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
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
    selectMask = (vec_uint4)vec_cmpgt( (vec_float4){_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL}, cosAngle );
    angle = acosf4( cosAngle );
    recipSinAngle = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sinf4( angle ) );
    scale0 = vec_sel( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), vec_madd( sinf4( vec_madd( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), recipSinAngle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), selectMask );
    scale1 = vec_sel( t, vec_madd( sinf4( vec_madd( t, angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), recipSinAngle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), selectMask );
    vmathSoaV3ScalarMul( &tmpV3_0, unitVec0, scale0 );
    vmathSoaV3ScalarMul( &tmpV3_1, unitVec1, scale1 );
    vmathSoaV3Add( result, &tmpV3_0, &tmpV3_1 );
}

static inline void vmathSoaV3Get4Aos( const VmathSoaVector3 *vec, VmathVector3 *result0, VmathVector3 *result1, VmathVector3 *result2, VmathVector3 *result3 )
{
    vec_float4 tmp0, tmp1;
    tmp0 = vec_mergeh( vec->x, vec->z );
    tmp1 = vec_mergel( vec->x, vec->z );
    vmathV3MakeFrom128( result0, vec_mergeh( tmp0, vec->y ) );
    vmathV3MakeFrom128( result1, vec_perm( tmp0, vec->y, _VECTORMATH_PERM_ZBWX ) );
    vmathV3MakeFrom128( result2, vec_perm( tmp1, vec->y, _VECTORMATH_PERM_XCYX ) );
    vmathV3MakeFrom128( result3, vec_perm( tmp1, vec->y, _VECTORMATH_PERM_ZDWX ) );
}

static inline void vmathSoaV3LoadXYZArray( VmathSoaVector3 *vec, const vec_float4 *threeQuads )
{
    vec_float4 xyxy, yzyz, zxzx, xyzx, yzxy, zxyz;
    xyzx = threeQuads[0];
    yzxy = threeQuads[1];
    zxyz = threeQuads[2];
    xyxy = vec_sld( yzxy, xyzx, 8 );
    zxzx = vec_sld( xyzx, zxyz, 8 );
    yzyz = vec_sld( zxyz, yzxy, 8 );
    vmathSoaV3SetX( vec, vec_perm( xyxy, zxzx, _VECTORMATH_PERM_ZBXD ) );
    vmathSoaV3SetY( vec, vec_perm( xyxy, yzyz, _VECTORMATH_PERM_WCYA ) );
    vmathSoaV3SetZ( vec, vec_perm( zxzx, yzyz, _VECTORMATH_PERM_XDZB ) );
}

static inline void vmathSoaV3StoreXYZArray( const VmathSoaVector3 *vec, vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz, xyxy, zxzx, yzyz;
    xyxy = vec_perm( vec->x, vec->y, _VECTORMATH_PERM_ZCXA );
    zxzx = vec_perm( vec->z, vec->x, _VECTORMATH_PERM_XBZD );
    yzyz = vec_perm( vec->y, vec->z, _VECTORMATH_PERM_WDYB );
    xyzx = vec_sld( xyxy, zxzx, 8 );
    yzxy = vec_sld( yzyz, xyxy, 8 );
    zxyz = vec_sld( zxzx, yzyz, 8 );
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
    result->x = vec_add( vec0->x, vec1->x );
    result->y = vec_add( vec0->y, vec1->y );
    result->z = vec_add( vec0->z, vec1->z );
}

static inline void vmathSoaV3Sub( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = vec_sub( vec0->x, vec1->x );
    result->y = vec_sub( vec0->y, vec1->y );
    result->z = vec_sub( vec0->z, vec1->z );
}

static inline void vmathSoaV3AddP3( VmathSoaPoint3 *result, const VmathSoaVector3 *vec, const VmathSoaPoint3 *pnt1 )
{
    result->x = vec_add( vec->x, pnt1->x );
    result->y = vec_add( vec->y, pnt1->y );
    result->z = vec_add( vec->z, pnt1->z );
}

static inline void vmathSoaV3ScalarMul( VmathSoaVector3 *result, const VmathSoaVector3 *vec, vec_float4 scalar )
{
    result->x = vec_madd( vec->x, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( vec->y, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( vec->z, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
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
    result->x = vec_madd( vec0->x, vec1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( vec0->y, vec1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( vec0->z, vec1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaV3DivPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    result->x = divf4( vec0->x, vec1->x );
    result->y = divf4( vec0->y, vec1->y );
    result->z = divf4( vec0->z, vec1->z );
}

static inline void vmathSoaV3RecipPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec->x );
    result->y = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec->y );
    result->z = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec->z );
}

static inline void vmathSoaV3SqrtPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = sqrtf4( vec->x );
    result->y = sqrtf4( vec->y );
    result->z = sqrtf4( vec->z );
}

static inline void vmathSoaV3RsqrtPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec )
{
    result->x = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( vec->x ) );
    result->y = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( vec->y ) );
    result->z = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( vec->z ) );
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
    result = vec_add( vec->x, vec->y );
    result = vec_add( result, vec->z );
    return result;
}

static inline vec_float4 vmathSoaV3Dot( const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    vec_float4 result;
    result = vec_madd( vec0->x, vec1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( vec0->y, vec1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( vec0->z, vec1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    return result;
}

static inline vec_float4 vmathSoaV3LengthSqr( const VmathSoaVector3 *vec )
{
    vec_float4 result;
    result = vec_madd( vec->x, vec->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( vec->y, vec->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( vec->z, vec->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
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
    lenInv = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( lenSqr ) );
    result->x = vec_madd( vec->x, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( vec->y, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( vec->z, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaV3Cross( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 )
{
    vec_float4 tmpX, tmpY, tmpZ;
    tmpX = vec_sub( vec_madd( vec0->y, vec1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( vec0->z, vec1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpY = vec_sub( vec_madd( vec0->z, vec1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( vec0->x, vec1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpZ = vec_sub( vec_madd( vec0->x, vec1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( vec0->y, vec1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    vmathSoaV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathSoaV3Select( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1, vec_uint4 select1 )
{
    result->x = vec_sel( vec0->x, vec1->x, select1 );
    result->y = vec_sel( vec0->y, vec1->y, select1 );
    result->z = vec_sel( vec0->z, vec1->z, select1 );
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
    result->w = ((vec_float4){0.0f,0.0f,0.0f,0.0f});
}

static inline void vmathSoaV4MakeFromP3( VmathSoaVector4 *result, const VmathSoaPoint3 *pnt )
{
    result->x = pnt->x;
    result->y = pnt->y;
    result->z = pnt->z;
    result->w = ((vec_float4){1.0f,1.0f,1.0f,1.0f});
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
    vec_float4 vec128 = vec->vec128;
    result->x = vec_splat( vec128, 0 );
    result->y = vec_splat( vec128, 1 );
    result->z = vec_splat( vec128, 2 );
    result->w = vec_splat( vec128, 3 );
}

static inline void vmathSoaV4MakeFrom4Aos( VmathSoaVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1, const VmathVector4 *vec2, const VmathVector4 *vec3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = vec_mergeh( vec0->vec128, vec2->vec128 );
    tmp1 = vec_mergeh( vec1->vec128, vec3->vec128 );
    tmp2 = vec_mergel( vec0->vec128, vec2->vec128 );
    tmp3 = vec_mergel( vec1->vec128, vec3->vec128 );
    result->x = vec_mergeh( tmp0, tmp1 );
    result->y = vec_mergel( tmp0, tmp1 );
    result->z = vec_mergeh( tmp2, tmp3 );
    result->w = vec_mergel( tmp2, tmp3 );
}

static inline void vmathSoaV4MakeXAxis( VmathSoaVector4 *result )
{
    vmathSoaV4MakeFromElems( result, ((vec_float4){1.0f,1.0f,1.0f,1.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaV4MakeYAxis( VmathSoaVector4 *result )
{
    vmathSoaV4MakeFromElems( result, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){1.0f,1.0f,1.0f,1.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaV4MakeZAxis( VmathSoaVector4 *result )
{
    vmathSoaV4MakeFromElems( result, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){1.0f,1.0f,1.0f,1.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaV4MakeWAxis( VmathSoaVector4 *result )
{
    vmathSoaV4MakeFromElems( result, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
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
    selectMask = (vec_uint4)vec_cmpgt( (vec_float4){_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL}, cosAngle );
    angle = acosf4( cosAngle );
    recipSinAngle = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sinf4( angle ) );
    scale0 = vec_sel( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), vec_madd( sinf4( vec_madd( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), recipSinAngle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), selectMask );
    scale1 = vec_sel( t, vec_madd( sinf4( vec_madd( t, angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), recipSinAngle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), selectMask );
    vmathSoaV4ScalarMul( &tmpV4_0, unitVec0, scale0 );
    vmathSoaV4ScalarMul( &tmpV4_1, unitVec1, scale1 );
    vmathSoaV4Add( result, &tmpV4_0, &tmpV4_1 );
}

static inline void vmathSoaV4Get4Aos( const VmathSoaVector4 *vec, VmathVector4 *result0, VmathVector4 *result1, VmathVector4 *result2, VmathVector4 *result3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = vec_mergeh( vec->x, vec->z );
    tmp1 = vec_mergeh( vec->y, vec->w );
    tmp2 = vec_mergel( vec->x, vec->z );
    tmp3 = vec_mergel( vec->y, vec->w );
    vmathV4MakeFrom128( result0, vec_mergeh( tmp0, tmp1 ) );
    vmathV4MakeFrom128( result1, vec_mergel( tmp0, tmp1 ) );
    vmathV4MakeFrom128( result2, vec_mergeh( tmp2, tmp3 ) );
    vmathV4MakeFrom128( result3, vec_mergel( tmp2, tmp3 ) );
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
    result->x = vec_add( vec0->x, vec1->x );
    result->y = vec_add( vec0->y, vec1->y );
    result->z = vec_add( vec0->z, vec1->z );
    result->w = vec_add( vec0->w, vec1->w );
}

static inline void vmathSoaV4Sub( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    result->x = vec_sub( vec0->x, vec1->x );
    result->y = vec_sub( vec0->y, vec1->y );
    result->z = vec_sub( vec0->z, vec1->z );
    result->w = vec_sub( vec0->w, vec1->w );
}

static inline void vmathSoaV4ScalarMul( VmathSoaVector4 *result, const VmathSoaVector4 *vec, vec_float4 scalar )
{
    result->x = vec_madd( vec->x, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( vec->y, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( vec->z, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->w = vec_madd( vec->w, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
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
    result->x = vec_madd( vec0->x, vec1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( vec0->y, vec1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( vec0->z, vec1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->w = vec_madd( vec0->w, vec1->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
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
    result->x = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec->x );
    result->y = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec->y );
    result->z = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec->z );
    result->w = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec->w );
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
    result->x = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( vec->x ) );
    result->y = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( vec->y ) );
    result->z = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( vec->z ) );
    result->w = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( vec->w ) );
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
    result = vec_add( vec->x, vec->y );
    result = vec_add( result, vec->z );
    result = vec_add( result, vec->w );
    return result;
}

static inline vec_float4 vmathSoaV4Dot( const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 )
{
    vec_float4 result;
    result = vec_madd( vec0->x, vec1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( vec0->y, vec1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( vec0->z, vec1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( vec0->w, vec1->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    return result;
}

static inline vec_float4 vmathSoaV4LengthSqr( const VmathSoaVector4 *vec )
{
    vec_float4 result;
    result = vec_madd( vec->x, vec->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( vec->y, vec->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( vec->z, vec->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( vec->w, vec->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
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
    lenInv = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( lenSqr ) );
    result->x = vec_madd( vec->x, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( vec->y, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( vec->z, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->w = vec_madd( vec->w, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaV4Select( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1, vec_uint4 select1 )
{
    result->x = vec_sel( vec0->x, vec1->x, select1 );
    result->y = vec_sel( vec0->y, vec1->y, select1 );
    result->z = vec_sel( vec0->z, vec1->z, select1 );
    result->w = vec_sel( vec0->w, vec1->w, select1 );
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
    vec_float4 vec128 = pnt->vec128;
    result->x = vec_splat( vec128, 0 );
    result->y = vec_splat( vec128, 1 );
    result->z = vec_splat( vec128, 2 );
}

static inline void vmathSoaP3MakeFrom4Aos( VmathSoaPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1, const VmathPoint3 *pnt2, const VmathPoint3 *pnt3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = vec_mergeh( pnt0->vec128, pnt2->vec128 );
    tmp1 = vec_mergeh( pnt1->vec128, pnt3->vec128 );
    tmp2 = vec_mergel( pnt0->vec128, pnt2->vec128 );
    tmp3 = vec_mergel( pnt1->vec128, pnt3->vec128 );
    result->x = vec_mergeh( tmp0, tmp1 );
    result->y = vec_mergel( tmp0, tmp1 );
    result->z = vec_mergeh( tmp2, tmp3 );
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
    tmp0 = vec_mergeh( pnt->x, pnt->z );
    tmp1 = vec_mergel( pnt->x, pnt->z );
    vmathP3MakeFrom128( result0, vec_mergeh( tmp0, pnt->y ) );
    vmathP3MakeFrom128( result1, vec_perm( tmp0, pnt->y, _VECTORMATH_PERM_ZBWX ) );
    vmathP3MakeFrom128( result2, vec_perm( tmp1, pnt->y, _VECTORMATH_PERM_XCYX ) );
    vmathP3MakeFrom128( result3, vec_perm( tmp1, pnt->y, _VECTORMATH_PERM_ZDWX ) );
}

static inline void vmathSoaP3LoadXYZArray( VmathSoaPoint3 *vec, const vec_float4 *threeQuads )
{
    vec_float4 xyxy, yzyz, zxzx, xyzx, yzxy, zxyz;
    xyzx = threeQuads[0];
    yzxy = threeQuads[1];
    zxyz = threeQuads[2];
    xyxy = vec_sld( yzxy, xyzx, 8 );
    zxzx = vec_sld( xyzx, zxyz, 8 );
    yzyz = vec_sld( zxyz, yzxy, 8 );
    vmathSoaP3SetX( vec, vec_perm( xyxy, zxzx, _VECTORMATH_PERM_ZBXD ) );
    vmathSoaP3SetY( vec, vec_perm( xyxy, yzyz, _VECTORMATH_PERM_WCYA ) );
    vmathSoaP3SetZ( vec, vec_perm( zxzx, yzyz, _VECTORMATH_PERM_XDZB ) );
}

static inline void vmathSoaP3StoreXYZArray( const VmathSoaPoint3 *vec, vec_float4 *threeQuads )
{
    vec_float4 xyzx, yzxy, zxyz, xyxy, zxzx, yzyz;
    xyxy = vec_perm( vec->x, vec->y, _VECTORMATH_PERM_ZCXA );
    zxzx = vec_perm( vec->z, vec->x, _VECTORMATH_PERM_XBZD );
    yzyz = vec_perm( vec->y, vec->z, _VECTORMATH_PERM_WDYB );
    xyzx = vec_sld( xyxy, zxzx, 8 );
    yzxy = vec_sld( yzyz, xyxy, 8 );
    zxyz = vec_sld( zxzx, yzyz, 8 );
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
    result->x = vec_sub( pnt0->x, pnt1->x );
    result->y = vec_sub( pnt0->y, pnt1->y );
    result->z = vec_sub( pnt0->z, pnt1->z );
}

static inline void vmathSoaP3AddV3( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, const VmathSoaVector3 *vec1 )
{
    result->x = vec_add( pnt->x, vec1->x );
    result->y = vec_add( pnt->y, vec1->y );
    result->z = vec_add( pnt->z, vec1->z );
}

static inline void vmathSoaP3SubV3( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, const VmathSoaVector3 *vec1 )
{
    result->x = vec_sub( pnt->x, vec1->x );
    result->y = vec_sub( pnt->y, vec1->y );
    result->z = vec_sub( pnt->z, vec1->z );
}

static inline void vmathSoaP3MulPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    result->x = vec_madd( pnt0->x, pnt1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( pnt0->y, pnt1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( pnt0->z, pnt1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaP3DivPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 )
{
    result->x = divf4( pnt0->x, pnt1->x );
    result->y = divf4( pnt0->y, pnt1->y );
    result->z = divf4( pnt0->z, pnt1->z );
}

static inline void vmathSoaP3RecipPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), pnt->x );
    result->y = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), pnt->y );
    result->z = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), pnt->z );
}

static inline void vmathSoaP3SqrtPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = sqrtf4( pnt->x );
    result->y = sqrtf4( pnt->y );
    result->z = sqrtf4( pnt->z );
}

static inline void vmathSoaP3RsqrtPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt )
{
    result->x = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( pnt->x ) );
    result->y = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( pnt->y ) );
    result->z = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( pnt->z ) );
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
    result = vec_add( pnt->x, pnt->y );
    result = vec_add( result, pnt->z );
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
    result = vec_madd( pnt->x, unitVec->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( pnt->y, unitVec->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( pnt->z, unitVec->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
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
    result->x = vec_sel( pnt0->x, pnt1->x, select1 );
    result->y = vec_sel( pnt0->y, pnt1->y, select1 );
    result->z = vec_sel( pnt0->z, pnt1->z, select1 );
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
