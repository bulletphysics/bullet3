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

#ifndef _VECTORMATH_QUAT_SOA_C_H
#define _VECTORMATH_QUAT_SOA_C_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Definitions
 */
#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

static inline void vmathSoaQCopy( VmathSoaQuat *result, const VmathSoaQuat *quat )
{
    result->x = quat->x;
    result->y = quat->y;
    result->z = quat->z;
    result->w = quat->w;
}

static inline void vmathSoaQMakeFromElems( VmathSoaQuat *result, vec_float4 _x, vec_float4 _y, vec_float4 _z, vec_float4 _w )
{
    result->x = _x;
    result->y = _y;
    result->z = _z;
    result->w = _w;
}

static inline void vmathSoaQMakeFromV3Scalar( VmathSoaQuat *result, const VmathSoaVector3 *xyz, vec_float4 _w )
{
    vmathSoaQSetXYZ( result, xyz );
    vmathSoaQSetW( result, _w );
}

static inline void vmathSoaQMakeFromV4( VmathSoaQuat *result, const VmathSoaVector4 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
    result->w = vec->w;
}

static inline void vmathSoaQMakeFromScalar( VmathSoaQuat *result, vec_float4 scalar )
{
    result->x = scalar;
    result->y = scalar;
    result->z = scalar;
    result->w = scalar;
}

static inline void vmathSoaQMakeFromAos( VmathSoaQuat *result, const VmathQuat *quat )
{
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    vec_uchar16 shuffle_wwww = (vec_uchar16)spu_splats((int)0x0c0d0e0f);
    vec_float4 vec128 = quat->vec128;
    result->x = spu_shuffle( vec128, vec128, shuffle_xxxx );
    result->y = spu_shuffle( vec128, vec128, shuffle_yyyy );
    result->z = spu_shuffle( vec128, vec128, shuffle_zzzz );
    result->w = spu_shuffle( vec128, vec128, shuffle_wwww );
}

static inline void vmathSoaQMakeFrom4Aos( VmathSoaQuat *result, const VmathQuat *quat0, const VmathQuat *quat1, const VmathQuat *quat2, const VmathQuat *quat3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = spu_shuffle( quat0->vec128, quat2->vec128, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( quat1->vec128, quat3->vec128, _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( quat0->vec128, quat2->vec128, _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( quat1->vec128, quat3->vec128, _VECTORMATH_SHUF_ZCWD );
    result->x = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB );
    result->y = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD );
    result->z = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB );
    result->w = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_ZCWD );
}

static inline void vmathSoaQMakeIdentity( VmathSoaQuat *result )
{
    vmathSoaQMakeFromElems( result, spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f), spu_splats(1.0f) );
}

static inline void vmathSoaQLerp( VmathSoaQuat *result, vec_float4 t, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 )
{
    VmathSoaQuat tmpQ_0, tmpQ_1;
    vmathSoaQSub( &tmpQ_0, quat1, quat0 );
    vmathSoaQScalarMul( &tmpQ_1, &tmpQ_0, t );
    vmathSoaQAdd( result, quat0, &tmpQ_1 );
}

static inline void vmathSoaQSlerp( VmathSoaQuat *result, vec_float4 t, const VmathSoaQuat *unitQuat0, const VmathSoaQuat *unitQuat1 )
{
    VmathSoaQuat start, tmpQ_0, tmpQ_1;
    vec_float4 recipSinAngle, scale0, scale1, cosAngle, angle;
    vec_uint4 selectMask;
    cosAngle = vmathSoaQDot( unitQuat0, unitQuat1 );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(0.0f), cosAngle );
    cosAngle = spu_sel( cosAngle, negatef4( cosAngle ), selectMask );
    vmathSoaQSetX( &start, spu_sel( unitQuat0->x, negatef4( unitQuat0->x ), selectMask ) );
    vmathSoaQSetY( &start, spu_sel( unitQuat0->y, negatef4( unitQuat0->y ), selectMask ) );
    vmathSoaQSetZ( &start, spu_sel( unitQuat0->z, negatef4( unitQuat0->z ), selectMask ) );
    vmathSoaQSetW( &start, spu_sel( unitQuat0->w, negatef4( unitQuat0->w ), selectMask ) );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(_VECTORMATH_SLERP_TOL), cosAngle );
    angle = acosf4( cosAngle );
    recipSinAngle = recipf4( sinf4( angle ) );
    scale0 = spu_sel( spu_sub( spu_splats(1.0f), t ), spu_mul( sinf4( spu_mul( spu_sub( spu_splats(1.0f), t ), angle ) ), recipSinAngle ), selectMask );
    scale1 = spu_sel( t, spu_mul( sinf4( spu_mul( t, angle ) ), recipSinAngle ), selectMask );
    vmathSoaQScalarMul( &tmpQ_0, &start, scale0 );
    vmathSoaQScalarMul( &tmpQ_1, unitQuat1, scale1 );
    vmathSoaQAdd( result, &tmpQ_0, &tmpQ_1 );
}

static inline void vmathSoaQSquad( VmathSoaQuat *result, vec_float4 t, const VmathSoaQuat *unitQuat0, const VmathSoaQuat *unitQuat1, const VmathSoaQuat *unitQuat2, const VmathSoaQuat *unitQuat3 )
{
    VmathSoaQuat tmp0, tmp1;
    vmathSoaQSlerp( &tmp0, t, unitQuat0, unitQuat3 );
    vmathSoaQSlerp( &tmp1, t, unitQuat1, unitQuat2 );
    vmathSoaQSlerp( result, spu_mul( spu_mul( spu_splats(2.0f), t ), spu_sub( spu_splats(1.0f), t ) ), &tmp0, &tmp1 );
}

static inline void vmathSoaQGet4Aos( const VmathSoaQuat *quat, VmathQuat *result0, VmathQuat *result1, VmathQuat *result2, VmathQuat *result3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = spu_shuffle( quat->x, quat->z, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( quat->y, quat->w, _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( quat->x, quat->z, _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( quat->y, quat->w, _VECTORMATH_SHUF_ZCWD );
    vmathQMakeFrom128( result0, spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB ) );
    vmathQMakeFrom128( result1, spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD ) );
    vmathQMakeFrom128( result2, spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB ) );
    vmathQMakeFrom128( result3, spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_ZCWD ) );
}

static inline void vmathSoaQSetXYZ( VmathSoaQuat *result, const VmathSoaVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
}

static inline void vmathSoaQGetXYZ( VmathSoaVector3 *result, const VmathSoaQuat *quat )
{
    vmathSoaV3MakeFromElems( result, quat->x, quat->y, quat->z );
}

static inline void vmathSoaQSetX( VmathSoaQuat *result, vec_float4 _x )
{
    result->x = _x;
}

static inline vec_float4 vmathSoaQGetX( const VmathSoaQuat *quat )
{
    return quat->x;
}

static inline void vmathSoaQSetY( VmathSoaQuat *result, vec_float4 _y )
{
    result->y = _y;
}

static inline vec_float4 vmathSoaQGetY( const VmathSoaQuat *quat )
{
    return quat->y;
}

static inline void vmathSoaQSetZ( VmathSoaQuat *result, vec_float4 _z )
{
    result->z = _z;
}

static inline vec_float4 vmathSoaQGetZ( const VmathSoaQuat *quat )
{
    return quat->z;
}

static inline void vmathSoaQSetW( VmathSoaQuat *result, vec_float4 _w )
{
    result->w = _w;
}

static inline vec_float4 vmathSoaQGetW( const VmathSoaQuat *quat )
{
    return quat->w;
}

static inline void vmathSoaQSetElem( VmathSoaQuat *result, int idx, vec_float4 value )
{
    *(&result->x + idx) = value;
}

static inline vec_float4 vmathSoaQGetElem( const VmathSoaQuat *quat, int idx )
{
    return *(&quat->x + idx);
}

static inline void vmathSoaQAdd( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 )
{
    result->x = spu_add( quat0->x, quat1->x );
    result->y = spu_add( quat0->y, quat1->y );
    result->z = spu_add( quat0->z, quat1->z );
    result->w = spu_add( quat0->w, quat1->w );
}

static inline void vmathSoaQSub( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 )
{
    result->x = spu_sub( quat0->x, quat1->x );
    result->y = spu_sub( quat0->y, quat1->y );
    result->z = spu_sub( quat0->z, quat1->z );
    result->w = spu_sub( quat0->w, quat1->w );
}

static inline void vmathSoaQScalarMul( VmathSoaQuat *result, const VmathSoaQuat *quat, vec_float4 scalar )
{
    result->x = spu_mul( quat->x, scalar );
    result->y = spu_mul( quat->y, scalar );
    result->z = spu_mul( quat->z, scalar );
    result->w = spu_mul( quat->w, scalar );
}

static inline void vmathSoaQScalarDiv( VmathSoaQuat *result, const VmathSoaQuat *quat, vec_float4 scalar )
{
    result->x = divf4( quat->x, scalar );
    result->y = divf4( quat->y, scalar );
    result->z = divf4( quat->z, scalar );
    result->w = divf4( quat->w, scalar );
}

static inline void vmathSoaQNeg( VmathSoaQuat *result, const VmathSoaQuat *quat )
{
    result->x = negatef4( quat->x );
    result->y = negatef4( quat->y );
    result->z = negatef4( quat->z );
    result->w = negatef4( quat->w );
}

static inline vec_float4 vmathSoaQDot( const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 )
{
    vec_float4 result;
    result = spu_mul( quat0->x, quat1->x );
    result = spu_add( result, spu_mul( quat0->y, quat1->y ) );
    result = spu_add( result, spu_mul( quat0->z, quat1->z ) );
    result = spu_add( result, spu_mul( quat0->w, quat1->w ) );
    return result;
}

static inline vec_float4 vmathSoaQNorm( const VmathSoaQuat *quat )
{
    vec_float4 result;
    result = spu_mul( quat->x, quat->x );
    result = spu_add( result, spu_mul( quat->y, quat->y ) );
    result = spu_add( result, spu_mul( quat->z, quat->z ) );
    result = spu_add( result, spu_mul( quat->w, quat->w ) );
    return result;
}

static inline vec_float4 vmathSoaQLength( const VmathSoaQuat *quat )
{
    return sqrtf4( vmathSoaQNorm( quat ) );
}

static inline void vmathSoaQNormalize( VmathSoaQuat *result, const VmathSoaQuat *quat )
{
    vec_float4 lenSqr, lenInv;
    lenSqr = vmathSoaQNorm( quat );
    lenInv = rsqrtf4( lenSqr );
    result->x = spu_mul( quat->x, lenInv );
    result->y = spu_mul( quat->y, lenInv );
    result->z = spu_mul( quat->z, lenInv );
    result->w = spu_mul( quat->w, lenInv );
}

static inline void vmathSoaQMakeRotationArc( VmathSoaQuat *result, const VmathSoaVector3 *unitVec0, const VmathSoaVector3 *unitVec1 )
{
    VmathSoaVector3 tmpV3_0, tmpV3_1;
    vec_float4 cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf4( spu_mul( spu_splats(2.0f), spu_add( spu_splats(1.0f), vmathSoaV3Dot( unitVec0, unitVec1 ) ) ) );
    recipCosHalfAngleX2 = recipf4( cosHalfAngleX2 );
    vmathSoaV3Cross( &tmpV3_0, unitVec0, unitVec1 );
    vmathSoaV3ScalarMul( &tmpV3_1, &tmpV3_0, recipCosHalfAngleX2 );
    vmathSoaQMakeFromV3Scalar( result, &tmpV3_1, spu_mul( cosHalfAngleX2, spu_splats(0.5f) ) );
}

static inline void vmathSoaQMakeRotationAxis( VmathSoaQuat *result, vec_float4 radians, const VmathSoaVector3 *unitVec )
{
    VmathSoaVector3 tmpV3_0;
    vec_float4 s, c, angle;
    angle = spu_mul( radians, spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    vmathSoaV3ScalarMul( &tmpV3_0, unitVec, s );
    vmathSoaQMakeFromV3Scalar( result, &tmpV3_0, c );
}

static inline void vmathSoaQMakeRotationX( VmathSoaQuat *result, vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = spu_mul( radians, spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    vmathSoaQMakeFromElems( result, s, spu_splats(0.0f), spu_splats(0.0f), c );
}

static inline void vmathSoaQMakeRotationY( VmathSoaQuat *result, vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = spu_mul( radians, spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    vmathSoaQMakeFromElems( result, spu_splats(0.0f), s, spu_splats(0.0f), c );
}

static inline void vmathSoaQMakeRotationZ( VmathSoaQuat *result, vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = spu_mul( radians, spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    vmathSoaQMakeFromElems( result, spu_splats(0.0f), spu_splats(0.0f), s, c );
}

static inline void vmathSoaQMul( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 )
{
    vec_float4 tmpX, tmpY, tmpZ, tmpW;
    tmpX = spu_sub( spu_add( spu_add( spu_mul( quat0->w, quat1->x ), spu_mul( quat0->x, quat1->w ) ), spu_mul( quat0->y, quat1->z ) ), spu_mul( quat0->z, quat1->y ) );
    tmpY = spu_sub( spu_add( spu_add( spu_mul( quat0->w, quat1->y ), spu_mul( quat0->y, quat1->w ) ), spu_mul( quat0->z, quat1->x ) ), spu_mul( quat0->x, quat1->z ) );
    tmpZ = spu_sub( spu_add( spu_add( spu_mul( quat0->w, quat1->z ), spu_mul( quat0->z, quat1->w ) ), spu_mul( quat0->x, quat1->y ) ), spu_mul( quat0->y, quat1->x ) );
    tmpW = spu_sub( spu_sub( spu_sub( spu_mul( quat0->w, quat1->w ), spu_mul( quat0->x, quat1->x ) ), spu_mul( quat0->y, quat1->y ) ), spu_mul( quat0->z, quat1->z ) );
    vmathSoaQMakeFromElems( result, tmpX, tmpY, tmpZ, tmpW );
}

static inline void vmathSoaQRotate( VmathSoaVector3 *result, const VmathSoaQuat *quat, const VmathSoaVector3 *vec )
{
    vec_float4 tmpX, tmpY, tmpZ, tmpW;
    tmpX = spu_sub( spu_add( spu_mul( quat->w, vec->x ), spu_mul( quat->y, vec->z ) ), spu_mul( quat->z, vec->y ) );
    tmpY = spu_sub( spu_add( spu_mul( quat->w, vec->y ), spu_mul( quat->z, vec->x ) ), spu_mul( quat->x, vec->z ) );
    tmpZ = spu_sub( spu_add( spu_mul( quat->w, vec->z ), spu_mul( quat->x, vec->y ) ), spu_mul( quat->y, vec->x ) );
    tmpW = spu_add( spu_add( spu_mul( quat->x, vec->x ), spu_mul( quat->y, vec->y ) ), spu_mul( quat->z, vec->z ) );
    result->x = spu_add( spu_sub( spu_add( spu_mul( tmpW, quat->x ), spu_mul( tmpX, quat->w ) ), spu_mul( tmpY, quat->z ) ), spu_mul( tmpZ, quat->y ) );
    result->y = spu_add( spu_sub( spu_add( spu_mul( tmpW, quat->y ), spu_mul( tmpY, quat->w ) ), spu_mul( tmpZ, quat->x ) ), spu_mul( tmpX, quat->z ) );
    result->z = spu_add( spu_sub( spu_add( spu_mul( tmpW, quat->z ), spu_mul( tmpZ, quat->w ) ), spu_mul( tmpX, quat->y ) ), spu_mul( tmpY, quat->x ) );
}

static inline void vmathSoaQConj( VmathSoaQuat *result, const VmathSoaQuat *quat )
{
    vmathSoaQMakeFromElems( result, negatef4( quat->x ), negatef4( quat->y ), negatef4( quat->z ), quat->w );
}

static inline void vmathSoaQSelect( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1, vec_uint4 select1 )
{
    result->x = spu_sel( quat0->x, quat1->x, select1 );
    result->y = spu_sel( quat0->y, quat1->y, select1 );
    result->z = spu_sel( quat0->z, quat1->z, select1 );
    result->w = spu_sel( quat0->w, quat1->w, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaQPrint( const VmathSoaQuat *quat )
{
    VmathQuat vec0, vec1, vec2, vec3;
    vmathSoaQGet4Aos( quat, &vec0, &vec1, &vec2, &vec3 );
    printf("slot 0:\n");
    vmathQPrint( &vec0 );
    printf("slot 1:\n");
    vmathQPrint( &vec1 );
    printf("slot 2:\n");
    vmathQPrint( &vec2 );
    printf("slot 3:\n");
    vmathQPrint( &vec3 );
}

static inline void vmathSoaQPrints( const VmathSoaQuat *quat, const char *name )
{
    VmathQuat vec0, vec1, vec2, vec3;
    printf( "%s:\n", name );
    vmathSoaQGet4Aos( quat, &vec0, &vec1, &vec2, &vec3 );
    printf("slot 0:\n");
    vmathQPrint( &vec0 );
    printf("slot 1:\n");
    vmathQPrint( &vec1 );
    printf("slot 2:\n");
    vmathQPrint( &vec2 );
    printf("slot 3:\n");
    vmathQPrint( &vec3 );
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
