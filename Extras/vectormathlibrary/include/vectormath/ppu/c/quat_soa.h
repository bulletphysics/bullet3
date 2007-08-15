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
    vec_float4 vec128 = quat->vec128;
    result->x = vec_splat( vec128, 0 );
    result->y = vec_splat( vec128, 1 );
    result->z = vec_splat( vec128, 2 );
    result->w = vec_splat( vec128, 3 );
}

static inline void vmathSoaQMakeFrom4Aos( VmathSoaQuat *result, const VmathQuat *quat0, const VmathQuat *quat1, const VmathQuat *quat2, const VmathQuat *quat3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = vec_mergeh( quat0->vec128, quat2->vec128 );
    tmp1 = vec_mergeh( quat1->vec128, quat3->vec128 );
    tmp2 = vec_mergel( quat0->vec128, quat2->vec128 );
    tmp3 = vec_mergel( quat1->vec128, quat3->vec128 );
    result->x = vec_mergeh( tmp0, tmp1 );
    result->y = vec_mergel( tmp0, tmp1 );
    result->z = vec_mergeh( tmp2, tmp3 );
    result->w = vec_mergel( tmp2, tmp3 );
}

static inline void vmathSoaQMakeIdentity( VmathSoaQuat *result )
{
    vmathSoaQMakeFromElems( result, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
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
    selectMask = (vec_uint4)vec_cmpgt( (vec_float4){0.0f,0.0f,0.0f,0.0f}, cosAngle );
    cosAngle = vec_sel( cosAngle, negatef4( cosAngle ), selectMask );
    vmathSoaQSetX( &start, vec_sel( unitQuat0->x, negatef4( unitQuat0->x ), selectMask ) );
    vmathSoaQSetY( &start, vec_sel( unitQuat0->y, negatef4( unitQuat0->y ), selectMask ) );
    vmathSoaQSetZ( &start, vec_sel( unitQuat0->z, negatef4( unitQuat0->z ), selectMask ) );
    vmathSoaQSetW( &start, vec_sel( unitQuat0->w, negatef4( unitQuat0->w ), selectMask ) );
    selectMask = (vec_uint4)vec_cmpgt( (vec_float4){_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL}, cosAngle );
    angle = acosf4( cosAngle );
    recipSinAngle = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sinf4( angle ) );
    scale0 = vec_sel( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), vec_madd( sinf4( vec_madd( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), recipSinAngle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), selectMask );
    scale1 = vec_sel( t, vec_madd( sinf4( vec_madd( t, angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), recipSinAngle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), selectMask );
    vmathSoaQScalarMul( &tmpQ_0, &start, scale0 );
    vmathSoaQScalarMul( &tmpQ_1, unitQuat1, scale1 );
    vmathSoaQAdd( result, &tmpQ_0, &tmpQ_1 );
}

static inline void vmathSoaQSquad( VmathSoaQuat *result, vec_float4 t, const VmathSoaQuat *unitQuat0, const VmathSoaQuat *unitQuat1, const VmathSoaQuat *unitQuat2, const VmathSoaQuat *unitQuat3 )
{
    VmathSoaQuat tmp0, tmp1;
    vmathSoaQSlerp( &tmp0, t, unitQuat0, unitQuat3 );
    vmathSoaQSlerp( &tmp1, t, unitQuat1, unitQuat2 );
    vmathSoaQSlerp( result, vec_madd( vec_madd( ((vec_float4){2.0f,2.0f,2.0f,2.0f}), t, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), &tmp0, &tmp1 );
}

static inline void vmathSoaQGet4Aos( const VmathSoaQuat *quat, VmathQuat *result0, VmathQuat *result1, VmathQuat *result2, VmathQuat *result3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = vec_mergeh( quat->x, quat->z );
    tmp1 = vec_mergeh( quat->y, quat->w );
    tmp2 = vec_mergel( quat->x, quat->z );
    tmp3 = vec_mergel( quat->y, quat->w );
    vmathQMakeFrom128( result0, vec_mergeh( tmp0, tmp1 ) );
    vmathQMakeFrom128( result1, vec_mergel( tmp0, tmp1 ) );
    vmathQMakeFrom128( result2, vec_mergeh( tmp2, tmp3 ) );
    vmathQMakeFrom128( result3, vec_mergel( tmp2, tmp3 ) );
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
    result->x = vec_add( quat0->x, quat1->x );
    result->y = vec_add( quat0->y, quat1->y );
    result->z = vec_add( quat0->z, quat1->z );
    result->w = vec_add( quat0->w, quat1->w );
}

static inline void vmathSoaQSub( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 )
{
    result->x = vec_sub( quat0->x, quat1->x );
    result->y = vec_sub( quat0->y, quat1->y );
    result->z = vec_sub( quat0->z, quat1->z );
    result->w = vec_sub( quat0->w, quat1->w );
}

static inline void vmathSoaQScalarMul( VmathSoaQuat *result, const VmathSoaQuat *quat, vec_float4 scalar )
{
    result->x = vec_madd( quat->x, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( quat->y, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( quat->z, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->w = vec_madd( quat->w, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
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
    result = vec_madd( quat0->x, quat1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( quat0->y, quat1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( quat0->z, quat1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( quat0->w, quat1->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    return result;
}

static inline vec_float4 vmathSoaQNorm( const VmathSoaQuat *quat )
{
    vec_float4 result;
    result = vec_madd( quat->x, quat->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( quat->y, quat->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( quat->z, quat->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( quat->w, quat->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
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
    lenInv = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( lenSqr ) );
    result->x = vec_madd( quat->x, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->y = vec_madd( quat->y, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->z = vec_madd( quat->z, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result->w = vec_madd( quat->w, lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathSoaQMakeRotationArc( VmathSoaQuat *result, const VmathSoaVector3 *unitVec0, const VmathSoaVector3 *unitVec1 )
{
    VmathSoaVector3 tmpV3_0, tmpV3_1;
    vec_float4 cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf4( vec_madd( ((vec_float4){2.0f,2.0f,2.0f,2.0f}), vec_add( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vmathSoaV3Dot( unitVec0, unitVec1 ) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    recipCosHalfAngleX2 = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), cosHalfAngleX2 );
    vmathSoaV3Cross( &tmpV3_0, unitVec0, unitVec1 );
    vmathSoaV3ScalarMul( &tmpV3_1, &tmpV3_0, recipCosHalfAngleX2 );
    vmathSoaQMakeFromV3Scalar( result, &tmpV3_1, vec_madd( cosHalfAngleX2, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

static inline void vmathSoaQMakeRotationAxis( VmathSoaQuat *result, vec_float4 radians, const VmathSoaVector3 *unitVec )
{
    VmathSoaVector3 tmpV3_0;
    vec_float4 s, c, angle;
    angle = vec_madd( radians, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    vmathSoaV3ScalarMul( &tmpV3_0, unitVec, s );
    vmathSoaQMakeFromV3Scalar( result, &tmpV3_0, c );
}

static inline void vmathSoaQMakeRotationX( VmathSoaQuat *result, vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = vec_madd( radians, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    vmathSoaQMakeFromElems( result, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c );
}

static inline void vmathSoaQMakeRotationY( VmathSoaQuat *result, vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = vec_madd( radians, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    vmathSoaQMakeFromElems( result, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c );
}

static inline void vmathSoaQMakeRotationZ( VmathSoaQuat *result, vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = vec_madd( radians, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    vmathSoaQMakeFromElems( result, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, c );
}

static inline void vmathSoaQMul( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 )
{
    vec_float4 tmpX, tmpY, tmpZ, tmpW;
    tmpX = vec_sub( vec_add( vec_add( vec_madd( quat0->w, quat1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat0->x, quat1->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat0->y, quat1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat0->z, quat1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpY = vec_sub( vec_add( vec_add( vec_madd( quat0->w, quat1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat0->y, quat1->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat0->z, quat1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat0->x, quat1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpZ = vec_sub( vec_add( vec_add( vec_madd( quat0->w, quat1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat0->z, quat1->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat0->x, quat1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat0->y, quat1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpW = vec_sub( vec_sub( vec_sub( vec_madd( quat0->w, quat1->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat0->x, quat1->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat0->y, quat1->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat0->z, quat1->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    vmathSoaQMakeFromElems( result, tmpX, tmpY, tmpZ, tmpW );
}

static inline void vmathSoaQRotate( VmathSoaVector3 *result, const VmathSoaQuat *quat, const VmathSoaVector3 *vec )
{
    vec_float4 tmpX, tmpY, tmpZ, tmpW;
    tmpX = vec_sub( vec_add( vec_madd( quat->w, vec->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat->y, vec->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat->z, vec->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpY = vec_sub( vec_add( vec_madd( quat->w, vec->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat->z, vec->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat->x, vec->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpZ = vec_sub( vec_add( vec_madd( quat->w, vec->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat->x, vec->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat->y, vec->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpW = vec_add( vec_add( vec_madd( quat->x, vec->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat->y, vec->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat->z, vec->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result->x = vec_add( vec_sub( vec_add( vec_madd( tmpW, quat->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmpX, quat->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpY, quat->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpZ, quat->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result->y = vec_add( vec_sub( vec_add( vec_madd( tmpW, quat->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmpY, quat->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpZ, quat->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpX, quat->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result->z = vec_add( vec_sub( vec_add( vec_madd( tmpW, quat->z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmpZ, quat->w, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpX, quat->y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpY, quat->x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

static inline void vmathSoaQConj( VmathSoaQuat *result, const VmathSoaQuat *quat )
{
    vmathSoaQMakeFromElems( result, negatef4( quat->x ), negatef4( quat->y ), negatef4( quat->z ), quat->w );
}

static inline void vmathSoaQSelect( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1, vec_uint4 select1 )
{
    result->x = vec_sel( quat0->x, quat1->x, select1 );
    result->y = vec_sel( quat0->y, quat1->y, select1 );
    result->z = vec_sel( quat0->z, quat1->z, select1 );
    result->w = vec_sel( quat0->w, quat1->w, select1 );
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
