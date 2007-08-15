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

#ifndef _VECTORMATH_QUAT_AOS_C_H
#define _VECTORMATH_QUAT_AOS_C_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Definitions
 */
#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

static inline void vmathQCopy( VmathQuat *result, const VmathQuat *quat )
{
    result->x = quat->x;
    result->y = quat->y;
    result->z = quat->z;
    result->w = quat->w;
}

static inline void vmathQMakeFromElems( VmathQuat *result, float _x, float _y, float _z, float _w )
{
    result->x = _x;
    result->y = _y;
    result->z = _z;
    result->w = _w;
}

static inline void vmathQMakeFromV3Scalar( VmathQuat *result, const VmathVector3 *xyz, float _w )
{
    vmathQSetXYZ( result, xyz );
    vmathQSetW( result, _w );
}

static inline void vmathQMakeFromV4( VmathQuat *result, const VmathVector4 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
    result->w = vec->w;
}

static inline void vmathQMakeFromScalar( VmathQuat *result, float scalar )
{
    result->x = scalar;
    result->y = scalar;
    result->z = scalar;
    result->w = scalar;
}

static inline void vmathQMakeIdentity( VmathQuat *result )
{
    vmathQMakeFromElems( result, 0.0f, 0.0f, 0.0f, 1.0f );
}

static inline void vmathQLerp( VmathQuat *result, float t, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    VmathQuat tmpQ_0, tmpQ_1;
    vmathQSub( &tmpQ_0, quat1, quat0 );
    vmathQScalarMul( &tmpQ_1, &tmpQ_0, t );
    vmathQAdd( result, quat0, &tmpQ_1 );
}

static inline void vmathQSlerp( VmathQuat *result, float t, const VmathQuat *unitQuat0, const VmathQuat *unitQuat1 )
{
    VmathQuat start, tmpQ_0, tmpQ_1;
    float recipSinAngle, scale0, scale1, cosAngle, angle;
    cosAngle = vmathQDot( unitQuat0, unitQuat1 );
    if ( cosAngle < 0.0f ) {
        cosAngle = -cosAngle;
        vmathQNeg( &start, unitQuat0 );
    } else {
        vmathQCopy( &start, unitQuat0 );
    }
    if ( cosAngle < _VECTORMATH_SLERP_TOL ) {
        angle = acosf( cosAngle );
        recipSinAngle = ( 1.0f / sinf( angle ) );
        scale0 = ( sinf( ( ( 1.0f - t ) * angle ) ) * recipSinAngle );
        scale1 = ( sinf( ( t * angle ) ) * recipSinAngle );
    } else {
        scale0 = ( 1.0f - t );
        scale1 = t;
    }
    vmathQScalarMul( &tmpQ_0, &start, scale0 );
    vmathQScalarMul( &tmpQ_1, unitQuat1, scale1 );
    vmathQAdd( result, &tmpQ_0, &tmpQ_1 );
}

static inline void vmathQSquad( VmathQuat *result, float t, const VmathQuat *unitQuat0, const VmathQuat *unitQuat1, const VmathQuat *unitQuat2, const VmathQuat *unitQuat3 )
{
    VmathQuat tmp0, tmp1;
    vmathQSlerp( &tmp0, t, unitQuat0, unitQuat3 );
    vmathQSlerp( &tmp1, t, unitQuat1, unitQuat2 );
    vmathQSlerp( result, ( ( 2.0f * t ) * ( 1.0f - t ) ), &tmp0, &tmp1 );
}

static inline void vmathQSetXYZ( VmathQuat *result, const VmathVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
}

static inline void vmathQGetXYZ( VmathVector3 *result, const VmathQuat *quat )
{
    vmathV3MakeFromElems( result, quat->x, quat->y, quat->z );
}

static inline void vmathQSetX( VmathQuat *result, float _x )
{
    result->x = _x;
}

static inline float vmathQGetX( const VmathQuat *quat )
{
    return quat->x;
}

static inline void vmathQSetY( VmathQuat *result, float _y )
{
    result->y = _y;
}

static inline float vmathQGetY( const VmathQuat *quat )
{
    return quat->y;
}

static inline void vmathQSetZ( VmathQuat *result, float _z )
{
    result->z = _z;
}

static inline float vmathQGetZ( const VmathQuat *quat )
{
    return quat->z;
}

static inline void vmathQSetW( VmathQuat *result, float _w )
{
    result->w = _w;
}

static inline float vmathQGetW( const VmathQuat *quat )
{
    return quat->w;
}

static inline void vmathQSetElem( VmathQuat *result, int idx, float value )
{
    *(&result->x + idx) = value;
}

static inline float vmathQGetElem( const VmathQuat *quat, int idx )
{
    return *(&quat->x + idx);
}

static inline void vmathQAdd( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    result->x = ( quat0->x + quat1->x );
    result->y = ( quat0->y + quat1->y );
    result->z = ( quat0->z + quat1->z );
    result->w = ( quat0->w + quat1->w );
}

static inline void vmathQSub( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    result->x = ( quat0->x - quat1->x );
    result->y = ( quat0->y - quat1->y );
    result->z = ( quat0->z - quat1->z );
    result->w = ( quat0->w - quat1->w );
}

static inline void vmathQScalarMul( VmathQuat *result, const VmathQuat *quat, float scalar )
{
    result->x = ( quat->x * scalar );
    result->y = ( quat->y * scalar );
    result->z = ( quat->z * scalar );
    result->w = ( quat->w * scalar );
}

static inline void vmathQScalarDiv( VmathQuat *result, const VmathQuat *quat, float scalar )
{
    result->x = ( quat->x / scalar );
    result->y = ( quat->y / scalar );
    result->z = ( quat->z / scalar );
    result->w = ( quat->w / scalar );
}

static inline void vmathQNeg( VmathQuat *result, const VmathQuat *quat )
{
    result->x = -quat->x;
    result->y = -quat->y;
    result->z = -quat->z;
    result->w = -quat->w;
}

static inline float vmathQDot( const VmathQuat *quat0, const VmathQuat *quat1 )
{
    float result;
    result = ( quat0->x * quat1->x );
    result = ( result + ( quat0->y * quat1->y ) );
    result = ( result + ( quat0->z * quat1->z ) );
    result = ( result + ( quat0->w * quat1->w ) );
    return result;
}

static inline float vmathQNorm( const VmathQuat *quat )
{
    float result;
    result = ( quat->x * quat->x );
    result = ( result + ( quat->y * quat->y ) );
    result = ( result + ( quat->z * quat->z ) );
    result = ( result + ( quat->w * quat->w ) );
    return result;
}

static inline float vmathQLength( const VmathQuat *quat )
{
    return sqrtf( vmathQNorm( quat ) );
}

static inline void vmathQNormalize( VmathQuat *result, const VmathQuat *quat )
{
    float lenSqr, lenInv;
    lenSqr = vmathQNorm( quat );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    result->x = ( quat->x * lenInv );
    result->y = ( quat->y * lenInv );
    result->z = ( quat->z * lenInv );
    result->w = ( quat->w * lenInv );
}

static inline void vmathQMakeRotationArc( VmathQuat *result, const VmathVector3 *unitVec0, const VmathVector3 *unitVec1 )
{
    VmathVector3 tmpV3_0, tmpV3_1;
    float cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf( ( 2.0f * ( 1.0f + vmathV3Dot( unitVec0, unitVec1 ) ) ) );
    recipCosHalfAngleX2 = ( 1.0f / cosHalfAngleX2 );
    vmathV3Cross( &tmpV3_0, unitVec0, unitVec1 );
    vmathV3ScalarMul( &tmpV3_1, &tmpV3_0, recipCosHalfAngleX2 );
    vmathQMakeFromV3Scalar( result, &tmpV3_1, ( cosHalfAngleX2 * 0.5f ) );
}

static inline void vmathQMakeRotationAxis( VmathQuat *result, float radians, const VmathVector3 *unitVec )
{
    VmathVector3 tmpV3_0;
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    vmathV3ScalarMul( &tmpV3_0, unitVec, s );
    vmathQMakeFromV3Scalar( result, &tmpV3_0, c );
}

static inline void vmathQMakeRotationX( VmathQuat *result, float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    vmathQMakeFromElems( result, s, 0.0f, 0.0f, c );
}

static inline void vmathQMakeRotationY( VmathQuat *result, float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    vmathQMakeFromElems( result, 0.0f, s, 0.0f, c );
}

static inline void vmathQMakeRotationZ( VmathQuat *result, float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    vmathQMakeFromElems( result, 0.0f, 0.0f, s, c );
}

static inline void vmathQMul( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    float tmpX, tmpY, tmpZ, tmpW;
    tmpX = ( ( ( ( quat0->w * quat1->x ) + ( quat0->x * quat1->w ) ) + ( quat0->y * quat1->z ) ) - ( quat0->z * quat1->y ) );
    tmpY = ( ( ( ( quat0->w * quat1->y ) + ( quat0->y * quat1->w ) ) + ( quat0->z * quat1->x ) ) - ( quat0->x * quat1->z ) );
    tmpZ = ( ( ( ( quat0->w * quat1->z ) + ( quat0->z * quat1->w ) ) + ( quat0->x * quat1->y ) ) - ( quat0->y * quat1->x ) );
    tmpW = ( ( ( ( quat0->w * quat1->w ) - ( quat0->x * quat1->x ) ) - ( quat0->y * quat1->y ) ) - ( quat0->z * quat1->z ) );
    vmathQMakeFromElems( result, tmpX, tmpY, tmpZ, tmpW );
}

static inline void vmathQRotate( VmathVector3 *result, const VmathQuat *quat, const VmathVector3 *vec )
{
    float tmpX, tmpY, tmpZ, tmpW;
    tmpX = ( ( ( quat->w * vec->x ) + ( quat->y * vec->z ) ) - ( quat->z * vec->y ) );
    tmpY = ( ( ( quat->w * vec->y ) + ( quat->z * vec->x ) ) - ( quat->x * vec->z ) );
    tmpZ = ( ( ( quat->w * vec->z ) + ( quat->x * vec->y ) ) - ( quat->y * vec->x ) );
    tmpW = ( ( ( quat->x * vec->x ) + ( quat->y * vec->y ) ) + ( quat->z * vec->z ) );
    result->x = ( ( ( ( tmpW * quat->x ) + ( tmpX * quat->w ) ) - ( tmpY * quat->z ) ) + ( tmpZ * quat->y ) );
    result->y = ( ( ( ( tmpW * quat->y ) + ( tmpY * quat->w ) ) - ( tmpZ * quat->x ) ) + ( tmpX * quat->z ) );
    result->z = ( ( ( ( tmpW * quat->z ) + ( tmpZ * quat->w ) ) - ( tmpX * quat->y ) ) + ( tmpY * quat->x ) );
}

static inline void vmathQConj( VmathQuat *result, const VmathQuat *quat )
{
    vmathQMakeFromElems( result, -quat->x, -quat->y, -quat->z, quat->w );
}

static inline void vmathQSelect( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1, unsigned int select1 )
{
    result->x = ( select1 )? quat1->x : quat0->x;
    result->y = ( select1 )? quat1->y : quat0->y;
    result->z = ( select1 )? quat1->z : quat0->z;
    result->w = ( select1 )? quat1->w : quat0->w;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathQPrint( const VmathQuat *quat )
{
    printf( "( %f %f %f %f )\n", quat->x, quat->y, quat->z, quat->w );
}

static inline void vmathQPrints( const VmathQuat *quat, const char *name )
{
    printf( "%s: ( %f %f %f %f )\n", name, quat->x, quat->y, quat->z, quat->w );
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
