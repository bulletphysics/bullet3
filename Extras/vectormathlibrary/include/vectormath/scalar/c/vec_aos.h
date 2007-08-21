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
 */
#define _VECTORMATH_SLERP_TOL 0.999f

/*-----------------------------------------------------------------------------
 * Definitions
 */
#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

static inline void vmathV3Copy( VmathVector3 *result, const VmathVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
}

static inline void vmathV3MakeFromElems( VmathVector3 *result, float _x, float _y, float _z )
{
    result->x = _x;
    result->y = _y;
    result->z = _z;
}

static inline void vmathV3MakeFromP3( VmathVector3 *result, const VmathPoint3 *pnt )
{
    result->x = pnt->x;
    result->y = pnt->y;
    result->z = pnt->z;
}

static inline void vmathV3MakeFromScalar( VmathVector3 *result, float scalar )
{
    result->x = scalar;
    result->y = scalar;
    result->z = scalar;
}

static inline void vmathV3MakeXAxis( VmathVector3 *result )
{
    vmathV3MakeFromElems( result, 1.0f, 0.0f, 0.0f );
}

static inline void vmathV3MakeYAxis( VmathVector3 *result )
{
    vmathV3MakeFromElems( result, 0.0f, 1.0f, 0.0f );
}

static inline void vmathV3MakeZAxis( VmathVector3 *result )
{
    vmathV3MakeFromElems( result, 0.0f, 0.0f, 1.0f );
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
    VmathVector3 tmpV3_0, tmpV3_1;
    float recipSinAngle, scale0, scale1, cosAngle, angle;
    cosAngle = vmathV3Dot( unitVec0, unitVec1 );
    if ( cosAngle < _VECTORMATH_SLERP_TOL ) {
        angle = acosf( cosAngle );
        recipSinAngle = ( 1.0f / sinf( angle ) );
        scale0 = ( sinf( ( ( 1.0f - t ) * angle ) ) * recipSinAngle );
        scale1 = ( sinf( ( t * angle ) ) * recipSinAngle );
    } else {
        scale0 = ( 1.0f - t );
        scale1 = t;
    }
    vmathV3ScalarMul( &tmpV3_0, unitVec0, scale0 );
    vmathV3ScalarMul( &tmpV3_1, unitVec1, scale1 );
    vmathV3Add( result, &tmpV3_0, &tmpV3_1 );
}

static inline void vmathV3SetX( VmathVector3 *result, float _x )
{
    result->x = _x;
}

static inline float vmathV3GetX( const VmathVector3 *vec )
{
    return vec->x;
}

static inline void vmathV3SetY( VmathVector3 *result, float _y )
{
    result->y = _y;
}

static inline float vmathV3GetY( const VmathVector3 *vec )
{
    return vec->y;
}

static inline void vmathV3SetZ( VmathVector3 *result, float _z )
{
    result->z = _z;
}

static inline float vmathV3GetZ( const VmathVector3 *vec )
{
    return vec->z;
}

static inline void vmathV3SetElem( VmathVector3 *result, int idx, float value )
{
    *(&result->x + idx) = value;
}

static inline float vmathV3GetElem( const VmathVector3 *vec, int idx )
{
    return *(&vec->x + idx);
}

static inline void vmathV3Add( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->x = ( vec0->x + vec1->x );
    result->y = ( vec0->y + vec1->y );
    result->z = ( vec0->z + vec1->z );
}

static inline void vmathV3Sub( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->x = ( vec0->x - vec1->x );
    result->y = ( vec0->y - vec1->y );
    result->z = ( vec0->z - vec1->z );
}

static inline void vmathV3AddP3( VmathPoint3 *result, const VmathVector3 *vec, const VmathPoint3 *pnt1 )
{
    result->x = ( vec->x + pnt1->x );
    result->y = ( vec->y + pnt1->y );
    result->z = ( vec->z + pnt1->z );
}

static inline void vmathV3ScalarMul( VmathVector3 *result, const VmathVector3 *vec, float scalar )
{
    result->x = ( vec->x * scalar );
    result->y = ( vec->y * scalar );
    result->z = ( vec->z * scalar );
}

static inline void vmathV3ScalarDiv( VmathVector3 *result, const VmathVector3 *vec, float scalar )
{
    result->x = ( vec->x / scalar );
    result->y = ( vec->y / scalar );
    result->z = ( vec->z / scalar );
}

static inline void vmathV3Neg( VmathVector3 *result, const VmathVector3 *vec )
{
    result->x = -vec->x;
    result->y = -vec->y;
    result->z = -vec->z;
}

static inline void vmathV3MulPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->x = ( vec0->x * vec1->x );
    result->y = ( vec0->y * vec1->y );
    result->z = ( vec0->z * vec1->z );
}

static inline void vmathV3DivPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->x = ( vec0->x / vec1->x );
    result->y = ( vec0->y / vec1->y );
    result->z = ( vec0->z / vec1->z );
}

static inline void vmathV3RecipPerElem( VmathVector3 *result, const VmathVector3 *vec )
{
    result->x = ( 1.0f / vec->x );
    result->y = ( 1.0f / vec->y );
    result->z = ( 1.0f / vec->z );
}

static inline void vmathV3SqrtPerElem( VmathVector3 *result, const VmathVector3 *vec )
{
    result->x = sqrtf( vec->x );
    result->y = sqrtf( vec->y );
    result->z = sqrtf( vec->z );
}

static inline void vmathV3RsqrtPerElem( VmathVector3 *result, const VmathVector3 *vec )
{
    result->x = ( 1.0f / sqrtf( vec->x ) );
    result->y = ( 1.0f / sqrtf( vec->y ) );
    result->z = ( 1.0f / sqrtf( vec->z ) );
}

static inline void vmathV3AbsPerElem( VmathVector3 *result, const VmathVector3 *vec )
{
    result->x = fabsf( vec->x );
    result->y = fabsf( vec->y );
    result->z = fabsf( vec->z );
}

static inline void vmathV3CopySignPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->x = ( vec1->x < 0.0f )? -fabsf( vec0->x ) : fabsf( vec0->x );
    result->y = ( vec1->y < 0.0f )? -fabsf( vec0->y ) : fabsf( vec0->y );
    result->z = ( vec1->z < 0.0f )? -fabsf( vec0->z ) : fabsf( vec0->z );
}

static inline void vmathV3MaxPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->x = (vec0->x > vec1->x)? vec0->x : vec1->x;
    result->y = (vec0->y > vec1->y)? vec0->y : vec1->y;
    result->z = (vec0->z > vec1->z)? vec0->z : vec1->z;
}

static inline float vmathV3MaxElem( const VmathVector3 *vec )
{
    float result;
    result = (vec->x > vec->y)? vec->x : vec->y;
    result = (vec->z > result)? vec->z : result;
    return result;
}

static inline void vmathV3MinPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    result->x = (vec0->x < vec1->x)? vec0->x : vec1->x;
    result->y = (vec0->y < vec1->y)? vec0->y : vec1->y;
    result->z = (vec0->z < vec1->z)? vec0->z : vec1->z;
}

static inline float vmathV3MinElem( const VmathVector3 *vec )
{
    float result;
    result = (vec->x < vec->y)? vec->x : vec->y;
    result = (vec->z < result)? vec->z : result;
    return result;
}

static inline float vmathV3Sum( const VmathVector3 *vec )
{
    float result;
    result = ( vec->x + vec->y );
    result = ( result + vec->z );
    return result;
}

static inline float vmathV3Dot( const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    float result;
    result = ( vec0->x * vec1->x );
    result = ( result + ( vec0->y * vec1->y ) );
    result = ( result + ( vec0->z * vec1->z ) );
    return result;
}

static inline float vmathV3LengthSqr( const VmathVector3 *vec )
{
    float result;
    result = ( vec->x * vec->x );
    result = ( result + ( vec->y * vec->y ) );
    result = ( result + ( vec->z * vec->z ) );
    return result;
}

static inline float vmathV3Length( const VmathVector3 *vec )
{
    return sqrtf( vmathV3LengthSqr( vec ) );
}

static inline void vmathV3Normalize( VmathVector3 *result, const VmathVector3 *vec )
{
    float lenSqr, lenInv;
    lenSqr = vmathV3LengthSqr( vec );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    result->x = ( vec->x * lenInv );
    result->y = ( vec->y * lenInv );
    result->z = ( vec->z * lenInv );
}

static inline void vmathV3Cross( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 )
{
    float tmpX, tmpY, tmpZ;
    tmpX = ( ( vec0->y * vec1->z ) - ( vec0->z * vec1->y ) );
    tmpY = ( ( vec0->z * vec1->x ) - ( vec0->x * vec1->z ) );
    tmpZ = ( ( vec0->x * vec1->y ) - ( vec0->y * vec1->x ) );
    vmathV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathV3Select( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1, unsigned int select1 )
{
    result->x = ( select1 )? vec1->x : vec0->x;
    result->y = ( select1 )? vec1->y : vec0->y;
    result->z = ( select1 )? vec1->z : vec0->z;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathV3Print( const VmathVector3 *vec )
{
    printf( "( %f %f %f )\n", vec->x, vec->y, vec->z );
}

static inline void vmathV3Prints( const VmathVector3 *vec, const char *name )
{
    printf( "%s: ( %f %f %f )\n", name, vec->x, vec->y, vec->z );
}

#endif

static inline void vmathV4Copy( VmathVector4 *result, const VmathVector4 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
    result->w = vec->w;
}

static inline void vmathV4MakeFromElems( VmathVector4 *result, float _x, float _y, float _z, float _w )
{
    result->x = _x;
    result->y = _y;
    result->z = _z;
    result->w = _w;
}

static inline void vmathV4MakeFromV3Scalar( VmathVector4 *result, const VmathVector3 *xyz, float _w )
{
    vmathV4SetXYZ( result, xyz );
    vmathV4SetW( result, _w );
}

static inline void vmathV4MakeFromV3( VmathVector4 *result, const VmathVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
    result->w = 0.0f;
}

static inline void vmathV4MakeFromP3( VmathVector4 *result, const VmathPoint3 *pnt )
{
    result->x = pnt->x;
    result->y = pnt->y;
    result->z = pnt->z;
    result->w = 1.0f;
}

static inline void vmathV4MakeFromQ( VmathVector4 *result, const VmathQuat *quat )
{
    result->x = quat->x;
    result->y = quat->y;
    result->z = quat->z;
    result->w = quat->w;
}

static inline void vmathV4MakeFromScalar( VmathVector4 *result, float scalar )
{
    result->x = scalar;
    result->y = scalar;
    result->z = scalar;
    result->w = scalar;
}

static inline void vmathV4MakeXAxis( VmathVector4 *result )
{
    vmathV4MakeFromElems( result, 1.0f, 0.0f, 0.0f, 0.0f );
}

static inline void vmathV4MakeYAxis( VmathVector4 *result )
{
    vmathV4MakeFromElems( result, 0.0f, 1.0f, 0.0f, 0.0f );
}

static inline void vmathV4MakeZAxis( VmathVector4 *result )
{
    vmathV4MakeFromElems( result, 0.0f, 0.0f, 1.0f, 0.0f );
}

static inline void vmathV4MakeWAxis( VmathVector4 *result )
{
    vmathV4MakeFromElems( result, 0.0f, 0.0f, 0.0f, 1.0f );
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
    VmathVector4 tmpV4_0, tmpV4_1;
    float recipSinAngle, scale0, scale1, cosAngle, angle;
    cosAngle = vmathV4Dot( unitVec0, unitVec1 );
    if ( cosAngle < _VECTORMATH_SLERP_TOL ) {
        angle = acosf( cosAngle );
        recipSinAngle = ( 1.0f / sinf( angle ) );
        scale0 = ( sinf( ( ( 1.0f - t ) * angle ) ) * recipSinAngle );
        scale1 = ( sinf( ( t * angle ) ) * recipSinAngle );
    } else {
        scale0 = ( 1.0f - t );
        scale1 = t;
    }
    vmathV4ScalarMul( &tmpV4_0, unitVec0, scale0 );
    vmathV4ScalarMul( &tmpV4_1, unitVec1, scale1 );
    vmathV4Add( result, &tmpV4_0, &tmpV4_1 );
}

static inline void vmathV4SetXYZ( VmathVector4 *result, const VmathVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
}

static inline void vmathV4GetXYZ( VmathVector3 *result, const VmathVector4 *vec )
{
    vmathV3MakeFromElems( result, vec->x, vec->y, vec->z );
}

static inline void vmathV4SetX( VmathVector4 *result, float _x )
{
    result->x = _x;
}

static inline float vmathV4GetX( const VmathVector4 *vec )
{
    return vec->x;
}

static inline void vmathV4SetY( VmathVector4 *result, float _y )
{
    result->y = _y;
}

static inline float vmathV4GetY( const VmathVector4 *vec )
{
    return vec->y;
}

static inline void vmathV4SetZ( VmathVector4 *result, float _z )
{
    result->z = _z;
}

static inline float vmathV4GetZ( const VmathVector4 *vec )
{
    return vec->z;
}

static inline void vmathV4SetW( VmathVector4 *result, float _w )
{
    result->w = _w;
}

static inline float vmathV4GetW( const VmathVector4 *vec )
{
    return vec->w;
}

static inline void vmathV4SetElem( VmathVector4 *result, int idx, float value )
{
    *(&result->x + idx) = value;
}

static inline float vmathV4GetElem( const VmathVector4 *vec, int idx )
{
    return *(&vec->x + idx);
}

static inline void vmathV4Add( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->x = ( vec0->x + vec1->x );
    result->y = ( vec0->y + vec1->y );
    result->z = ( vec0->z + vec1->z );
    result->w = ( vec0->w + vec1->w );
}

static inline void vmathV4Sub( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->x = ( vec0->x - vec1->x );
    result->y = ( vec0->y - vec1->y );
    result->z = ( vec0->z - vec1->z );
    result->w = ( vec0->w - vec1->w );
}

static inline void vmathV4ScalarMul( VmathVector4 *result, const VmathVector4 *vec, float scalar )
{
    result->x = ( vec->x * scalar );
    result->y = ( vec->y * scalar );
    result->z = ( vec->z * scalar );
    result->w = ( vec->w * scalar );
}

static inline void vmathV4ScalarDiv( VmathVector4 *result, const VmathVector4 *vec, float scalar )
{
    result->x = ( vec->x / scalar );
    result->y = ( vec->y / scalar );
    result->z = ( vec->z / scalar );
    result->w = ( vec->w / scalar );
}

static inline void vmathV4Neg( VmathVector4 *result, const VmathVector4 *vec )
{
    result->x = -vec->x;
    result->y = -vec->y;
    result->z = -vec->z;
    result->w = -vec->w;
}

static inline void vmathV4MulPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->x = ( vec0->x * vec1->x );
    result->y = ( vec0->y * vec1->y );
    result->z = ( vec0->z * vec1->z );
    result->w = ( vec0->w * vec1->w );
}

static inline void vmathV4DivPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->x = ( vec0->x / vec1->x );
    result->y = ( vec0->y / vec1->y );
    result->z = ( vec0->z / vec1->z );
    result->w = ( vec0->w / vec1->w );
}

static inline void vmathV4RecipPerElem( VmathVector4 *result, const VmathVector4 *vec )
{
    result->x = ( 1.0f / vec->x );
    result->y = ( 1.0f / vec->y );
    result->z = ( 1.0f / vec->z );
    result->w = ( 1.0f / vec->w );
}

static inline void vmathV4SqrtPerElem( VmathVector4 *result, const VmathVector4 *vec )
{
    result->x = sqrtf( vec->x );
    result->y = sqrtf( vec->y );
    result->z = sqrtf( vec->z );
    result->w = sqrtf( vec->w );
}

static inline void vmathV4RsqrtPerElem( VmathVector4 *result, const VmathVector4 *vec )
{
    result->x = ( 1.0f / sqrtf( vec->x ) );
    result->y = ( 1.0f / sqrtf( vec->y ) );
    result->z = ( 1.0f / sqrtf( vec->z ) );
    result->w = ( 1.0f / sqrtf( vec->w ) );
}

static inline void vmathV4AbsPerElem( VmathVector4 *result, const VmathVector4 *vec )
{
    result->x = fabsf( vec->x );
    result->y = fabsf( vec->y );
    result->z = fabsf( vec->z );
    result->w = fabsf( vec->w );
}

static inline void vmathV4CopySignPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->x = ( vec1->x < 0.0f )? -fabsf( vec0->x ) : fabsf( vec0->x );
    result->y = ( vec1->y < 0.0f )? -fabsf( vec0->y ) : fabsf( vec0->y );
    result->z = ( vec1->z < 0.0f )? -fabsf( vec0->z ) : fabsf( vec0->z );
    result->w = ( vec1->w < 0.0f )? -fabsf( vec0->w ) : fabsf( vec0->w );
}

static inline void vmathV4MaxPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->x = (vec0->x > vec1->x)? vec0->x : vec1->x;
    result->y = (vec0->y > vec1->y)? vec0->y : vec1->y;
    result->z = (vec0->z > vec1->z)? vec0->z : vec1->z;
    result->w = (vec0->w > vec1->w)? vec0->w : vec1->w;
}

static inline float vmathV4MaxElem( const VmathVector4 *vec )
{
    float result;
    result = (vec->x > vec->y)? vec->x : vec->y;
    result = (vec->z > result)? vec->z : result;
    result = (vec->w > result)? vec->w : result;
    return result;
}

static inline void vmathV4MinPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    result->x = (vec0->x < vec1->x)? vec0->x : vec1->x;
    result->y = (vec0->y < vec1->y)? vec0->y : vec1->y;
    result->z = (vec0->z < vec1->z)? vec0->z : vec1->z;
    result->w = (vec0->w < vec1->w)? vec0->w : vec1->w;
}

static inline float vmathV4MinElem( const VmathVector4 *vec )
{
    float result;
    result = (vec->x < vec->y)? vec->x : vec->y;
    result = (vec->z < result)? vec->z : result;
    result = (vec->w < result)? vec->w : result;
    return result;
}

static inline float vmathV4Sum( const VmathVector4 *vec )
{
    float result;
    result = ( vec->x + vec->y );
    result = ( result + vec->z );
    result = ( result + vec->w );
    return result;
}

static inline float vmathV4Dot( const VmathVector4 *vec0, const VmathVector4 *vec1 )
{
    float result;
    result = ( vec0->x * vec1->x );
    result = ( result + ( vec0->y * vec1->y ) );
    result = ( result + ( vec0->z * vec1->z ) );
    result = ( result + ( vec0->w * vec1->w ) );
    return result;
}

static inline float vmathV4LengthSqr( const VmathVector4 *vec )
{
    float result;
    result = ( vec->x * vec->x );
    result = ( result + ( vec->y * vec->y ) );
    result = ( result + ( vec->z * vec->z ) );
    result = ( result + ( vec->w * vec->w ) );
    return result;
}

static inline float vmathV4Length( const VmathVector4 *vec )
{
    return sqrtf( vmathV4LengthSqr( vec ) );
}

static inline void vmathV4Normalize( VmathVector4 *result, const VmathVector4 *vec )
{
    float lenSqr, lenInv;
    lenSqr = vmathV4LengthSqr( vec );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    result->x = ( vec->x * lenInv );
    result->y = ( vec->y * lenInv );
    result->z = ( vec->z * lenInv );
    result->w = ( vec->w * lenInv );
}

static inline void vmathV4Select( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1, unsigned int select1 )
{
    result->x = ( select1 )? vec1->x : vec0->x;
    result->y = ( select1 )? vec1->y : vec0->y;
    result->z = ( select1 )? vec1->z : vec0->z;
    result->w = ( select1 )? vec1->w : vec0->w;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathV4Print( const VmathVector4 *vec )
{
    printf( "( %f %f %f %f )\n", vec->x, vec->y, vec->z, vec->w );
}

static inline void vmathV4Prints( const VmathVector4 *vec, const char *name )
{
    printf( "%s: ( %f %f %f %f )\n", name, vec->x, vec->y, vec->z, vec->w );
}

#endif

static inline void vmathP3Copy( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->x = pnt->x;
    result->y = pnt->y;
    result->z = pnt->z;
}

static inline void vmathP3MakeFromElems( VmathPoint3 *result, float _x, float _y, float _z )
{
    result->x = _x;
    result->y = _y;
    result->z = _z;
}

static inline void vmathP3MakeFromV3( VmathPoint3 *result, const VmathVector3 *vec )
{
    result->x = vec->x;
    result->y = vec->y;
    result->z = vec->z;
}

static inline void vmathP3MakeFromScalar( VmathPoint3 *result, float scalar )
{
    result->x = scalar;
    result->y = scalar;
    result->z = scalar;
}

static inline void vmathP3Lerp( VmathPoint3 *result, float t, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    VmathVector3 tmpV3_0, tmpV3_1;
    vmathP3Sub( &tmpV3_0, pnt1, pnt0 );
    vmathV3ScalarMul( &tmpV3_1, &tmpV3_0, t );
    vmathP3AddV3( result, pnt0, &tmpV3_1 );
}

static inline void vmathP3SetX( VmathPoint3 *result, float _x )
{
    result->x = _x;
}

static inline float vmathP3GetX( const VmathPoint3 *pnt )
{
    return pnt->x;
}

static inline void vmathP3SetY( VmathPoint3 *result, float _y )
{
    result->y = _y;
}

static inline float vmathP3GetY( const VmathPoint3 *pnt )
{
    return pnt->y;
}

static inline void vmathP3SetZ( VmathPoint3 *result, float _z )
{
    result->z = _z;
}

static inline float vmathP3GetZ( const VmathPoint3 *pnt )
{
    return pnt->z;
}

static inline void vmathP3SetElem( VmathPoint3 *result, int idx, float value )
{
    *(&result->x + idx) = value;
}

static inline float vmathP3GetElem( const VmathPoint3 *pnt, int idx )
{
    return *(&pnt->x + idx);
}

static inline void vmathP3Sub( VmathVector3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->x = ( pnt0->x - pnt1->x );
    result->y = ( pnt0->y - pnt1->y );
    result->z = ( pnt0->z - pnt1->z );
}

static inline void vmathP3AddV3( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *vec1 )
{
    result->x = ( pnt->x + vec1->x );
    result->y = ( pnt->y + vec1->y );
    result->z = ( pnt->z + vec1->z );
}

static inline void vmathP3SubV3( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *vec1 )
{
    result->x = ( pnt->x - vec1->x );
    result->y = ( pnt->y - vec1->y );
    result->z = ( pnt->z - vec1->z );
}

static inline void vmathP3MulPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->x = ( pnt0->x * pnt1->x );
    result->y = ( pnt0->y * pnt1->y );
    result->z = ( pnt0->z * pnt1->z );
}

static inline void vmathP3DivPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->x = ( pnt0->x / pnt1->x );
    result->y = ( pnt0->y / pnt1->y );
    result->z = ( pnt0->z / pnt1->z );
}

static inline void vmathP3RecipPerElem( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->x = ( 1.0f / pnt->x );
    result->y = ( 1.0f / pnt->y );
    result->z = ( 1.0f / pnt->z );
}

static inline void vmathP3SqrtPerElem( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->x = sqrtf( pnt->x );
    result->y = sqrtf( pnt->y );
    result->z = sqrtf( pnt->z );
}

static inline void vmathP3RsqrtPerElem( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->x = ( 1.0f / sqrtf( pnt->x ) );
    result->y = ( 1.0f / sqrtf( pnt->y ) );
    result->z = ( 1.0f / sqrtf( pnt->z ) );
}

static inline void vmathP3AbsPerElem( VmathPoint3 *result, const VmathPoint3 *pnt )
{
    result->x = fabsf( pnt->x );
    result->y = fabsf( pnt->y );
    result->z = fabsf( pnt->z );
}

static inline void vmathP3CopySignPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->x = ( pnt1->x < 0.0f )? -fabsf( pnt0->x ) : fabsf( pnt0->x );
    result->y = ( pnt1->y < 0.0f )? -fabsf( pnt0->y ) : fabsf( pnt0->y );
    result->z = ( pnt1->z < 0.0f )? -fabsf( pnt0->z ) : fabsf( pnt0->z );
}

static inline void vmathP3MaxPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->x = (pnt0->x > pnt1->x)? pnt0->x : pnt1->x;
    result->y = (pnt0->y > pnt1->y)? pnt0->y : pnt1->y;
    result->z = (pnt0->z > pnt1->z)? pnt0->z : pnt1->z;
}

static inline float vmathP3MaxElem( const VmathPoint3 *pnt )
{
    float result;
    result = (pnt->x > pnt->y)? pnt->x : pnt->y;
    result = (pnt->z > result)? pnt->z : result;
    return result;
}

static inline void vmathP3MinPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 )
{
    result->x = (pnt0->x < pnt1->x)? pnt0->x : pnt1->x;
    result->y = (pnt0->y < pnt1->y)? pnt0->y : pnt1->y;
    result->z = (pnt0->z < pnt1->z)? pnt0->z : pnt1->z;
}

static inline float vmathP3MinElem( const VmathPoint3 *pnt )
{
    float result;
    result = (pnt->x < pnt->y)? pnt->x : pnt->y;
    result = (pnt->z < result)? pnt->z : result;
    return result;
}

static inline float vmathP3Sum( const VmathPoint3 *pnt )
{
    float result;
    result = ( pnt->x + pnt->y );
    result = ( result + pnt->z );
    return result;
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
    float result;
    result = ( pnt->x * unitVec->x );
    result = ( result + ( pnt->y * unitVec->y ) );
    result = ( result + ( pnt->z * unitVec->z ) );
    return result;
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
    result->x = ( select1 )? pnt1->x : pnt0->x;
    result->y = ( select1 )? pnt1->y : pnt0->y;
    result->z = ( select1 )? pnt1->z : pnt0->z;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathP3Print( const VmathPoint3 *pnt )
{
    printf( "( %f %f %f )\n", pnt->x, pnt->y, pnt->z );
}

static inline void vmathP3Prints( const VmathPoint3 *pnt, const char *name )
{
    printf( "%s: ( %f %f %f )\n", name, pnt->x, pnt->y, pnt->z );
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
