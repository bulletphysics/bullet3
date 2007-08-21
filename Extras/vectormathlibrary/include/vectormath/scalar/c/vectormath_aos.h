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

#ifndef _VECTORMATH_AOS_C_SCALAR_H
#define _VECTORMATH_AOS_C_SCALAR_H

#include <math.h>

#ifdef _VECTORMATH_DEBUG
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef _VECTORMATH_AOS_C_TYPES_H
#define _VECTORMATH_AOS_C_TYPES_H

/* A 3-D vector in array-of-structures format
 */
typedef struct _VmathVector3
{
    float x;
    float y;
    float z;
#ifndef __GNUC__
    float d;
#endif
}
#ifdef __GNUC__
__attribute__ ((aligned(16)))
#endif
VmathVector3;

/* A 4-D vector in array-of-structures format
 */
typedef struct _VmathVector4
{
    float x;
    float y;
    float z;
    float w;
}
#ifdef __GNUC__
__attribute__ ((aligned(16)))
#endif
VmathVector4;

/* A 3-D point in array-of-structures format
 */
typedef struct _VmathPoint3
{
    float x;
    float y;
    float z;
#ifndef __GNUC__
    float d;
#endif
}
#ifdef __GNUC__
__attribute__ ((aligned(16)))
#endif
VmathPoint3;

/* A quaternion in array-of-structures format
 */
typedef struct _VmathQuat
{
    float x;
    float y;
    float z;
    float w;
}
#ifdef __GNUC__
__attribute__ ((aligned(16)))
#endif
VmathQuat;

/* A 3x3 matrix in array-of-structures format
 */
typedef struct _VmathMatrix3
{
    VmathVector3 col0;
    VmathVector3 col1;
    VmathVector3 col2;
} VmathMatrix3;

/* A 4x4 matrix in array-of-structures format
 */
typedef struct _VmathMatrix4
{
    VmathVector4 col0;
    VmathVector4 col1;
    VmathVector4 col2;
    VmathVector4 col3;
} VmathMatrix4;

/* A 3x4 transformation matrix in array-of-structures format
 */
typedef struct _VmathTransform3
{
    VmathVector3 col0;
    VmathVector3 col1;
    VmathVector3 col2;
    VmathVector3 col3;
} VmathTransform3;

#endif

/*
 * Copy a 3-D vector
 */
static inline void vmathV3Copy( VmathVector3 *result, const VmathVector3 *vec );

/*
 * Construct a 3-D vector from x, y, and z elements
 */
static inline void vmathV3MakeFromElems( VmathVector3 *result, float x, float y, float z );

/*
 * Copy elements from a 3-D point into a 3-D vector
 */
static inline void vmathV3MakeFromP3( VmathVector3 *result, const VmathPoint3 *pnt );

/*
 * Set all elements of a 3-D vector to the same scalar value
 */
static inline void vmathV3MakeFromScalar( VmathVector3 *result, float scalar );

/*
 * Set the x element of a 3-D vector
 */
static inline void vmathV3SetX( VmathVector3 *result, float x );

/*
 * Set the y element of a 3-D vector
 */
static inline void vmathV3SetY( VmathVector3 *result, float y );

/*
 * Set the z element of a 3-D vector
 */
static inline void vmathV3SetZ( VmathVector3 *result, float z );

/*
 * Get the x element of a 3-D vector
 */
static inline float vmathV3GetX( const VmathVector3 *vec );

/*
 * Get the y element of a 3-D vector
 */
static inline float vmathV3GetY( const VmathVector3 *vec );

/*
 * Get the z element of a 3-D vector
 */
static inline float vmathV3GetZ( const VmathVector3 *vec );

/*
 * Set an x, y, or z element of a 3-D vector by index
 */
static inline void vmathV3SetElem( VmathVector3 *result, int idx, float value );

/*
 * Get an x, y, or z element of a 3-D vector by index
 */
static inline float vmathV3GetElem( const VmathVector3 *vec, int idx );

/*
 * Add two 3-D vectors
 */
static inline void vmathV3Add( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Subtract a 3-D vector from another 3-D vector
 */
static inline void vmathV3Sub( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Add a 3-D vector to a 3-D point
 */
static inline void vmathV3AddP3( VmathPoint3 *result, const VmathVector3 *vec, const VmathPoint3 *pnt );

/*
 * Multiply a 3-D vector by a scalar
 */
static inline void vmathV3ScalarMul( VmathVector3 *result, const VmathVector3 *vec, float scalar );

/*
 * Divide a 3-D vector by a scalar
 */
static inline void vmathV3ScalarDiv( VmathVector3 *result, const VmathVector3 *vec, float scalar );

/*
 * Negate all elements of a 3-D vector
 */
static inline void vmathV3Neg( VmathVector3 *result, const VmathVector3 *vec );

/*
 * Construct x axis
 */
static inline void vmathV3MakeXAxis( VmathVector3 *result );

/*
 * Construct y axis
 */
static inline void vmathV3MakeYAxis( VmathVector3 *result );

/*
 * Construct z axis
 */
static inline void vmathV3MakeZAxis( VmathVector3 *result );

/*
 * Multiply two 3-D vectors per element
 */
static inline void vmathV3MulPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Divide two 3-D vectors per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline void vmathV3DivPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Compute the reciprocal of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline void vmathV3RecipPerElem( VmathVector3 *result, const VmathVector3 *vec );

/*
 * Compute the square root of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline void vmathV3SqrtPerElem( VmathVector3 *result, const VmathVector3 *vec );

/*
 * Compute the reciprocal square root of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline void vmathV3RsqrtPerElem( VmathVector3 *result, const VmathVector3 *vec );

/*
 * Compute the absolute value of a 3-D vector per element
 */
static inline void vmathV3AbsPerElem( VmathVector3 *result, const VmathVector3 *vec );

/*
 * Copy sign from one 3-D vector to another, per element
 */
static inline void vmathV3CopySignPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Maximum of two 3-D vectors per element
 */
static inline void vmathV3MaxPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Minimum of two 3-D vectors per element
 */
static inline void vmathV3MinPerElem( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Maximum element of a 3-D vector
 */
static inline float vmathV3MaxElem( const VmathVector3 *vec );

/*
 * Minimum element of a 3-D vector
 */
static inline float vmathV3MinElem( const VmathVector3 *vec );

/*
 * Compute the sum of all elements of a 3-D vector
 */
static inline float vmathV3Sum( const VmathVector3 *vec );

/*
 * Compute the dot product of two 3-D vectors
 */
static inline float vmathV3Dot( const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Compute the square of the length of a 3-D vector
 */
static inline float vmathV3LengthSqr( const VmathVector3 *vec );

/*
 * Compute the length of a 3-D vector
 */
static inline float vmathV3Length( const VmathVector3 *vec );

/*
 * Normalize a 3-D vector
 * NOTE: 
 * The result is unpredictable when all elements of vec are at or near zero.
 */
static inline void vmathV3Normalize( VmathVector3 *result, const VmathVector3 *vec );

/*
 * Compute cross product of two 3-D vectors
 */
static inline void vmathV3Cross( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Outer product of two 3-D vectors
 */
static inline void vmathV3Outer( VmathMatrix3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Pre-multiply a row vector by a 3x3 matrix
 */
static inline void vmathV3RowMul( VmathVector3 *result, const VmathVector3 *vec, const VmathMatrix3 *mat );

/*
 * Cross-product matrix of a 3-D vector
 */
static inline void vmathV3CrossMatrix( VmathMatrix3 *result, const VmathVector3 *vec );

/*
 * Create cross-product matrix and multiply
 * NOTE: 
 * Faster than separately creating a cross-product matrix and multiplying.
 */
static inline void vmathV3CrossMatrixMul( VmathMatrix3 *result, const VmathVector3 *vec, const VmathMatrix3 *mat );

/*
 * Linear interpolation between two 3-D vectors
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline void vmathV3Lerp( VmathVector3 *result, float t, const VmathVector3 *vec0, const VmathVector3 *vec1 );

/*
 * Spherical linear interpolation between two 3-D vectors
 * NOTE: 
 * The result is unpredictable if the vectors point in opposite directions.
 * Does not clamp t between 0 and 1.
 */
static inline void vmathV3Slerp( VmathVector3 *result, float t, const VmathVector3 *unitVec0, const VmathVector3 *unitVec1 );

/*
 * Conditionally select between two 3-D vectors
 */
static inline void vmathV3Select( VmathVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3-D vector
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathV3Print( const VmathVector3 *vec );

/*
 * Print a 3-D vector and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathV3Prints( const VmathVector3 *vec, const char *name );

#endif

/*
 * Copy a 4-D vector
 */
static inline void vmathV4Copy( VmathVector4 *result, const VmathVector4 *vec );

/*
 * Construct a 4-D vector from x, y, z, and w elements
 */
static inline void vmathV4MakeFromElems( VmathVector4 *result, float x, float y, float z, float w );

/*
 * Construct a 4-D vector from a 3-D vector and a scalar
 */
static inline void vmathV4MakeFromV3Scalar( VmathVector4 *result, const VmathVector3 *xyz, float w );

/*
 * Copy x, y, and z from a 3-D vector into a 4-D vector, and set w to 0
 */
static inline void vmathV4MakeFromV3( VmathVector4 *result, const VmathVector3 *vec );

/*
 * Copy x, y, and z from a 3-D point into a 4-D vector, and set w to 1
 */
static inline void vmathV4MakeFromP3( VmathVector4 *result, const VmathPoint3 *pnt );

/*
 * Copy elements from a quaternion into a 4-D vector
 */
static inline void vmathV4MakeFromQ( VmathVector4 *result, const VmathQuat *quat );

/*
 * Set all elements of a 4-D vector to the same scalar value
 */
static inline void vmathV4MakeFromScalar( VmathVector4 *result, float scalar );

/*
 * Set the x, y, and z elements of a 4-D vector
 * NOTE: 
 * This function does not change the w element.
 */
static inline void vmathV4SetXYZ( VmathVector4 *result, const VmathVector3 *vec );

/*
 * Get the x, y, and z elements of a 4-D vector
 */
static inline void vmathV4GetXYZ( VmathVector3 *result, const VmathVector4 *vec );

/*
 * Set the x element of a 4-D vector
 */
static inline void vmathV4SetX( VmathVector4 *result, float x );

/*
 * Set the y element of a 4-D vector
 */
static inline void vmathV4SetY( VmathVector4 *result, float y );

/*
 * Set the z element of a 4-D vector
 */
static inline void vmathV4SetZ( VmathVector4 *result, float z );

/*
 * Set the w element of a 4-D vector
 */
static inline void vmathV4SetW( VmathVector4 *result, float w );

/*
 * Get the x element of a 4-D vector
 */
static inline float vmathV4GetX( const VmathVector4 *vec );

/*
 * Get the y element of a 4-D vector
 */
static inline float vmathV4GetY( const VmathVector4 *vec );

/*
 * Get the z element of a 4-D vector
 */
static inline float vmathV4GetZ( const VmathVector4 *vec );

/*
 * Get the w element of a 4-D vector
 */
static inline float vmathV4GetW( const VmathVector4 *vec );

/*
 * Set an x, y, z, or w element of a 4-D vector by index
 */
static inline void vmathV4SetElem( VmathVector4 *result, int idx, float value );

/*
 * Get an x, y, z, or w element of a 4-D vector by index
 */
static inline float vmathV4GetElem( const VmathVector4 *vec, int idx );

/*
 * Add two 4-D vectors
 */
static inline void vmathV4Add( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Subtract a 4-D vector from another 4-D vector
 */
static inline void vmathV4Sub( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Multiply a 4-D vector by a scalar
 */
static inline void vmathV4ScalarMul( VmathVector4 *result, const VmathVector4 *vec, float scalar );

/*
 * Divide a 4-D vector by a scalar
 */
static inline void vmathV4ScalarDiv( VmathVector4 *result, const VmathVector4 *vec, float scalar );

/*
 * Negate all elements of a 4-D vector
 */
static inline void vmathV4Neg( VmathVector4 *result, const VmathVector4 *vec );

/*
 * Construct x axis
 */
static inline void vmathV4MakeXAxis( VmathVector4 *result );

/*
 * Construct y axis
 */
static inline void vmathV4MakeYAxis( VmathVector4 *result );

/*
 * Construct z axis
 */
static inline void vmathV4MakeZAxis( VmathVector4 *result );

/*
 * Construct w axis
 */
static inline void vmathV4MakeWAxis( VmathVector4 *result );

/*
 * Multiply two 4-D vectors per element
 */
static inline void vmathV4MulPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Divide two 4-D vectors per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline void vmathV4DivPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Compute the reciprocal of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline void vmathV4RecipPerElem( VmathVector4 *result, const VmathVector4 *vec );

/*
 * Compute the square root of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline void vmathV4SqrtPerElem( VmathVector4 *result, const VmathVector4 *vec );

/*
 * Compute the reciprocal square root of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline void vmathV4RsqrtPerElem( VmathVector4 *result, const VmathVector4 *vec );

/*
 * Compute the absolute value of a 4-D vector per element
 */
static inline void vmathV4AbsPerElem( VmathVector4 *result, const VmathVector4 *vec );

/*
 * Copy sign from one 4-D vector to another, per element
 */
static inline void vmathV4CopySignPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Maximum of two 4-D vectors per element
 */
static inline void vmathV4MaxPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Minimum of two 4-D vectors per element
 */
static inline void vmathV4MinPerElem( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Maximum element of a 4-D vector
 */
static inline float vmathV4MaxElem( const VmathVector4 *vec );

/*
 * Minimum element of a 4-D vector
 */
static inline float vmathV4MinElem( const VmathVector4 *vec );

/*
 * Compute the sum of all elements of a 4-D vector
 */
static inline float vmathV4Sum( const VmathVector4 *vec );

/*
 * Compute the dot product of two 4-D vectors
 */
static inline float vmathV4Dot( const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Compute the square of the length of a 4-D vector
 */
static inline float vmathV4LengthSqr( const VmathVector4 *vec );

/*
 * Compute the length of a 4-D vector
 */
static inline float vmathV4Length( const VmathVector4 *vec );

/*
 * Normalize a 4-D vector
 * NOTE: 
 * The result is unpredictable when all elements of vec are at or near zero.
 */
static inline void vmathV4Normalize( VmathVector4 *result, const VmathVector4 *vec );

/*
 * Outer product of two 4-D vectors
 */
static inline void vmathV4Outer( VmathMatrix4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Linear interpolation between two 4-D vectors
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline void vmathV4Lerp( VmathVector4 *result, float t, const VmathVector4 *vec0, const VmathVector4 *vec1 );

/*
 * Spherical linear interpolation between two 4-D vectors
 * NOTE: 
 * The result is unpredictable if the vectors point in opposite directions.
 * Does not clamp t between 0 and 1.
 */
static inline void vmathV4Slerp( VmathVector4 *result, float t, const VmathVector4 *unitVec0, const VmathVector4 *unitVec1 );

/*
 * Conditionally select between two 4-D vectors
 */
static inline void vmathV4Select( VmathVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 4-D vector
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathV4Print( const VmathVector4 *vec );

/*
 * Print a 4-D vector and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathV4Prints( const VmathVector4 *vec, const char *name );

#endif

/*
 * Copy a 3-D point
 */
static inline void vmathP3Copy( VmathPoint3 *result, const VmathPoint3 *pnt );

/*
 * Construct a 3-D point from x, y, and z elements
 */
static inline void vmathP3MakeFromElems( VmathPoint3 *result, float x, float y, float z );

/*
 * Copy elements from a 3-D vector into a 3-D point
 */
static inline void vmathP3MakeFromV3( VmathPoint3 *result, const VmathVector3 *vec );

/*
 * Set all elements of a 3-D point to the same scalar value
 */
static inline void vmathP3MakeFromScalar( VmathPoint3 *result, float scalar );

/*
 * Set the x element of a 3-D point
 */
static inline void vmathP3SetX( VmathPoint3 *result, float x );

/*
 * Set the y element of a 3-D point
 */
static inline void vmathP3SetY( VmathPoint3 *result, float y );

/*
 * Set the z element of a 3-D point
 */
static inline void vmathP3SetZ( VmathPoint3 *result, float z );

/*
 * Get the x element of a 3-D point
 */
static inline float vmathP3GetX( const VmathPoint3 *pnt );

/*
 * Get the y element of a 3-D point
 */
static inline float vmathP3GetY( const VmathPoint3 *pnt );

/*
 * Get the z element of a 3-D point
 */
static inline float vmathP3GetZ( const VmathPoint3 *pnt );

/*
 * Set an x, y, or z element of a 3-D point by index
 */
static inline void vmathP3SetElem( VmathPoint3 *result, int idx, float value );

/*
 * Get an x, y, or z element of a 3-D point by index
 */
static inline float vmathP3GetElem( const VmathPoint3 *pnt, int idx );

/*
 * Subtract a 3-D point from another 3-D point
 */
static inline void vmathP3Sub( VmathVector3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Add a 3-D point to a 3-D vector
 */
static inline void vmathP3AddV3( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *vec );

/*
 * Subtract a 3-D vector from a 3-D point
 */
static inline void vmathP3SubV3( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *vec );

/*
 * Multiply two 3-D points per element
 */
static inline void vmathP3MulPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Divide two 3-D points per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline void vmathP3DivPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Compute the reciprocal of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline void vmathP3RecipPerElem( VmathPoint3 *result, const VmathPoint3 *pnt );

/*
 * Compute the square root of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline void vmathP3SqrtPerElem( VmathPoint3 *result, const VmathPoint3 *pnt );

/*
 * Compute the reciprocal square root of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline void vmathP3RsqrtPerElem( VmathPoint3 *result, const VmathPoint3 *pnt );

/*
 * Compute the absolute value of a 3-D point per element
 */
static inline void vmathP3AbsPerElem( VmathPoint3 *result, const VmathPoint3 *pnt );

/*
 * Copy sign from one 3-D point to another, per element
 */
static inline void vmathP3CopySignPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Maximum of two 3-D points per element
 */
static inline void vmathP3MaxPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Minimum of two 3-D points per element
 */
static inline void vmathP3MinPerElem( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Maximum element of a 3-D point
 */
static inline float vmathP3MaxElem( const VmathPoint3 *pnt );

/*
 * Minimum element of a 3-D point
 */
static inline float vmathP3MinElem( const VmathPoint3 *pnt );

/*
 * Compute the sum of all elements of a 3-D point
 */
static inline float vmathP3Sum( const VmathPoint3 *pnt );

/*
 * Apply uniform scale to a 3-D point
 */
static inline void vmathP3Scale( VmathPoint3 *result, const VmathPoint3 *pnt, float scaleVal );

/*
 * Apply non-uniform scale to a 3-D point
 */
static inline void vmathP3NonUniformScale( VmathPoint3 *result, const VmathPoint3 *pnt, const VmathVector3 *scaleVec );

/*
 * Scalar projection of a 3-D point on a unit-length 3-D vector
 */
static inline float vmathP3Projection( const VmathPoint3 *pnt, const VmathVector3 *unitVec );

/*
 * Compute the square of the distance of a 3-D point from the coordinate-system origin
 */
static inline float vmathP3DistSqrFromOrigin( const VmathPoint3 *pnt );

/*
 * Compute the distance of a 3-D point from the coordinate-system origin
 */
static inline float vmathP3DistFromOrigin( const VmathPoint3 *pnt );

/*
 * Compute the square of the distance between two 3-D points
 */
static inline float vmathP3DistSqr( const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Compute the distance between two 3-D points
 */
static inline float vmathP3Dist( const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Linear interpolation between two 3-D points
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline void vmathP3Lerp( VmathPoint3 *result, float t, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1 );

/*
 * Conditionally select between two 3-D points
 */
static inline void vmathP3Select( VmathPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3-D point
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathP3Print( const VmathPoint3 *pnt );

/*
 * Print a 3-D point and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathP3Prints( const VmathPoint3 *pnt, const char *name );

#endif

/*
 * Copy a quaternion
 */
static inline void vmathQCopy( VmathQuat *result, const VmathQuat *quat );

/*
 * Construct a quaternion from x, y, z, and w elements
 */
static inline void vmathQMakeFromElems( VmathQuat *result, float x, float y, float z, float w );

/*
 * Construct a quaternion from a 3-D vector and a scalar
 */
static inline void vmathQMakeFromV3Scalar( VmathQuat *result, const VmathVector3 *xyz, float w );

/*
 * Copy elements from a 4-D vector into a quaternion
 */
static inline void vmathQMakeFromV4( VmathQuat *result, const VmathVector4 *vec );

/*
 * Convert a rotation matrix to a unit-length quaternion
 */
static inline void vmathQMakeFromM3( VmathQuat *result, const VmathMatrix3 *rotMat );

/*
 * Set all elements of a quaternion to the same scalar value
 */
static inline void vmathQMakeFromScalar( VmathQuat *result, float scalar );

/*
 * Set the x, y, and z elements of a quaternion
 * NOTE: 
 * This function does not change the w element.
 */
static inline void vmathQSetXYZ( VmathQuat *result, const VmathVector3 *vec );

/*
 * Get the x, y, and z elements of a quaternion
 */
static inline void vmathQGetXYZ( VmathVector3 *result, const VmathQuat *quat );

/*
 * Set the x element of a quaternion
 */
static inline void vmathQSetX( VmathQuat *result, float x );

/*
 * Set the y element of a quaternion
 */
static inline void vmathQSetY( VmathQuat *result, float y );

/*
 * Set the z element of a quaternion
 */
static inline void vmathQSetZ( VmathQuat *result, float z );

/*
 * Set the w element of a quaternion
 */
static inline void vmathQSetW( VmathQuat *result, float w );

/*
 * Get the x element of a quaternion
 */
static inline float vmathQGetX( const VmathQuat *quat );

/*
 * Get the y element of a quaternion
 */
static inline float vmathQGetY( const VmathQuat *quat );

/*
 * Get the z element of a quaternion
 */
static inline float vmathQGetZ( const VmathQuat *quat );

/*
 * Get the w element of a quaternion
 */
static inline float vmathQGetW( const VmathQuat *quat );

/*
 * Set an x, y, z, or w element of a quaternion by index
 */
static inline void vmathQSetElem( VmathQuat *result, int idx, float value );

/*
 * Get an x, y, z, or w element of a quaternion by index
 */
static inline float vmathQGetElem( const VmathQuat *quat, int idx );

/*
 * Add two quaternions
 */
static inline void vmathQAdd( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 );

/*
 * Subtract a quaternion from another quaternion
 */
static inline void vmathQSub( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 );

/*
 * Multiply two quaternions
 */
static inline void vmathQMul( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 );

/*
 * Multiply a quaternion by a scalar
 */
static inline void vmathQScalarMul( VmathQuat *result, const VmathQuat *quat, float scalar );

/*
 * Divide a quaternion by a scalar
 */
static inline void vmathQScalarDiv( VmathQuat *result, const VmathQuat *quat, float scalar );

/*
 * Negate all elements of a quaternion
 */
static inline void vmathQNeg( VmathQuat *result, const VmathQuat *quat );

/*
 * Construct an identity quaternion
 */
static inline void vmathQMakeIdentity( VmathQuat *result );

/*
 * Construct a quaternion to rotate between two unit-length 3-D vectors
 * NOTE: 
 * The result is unpredictable if unitVec0 and unitVec1 point in opposite directions.
 */
static inline void vmathQMakeRotationArc( VmathQuat *result, const VmathVector3 *unitVec0, const VmathVector3 *unitVec1 );

/*
 * Construct a quaternion to rotate around a unit-length 3-D vector
 */
static inline void vmathQMakeRotationAxis( VmathQuat *result, float radians, const VmathVector3 *unitVec );

/*
 * Construct a quaternion to rotate around the x axis
 */
static inline void vmathQMakeRotationX( VmathQuat *result, float radians );

/*
 * Construct a quaternion to rotate around the y axis
 */
static inline void vmathQMakeRotationY( VmathQuat *result, float radians );

/*
 * Construct a quaternion to rotate around the z axis
 */
static inline void vmathQMakeRotationZ( VmathQuat *result, float radians );

/*
 * Compute the conjugate of a quaternion
 */
static inline void vmathQConj( VmathQuat *result, const VmathQuat *quat );

/*
 * Use a unit-length quaternion to rotate a 3-D vector
 */
static inline void vmathQRotate( VmathVector3 *result, const VmathQuat *unitQuat, const VmathVector3 *vec );

/*
 * Compute the dot product of two quaternions
 */
static inline float vmathQDot( const VmathQuat *quat0, const VmathQuat *quat1 );

/*
 * Compute the norm of a quaternion
 */
static inline float vmathQNorm( const VmathQuat *quat );

/*
 * Compute the length of a quaternion
 */
static inline float vmathQLength( const VmathQuat *quat );

/*
 * Normalize a quaternion
 * NOTE: 
 * The result is unpredictable when all elements of quat are at or near zero.
 */
static inline void vmathQNormalize( VmathQuat *result, const VmathQuat *quat );

/*
 * Linear interpolation between two quaternions
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline void vmathQLerp( VmathQuat *result, float t, const VmathQuat *quat0, const VmathQuat *quat1 );

/*
 * Spherical linear interpolation between two quaternions
 * NOTE: 
 * Interpolates along the shortest path between orientations.
 * Does not clamp t between 0 and 1.
 */
static inline void vmathQSlerp( VmathQuat *result, float t, const VmathQuat *unitQuat0, const VmathQuat *unitQuat1 );

/*
 * Spherical quadrangle interpolation
 */
static inline void vmathQSquad( VmathQuat *result, float t, const VmathQuat *unitQuat0, const VmathQuat *unitQuat1, const VmathQuat *unitQuat2, const VmathQuat *unitQuat3 );

/*
 * Conditionally select between two quaternions
 */
static inline void vmathQSelect( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a quaternion
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathQPrint( const VmathQuat *quat );

/*
 * Print a quaternion and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathQPrints( const VmathQuat *quat, const char *name );

#endif

/*
 * Copy a 3x3 matrix
 */
static inline void vmathM3Copy( VmathMatrix3 *result, const VmathMatrix3 *mat );

/*
 * Construct a 3x3 matrix containing the specified columns
 */
static inline void vmathM3MakeFromCols( VmathMatrix3 *result, const VmathVector3 *col0, const VmathVector3 *col1, const VmathVector3 *col2 );

/*
 * Construct a 3x3 rotation matrix from a unit-length quaternion
 */
static inline void vmathM3MakeFromQ( VmathMatrix3 *result, const VmathQuat *unitQuat );

/*
 * Set all elements of a 3x3 matrix to the same scalar value
 */
static inline void vmathM3MakeFromScalar( VmathMatrix3 *result, float scalar );

/*
 * Set column 0 of a 3x3 matrix
 */
static inline void vmathM3SetCol0( VmathMatrix3 *result, const VmathVector3 *col0 );

/*
 * Set column 1 of a 3x3 matrix
 */
static inline void vmathM3SetCol1( VmathMatrix3 *result, const VmathVector3 *col1 );

/*
 * Set column 2 of a 3x3 matrix
 */
static inline void vmathM3SetCol2( VmathMatrix3 *result, const VmathVector3 *col2 );

/*
 * Get column 0 of a 3x3 matrix
 */
static inline void vmathM3GetCol0( VmathVector3 *result, const VmathMatrix3 *mat );

/*
 * Get column 1 of a 3x3 matrix
 */
static inline void vmathM3GetCol1( VmathVector3 *result, const VmathMatrix3 *mat );

/*
 * Get column 2 of a 3x3 matrix
 */
static inline void vmathM3GetCol2( VmathVector3 *result, const VmathMatrix3 *mat );

/*
 * Set the column of a 3x3 matrix referred to by the specified index
 */
static inline void vmathM3SetCol( VmathMatrix3 *result, int col, const VmathVector3 *vec );

/*
 * Set the row of a 3x3 matrix referred to by the specified index
 */
static inline void vmathM3SetRow( VmathMatrix3 *result, int row, const VmathVector3 *vec );

/*
 * Get the column of a 3x3 matrix referred to by the specified index
 */
static inline void vmathM3GetCol( VmathVector3 *result, const VmathMatrix3 *mat, int col );

/*
 * Get the row of a 3x3 matrix referred to by the specified index
 */
static inline void vmathM3GetRow( VmathVector3 *result, const VmathMatrix3 *mat, int row );

/*
 * Set the element of a 3x3 matrix referred to by column and row indices
 */
static inline void vmathM3SetElem( VmathMatrix3 *result, int col, int row, float val );

/*
 * Get the element of a 3x3 matrix referred to by column and row indices
 */
static inline float vmathM3GetElem( const VmathMatrix3 *mat, int col, int row );

/*
 * Add two 3x3 matrices
 */
static inline void vmathM3Add( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 );

/*
 * Subtract a 3x3 matrix from another 3x3 matrix
 */
static inline void vmathM3Sub( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 );

/*
 * Negate all elements of a 3x3 matrix
 */
static inline void vmathM3Neg( VmathMatrix3 *result, const VmathMatrix3 *mat );

/*
 * Multiply a 3x3 matrix by a scalar
 */
static inline void vmathM3ScalarMul( VmathMatrix3 *result, const VmathMatrix3 *mat, float scalar );

/*
 * Multiply a 3x3 matrix by a 3-D vector
 */
static inline void vmathM3MulV3( VmathVector3 *result, const VmathMatrix3 *mat, const VmathVector3 *vec );

/*
 * Multiply two 3x3 matrices
 */
static inline void vmathM3Mul( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 );

/*
 * Construct an identity 3x3 matrix
 */
static inline void vmathM3MakeIdentity( VmathMatrix3 *result );

/*
 * Construct a 3x3 matrix to rotate around the x axis
 */
static inline void vmathM3MakeRotationX( VmathMatrix3 *result, float radians );

/*
 * Construct a 3x3 matrix to rotate around the y axis
 */
static inline void vmathM3MakeRotationY( VmathMatrix3 *result, float radians );

/*
 * Construct a 3x3 matrix to rotate around the z axis
 */
static inline void vmathM3MakeRotationZ( VmathMatrix3 *result, float radians );

/*
 * Construct a 3x3 matrix to rotate around the x, y, and z axes
 */
static inline void vmathM3MakeRotationZYX( VmathMatrix3 *result, const VmathVector3 *radiansXYZ );

/*
 * Construct a 3x3 matrix to rotate around a unit-length 3-D vector
 */
static inline void vmathM3MakeRotationAxis( VmathMatrix3 *result, float radians, const VmathVector3 *unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline void vmathM3MakeRotationQ( VmathMatrix3 *result, const VmathQuat *unitQuat );

/*
 * Construct a 3x3 matrix to perform scaling
 */
static inline void vmathM3MakeScale( VmathMatrix3 *result, const VmathVector3 *scaleVec );

/*
 * Append (post-multiply) a scale transformation to a 3x3 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathM3AppendScale( VmathMatrix3 *result, const VmathMatrix3 *mat, const VmathVector3 *scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 3x3 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathM3PrependScale( VmathMatrix3 *result, const VmathVector3 *scaleVec, const VmathMatrix3 *mat );

/*
 * Multiply two 3x3 matrices per element
 */
static inline void vmathM3MulPerElem( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 );

/*
 * Compute the absolute value of a 3x3 matrix per element
 */
static inline void vmathM3AbsPerElem( VmathMatrix3 *result, const VmathMatrix3 *mat );

/*
 * Transpose of a 3x3 matrix
 */
static inline void vmathM3Transpose( VmathMatrix3 *result, const VmathMatrix3 *mat );

/*
 * Compute the inverse of a 3x3 matrix
 * NOTE: 
 * Result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline void vmathM3Inverse( VmathMatrix3 *result, const VmathMatrix3 *mat );

/*
 * Determinant of a 3x3 matrix
 */
static inline float vmathM3Determinant( const VmathMatrix3 *mat );

/*
 * Conditionally select between two 3x3 matrices
 */
static inline void vmathM3Select( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3x3 matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathM3Print( const VmathMatrix3 *mat );

/*
 * Print a 3x3 matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathM3Prints( const VmathMatrix3 *mat, const char *name );

#endif

/*
 * Copy a 4x4 matrix
 */
static inline void vmathM4Copy( VmathMatrix4 *result, const VmathMatrix4 *mat );

/*
 * Construct a 4x4 matrix containing the specified columns
 */
static inline void vmathM4MakeFromCols( VmathMatrix4 *result, const VmathVector4 *col0, const VmathVector4 *col1, const VmathVector4 *col2, const VmathVector4 *col3 );

/*
 * Construct a 4x4 matrix from a 3x4 transformation matrix
 */
static inline void vmathM4MakeFromT3( VmathMatrix4 *result, const VmathTransform3 *mat );

/*
 * Construct a 4x4 matrix from a 3x3 matrix and a 3-D vector
 */
static inline void vmathM4MakeFromM3V3( VmathMatrix4 *result, const VmathMatrix3 *mat, const VmathVector3 *translateVec );

/*
 * Construct a 4x4 matrix from a unit-length quaternion and a 3-D vector
 */
static inline void vmathM4MakeFromQV3( VmathMatrix4 *result, const VmathQuat *unitQuat, const VmathVector3 *translateVec );

/*
 * Set all elements of a 4x4 matrix to the same scalar value
 */
static inline void vmathM4MakeFromScalar( VmathMatrix4 *result, float scalar );

/*
 * Set the upper-left 3x3 submatrix
 * NOTE: 
 * This function does not change the bottom row elements.
 */
static inline void vmathM4SetUpper3x3( VmathMatrix4 *result, const VmathMatrix3 *mat3 );

/*
 * Get the upper-left 3x3 submatrix of a 4x4 matrix
 */
static inline void vmathM4GetUpper3x3( VmathMatrix3 *result, const VmathMatrix4 *mat );

/*
 * Set translation component
 * NOTE: 
 * This function does not change the bottom row elements.
 */
static inline void vmathM4SetTranslation( VmathMatrix4 *result, const VmathVector3 *translateVec );

/*
 * Get the translation component of a 4x4 matrix
 */
static inline void vmathM4GetTranslation( VmathVector3 *result, const VmathMatrix4 *mat );

/*
 * Set column 0 of a 4x4 matrix
 */
static inline void vmathM4SetCol0( VmathMatrix4 *result, const VmathVector4 *col0 );

/*
 * Set column 1 of a 4x4 matrix
 */
static inline void vmathM4SetCol1( VmathMatrix4 *result, const VmathVector4 *col1 );

/*
 * Set column 2 of a 4x4 matrix
 */
static inline void vmathM4SetCol2( VmathMatrix4 *result, const VmathVector4 *col2 );

/*
 * Set column 3 of a 4x4 matrix
 */
static inline void vmathM4SetCol3( VmathMatrix4 *result, const VmathVector4 *col3 );

/*
 * Get column 0 of a 4x4 matrix
 */
static inline void vmathM4GetCol0( VmathVector4 *result, const VmathMatrix4 *mat );

/*
 * Get column 1 of a 4x4 matrix
 */
static inline void vmathM4GetCol1( VmathVector4 *result, const VmathMatrix4 *mat );

/*
 * Get column 2 of a 4x4 matrix
 */
static inline void vmathM4GetCol2( VmathVector4 *result, const VmathMatrix4 *mat );

/*
 * Get column 3 of a 4x4 matrix
 */
static inline void vmathM4GetCol3( VmathVector4 *result, const VmathMatrix4 *mat );

/*
 * Set the column of a 4x4 matrix referred to by the specified index
 */
static inline void vmathM4SetCol( VmathMatrix4 *result, int col, const VmathVector4 *vec );

/*
 * Set the row of a 4x4 matrix referred to by the specified index
 */
static inline void vmathM4SetRow( VmathMatrix4 *result, int row, const VmathVector4 *vec );

/*
 * Get the column of a 4x4 matrix referred to by the specified index
 */
static inline void vmathM4GetCol( VmathVector4 *result, const VmathMatrix4 *mat, int col );

/*
 * Get the row of a 4x4 matrix referred to by the specified index
 */
static inline void vmathM4GetRow( VmathVector4 *result, const VmathMatrix4 *mat, int row );

/*
 * Set the element of a 4x4 matrix referred to by column and row indices
 */
static inline void vmathM4SetElem( VmathMatrix4 *result, int col, int row, float val );

/*
 * Get the element of a 4x4 matrix referred to by column and row indices
 */
static inline float vmathM4GetElem( const VmathMatrix4 *mat, int col, int row );

/*
 * Add two 4x4 matrices
 */
static inline void vmathM4Add( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 );

/*
 * Subtract a 4x4 matrix from another 4x4 matrix
 */
static inline void vmathM4Sub( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 );

/*
 * Negate all elements of a 4x4 matrix
 */
static inline void vmathM4Neg( VmathMatrix4 *result, const VmathMatrix4 *mat );

/*
 * Multiply a 4x4 matrix by a scalar
 */
static inline void vmathM4ScalarMul( VmathMatrix4 *result, const VmathMatrix4 *mat, float scalar );

/*
 * Multiply a 4x4 matrix by a 4-D vector
 */
static inline void vmathM4MulV4( VmathVector4 *result, const VmathMatrix4 *mat, const VmathVector4 *vec );

/*
 * Multiply a 4x4 matrix by a 3-D vector
 */
static inline void vmathM4MulV3( VmathVector4 *result, const VmathMatrix4 *mat, const VmathVector3 *vec );

/*
 * Multiply a 4x4 matrix by a 3-D point
 */
static inline void vmathM4MulP3( VmathVector4 *result, const VmathMatrix4 *mat, const VmathPoint3 *pnt );

/*
 * Multiply two 4x4 matrices
 */
static inline void vmathM4Mul( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 );

/*
 * Multiply a 4x4 matrix by a 3x4 transformation matrix
 */
static inline void vmathM4MulT3( VmathMatrix4 *result, const VmathMatrix4 *mat, const VmathTransform3 *tfrm );

/*
 * Construct an identity 4x4 matrix
 */
static inline void vmathM4MakeIdentity( VmathMatrix4 *result );

/*
 * Construct a 4x4 matrix to rotate around the x axis
 */
static inline void vmathM4MakeRotationX( VmathMatrix4 *result, float radians );

/*
 * Construct a 4x4 matrix to rotate around the y axis
 */
static inline void vmathM4MakeRotationY( VmathMatrix4 *result, float radians );

/*
 * Construct a 4x4 matrix to rotate around the z axis
 */
static inline void vmathM4MakeRotationZ( VmathMatrix4 *result, float radians );

/*
 * Construct a 4x4 matrix to rotate around the x, y, and z axes
 */
static inline void vmathM4MakeRotationZYX( VmathMatrix4 *result, const VmathVector3 *radiansXYZ );

/*
 * Construct a 4x4 matrix to rotate around a unit-length 3-D vector
 */
static inline void vmathM4MakeRotationAxis( VmathMatrix4 *result, float radians, const VmathVector3 *unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline void vmathM4MakeRotationQ( VmathMatrix4 *result, const VmathQuat *unitQuat );

/*
 * Construct a 4x4 matrix to perform scaling
 */
static inline void vmathM4MakeScale( VmathMatrix4 *result, const VmathVector3 *scaleVec );

/*
 * Construct a 4x4 matrix to perform translation
 */
static inline void vmathM4MakeTranslation( VmathMatrix4 *result, const VmathVector3 *translateVec );

/*
 * Construct viewing matrix based on eye position, position looked at, and up direction
 */
static inline void vmathM4MakeLookAt( VmathMatrix4 *result, const VmathPoint3 *eyePos, const VmathPoint3 *lookAtPos, const VmathVector3 *upVec );

/*
 * Construct a perspective projection matrix
 */
static inline void vmathM4MakePerspective( VmathMatrix4 *result, float fovyRadians, float aspect, float zNear, float zFar );

/*
 * Construct a perspective projection matrix based on frustum
 */
static inline void vmathM4MakeFrustum( VmathMatrix4 *result, float left, float right, float bottom, float top, float zNear, float zFar );

/*
 * Construct an orthographic projection matrix
 */
static inline void vmathM4MakeOrthographic( VmathMatrix4 *result, float left, float right, float bottom, float top, float zNear, float zFar );

/*
 * Append (post-multiply) a scale transformation to a 4x4 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathM4AppendScale( VmathMatrix4 *result, const VmathMatrix4 *mat, const VmathVector3 *scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 4x4 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathM4PrependScale( VmathMatrix4 *result, const VmathVector3 *scaleVec, const VmathMatrix4 *mat );

/*
 * Multiply two 4x4 matrices per element
 */
static inline void vmathM4MulPerElem( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 );

/*
 * Compute the absolute value of a 4x4 matrix per element
 */
static inline void vmathM4AbsPerElem( VmathMatrix4 *result, const VmathMatrix4 *mat );

/*
 * Transpose of a 4x4 matrix
 */
static inline void vmathM4Transpose( VmathMatrix4 *result, const VmathMatrix4 *mat );

/*
 * Compute the inverse of a 4x4 matrix
 * NOTE: 
 * Result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline void vmathM4Inverse( VmathMatrix4 *result, const VmathMatrix4 *mat );

/*
 * Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.  The result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline void vmathM4AffineInverse( VmathMatrix4 *result, const VmathMatrix4 *mat );

/*
 * Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix with an orthogonal upper-left 3x3 submatrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.
 */
static inline void vmathM4OrthoInverse( VmathMatrix4 *result, const VmathMatrix4 *mat );

/*
 * Determinant of a 4x4 matrix
 */
static inline float vmathM4Determinant( const VmathMatrix4 *mat );

/*
 * Conditionally select between two 4x4 matrices
 */
static inline void vmathM4Select( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 4x4 matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathM4Print( const VmathMatrix4 *mat );

/*
 * Print a 4x4 matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathM4Prints( const VmathMatrix4 *mat, const char *name );

#endif

/*
 * Copy a 3x4 transformation matrix
 */
static inline void vmathT3Copy( VmathTransform3 *result, const VmathTransform3 *tfrm );

/*
 * Construct a 3x4 transformation matrix containing the specified columns
 */
static inline void vmathT3MakeFromCols( VmathTransform3 *result, const VmathVector3 *col0, const VmathVector3 *col1, const VmathVector3 *col2, const VmathVector3 *col3 );

/*
 * Construct a 3x4 transformation matrix from a 3x3 matrix and a 3-D vector
 */
static inline void vmathT3MakeFromM3V3( VmathTransform3 *result, const VmathMatrix3 *tfrm, const VmathVector3 *translateVec );

/*
 * Construct a 3x4 transformation matrix from a unit-length quaternion and a 3-D vector
 */
static inline void vmathT3MakeFromQV3( VmathTransform3 *result, const VmathQuat *unitQuat, const VmathVector3 *translateVec );

/*
 * Set all elements of a 3x4 transformation matrix to the same scalar value
 */
static inline void vmathT3MakeFromScalar( VmathTransform3 *result, float scalar );

/*
 * Set the upper-left 3x3 submatrix
 */
static inline void vmathT3SetUpper3x3( VmathTransform3 *result, const VmathMatrix3 *mat3 );

/*
 * Get the upper-left 3x3 submatrix of a 3x4 transformation matrix
 */
static inline void vmathT3GetUpper3x3( VmathMatrix3 *result, const VmathTransform3 *tfrm );

/*
 * Set translation component
 */
static inline void vmathT3SetTranslation( VmathTransform3 *result, const VmathVector3 *translateVec );

/*
 * Get the translation component of a 3x4 transformation matrix
 */
static inline void vmathT3GetTranslation( VmathVector3 *result, const VmathTransform3 *tfrm );

/*
 * Set column 0 of a 3x4 transformation matrix
 */
static inline void vmathT3SetCol0( VmathTransform3 *result, const VmathVector3 *col0 );

/*
 * Set column 1 of a 3x4 transformation matrix
 */
static inline void vmathT3SetCol1( VmathTransform3 *result, const VmathVector3 *col1 );

/*
 * Set column 2 of a 3x4 transformation matrix
 */
static inline void vmathT3SetCol2( VmathTransform3 *result, const VmathVector3 *col2 );

/*
 * Set column 3 of a 3x4 transformation matrix
 */
static inline void vmathT3SetCol3( VmathTransform3 *result, const VmathVector3 *col3 );

/*
 * Get column 0 of a 3x4 transformation matrix
 */
static inline void vmathT3GetCol0( VmathVector3 *result, const VmathTransform3 *tfrm );

/*
 * Get column 1 of a 3x4 transformation matrix
 */
static inline void vmathT3GetCol1( VmathVector3 *result, const VmathTransform3 *tfrm );

/*
 * Get column 2 of a 3x4 transformation matrix
 */
static inline void vmathT3GetCol2( VmathVector3 *result, const VmathTransform3 *tfrm );

/*
 * Get column 3 of a 3x4 transformation matrix
 */
static inline void vmathT3GetCol3( VmathVector3 *result, const VmathTransform3 *tfrm );

/*
 * Set the column of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathT3SetCol( VmathTransform3 *result, int col, const VmathVector3 *vec );

/*
 * Set the row of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathT3SetRow( VmathTransform3 *result, int row, const VmathVector4 *vec );

/*
 * Get the column of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathT3GetCol( VmathVector3 *result, const VmathTransform3 *tfrm, int col );

/*
 * Get the row of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathT3GetRow( VmathVector4 *result, const VmathTransform3 *tfrm, int row );

/*
 * Set the element of a 3x4 transformation matrix referred to by column and row indices
 */
static inline void vmathT3SetElem( VmathTransform3 *result, int col, int row, float val );

/*
 * Get the element of a 3x4 transformation matrix referred to by column and row indices
 */
static inline float vmathT3GetElem( const VmathTransform3 *tfrm, int col, int row );

/*
 * Multiply a 3x4 transformation matrix by a 3-D vector
 */
static inline void vmathT3MulV3( VmathVector3 *result, const VmathTransform3 *tfrm, const VmathVector3 *vec );

/*
 * Multiply a 3x4 transformation matrix by a 3-D point
 */
static inline void vmathT3MulP3( VmathPoint3 *result, const VmathTransform3 *tfrm, const VmathPoint3 *pnt );

/*
 * Multiply two 3x4 transformation matrices
 */
static inline void vmathT3Mul( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1 );

/*
 * Construct an identity 3x4 transformation matrix
 */
static inline void vmathT3MakeIdentity( VmathTransform3 *result );

/*
 * Construct a 3x4 transformation matrix to rotate around the x axis
 */
static inline void vmathT3MakeRotationX( VmathTransform3 *result, float radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the y axis
 */
static inline void vmathT3MakeRotationY( VmathTransform3 *result, float radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the z axis
 */
static inline void vmathT3MakeRotationZ( VmathTransform3 *result, float radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the x, y, and z axes
 */
static inline void vmathT3MakeRotationZYX( VmathTransform3 *result, const VmathVector3 *radiansXYZ );

/*
 * Construct a 3x4 transformation matrix to rotate around a unit-length 3-D vector
 */
static inline void vmathT3MakeRotationAxis( VmathTransform3 *result, float radians, const VmathVector3 *unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline void vmathT3MakeRotationQ( VmathTransform3 *result, const VmathQuat *unitQuat );

/*
 * Construct a 3x4 transformation matrix to perform scaling
 */
static inline void vmathT3MakeScale( VmathTransform3 *result, const VmathVector3 *scaleVec );

/*
 * Construct a 3x4 transformation matrix to perform translation
 */
static inline void vmathT3MakeTranslation( VmathTransform3 *result, const VmathVector3 *translateVec );

/*
 * Append (post-multiply) a scale transformation to a 3x4 transformation matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathT3AppendScale( VmathTransform3 *result, const VmathTransform3 *tfrm, const VmathVector3 *scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 3x4 transformation matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathT3PrependScale( VmathTransform3 *result, const VmathVector3 *scaleVec, const VmathTransform3 *tfrm );

/*
 * Multiply two 3x4 transformation matrices per element
 */
static inline void vmathT3MulPerElem( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1 );

/*
 * Compute the absolute value of a 3x4 transformation matrix per element
 */
static inline void vmathT3AbsPerElem( VmathTransform3 *result, const VmathTransform3 *tfrm );

/*
 * Inverse of a 3x4 transformation matrix
 * NOTE: 
 * Result is unpredictable when the determinant of the left 3x3 submatrix is equal to or near 0.
 */
static inline void vmathT3Inverse( VmathTransform3 *result, const VmathTransform3 *tfrm );

/*
 * Compute the inverse of a 3x4 transformation matrix, expected to have an orthogonal upper-left 3x3 submatrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 3x4 transformation matrix meets the given restrictions.
 */
static inline void vmathT3OrthoInverse( VmathTransform3 *result, const VmathTransform3 *tfrm );

/*
 * Conditionally select between two 3x4 transformation matrices
 */
static inline void vmathT3Select( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3x4 transformation matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathT3Print( const VmathTransform3 *tfrm );

/*
 * Print a 3x4 transformation matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathT3Prints( const VmathTransform3 *tfrm, const char *name );

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#include "vec_aos.h"
#include "quat_aos.h"
#include "mat_aos.h"

#endif
