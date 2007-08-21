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

#ifndef _VECTORMATH_AOS_C_V_SCALAR_H
#define _VECTORMATH_AOS_C_V_SCALAR_H

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
 * Construct a 3-D vector from x, y, and z elements
 */
static inline VmathVector3 vmathV3MakeFromElems_V( float x, float y, float z );

/*
 * Copy elements from a 3-D point into a 3-D vector
 */
static inline VmathVector3 vmathV3MakeFromP3_V( VmathPoint3 pnt );

/*
 * Set all elements of a 3-D vector to the same scalar value
 */
static inline VmathVector3 vmathV3MakeFromScalar_V( float scalar );

/*
 * Set the x element of a 3-D vector
 */
static inline void vmathV3SetX_V( VmathVector3 *result, float x );

/*
 * Set the y element of a 3-D vector
 */
static inline void vmathV3SetY_V( VmathVector3 *result, float y );

/*
 * Set the z element of a 3-D vector
 */
static inline void vmathV3SetZ_V( VmathVector3 *result, float z );

/*
 * Get the x element of a 3-D vector
 */
static inline float vmathV3GetX_V( VmathVector3 vec );

/*
 * Get the y element of a 3-D vector
 */
static inline float vmathV3GetY_V( VmathVector3 vec );

/*
 * Get the z element of a 3-D vector
 */
static inline float vmathV3GetZ_V( VmathVector3 vec );

/*
 * Set an x, y, or z element of a 3-D vector by index
 */
static inline void vmathV3SetElem_V( VmathVector3 *result, int idx, float value );

/*
 * Get an x, y, or z element of a 3-D vector by index
 */
static inline float vmathV3GetElem_V( VmathVector3 vec, int idx );

/*
 * Add two 3-D vectors
 */
static inline VmathVector3 vmathV3Add_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Subtract a 3-D vector from another 3-D vector
 */
static inline VmathVector3 vmathV3Sub_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Add a 3-D vector to a 3-D point
 */
static inline VmathPoint3 vmathV3AddP3_V( VmathVector3 vec, VmathPoint3 pnt );

/*
 * Multiply a 3-D vector by a scalar
 */
static inline VmathVector3 vmathV3ScalarMul_V( VmathVector3 vec, float scalar );

/*
 * Divide a 3-D vector by a scalar
 */
static inline VmathVector3 vmathV3ScalarDiv_V( VmathVector3 vec, float scalar );

/*
 * Negate all elements of a 3-D vector
 */
static inline VmathVector3 vmathV3Neg_V( VmathVector3 vec );

/*
 * Construct x axis
 */
static inline VmathVector3 vmathV3MakeXAxis_V( );

/*
 * Construct y axis
 */
static inline VmathVector3 vmathV3MakeYAxis_V( );

/*
 * Construct z axis
 */
static inline VmathVector3 vmathV3MakeZAxis_V( );

/*
 * Multiply two 3-D vectors per element
 */
static inline VmathVector3 vmathV3MulPerElem_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Divide two 3-D vectors per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline VmathVector3 vmathV3DivPerElem_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Compute the reciprocal of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline VmathVector3 vmathV3RecipPerElem_V( VmathVector3 vec );

/*
 * Compute the square root of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline VmathVector3 vmathV3SqrtPerElem_V( VmathVector3 vec );

/*
 * Compute the reciprocal square root of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline VmathVector3 vmathV3RsqrtPerElem_V( VmathVector3 vec );

/*
 * Compute the absolute value of a 3-D vector per element
 */
static inline VmathVector3 vmathV3AbsPerElem_V( VmathVector3 vec );

/*
 * Copy sign from one 3-D vector to another, per element
 */
static inline VmathVector3 vmathV3CopySignPerElem_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Maximum of two 3-D vectors per element
 */
static inline VmathVector3 vmathV3MaxPerElem_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Minimum of two 3-D vectors per element
 */
static inline VmathVector3 vmathV3MinPerElem_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Maximum element of a 3-D vector
 */
static inline float vmathV3MaxElem_V( VmathVector3 vec );

/*
 * Minimum element of a 3-D vector
 */
static inline float vmathV3MinElem_V( VmathVector3 vec );

/*
 * Compute the sum of all elements of a 3-D vector
 */
static inline float vmathV3Sum_V( VmathVector3 vec );

/*
 * Compute the dot product of two 3-D vectors
 */
static inline float vmathV3Dot_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Compute the square of the length of a 3-D vector
 */
static inline float vmathV3LengthSqr_V( VmathVector3 vec );

/*
 * Compute the length of a 3-D vector
 */
static inline float vmathV3Length_V( VmathVector3 vec );

/*
 * Normalize a 3-D vector
 * NOTE: 
 * The result is unpredictable when all elements of vec are at or near zero.
 */
static inline VmathVector3 vmathV3Normalize_V( VmathVector3 vec );

/*
 * Compute cross product of two 3-D vectors
 */
static inline VmathVector3 vmathV3Cross_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Outer product of two 3-D vectors
 */
static inline VmathMatrix3 vmathV3Outer_V( VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Pre-multiply a row vector by a 3x3 matrix
 */
static inline VmathVector3 vmathV3RowMul_V( VmathVector3 vec, VmathMatrix3 mat );

/*
 * Cross-product matrix of a 3-D vector
 */
static inline VmathMatrix3 vmathV3CrossMatrix_V( VmathVector3 vec );

/*
 * Create cross-product matrix and multiply
 * NOTE: 
 * Faster than separately creating a cross-product matrix and multiplying.
 */
static inline VmathMatrix3 vmathV3CrossMatrixMul_V( VmathVector3 vec, VmathMatrix3 mat );

/*
 * Linear interpolation between two 3-D vectors
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline VmathVector3 vmathV3Lerp_V( float t, VmathVector3 vec0, VmathVector3 vec1 );

/*
 * Spherical linear interpolation between two 3-D vectors
 * NOTE: 
 * The result is unpredictable if the vectors point in opposite directions.
 * Does not clamp t between 0 and 1.
 */
static inline VmathVector3 vmathV3Slerp_V( float t, VmathVector3 unitVec0, VmathVector3 unitVec1 );

/*
 * Conditionally select between two 3-D vectors
 */
static inline VmathVector3 vmathV3Select_V( VmathVector3 vec0, VmathVector3 vec1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3-D vector
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathV3Print_V( VmathVector3 vec );

/*
 * Print a 3-D vector and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathV3Prints_V( VmathVector3 vec, const char *name );

#endif

/*
 * Construct a 4-D vector from x, y, z, and w elements
 */
static inline VmathVector4 vmathV4MakeFromElems_V( float x, float y, float z, float w );

/*
 * Construct a 4-D vector from a 3-D vector and a scalar
 */
static inline VmathVector4 vmathV4MakeFromV3Scalar_V( VmathVector3 xyz, float w );

/*
 * Copy x, y, and z from a 3-D vector into a 4-D vector, and set w to 0
 */
static inline VmathVector4 vmathV4MakeFromV3_V( VmathVector3 vec );

/*
 * Copy x, y, and z from a 3-D point into a 4-D vector, and set w to 1
 */
static inline VmathVector4 vmathV4MakeFromP3_V( VmathPoint3 pnt );

/*
 * Copy elements from a quaternion into a 4-D vector
 */
static inline VmathVector4 vmathV4MakeFromQ_V( VmathQuat quat );

/*
 * Set all elements of a 4-D vector to the same scalar value
 */
static inline VmathVector4 vmathV4MakeFromScalar_V( float scalar );

/*
 * Set the x, y, and z elements of a 4-D vector
 * NOTE: 
 * This function does not change the w element.
 */
static inline void vmathV4SetXYZ_V( VmathVector4 *result, VmathVector3 vec );

/*
 * Get the x, y, and z elements of a 4-D vector
 */
static inline VmathVector3 vmathV4GetXYZ_V( VmathVector4 vec );

/*
 * Set the x element of a 4-D vector
 */
static inline void vmathV4SetX_V( VmathVector4 *result, float x );

/*
 * Set the y element of a 4-D vector
 */
static inline void vmathV4SetY_V( VmathVector4 *result, float y );

/*
 * Set the z element of a 4-D vector
 */
static inline void vmathV4SetZ_V( VmathVector4 *result, float z );

/*
 * Set the w element of a 4-D vector
 */
static inline void vmathV4SetW_V( VmathVector4 *result, float w );

/*
 * Get the x element of a 4-D vector
 */
static inline float vmathV4GetX_V( VmathVector4 vec );

/*
 * Get the y element of a 4-D vector
 */
static inline float vmathV4GetY_V( VmathVector4 vec );

/*
 * Get the z element of a 4-D vector
 */
static inline float vmathV4GetZ_V( VmathVector4 vec );

/*
 * Get the w element of a 4-D vector
 */
static inline float vmathV4GetW_V( VmathVector4 vec );

/*
 * Set an x, y, z, or w element of a 4-D vector by index
 */
static inline void vmathV4SetElem_V( VmathVector4 *result, int idx, float value );

/*
 * Get an x, y, z, or w element of a 4-D vector by index
 */
static inline float vmathV4GetElem_V( VmathVector4 vec, int idx );

/*
 * Add two 4-D vectors
 */
static inline VmathVector4 vmathV4Add_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Subtract a 4-D vector from another 4-D vector
 */
static inline VmathVector4 vmathV4Sub_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Multiply a 4-D vector by a scalar
 */
static inline VmathVector4 vmathV4ScalarMul_V( VmathVector4 vec, float scalar );

/*
 * Divide a 4-D vector by a scalar
 */
static inline VmathVector4 vmathV4ScalarDiv_V( VmathVector4 vec, float scalar );

/*
 * Negate all elements of a 4-D vector
 */
static inline VmathVector4 vmathV4Neg_V( VmathVector4 vec );

/*
 * Construct x axis
 */
static inline VmathVector4 vmathV4MakeXAxis_V( );

/*
 * Construct y axis
 */
static inline VmathVector4 vmathV4MakeYAxis_V( );

/*
 * Construct z axis
 */
static inline VmathVector4 vmathV4MakeZAxis_V( );

/*
 * Construct w axis
 */
static inline VmathVector4 vmathV4MakeWAxis_V( );

/*
 * Multiply two 4-D vectors per element
 */
static inline VmathVector4 vmathV4MulPerElem_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Divide two 4-D vectors per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline VmathVector4 vmathV4DivPerElem_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Compute the reciprocal of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline VmathVector4 vmathV4RecipPerElem_V( VmathVector4 vec );

/*
 * Compute the square root of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline VmathVector4 vmathV4SqrtPerElem_V( VmathVector4 vec );

/*
 * Compute the reciprocal square root of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline VmathVector4 vmathV4RsqrtPerElem_V( VmathVector4 vec );

/*
 * Compute the absolute value of a 4-D vector per element
 */
static inline VmathVector4 vmathV4AbsPerElem_V( VmathVector4 vec );

/*
 * Copy sign from one 4-D vector to another, per element
 */
static inline VmathVector4 vmathV4CopySignPerElem_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Maximum of two 4-D vectors per element
 */
static inline VmathVector4 vmathV4MaxPerElem_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Minimum of two 4-D vectors per element
 */
static inline VmathVector4 vmathV4MinPerElem_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Maximum element of a 4-D vector
 */
static inline float vmathV4MaxElem_V( VmathVector4 vec );

/*
 * Minimum element of a 4-D vector
 */
static inline float vmathV4MinElem_V( VmathVector4 vec );

/*
 * Compute the sum of all elements of a 4-D vector
 */
static inline float vmathV4Sum_V( VmathVector4 vec );

/*
 * Compute the dot product of two 4-D vectors
 */
static inline float vmathV4Dot_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Compute the square of the length of a 4-D vector
 */
static inline float vmathV4LengthSqr_V( VmathVector4 vec );

/*
 * Compute the length of a 4-D vector
 */
static inline float vmathV4Length_V( VmathVector4 vec );

/*
 * Normalize a 4-D vector
 * NOTE: 
 * The result is unpredictable when all elements of vec are at or near zero.
 */
static inline VmathVector4 vmathV4Normalize_V( VmathVector4 vec );

/*
 * Outer product of two 4-D vectors
 */
static inline VmathMatrix4 vmathV4Outer_V( VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Linear interpolation between two 4-D vectors
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline VmathVector4 vmathV4Lerp_V( float t, VmathVector4 vec0, VmathVector4 vec1 );

/*
 * Spherical linear interpolation between two 4-D vectors
 * NOTE: 
 * The result is unpredictable if the vectors point in opposite directions.
 * Does not clamp t between 0 and 1.
 */
static inline VmathVector4 vmathV4Slerp_V( float t, VmathVector4 unitVec0, VmathVector4 unitVec1 );

/*
 * Conditionally select between two 4-D vectors
 */
static inline VmathVector4 vmathV4Select_V( VmathVector4 vec0, VmathVector4 vec1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 4-D vector
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathV4Print_V( VmathVector4 vec );

/*
 * Print a 4-D vector and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathV4Prints_V( VmathVector4 vec, const char *name );

#endif

/*
 * Construct a 3-D point from x, y, and z elements
 */
static inline VmathPoint3 vmathP3MakeFromElems_V( float x, float y, float z );

/*
 * Copy elements from a 3-D vector into a 3-D point
 */
static inline VmathPoint3 vmathP3MakeFromV3_V( VmathVector3 vec );

/*
 * Set all elements of a 3-D point to the same scalar value
 */
static inline VmathPoint3 vmathP3MakeFromScalar_V( float scalar );

/*
 * Set the x element of a 3-D point
 */
static inline void vmathP3SetX_V( VmathPoint3 *result, float x );

/*
 * Set the y element of a 3-D point
 */
static inline void vmathP3SetY_V( VmathPoint3 *result, float y );

/*
 * Set the z element of a 3-D point
 */
static inline void vmathP3SetZ_V( VmathPoint3 *result, float z );

/*
 * Get the x element of a 3-D point
 */
static inline float vmathP3GetX_V( VmathPoint3 pnt );

/*
 * Get the y element of a 3-D point
 */
static inline float vmathP3GetY_V( VmathPoint3 pnt );

/*
 * Get the z element of a 3-D point
 */
static inline float vmathP3GetZ_V( VmathPoint3 pnt );

/*
 * Set an x, y, or z element of a 3-D point by index
 */
static inline void vmathP3SetElem_V( VmathPoint3 *result, int idx, float value );

/*
 * Get an x, y, or z element of a 3-D point by index
 */
static inline float vmathP3GetElem_V( VmathPoint3 pnt, int idx );

/*
 * Subtract a 3-D point from another 3-D point
 */
static inline VmathVector3 vmathP3Sub_V( VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Add a 3-D point to a 3-D vector
 */
static inline VmathPoint3 vmathP3AddV3_V( VmathPoint3 pnt, VmathVector3 vec );

/*
 * Subtract a 3-D vector from a 3-D point
 */
static inline VmathPoint3 vmathP3SubV3_V( VmathPoint3 pnt, VmathVector3 vec );

/*
 * Multiply two 3-D points per element
 */
static inline VmathPoint3 vmathP3MulPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Divide two 3-D points per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline VmathPoint3 vmathP3DivPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Compute the reciprocal of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline VmathPoint3 vmathP3RecipPerElem_V( VmathPoint3 pnt );

/*
 * Compute the square root of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline VmathPoint3 vmathP3SqrtPerElem_V( VmathPoint3 pnt );

/*
 * Compute the reciprocal square root of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline VmathPoint3 vmathP3RsqrtPerElem_V( VmathPoint3 pnt );

/*
 * Compute the absolute value of a 3-D point per element
 */
static inline VmathPoint3 vmathP3AbsPerElem_V( VmathPoint3 pnt );

/*
 * Copy sign from one 3-D point to another, per element
 */
static inline VmathPoint3 vmathP3CopySignPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Maximum of two 3-D points per element
 */
static inline VmathPoint3 vmathP3MaxPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Minimum of two 3-D points per element
 */
static inline VmathPoint3 vmathP3MinPerElem_V( VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Maximum element of a 3-D point
 */
static inline float vmathP3MaxElem_V( VmathPoint3 pnt );

/*
 * Minimum element of a 3-D point
 */
static inline float vmathP3MinElem_V( VmathPoint3 pnt );

/*
 * Compute the sum of all elements of a 3-D point
 */
static inline float vmathP3Sum_V( VmathPoint3 pnt );

/*
 * Apply uniform scale to a 3-D point
 */
static inline VmathPoint3 vmathP3Scale_V( VmathPoint3 pnt, float scaleVal );

/*
 * Apply non-uniform scale to a 3-D point
 */
static inline VmathPoint3 vmathP3NonUniformScale_V( VmathPoint3 pnt, VmathVector3 scaleVec );

/*
 * Scalar projection of a 3-D point on a unit-length 3-D vector
 */
static inline float vmathP3Projection_V( VmathPoint3 pnt, VmathVector3 unitVec );

/*
 * Compute the square of the distance of a 3-D point from the coordinate-system origin
 */
static inline float vmathP3DistSqrFromOrigin_V( VmathPoint3 pnt );

/*
 * Compute the distance of a 3-D point from the coordinate-system origin
 */
static inline float vmathP3DistFromOrigin_V( VmathPoint3 pnt );

/*
 * Compute the square of the distance between two 3-D points
 */
static inline float vmathP3DistSqr_V( VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Compute the distance between two 3-D points
 */
static inline float vmathP3Dist_V( VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Linear interpolation between two 3-D points
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline VmathPoint3 vmathP3Lerp_V( float t, VmathPoint3 pnt0, VmathPoint3 pnt1 );

/*
 * Conditionally select between two 3-D points
 */
static inline VmathPoint3 vmathP3Select_V( VmathPoint3 pnt0, VmathPoint3 pnt1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3-D point
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathP3Print_V( VmathPoint3 pnt );

/*
 * Print a 3-D point and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathP3Prints_V( VmathPoint3 pnt, const char *name );

#endif

/*
 * Construct a quaternion from x, y, z, and w elements
 */
static inline VmathQuat vmathQMakeFromElems_V( float x, float y, float z, float w );

/*
 * Construct a quaternion from a 3-D vector and a scalar
 */
static inline VmathQuat vmathQMakeFromV3Scalar_V( VmathVector3 xyz, float w );

/*
 * Copy elements from a 4-D vector into a quaternion
 */
static inline VmathQuat vmathQMakeFromV4_V( VmathVector4 vec );

/*
 * Convert a rotation matrix to a unit-length quaternion
 */
static inline VmathQuat vmathQMakeFromM3_V( VmathMatrix3 rotMat );

/*
 * Set all elements of a quaternion to the same scalar value
 */
static inline VmathQuat vmathQMakeFromScalar_V( float scalar );

/*
 * Set the x, y, and z elements of a quaternion
 * NOTE: 
 * This function does not change the w element.
 */
static inline void vmathQSetXYZ_V( VmathQuat *result, VmathVector3 vec );

/*
 * Get the x, y, and z elements of a quaternion
 */
static inline VmathVector3 vmathQGetXYZ_V( VmathQuat quat );

/*
 * Set the x element of a quaternion
 */
static inline void vmathQSetX_V( VmathQuat *result, float x );

/*
 * Set the y element of a quaternion
 */
static inline void vmathQSetY_V( VmathQuat *result, float y );

/*
 * Set the z element of a quaternion
 */
static inline void vmathQSetZ_V( VmathQuat *result, float z );

/*
 * Set the w element of a quaternion
 */
static inline void vmathQSetW_V( VmathQuat *result, float w );

/*
 * Get the x element of a quaternion
 */
static inline float vmathQGetX_V( VmathQuat quat );

/*
 * Get the y element of a quaternion
 */
static inline float vmathQGetY_V( VmathQuat quat );

/*
 * Get the z element of a quaternion
 */
static inline float vmathQGetZ_V( VmathQuat quat );

/*
 * Get the w element of a quaternion
 */
static inline float vmathQGetW_V( VmathQuat quat );

/*
 * Set an x, y, z, or w element of a quaternion by index
 */
static inline void vmathQSetElem_V( VmathQuat *result, int idx, float value );

/*
 * Get an x, y, z, or w element of a quaternion by index
 */
static inline float vmathQGetElem_V( VmathQuat quat, int idx );

/*
 * Add two quaternions
 */
static inline VmathQuat vmathQAdd_V( VmathQuat quat0, VmathQuat quat1 );

/*
 * Subtract a quaternion from another quaternion
 */
static inline VmathQuat vmathQSub_V( VmathQuat quat0, VmathQuat quat1 );

/*
 * Multiply two quaternions
 */
static inline VmathQuat vmathQMul_V( VmathQuat quat0, VmathQuat quat1 );

/*
 * Multiply a quaternion by a scalar
 */
static inline VmathQuat vmathQScalarMul_V( VmathQuat quat, float scalar );

/*
 * Divide a quaternion by a scalar
 */
static inline VmathQuat vmathQScalarDiv_V( VmathQuat quat, float scalar );

/*
 * Negate all elements of a quaternion
 */
static inline VmathQuat vmathQNeg_V( VmathQuat quat );

/*
 * Construct an identity quaternion
 */
static inline VmathQuat vmathQMakeIdentity_V( );

/*
 * Construct a quaternion to rotate between two unit-length 3-D vectors
 * NOTE: 
 * The result is unpredictable if unitVec0 and unitVec1 point in opposite directions.
 */
static inline VmathQuat vmathQMakeRotationArc_V( VmathVector3 unitVec0, VmathVector3 unitVec1 );

/*
 * Construct a quaternion to rotate around a unit-length 3-D vector
 */
static inline VmathQuat vmathQMakeRotationAxis_V( float radians, VmathVector3 unitVec );

/*
 * Construct a quaternion to rotate around the x axis
 */
static inline VmathQuat vmathQMakeRotationX_V( float radians );

/*
 * Construct a quaternion to rotate around the y axis
 */
static inline VmathQuat vmathQMakeRotationY_V( float radians );

/*
 * Construct a quaternion to rotate around the z axis
 */
static inline VmathQuat vmathQMakeRotationZ_V( float radians );

/*
 * Compute the conjugate of a quaternion
 */
static inline VmathQuat vmathQConj_V( VmathQuat quat );

/*
 * Use a unit-length quaternion to rotate a 3-D vector
 */
static inline VmathVector3 vmathQRotate_V( VmathQuat unitQuat, VmathVector3 vec );

/*
 * Compute the dot product of two quaternions
 */
static inline float vmathQDot_V( VmathQuat quat0, VmathQuat quat1 );

/*
 * Compute the norm of a quaternion
 */
static inline float vmathQNorm_V( VmathQuat quat );

/*
 * Compute the length of a quaternion
 */
static inline float vmathQLength_V( VmathQuat quat );

/*
 * Normalize a quaternion
 * NOTE: 
 * The result is unpredictable when all elements of quat are at or near zero.
 */
static inline VmathQuat vmathQNormalize_V( VmathQuat quat );

/*
 * Linear interpolation between two quaternions
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline VmathQuat vmathQLerp_V( float t, VmathQuat quat0, VmathQuat quat1 );

/*
 * Spherical linear interpolation between two quaternions
 * NOTE: 
 * Interpolates along the shortest path between orientations.
 * Does not clamp t between 0 and 1.
 */
static inline VmathQuat vmathQSlerp_V( float t, VmathQuat unitQuat0, VmathQuat unitQuat1 );

/*
 * Spherical quadrangle interpolation
 */
static inline VmathQuat vmathQSquad_V( float t, VmathQuat unitQuat0, VmathQuat unitQuat1, VmathQuat unitQuat2, VmathQuat unitQuat3 );

/*
 * Conditionally select between two quaternions
 */
static inline VmathQuat vmathQSelect_V( VmathQuat quat0, VmathQuat quat1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a quaternion
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathQPrint_V( VmathQuat quat );

/*
 * Print a quaternion and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathQPrints_V( VmathQuat quat, const char *name );

#endif

/*
 * Construct a 3x3 matrix containing the specified columns
 */
static inline VmathMatrix3 vmathM3MakeFromCols_V( VmathVector3 col0, VmathVector3 col1, VmathVector3 col2 );

/*
 * Construct a 3x3 rotation matrix from a unit-length quaternion
 */
static inline VmathMatrix3 vmathM3MakeFromQ_V( VmathQuat unitQuat );

/*
 * Set all elements of a 3x3 matrix to the same scalar value
 */
static inline VmathMatrix3 vmathM3MakeFromScalar_V( float scalar );

/*
 * Set column 0 of a 3x3 matrix
 */
static inline void vmathM3SetCol0_V( VmathMatrix3 *result, VmathVector3 col0 );

/*
 * Set column 1 of a 3x3 matrix
 */
static inline void vmathM3SetCol1_V( VmathMatrix3 *result, VmathVector3 col1 );

/*
 * Set column 2 of a 3x3 matrix
 */
static inline void vmathM3SetCol2_V( VmathMatrix3 *result, VmathVector3 col2 );

/*
 * Get column 0 of a 3x3 matrix
 */
static inline VmathVector3 vmathM3GetCol0_V( VmathMatrix3 mat );

/*
 * Get column 1 of a 3x3 matrix
 */
static inline VmathVector3 vmathM3GetCol1_V( VmathMatrix3 mat );

/*
 * Get column 2 of a 3x3 matrix
 */
static inline VmathVector3 vmathM3GetCol2_V( VmathMatrix3 mat );

/*
 * Set the column of a 3x3 matrix referred to by the specified index
 */
static inline void vmathM3SetCol_V( VmathMatrix3 *result, int col, VmathVector3 vec );

/*
 * Set the row of a 3x3 matrix referred to by the specified index
 */
static inline void vmathM3SetRow_V( VmathMatrix3 *result, int row, VmathVector3 vec );

/*
 * Get the column of a 3x3 matrix referred to by the specified index
 */
static inline VmathVector3 vmathM3GetCol_V( VmathMatrix3 mat, int col );

/*
 * Get the row of a 3x3 matrix referred to by the specified index
 */
static inline VmathVector3 vmathM3GetRow_V( VmathMatrix3 mat, int row );

/*
 * Set the element of a 3x3 matrix referred to by column and row indices
 */
static inline void vmathM3SetElem_V( VmathMatrix3 *result, int col, int row, float val );

/*
 * Get the element of a 3x3 matrix referred to by column and row indices
 */
static inline float vmathM3GetElem_V( VmathMatrix3 mat, int col, int row );

/*
 * Add two 3x3 matrices
 */
static inline VmathMatrix3 vmathM3Add_V( VmathMatrix3 mat0, VmathMatrix3 mat1 );

/*
 * Subtract a 3x3 matrix from another 3x3 matrix
 */
static inline VmathMatrix3 vmathM3Sub_V( VmathMatrix3 mat0, VmathMatrix3 mat1 );

/*
 * Negate all elements of a 3x3 matrix
 */
static inline VmathMatrix3 vmathM3Neg_V( VmathMatrix3 mat );

/*
 * Multiply a 3x3 matrix by a scalar
 */
static inline VmathMatrix3 vmathM3ScalarMul_V( VmathMatrix3 mat, float scalar );

/*
 * Multiply a 3x3 matrix by a 3-D vector
 */
static inline VmathVector3 vmathM3MulV3_V( VmathMatrix3 mat, VmathVector3 vec );

/*
 * Multiply two 3x3 matrices
 */
static inline VmathMatrix3 vmathM3Mul_V( VmathMatrix3 mat0, VmathMatrix3 mat1 );

/*
 * Construct an identity 3x3 matrix
 */
static inline VmathMatrix3 vmathM3MakeIdentity_V( );

/*
 * Construct a 3x3 matrix to rotate around the x axis
 */
static inline VmathMatrix3 vmathM3MakeRotationX_V( float radians );

/*
 * Construct a 3x3 matrix to rotate around the y axis
 */
static inline VmathMatrix3 vmathM3MakeRotationY_V( float radians );

/*
 * Construct a 3x3 matrix to rotate around the z axis
 */
static inline VmathMatrix3 vmathM3MakeRotationZ_V( float radians );

/*
 * Construct a 3x3 matrix to rotate around the x, y, and z axes
 */
static inline VmathMatrix3 vmathM3MakeRotationZYX_V( VmathVector3 radiansXYZ );

/*
 * Construct a 3x3 matrix to rotate around a unit-length 3-D vector
 */
static inline VmathMatrix3 vmathM3MakeRotationAxis_V( float radians, VmathVector3 unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline VmathMatrix3 vmathM3MakeRotationQ_V( VmathQuat unitQuat );

/*
 * Construct a 3x3 matrix to perform scaling
 */
static inline VmathMatrix3 vmathM3MakeScale_V( VmathVector3 scaleVec );

/*
 * Append (post-multiply) a scale transformation to a 3x3 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathMatrix3 vmathM3AppendScale_V( VmathMatrix3 mat, VmathVector3 scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 3x3 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathMatrix3 vmathM3PrependScale_V( VmathVector3 scaleVec, VmathMatrix3 mat );

/*
 * Multiply two 3x3 matrices per element
 */
static inline VmathMatrix3 vmathM3MulPerElem_V( VmathMatrix3 mat0, VmathMatrix3 mat1 );

/*
 * Compute the absolute value of a 3x3 matrix per element
 */
static inline VmathMatrix3 vmathM3AbsPerElem_V( VmathMatrix3 mat );

/*
 * Transpose of a 3x3 matrix
 */
static inline VmathMatrix3 vmathM3Transpose_V( VmathMatrix3 mat );

/*
 * Compute the inverse of a 3x3 matrix
 * NOTE: 
 * Result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline VmathMatrix3 vmathM3Inverse_V( VmathMatrix3 mat );

/*
 * Determinant of a 3x3 matrix
 */
static inline float vmathM3Determinant_V( VmathMatrix3 mat );

/*
 * Conditionally select between two 3x3 matrices
 */
static inline VmathMatrix3 vmathM3Select_V( VmathMatrix3 mat0, VmathMatrix3 mat1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3x3 matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathM3Print_V( VmathMatrix3 mat );

/*
 * Print a 3x3 matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathM3Prints_V( VmathMatrix3 mat, const char *name );

#endif

/*
 * Construct a 4x4 matrix containing the specified columns
 */
static inline VmathMatrix4 vmathM4MakeFromCols_V( VmathVector4 col0, VmathVector4 col1, VmathVector4 col2, VmathVector4 col3 );

/*
 * Construct a 4x4 matrix from a 3x4 transformation matrix
 */
static inline VmathMatrix4 vmathM4MakeFromT3_V( VmathTransform3 mat );

/*
 * Construct a 4x4 matrix from a 3x3 matrix and a 3-D vector
 */
static inline VmathMatrix4 vmathM4MakeFromM3V3_V( VmathMatrix3 mat, VmathVector3 translateVec );

/*
 * Construct a 4x4 matrix from a unit-length quaternion and a 3-D vector
 */
static inline VmathMatrix4 vmathM4MakeFromQV3_V( VmathQuat unitQuat, VmathVector3 translateVec );

/*
 * Set all elements of a 4x4 matrix to the same scalar value
 */
static inline VmathMatrix4 vmathM4MakeFromScalar_V( float scalar );

/*
 * Set the upper-left 3x3 submatrix
 * NOTE: 
 * This function does not change the bottom row elements.
 */
static inline void vmathM4SetUpper3x3_V( VmathMatrix4 *result, VmathMatrix3 mat3 );

/*
 * Get the upper-left 3x3 submatrix of a 4x4 matrix
 */
static inline VmathMatrix3 vmathM4GetUpper3x3_V( VmathMatrix4 mat );

/*
 * Set translation component
 * NOTE: 
 * This function does not change the bottom row elements.
 */
static inline void vmathM4SetTranslation_V( VmathMatrix4 *result, VmathVector3 translateVec );

/*
 * Get the translation component of a 4x4 matrix
 */
static inline VmathVector3 vmathM4GetTranslation_V( VmathMatrix4 mat );

/*
 * Set column 0 of a 4x4 matrix
 */
static inline void vmathM4SetCol0_V( VmathMatrix4 *result, VmathVector4 col0 );

/*
 * Set column 1 of a 4x4 matrix
 */
static inline void vmathM4SetCol1_V( VmathMatrix4 *result, VmathVector4 col1 );

/*
 * Set column 2 of a 4x4 matrix
 */
static inline void vmathM4SetCol2_V( VmathMatrix4 *result, VmathVector4 col2 );

/*
 * Set column 3 of a 4x4 matrix
 */
static inline void vmathM4SetCol3_V( VmathMatrix4 *result, VmathVector4 col3 );

/*
 * Get column 0 of a 4x4 matrix
 */
static inline VmathVector4 vmathM4GetCol0_V( VmathMatrix4 mat );

/*
 * Get column 1 of a 4x4 matrix
 */
static inline VmathVector4 vmathM4GetCol1_V( VmathMatrix4 mat );

/*
 * Get column 2 of a 4x4 matrix
 */
static inline VmathVector4 vmathM4GetCol2_V( VmathMatrix4 mat );

/*
 * Get column 3 of a 4x4 matrix
 */
static inline VmathVector4 vmathM4GetCol3_V( VmathMatrix4 mat );

/*
 * Set the column of a 4x4 matrix referred to by the specified index
 */
static inline void vmathM4SetCol_V( VmathMatrix4 *result, int col, VmathVector4 vec );

/*
 * Set the row of a 4x4 matrix referred to by the specified index
 */
static inline void vmathM4SetRow_V( VmathMatrix4 *result, int row, VmathVector4 vec );

/*
 * Get the column of a 4x4 matrix referred to by the specified index
 */
static inline VmathVector4 vmathM4GetCol_V( VmathMatrix4 mat, int col );

/*
 * Get the row of a 4x4 matrix referred to by the specified index
 */
static inline VmathVector4 vmathM4GetRow_V( VmathMatrix4 mat, int row );

/*
 * Set the element of a 4x4 matrix referred to by column and row indices
 */
static inline void vmathM4SetElem_V( VmathMatrix4 *result, int col, int row, float val );

/*
 * Get the element of a 4x4 matrix referred to by column and row indices
 */
static inline float vmathM4GetElem_V( VmathMatrix4 mat, int col, int row );

/*
 * Add two 4x4 matrices
 */
static inline VmathMatrix4 vmathM4Add_V( VmathMatrix4 mat0, VmathMatrix4 mat1 );

/*
 * Subtract a 4x4 matrix from another 4x4 matrix
 */
static inline VmathMatrix4 vmathM4Sub_V( VmathMatrix4 mat0, VmathMatrix4 mat1 );

/*
 * Negate all elements of a 4x4 matrix
 */
static inline VmathMatrix4 vmathM4Neg_V( VmathMatrix4 mat );

/*
 * Multiply a 4x4 matrix by a scalar
 */
static inline VmathMatrix4 vmathM4ScalarMul_V( VmathMatrix4 mat, float scalar );

/*
 * Multiply a 4x4 matrix by a 4-D vector
 */
static inline VmathVector4 vmathM4MulV4_V( VmathMatrix4 mat, VmathVector4 vec );

/*
 * Multiply a 4x4 matrix by a 3-D vector
 */
static inline VmathVector4 vmathM4MulV3_V( VmathMatrix4 mat, VmathVector3 vec );

/*
 * Multiply a 4x4 matrix by a 3-D point
 */
static inline VmathVector4 vmathM4MulP3_V( VmathMatrix4 mat, VmathPoint3 pnt );

/*
 * Multiply two 4x4 matrices
 */
static inline VmathMatrix4 vmathM4Mul_V( VmathMatrix4 mat0, VmathMatrix4 mat1 );

/*
 * Multiply a 4x4 matrix by a 3x4 transformation matrix
 */
static inline VmathMatrix4 vmathM4MulT3_V( VmathMatrix4 mat, VmathTransform3 tfrm );

/*
 * Construct an identity 4x4 matrix
 */
static inline VmathMatrix4 vmathM4MakeIdentity_V( );

/*
 * Construct a 4x4 matrix to rotate around the x axis
 */
static inline VmathMatrix4 vmathM4MakeRotationX_V( float radians );

/*
 * Construct a 4x4 matrix to rotate around the y axis
 */
static inline VmathMatrix4 vmathM4MakeRotationY_V( float radians );

/*
 * Construct a 4x4 matrix to rotate around the z axis
 */
static inline VmathMatrix4 vmathM4MakeRotationZ_V( float radians );

/*
 * Construct a 4x4 matrix to rotate around the x, y, and z axes
 */
static inline VmathMatrix4 vmathM4MakeRotationZYX_V( VmathVector3 radiansXYZ );

/*
 * Construct a 4x4 matrix to rotate around a unit-length 3-D vector
 */
static inline VmathMatrix4 vmathM4MakeRotationAxis_V( float radians, VmathVector3 unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline VmathMatrix4 vmathM4MakeRotationQ_V( VmathQuat unitQuat );

/*
 * Construct a 4x4 matrix to perform scaling
 */
static inline VmathMatrix4 vmathM4MakeScale_V( VmathVector3 scaleVec );

/*
 * Construct a 4x4 matrix to perform translation
 */
static inline VmathMatrix4 vmathM4MakeTranslation_V( VmathVector3 translateVec );

/*
 * Construct viewing matrix based on eye position, position looked at, and up direction
 */
static inline VmathMatrix4 vmathM4MakeLookAt_V( VmathPoint3 eyePos, VmathPoint3 lookAtPos, VmathVector3 upVec );

/*
 * Construct a perspective projection matrix
 */
static inline VmathMatrix4 vmathM4MakePerspective_V( float fovyRadians, float aspect, float zNear, float zFar );

/*
 * Construct a perspective projection matrix based on frustum
 */
static inline VmathMatrix4 vmathM4MakeFrustum_V( float left, float right, float bottom, float top, float zNear, float zFar );

/*
 * Construct an orthographic projection matrix
 */
static inline VmathMatrix4 vmathM4MakeOrthographic_V( float left, float right, float bottom, float top, float zNear, float zFar );

/*
 * Append (post-multiply) a scale transformation to a 4x4 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathMatrix4 vmathM4AppendScale_V( VmathMatrix4 mat, VmathVector3 scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 4x4 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathMatrix4 vmathM4PrependScale_V( VmathVector3 scaleVec, VmathMatrix4 mat );

/*
 * Multiply two 4x4 matrices per element
 */
static inline VmathMatrix4 vmathM4MulPerElem_V( VmathMatrix4 mat0, VmathMatrix4 mat1 );

/*
 * Compute the absolute value of a 4x4 matrix per element
 */
static inline VmathMatrix4 vmathM4AbsPerElem_V( VmathMatrix4 mat );

/*
 * Transpose of a 4x4 matrix
 */
static inline VmathMatrix4 vmathM4Transpose_V( VmathMatrix4 mat );

/*
 * Compute the inverse of a 4x4 matrix
 * NOTE: 
 * Result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline VmathMatrix4 vmathM4Inverse_V( VmathMatrix4 mat );

/*
 * Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.  The result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline VmathMatrix4 vmathM4AffineInverse_V( VmathMatrix4 mat );

/*
 * Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix with an orthogonal upper-left 3x3 submatrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.
 */
static inline VmathMatrix4 vmathM4OrthoInverse_V( VmathMatrix4 mat );

/*
 * Determinant of a 4x4 matrix
 */
static inline float vmathM4Determinant_V( VmathMatrix4 mat );

/*
 * Conditionally select between two 4x4 matrices
 */
static inline VmathMatrix4 vmathM4Select_V( VmathMatrix4 mat0, VmathMatrix4 mat1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 4x4 matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathM4Print_V( VmathMatrix4 mat );

/*
 * Print a 4x4 matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathM4Prints_V( VmathMatrix4 mat, const char *name );

#endif

/*
 * Construct a 3x4 transformation matrix containing the specified columns
 */
static inline VmathTransform3 vmathT3MakeFromCols_V( VmathVector3 col0, VmathVector3 col1, VmathVector3 col2, VmathVector3 col3 );

/*
 * Construct a 3x4 transformation matrix from a 3x3 matrix and a 3-D vector
 */
static inline VmathTransform3 vmathT3MakeFromM3V3_V( VmathMatrix3 tfrm, VmathVector3 translateVec );

/*
 * Construct a 3x4 transformation matrix from a unit-length quaternion and a 3-D vector
 */
static inline VmathTransform3 vmathT3MakeFromQV3_V( VmathQuat unitQuat, VmathVector3 translateVec );

/*
 * Set all elements of a 3x4 transformation matrix to the same scalar value
 */
static inline VmathTransform3 vmathT3MakeFromScalar_V( float scalar );

/*
 * Set the upper-left 3x3 submatrix
 */
static inline void vmathT3SetUpper3x3_V( VmathTransform3 *result, VmathMatrix3 mat3 );

/*
 * Get the upper-left 3x3 submatrix of a 3x4 transformation matrix
 */
static inline VmathMatrix3 vmathT3GetUpper3x3_V( VmathTransform3 tfrm );

/*
 * Set translation component
 */
static inline void vmathT3SetTranslation_V( VmathTransform3 *result, VmathVector3 translateVec );

/*
 * Get the translation component of a 3x4 transformation matrix
 */
static inline VmathVector3 vmathT3GetTranslation_V( VmathTransform3 tfrm );

/*
 * Set column 0 of a 3x4 transformation matrix
 */
static inline void vmathT3SetCol0_V( VmathTransform3 *result, VmathVector3 col0 );

/*
 * Set column 1 of a 3x4 transformation matrix
 */
static inline void vmathT3SetCol1_V( VmathTransform3 *result, VmathVector3 col1 );

/*
 * Set column 2 of a 3x4 transformation matrix
 */
static inline void vmathT3SetCol2_V( VmathTransform3 *result, VmathVector3 col2 );

/*
 * Set column 3 of a 3x4 transformation matrix
 */
static inline void vmathT3SetCol3_V( VmathTransform3 *result, VmathVector3 col3 );

/*
 * Get column 0 of a 3x4 transformation matrix
 */
static inline VmathVector3 vmathT3GetCol0_V( VmathTransform3 tfrm );

/*
 * Get column 1 of a 3x4 transformation matrix
 */
static inline VmathVector3 vmathT3GetCol1_V( VmathTransform3 tfrm );

/*
 * Get column 2 of a 3x4 transformation matrix
 */
static inline VmathVector3 vmathT3GetCol2_V( VmathTransform3 tfrm );

/*
 * Get column 3 of a 3x4 transformation matrix
 */
static inline VmathVector3 vmathT3GetCol3_V( VmathTransform3 tfrm );

/*
 * Set the column of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathT3SetCol_V( VmathTransform3 *result, int col, VmathVector3 vec );

/*
 * Set the row of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathT3SetRow_V( VmathTransform3 *result, int row, VmathVector4 vec );

/*
 * Get the column of a 3x4 transformation matrix referred to by the specified index
 */
static inline VmathVector3 vmathT3GetCol_V( VmathTransform3 tfrm, int col );

/*
 * Get the row of a 3x4 transformation matrix referred to by the specified index
 */
static inline VmathVector4 vmathT3GetRow_V( VmathTransform3 tfrm, int row );

/*
 * Set the element of a 3x4 transformation matrix referred to by column and row indices
 */
static inline void vmathT3SetElem_V( VmathTransform3 *result, int col, int row, float val );

/*
 * Get the element of a 3x4 transformation matrix referred to by column and row indices
 */
static inline float vmathT3GetElem_V( VmathTransform3 tfrm, int col, int row );

/*
 * Multiply a 3x4 transformation matrix by a 3-D vector
 */
static inline VmathVector3 vmathT3MulV3_V( VmathTransform3 tfrm, VmathVector3 vec );

/*
 * Multiply a 3x4 transformation matrix by a 3-D point
 */
static inline VmathPoint3 vmathT3MulP3_V( VmathTransform3 tfrm, VmathPoint3 pnt );

/*
 * Multiply two 3x4 transformation matrices
 */
static inline VmathTransform3 vmathT3Mul_V( VmathTransform3 tfrm0, VmathTransform3 tfrm1 );

/*
 * Construct an identity 3x4 transformation matrix
 */
static inline VmathTransform3 vmathT3MakeIdentity_V( );

/*
 * Construct a 3x4 transformation matrix to rotate around the x axis
 */
static inline VmathTransform3 vmathT3MakeRotationX_V( float radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the y axis
 */
static inline VmathTransform3 vmathT3MakeRotationY_V( float radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the z axis
 */
static inline VmathTransform3 vmathT3MakeRotationZ_V( float radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the x, y, and z axes
 */
static inline VmathTransform3 vmathT3MakeRotationZYX_V( VmathVector3 radiansXYZ );

/*
 * Construct a 3x4 transformation matrix to rotate around a unit-length 3-D vector
 */
static inline VmathTransform3 vmathT3MakeRotationAxis_V( float radians, VmathVector3 unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline VmathTransform3 vmathT3MakeRotationQ_V( VmathQuat unitQuat );

/*
 * Construct a 3x4 transformation matrix to perform scaling
 */
static inline VmathTransform3 vmathT3MakeScale_V( VmathVector3 scaleVec );

/*
 * Construct a 3x4 transformation matrix to perform translation
 */
static inline VmathTransform3 vmathT3MakeTranslation_V( VmathVector3 translateVec );

/*
 * Append (post-multiply) a scale transformation to a 3x4 transformation matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathTransform3 vmathT3AppendScale_V( VmathTransform3 tfrm, VmathVector3 scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 3x4 transformation matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathTransform3 vmathT3PrependScale_V( VmathVector3 scaleVec, VmathTransform3 tfrm );

/*
 * Multiply two 3x4 transformation matrices per element
 */
static inline VmathTransform3 vmathT3MulPerElem_V( VmathTransform3 tfrm0, VmathTransform3 tfrm1 );

/*
 * Compute the absolute value of a 3x4 transformation matrix per element
 */
static inline VmathTransform3 vmathT3AbsPerElem_V( VmathTransform3 tfrm );

/*
 * Inverse of a 3x4 transformation matrix
 * NOTE: 
 * Result is unpredictable when the determinant of the left 3x3 submatrix is equal to or near 0.
 */
static inline VmathTransform3 vmathT3Inverse_V( VmathTransform3 tfrm );

/*
 * Compute the inverse of a 3x4 transformation matrix, expected to have an orthogonal upper-left 3x3 submatrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 3x4 transformation matrix meets the given restrictions.
 */
static inline VmathTransform3 vmathT3OrthoInverse_V( VmathTransform3 tfrm );

/*
 * Conditionally select between two 3x4 transformation matrices
 */
static inline VmathTransform3 vmathT3Select_V( VmathTransform3 tfrm0, VmathTransform3 tfrm1, unsigned int select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3x4 transformation matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathT3Print_V( VmathTransform3 tfrm );

/*
 * Print a 3x4 transformation matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathT3Prints_V( VmathTransform3 tfrm, const char *name );

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#include "vectormath_aos.h"
#include "vec_aos_v.h"
#include "quat_aos_v.h"
#include "mat_aos_v.h"

#endif
