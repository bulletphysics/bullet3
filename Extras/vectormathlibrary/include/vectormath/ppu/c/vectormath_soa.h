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

#ifndef _VECTORMATH_SOA_C_PPU_H
#define _VECTORMATH_SOA_C_PPU_H

#include <math.h>
#include <altivec.h>
#include "vectormath_aos.h"

#ifdef _VECTORMATH_DEBUG
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef _VECTORMATH_SOA_C_TYPES_H
#define _VECTORMATH_SOA_C_TYPES_H

/* A set of four 3-D vectors in structure-of-arrays format
 */
typedef struct _VmathSoaVector3
{
    vec_float4 x;
    vec_float4 y;
    vec_float4 z;
} VmathSoaVector3;

/* A set of four 4-D vectors in structure-of-arrays format
 */
typedef struct _VmathSoaVector4
{
    vec_float4 x;
    vec_float4 y;
    vec_float4 z;
    vec_float4 w;
} VmathSoaVector4;

/* A set of four 3-D points in structure-of-arrays format
 */
typedef struct _VmathSoaPoint3
{
    vec_float4 x;
    vec_float4 y;
    vec_float4 z;
} VmathSoaPoint3;

/* A set of four quaternions in structure-of-arrays format
 */
typedef struct _VmathSoaQuat
{
    vec_float4 x;
    vec_float4 y;
    vec_float4 z;
    vec_float4 w;
} VmathSoaQuat;

/* A set of four 3x3 matrices in structure-of-arrays format
 */
typedef struct _VmathSoaMatrix3
{
    VmathSoaVector3 col0;
    VmathSoaVector3 col1;
    VmathSoaVector3 col2;
} VmathSoaMatrix3;

/* A set of four 4x4 matrices in structure-of-arrays format
 */
typedef struct _VmathSoaMatrix4
{
    VmathSoaVector4 col0;
    VmathSoaVector4 col1;
    VmathSoaVector4 col2;
    VmathSoaVector4 col3;
} VmathSoaMatrix4;

/* A set of four 3x4 transformation matrices in structure-of-arrays format
 */
typedef struct _VmathSoaTransform3
{
    VmathSoaVector3 col0;
    VmathSoaVector3 col1;
    VmathSoaVector3 col2;
    VmathSoaVector3 col3;
} VmathSoaTransform3;

#endif

/*
 * Copy a 3-D vector
 */
static inline void vmathSoaV3Copy( VmathSoaVector3 *result, const VmathSoaVector3 *vec );

/*
 * Construct a 3-D vector from x, y, and z elements
 */
static inline void vmathSoaV3MakeFromElems( VmathSoaVector3 *result, vec_float4 x, vec_float4 y, vec_float4 z );

/*
 * Copy elements from a 3-D point into a 3-D vector
 */
static inline void vmathSoaV3MakeFromP3( VmathSoaVector3 *result, const VmathSoaPoint3 *pnt );

/*
 * Set all elements of a 3-D vector to the same scalar value
 */
static inline void vmathSoaV3MakeFromScalar( VmathSoaVector3 *result, vec_float4 scalar );

/*
 * Replicate an AoS 3-D vector
 */
static inline void vmathSoaV3MakeFromAos( VmathSoaVector3 *result, const VmathVector3 *vec );

/*
 * Insert four AoS 3-D vectors
 */
static inline void vmathSoaV3MakeFrom4Aos( VmathSoaVector3 *result, const VmathVector3 *vec0, const VmathVector3 *vec1, const VmathVector3 *vec2, const VmathVector3 *vec3 );

/*
 * Extract four AoS 3-D vectors
 */
static inline void vmathSoaV3Get4Aos( const VmathSoaVector3 *vec, VmathVector3 *result0, VmathVector3 *result1, VmathVector3 *result2, VmathVector3 *result3 );

/*
 * Set the x element of a 3-D vector
 */
static inline void vmathSoaV3SetX( VmathSoaVector3 *result, vec_float4 x );

/*
 * Set the y element of a 3-D vector
 */
static inline void vmathSoaV3SetY( VmathSoaVector3 *result, vec_float4 y );

/*
 * Set the z element of a 3-D vector
 */
static inline void vmathSoaV3SetZ( VmathSoaVector3 *result, vec_float4 z );

/*
 * Get the x element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3GetX( const VmathSoaVector3 *vec );

/*
 * Get the y element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3GetY( const VmathSoaVector3 *vec );

/*
 * Get the z element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3GetZ( const VmathSoaVector3 *vec );

/*
 * Set an x, y, or z element of a 3-D vector by index
 */
static inline void vmathSoaV3SetElem( VmathSoaVector3 *result, int idx, vec_float4 value );

/*
 * Get an x, y, or z element of a 3-D vector by index
 */
static inline vec_float4 vmathSoaV3GetElem( const VmathSoaVector3 *vec, int idx );

/*
 * Add two 3-D vectors
 */
static inline void vmathSoaV3Add( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Subtract a 3-D vector from another 3-D vector
 */
static inline void vmathSoaV3Sub( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Add a 3-D vector to a 3-D point
 */
static inline void vmathSoaV3AddP3( VmathSoaPoint3 *result, const VmathSoaVector3 *vec, const VmathSoaPoint3 *pnt );

/*
 * Multiply a 3-D vector by a scalar
 */
static inline void vmathSoaV3ScalarMul( VmathSoaVector3 *result, const VmathSoaVector3 *vec, vec_float4 scalar );

/*
 * Divide a 3-D vector by a scalar
 */
static inline void vmathSoaV3ScalarDiv( VmathSoaVector3 *result, const VmathSoaVector3 *vec, vec_float4 scalar );

/*
 * Negate all elements of a 3-D vector
 */
static inline void vmathSoaV3Neg( VmathSoaVector3 *result, const VmathSoaVector3 *vec );

/*
 * Construct x axis
 */
static inline void vmathSoaV3MakeXAxis( VmathSoaVector3 *result );

/*
 * Construct y axis
 */
static inline void vmathSoaV3MakeYAxis( VmathSoaVector3 *result );

/*
 * Construct z axis
 */
static inline void vmathSoaV3MakeZAxis( VmathSoaVector3 *result );

/*
 * Multiply two 3-D vectors per element
 */
static inline void vmathSoaV3MulPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Divide two 3-D vectors per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline void vmathSoaV3DivPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Compute the reciprocal of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline void vmathSoaV3RecipPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec );

/*
 * Compute the square root of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline void vmathSoaV3SqrtPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec );

/*
 * Compute the reciprocal square root of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline void vmathSoaV3RsqrtPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec );

/*
 * Compute the absolute value of a 3-D vector per element
 */
static inline void vmathSoaV3AbsPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec );

/*
 * Copy sign from one 3-D vector to another, per element
 */
static inline void vmathSoaV3CopySignPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Maximum of two 3-D vectors per element
 */
static inline void vmathSoaV3MaxPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Minimum of two 3-D vectors per element
 */
static inline void vmathSoaV3MinPerElem( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Maximum element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3MaxElem( const VmathSoaVector3 *vec );

/*
 * Minimum element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3MinElem( const VmathSoaVector3 *vec );

/*
 * Compute the sum of all elements of a 3-D vector
 */
static inline vec_float4 vmathSoaV3Sum( const VmathSoaVector3 *vec );

/*
 * Compute the dot product of two 3-D vectors
 */
static inline vec_float4 vmathSoaV3Dot( const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Compute the square of the length of a 3-D vector
 */
static inline vec_float4 vmathSoaV3LengthSqr( const VmathSoaVector3 *vec );

/*
 * Compute the length of a 3-D vector
 */
static inline vec_float4 vmathSoaV3Length( const VmathSoaVector3 *vec );

/*
 * Normalize a 3-D vector
 * NOTE: 
 * The result is unpredictable when all elements of vec are at or near zero.
 */
static inline void vmathSoaV3Normalize( VmathSoaVector3 *result, const VmathSoaVector3 *vec );

/*
 * Compute cross product of two 3-D vectors
 */
static inline void vmathSoaV3Cross( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Outer product of two 3-D vectors
 */
static inline void vmathSoaV3Outer( VmathSoaMatrix3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Pre-multiply a row vector by a 3x3 matrix
 */
static inline void vmathSoaV3RowMul( VmathSoaVector3 *result, const VmathSoaVector3 *vec, const VmathSoaMatrix3 *mat );

/*
 * Cross-product matrix of a 3-D vector
 */
static inline void vmathSoaV3CrossMatrix( VmathSoaMatrix3 *result, const VmathSoaVector3 *vec );

/*
 * Create cross-product matrix and multiply
 * NOTE: 
 * Faster than separately creating a cross-product matrix and multiplying.
 */
static inline void vmathSoaV3CrossMatrixMul( VmathSoaMatrix3 *result, const VmathSoaVector3 *vec, const VmathSoaMatrix3 *mat );

/*
 * Linear interpolation between two 3-D vectors
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline void vmathSoaV3Lerp( VmathSoaVector3 *result, vec_float4 t, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1 );

/*
 * Spherical linear interpolation between two 3-D vectors
 * NOTE: 
 * The result is unpredictable if the vectors point in opposite directions.
 * Does not clamp t between 0 and 1.
 */
static inline void vmathSoaV3Slerp( VmathSoaVector3 *result, vec_float4 t, const VmathSoaVector3 *unitVec0, const VmathSoaVector3 *unitVec1 );

/*
 * Conditionally select between two 3-D vectors
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline void vmathSoaV3Select( VmathSoaVector3 *result, const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1, vec_uint4 select1 );

/*
 * Load four three-float 3-D vectors, stored in three quadwords
 */
static inline void vmathSoaV3LoadXYZArray( VmathSoaVector3 *vec, const vec_float4 *threeQuads );

/*
 * Store four slots of an SoA 3-D vector in three quadwords
 */
static inline void vmathSoaV3StoreXYZArray( const VmathSoaVector3 *vec, vec_float4 *threeQuads );

/*
 * Store eight slots of two SoA 3-D vectors as half-floats
 */
static inline void vmathSoaV3StoreHalfFloats( const VmathSoaVector3 *vec0, const VmathSoaVector3 *vec1, vec_ushort8 *threeQuads );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3-D vector
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaV3Print( const VmathSoaVector3 *vec );

/*
 * Print a 3-D vector and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaV3Prints( const VmathSoaVector3 *vec, const char *name );

#endif

/*
 * Copy a 4-D vector
 */
static inline void vmathSoaV4Copy( VmathSoaVector4 *result, const VmathSoaVector4 *vec );

/*
 * Construct a 4-D vector from x, y, z, and w elements
 */
static inline void vmathSoaV4MakeFromElems( VmathSoaVector4 *result, vec_float4 x, vec_float4 y, vec_float4 z, vec_float4 w );

/*
 * Construct a 4-D vector from a 3-D vector and a scalar
 */
static inline void vmathSoaV4MakeFromV3Scalar( VmathSoaVector4 *result, const VmathSoaVector3 *xyz, vec_float4 w );

/*
 * Copy x, y, and z from a 3-D vector into a 4-D vector, and set w to 0
 */
static inline void vmathSoaV4MakeFromV3( VmathSoaVector4 *result, const VmathSoaVector3 *vec );

/*
 * Copy x, y, and z from a 3-D point into a 4-D vector, and set w to 1
 */
static inline void vmathSoaV4MakeFromP3( VmathSoaVector4 *result, const VmathSoaPoint3 *pnt );

/*
 * Copy elements from a quaternion into a 4-D vector
 */
static inline void vmathSoaV4MakeFromQ( VmathSoaVector4 *result, const VmathSoaQuat *quat );

/*
 * Set all elements of a 4-D vector to the same scalar value
 */
static inline void vmathSoaV4MakeFromScalar( VmathSoaVector4 *result, vec_float4 scalar );

/*
 * Replicate an AoS 4-D vector
 */
static inline void vmathSoaV4MakeFromAos( VmathSoaVector4 *result, const VmathVector4 *vec );

/*
 * Insert four AoS 4-D vectors
 */
static inline void vmathSoaV4MakeFrom4Aos( VmathSoaVector4 *result, const VmathVector4 *vec0, const VmathVector4 *vec1, const VmathVector4 *vec2, const VmathVector4 *vec3 );

/*
 * Extract four AoS 4-D vectors
 */
static inline void vmathSoaV4Get4Aos( const VmathSoaVector4 *vec, VmathVector4 *result0, VmathVector4 *result1, VmathVector4 *result2, VmathVector4 *result3 );

/*
 * Set the x, y, and z elements of a 4-D vector
 * NOTE: 
 * This function does not change the w element.
 */
static inline void vmathSoaV4SetXYZ( VmathSoaVector4 *result, const VmathSoaVector3 *vec );

/*
 * Get the x, y, and z elements of a 4-D vector
 */
static inline void vmathSoaV4GetXYZ( VmathSoaVector3 *result, const VmathSoaVector4 *vec );

/*
 * Set the x element of a 4-D vector
 */
static inline void vmathSoaV4SetX( VmathSoaVector4 *result, vec_float4 x );

/*
 * Set the y element of a 4-D vector
 */
static inline void vmathSoaV4SetY( VmathSoaVector4 *result, vec_float4 y );

/*
 * Set the z element of a 4-D vector
 */
static inline void vmathSoaV4SetZ( VmathSoaVector4 *result, vec_float4 z );

/*
 * Set the w element of a 4-D vector
 */
static inline void vmathSoaV4SetW( VmathSoaVector4 *result, vec_float4 w );

/*
 * Get the x element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4GetX( const VmathSoaVector4 *vec );

/*
 * Get the y element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4GetY( const VmathSoaVector4 *vec );

/*
 * Get the z element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4GetZ( const VmathSoaVector4 *vec );

/*
 * Get the w element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4GetW( const VmathSoaVector4 *vec );

/*
 * Set an x, y, z, or w element of a 4-D vector by index
 */
static inline void vmathSoaV4SetElem( VmathSoaVector4 *result, int idx, vec_float4 value );

/*
 * Get an x, y, z, or w element of a 4-D vector by index
 */
static inline vec_float4 vmathSoaV4GetElem( const VmathSoaVector4 *vec, int idx );

/*
 * Add two 4-D vectors
 */
static inline void vmathSoaV4Add( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Subtract a 4-D vector from another 4-D vector
 */
static inline void vmathSoaV4Sub( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Multiply a 4-D vector by a scalar
 */
static inline void vmathSoaV4ScalarMul( VmathSoaVector4 *result, const VmathSoaVector4 *vec, vec_float4 scalar );

/*
 * Divide a 4-D vector by a scalar
 */
static inline void vmathSoaV4ScalarDiv( VmathSoaVector4 *result, const VmathSoaVector4 *vec, vec_float4 scalar );

/*
 * Negate all elements of a 4-D vector
 */
static inline void vmathSoaV4Neg( VmathSoaVector4 *result, const VmathSoaVector4 *vec );

/*
 * Construct x axis
 */
static inline void vmathSoaV4MakeXAxis( VmathSoaVector4 *result );

/*
 * Construct y axis
 */
static inline void vmathSoaV4MakeYAxis( VmathSoaVector4 *result );

/*
 * Construct z axis
 */
static inline void vmathSoaV4MakeZAxis( VmathSoaVector4 *result );

/*
 * Construct w axis
 */
static inline void vmathSoaV4MakeWAxis( VmathSoaVector4 *result );

/*
 * Multiply two 4-D vectors per element
 */
static inline void vmathSoaV4MulPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Divide two 4-D vectors per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline void vmathSoaV4DivPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Compute the reciprocal of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline void vmathSoaV4RecipPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec );

/*
 * Compute the square root of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline void vmathSoaV4SqrtPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec );

/*
 * Compute the reciprocal square root of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline void vmathSoaV4RsqrtPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec );

/*
 * Compute the absolute value of a 4-D vector per element
 */
static inline void vmathSoaV4AbsPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec );

/*
 * Copy sign from one 4-D vector to another, per element
 */
static inline void vmathSoaV4CopySignPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Maximum of two 4-D vectors per element
 */
static inline void vmathSoaV4MaxPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Minimum of two 4-D vectors per element
 */
static inline void vmathSoaV4MinPerElem( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Maximum element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4MaxElem( const VmathSoaVector4 *vec );

/*
 * Minimum element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4MinElem( const VmathSoaVector4 *vec );

/*
 * Compute the sum of all elements of a 4-D vector
 */
static inline vec_float4 vmathSoaV4Sum( const VmathSoaVector4 *vec );

/*
 * Compute the dot product of two 4-D vectors
 */
static inline vec_float4 vmathSoaV4Dot( const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Compute the square of the length of a 4-D vector
 */
static inline vec_float4 vmathSoaV4LengthSqr( const VmathSoaVector4 *vec );

/*
 * Compute the length of a 4-D vector
 */
static inline vec_float4 vmathSoaV4Length( const VmathSoaVector4 *vec );

/*
 * Normalize a 4-D vector
 * NOTE: 
 * The result is unpredictable when all elements of vec are at or near zero.
 */
static inline void vmathSoaV4Normalize( VmathSoaVector4 *result, const VmathSoaVector4 *vec );

/*
 * Outer product of two 4-D vectors
 */
static inline void vmathSoaV4Outer( VmathSoaMatrix4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Linear interpolation between two 4-D vectors
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline void vmathSoaV4Lerp( VmathSoaVector4 *result, vec_float4 t, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1 );

/*
 * Spherical linear interpolation between two 4-D vectors
 * NOTE: 
 * The result is unpredictable if the vectors point in opposite directions.
 * Does not clamp t between 0 and 1.
 */
static inline void vmathSoaV4Slerp( VmathSoaVector4 *result, vec_float4 t, const VmathSoaVector4 *unitVec0, const VmathSoaVector4 *unitVec1 );

/*
 * Conditionally select between two 4-D vectors
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline void vmathSoaV4Select( VmathSoaVector4 *result, const VmathSoaVector4 *vec0, const VmathSoaVector4 *vec1, vec_uint4 select1 );

/*
 * Store four slots of an SoA 4-D vector as half-floats
 */
static inline void vmathSoaV4StoreHalfFloats( const VmathSoaVector4 *vec, vec_ushort8 *twoQuads );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 4-D vector
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaV4Print( const VmathSoaVector4 *vec );

/*
 * Print a 4-D vector and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaV4Prints( const VmathSoaVector4 *vec, const char *name );

#endif

/*
 * Copy a 3-D point
 */
static inline void vmathSoaP3Copy( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt );

/*
 * Construct a 3-D point from x, y, and z elements
 */
static inline void vmathSoaP3MakeFromElems( VmathSoaPoint3 *result, vec_float4 x, vec_float4 y, vec_float4 z );

/*
 * Copy elements from a 3-D vector into a 3-D point
 */
static inline void vmathSoaP3MakeFromV3( VmathSoaPoint3 *result, const VmathSoaVector3 *vec );

/*
 * Set all elements of a 3-D point to the same scalar value
 */
static inline void vmathSoaP3MakeFromScalar( VmathSoaPoint3 *result, vec_float4 scalar );

/*
 * Replicate an AoS 3-D point
 */
static inline void vmathSoaP3MakeFromAos( VmathSoaPoint3 *result, const VmathPoint3 *pnt );

/*
 * Insert four AoS 3-D points
 */
static inline void vmathSoaP3MakeFrom4Aos( VmathSoaPoint3 *result, const VmathPoint3 *pnt0, const VmathPoint3 *pnt1, const VmathPoint3 *pnt2, const VmathPoint3 *pnt3 );

/*
 * Extract four AoS 3-D points
 */
static inline void vmathSoaP3Get4Aos( const VmathSoaPoint3 *pnt, VmathPoint3 *result0, VmathPoint3 *result1, VmathPoint3 *result2, VmathPoint3 *result3 );

/*
 * Set the x element of a 3-D point
 */
static inline void vmathSoaP3SetX( VmathSoaPoint3 *result, vec_float4 x );

/*
 * Set the y element of a 3-D point
 */
static inline void vmathSoaP3SetY( VmathSoaPoint3 *result, vec_float4 y );

/*
 * Set the z element of a 3-D point
 */
static inline void vmathSoaP3SetZ( VmathSoaPoint3 *result, vec_float4 z );

/*
 * Get the x element of a 3-D point
 */
static inline vec_float4 vmathSoaP3GetX( const VmathSoaPoint3 *pnt );

/*
 * Get the y element of a 3-D point
 */
static inline vec_float4 vmathSoaP3GetY( const VmathSoaPoint3 *pnt );

/*
 * Get the z element of a 3-D point
 */
static inline vec_float4 vmathSoaP3GetZ( const VmathSoaPoint3 *pnt );

/*
 * Set an x, y, or z element of a 3-D point by index
 */
static inline void vmathSoaP3SetElem( VmathSoaPoint3 *result, int idx, vec_float4 value );

/*
 * Get an x, y, or z element of a 3-D point by index
 */
static inline vec_float4 vmathSoaP3GetElem( const VmathSoaPoint3 *pnt, int idx );

/*
 * Subtract a 3-D point from another 3-D point
 */
static inline void vmathSoaP3Sub( VmathSoaVector3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Add a 3-D point to a 3-D vector
 */
static inline void vmathSoaP3AddV3( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, const VmathSoaVector3 *vec );

/*
 * Subtract a 3-D vector from a 3-D point
 */
static inline void vmathSoaP3SubV3( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, const VmathSoaVector3 *vec );

/*
 * Multiply two 3-D points per element
 */
static inline void vmathSoaP3MulPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Divide two 3-D points per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline void vmathSoaP3DivPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Compute the reciprocal of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline void vmathSoaP3RecipPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt );

/*
 * Compute the square root of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline void vmathSoaP3SqrtPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt );

/*
 * Compute the reciprocal square root of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline void vmathSoaP3RsqrtPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt );

/*
 * Compute the absolute value of a 3-D point per element
 */
static inline void vmathSoaP3AbsPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt );

/*
 * Copy sign from one 3-D point to another, per element
 */
static inline void vmathSoaP3CopySignPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Maximum of two 3-D points per element
 */
static inline void vmathSoaP3MaxPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Minimum of two 3-D points per element
 */
static inline void vmathSoaP3MinPerElem( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Maximum element of a 3-D point
 */
static inline vec_float4 vmathSoaP3MaxElem( const VmathSoaPoint3 *pnt );

/*
 * Minimum element of a 3-D point
 */
static inline vec_float4 vmathSoaP3MinElem( const VmathSoaPoint3 *pnt );

/*
 * Compute the sum of all elements of a 3-D point
 */
static inline vec_float4 vmathSoaP3Sum( const VmathSoaPoint3 *pnt );

/*
 * Apply uniform scale to a 3-D point
 */
static inline void vmathSoaP3Scale( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, vec_float4 scaleVal );

/*
 * Apply non-uniform scale to a 3-D point
 */
static inline void vmathSoaP3NonUniformScale( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt, const VmathSoaVector3 *scaleVec );

/*
 * Scalar projection of a 3-D point on a unit-length 3-D vector
 */
static inline vec_float4 vmathSoaP3Projection( const VmathSoaPoint3 *pnt, const VmathSoaVector3 *unitVec );

/*
 * Compute the square of the distance of a 3-D point from the coordinate-system origin
 */
static inline vec_float4 vmathSoaP3DistSqrFromOrigin( const VmathSoaPoint3 *pnt );

/*
 * Compute the distance of a 3-D point from the coordinate-system origin
 */
static inline vec_float4 vmathSoaP3DistFromOrigin( const VmathSoaPoint3 *pnt );

/*
 * Compute the square of the distance between two 3-D points
 */
static inline vec_float4 vmathSoaP3DistSqr( const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Compute the distance between two 3-D points
 */
static inline vec_float4 vmathSoaP3Dist( const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Linear interpolation between two 3-D points
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline void vmathSoaP3Lerp( VmathSoaPoint3 *result, vec_float4 t, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1 );

/*
 * Conditionally select between two 3-D points
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline void vmathSoaP3Select( VmathSoaPoint3 *result, const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1, vec_uint4 select1 );

/*
 * Load four three-float 3-D points, stored in three quadwords
 */
static inline void vmathSoaP3LoadXYZArray( VmathSoaPoint3 *pnt, const vec_float4 *threeQuads );

/*
 * Store four slots of an SoA 3-D point in three quadwords
 */
static inline void vmathSoaP3StoreXYZArray( const VmathSoaPoint3 *pnt, vec_float4 *threeQuads );

/*
 * Store eight slots of two SoA 3-D points as half-floats
 */
static inline void vmathSoaP3StoreHalfFloats( const VmathSoaPoint3 *pnt0, const VmathSoaPoint3 *pnt1, vec_ushort8 *threeQuads );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3-D point
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaP3Print( const VmathSoaPoint3 *pnt );

/*
 * Print a 3-D point and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaP3Prints( const VmathSoaPoint3 *pnt, const char *name );

#endif

/*
 * Copy a quaternion
 */
static inline void vmathSoaQCopy( VmathSoaQuat *result, const VmathSoaQuat *quat );

/*
 * Construct a quaternion from x, y, z, and w elements
 */
static inline void vmathSoaQMakeFromElems( VmathSoaQuat *result, vec_float4 x, vec_float4 y, vec_float4 z, vec_float4 w );

/*
 * Construct a quaternion from a 3-D vector and a scalar
 */
static inline void vmathSoaQMakeFromV3Scalar( VmathSoaQuat *result, const VmathSoaVector3 *xyz, vec_float4 w );

/*
 * Copy elements from a 4-D vector into a quaternion
 */
static inline void vmathSoaQMakeFromV4( VmathSoaQuat *result, const VmathSoaVector4 *vec );

/*
 * Convert a rotation matrix to a unit-length quaternion
 */
static inline void vmathSoaQMakeFromM3( VmathSoaQuat *result, const VmathSoaMatrix3 *rotMat );

/*
 * Set all elements of a quaternion to the same scalar value
 */
static inline void vmathSoaQMakeFromScalar( VmathSoaQuat *result, vec_float4 scalar );

/*
 * Replicate an AoS quaternion
 */
static inline void vmathSoaQMakeFromAos( VmathSoaQuat *result, const VmathQuat *quat );

/*
 * Insert four AoS quaternions
 */
static inline void vmathSoaQMakeFrom4Aos( VmathSoaQuat *result, const VmathQuat *quat0, const VmathQuat *quat1, const VmathQuat *quat2, const VmathQuat *quat3 );

/*
 * Extract four AoS quaternions
 */
static inline void vmathSoaQGet4Aos( const VmathSoaQuat *quat, VmathQuat *result0, VmathQuat *result1, VmathQuat *result2, VmathQuat *result3 );

/*
 * Set the x, y, and z elements of a quaternion
 * NOTE: 
 * This function does not change the w element.
 */
static inline void vmathSoaQSetXYZ( VmathSoaQuat *result, const VmathSoaVector3 *vec );

/*
 * Get the x, y, and z elements of a quaternion
 */
static inline void vmathSoaQGetXYZ( VmathSoaVector3 *result, const VmathSoaQuat *quat );

/*
 * Set the x element of a quaternion
 */
static inline void vmathSoaQSetX( VmathSoaQuat *result, vec_float4 x );

/*
 * Set the y element of a quaternion
 */
static inline void vmathSoaQSetY( VmathSoaQuat *result, vec_float4 y );

/*
 * Set the z element of a quaternion
 */
static inline void vmathSoaQSetZ( VmathSoaQuat *result, vec_float4 z );

/*
 * Set the w element of a quaternion
 */
static inline void vmathSoaQSetW( VmathSoaQuat *result, vec_float4 w );

/*
 * Get the x element of a quaternion
 */
static inline vec_float4 vmathSoaQGetX( const VmathSoaQuat *quat );

/*
 * Get the y element of a quaternion
 */
static inline vec_float4 vmathSoaQGetY( const VmathSoaQuat *quat );

/*
 * Get the z element of a quaternion
 */
static inline vec_float4 vmathSoaQGetZ( const VmathSoaQuat *quat );

/*
 * Get the w element of a quaternion
 */
static inline vec_float4 vmathSoaQGetW( const VmathSoaQuat *quat );

/*
 * Set an x, y, z, or w element of a quaternion by index
 */
static inline void vmathSoaQSetElem( VmathSoaQuat *result, int idx, vec_float4 value );

/*
 * Get an x, y, z, or w element of a quaternion by index
 */
static inline vec_float4 vmathSoaQGetElem( const VmathSoaQuat *quat, int idx );

/*
 * Add two quaternions
 */
static inline void vmathSoaQAdd( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 );

/*
 * Subtract a quaternion from another quaternion
 */
static inline void vmathSoaQSub( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 );

/*
 * Multiply two quaternions
 */
static inline void vmathSoaQMul( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 );

/*
 * Multiply a quaternion by a scalar
 */
static inline void vmathSoaQScalarMul( VmathSoaQuat *result, const VmathSoaQuat *quat, vec_float4 scalar );

/*
 * Divide a quaternion by a scalar
 */
static inline void vmathSoaQScalarDiv( VmathSoaQuat *result, const VmathSoaQuat *quat, vec_float4 scalar );

/*
 * Negate all elements of a quaternion
 */
static inline void vmathSoaQNeg( VmathSoaQuat *result, const VmathSoaQuat *quat );

/*
 * Construct an identity quaternion
 */
static inline void vmathSoaQMakeIdentity( VmathSoaQuat *result );

/*
 * Construct a quaternion to rotate between two unit-length 3-D vectors
 * NOTE: 
 * The result is unpredictable if unitVec0 and unitVec1 point in opposite directions.
 */
static inline void vmathSoaQMakeRotationArc( VmathSoaQuat *result, const VmathSoaVector3 *unitVec0, const VmathSoaVector3 *unitVec1 );

/*
 * Construct a quaternion to rotate around a unit-length 3-D vector
 */
static inline void vmathSoaQMakeRotationAxis( VmathSoaQuat *result, vec_float4 radians, const VmathSoaVector3 *unitVec );

/*
 * Construct a quaternion to rotate around the x axis
 */
static inline void vmathSoaQMakeRotationX( VmathSoaQuat *result, vec_float4 radians );

/*
 * Construct a quaternion to rotate around the y axis
 */
static inline void vmathSoaQMakeRotationY( VmathSoaQuat *result, vec_float4 radians );

/*
 * Construct a quaternion to rotate around the z axis
 */
static inline void vmathSoaQMakeRotationZ( VmathSoaQuat *result, vec_float4 radians );

/*
 * Compute the conjugate of a quaternion
 */
static inline void vmathSoaQConj( VmathSoaQuat *result, const VmathSoaQuat *quat );

/*
 * Use a unit-length quaternion to rotate a 3-D vector
 */
static inline void vmathSoaQRotate( VmathSoaVector3 *result, const VmathSoaQuat *unitQuat, const VmathSoaVector3 *vec );

/*
 * Compute the dot product of two quaternions
 */
static inline vec_float4 vmathSoaQDot( const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 );

/*
 * Compute the norm of a quaternion
 */
static inline vec_float4 vmathSoaQNorm( const VmathSoaQuat *quat );

/*
 * Compute the length of a quaternion
 */
static inline vec_float4 vmathSoaQLength( const VmathSoaQuat *quat );

/*
 * Normalize a quaternion
 * NOTE: 
 * The result is unpredictable when all elements of quat are at or near zero.
 */
static inline void vmathSoaQNormalize( VmathSoaQuat *result, const VmathSoaQuat *quat );

/*
 * Linear interpolation between two quaternions
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline void vmathSoaQLerp( VmathSoaQuat *result, vec_float4 t, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1 );

/*
 * Spherical linear interpolation between two quaternions
 * NOTE: 
 * Interpolates along the shortest path between orientations.
 * Does not clamp t between 0 and 1.
 */
static inline void vmathSoaQSlerp( VmathSoaQuat *result, vec_float4 t, const VmathSoaQuat *unitQuat0, const VmathSoaQuat *unitQuat1 );

/*
 * Spherical quadrangle interpolation
 */
static inline void vmathSoaQSquad( VmathSoaQuat *result, vec_float4 t, const VmathSoaQuat *unitQuat0, const VmathSoaQuat *unitQuat1, const VmathSoaQuat *unitQuat2, const VmathSoaQuat *unitQuat3 );

/*
 * Conditionally select between two quaternions
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline void vmathSoaQSelect( VmathSoaQuat *result, const VmathSoaQuat *quat0, const VmathSoaQuat *quat1, vec_uint4 select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a quaternion
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaQPrint( const VmathSoaQuat *quat );

/*
 * Print a quaternion and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaQPrints( const VmathSoaQuat *quat, const char *name );

#endif

/*
 * Copy a 3x3 matrix
 */
static inline void vmathSoaM3Copy( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat );

/*
 * Construct a 3x3 matrix containing the specified columns
 */
static inline void vmathSoaM3MakeFromCols( VmathSoaMatrix3 *result, const VmathSoaVector3 *col0, const VmathSoaVector3 *col1, const VmathSoaVector3 *col2 );

/*
 * Construct a 3x3 rotation matrix from a unit-length quaternion
 */
static inline void vmathSoaM3MakeFromQ( VmathSoaMatrix3 *result, const VmathSoaQuat *unitQuat );

/*
 * Set all elements of a 3x3 matrix to the same scalar value
 */
static inline void vmathSoaM3MakeFromScalar( VmathSoaMatrix3 *result, vec_float4 scalar );

/*
 * Replicate an AoS 3x3 matrix
 */
static inline void vmathSoaM3MakeFromAos( VmathSoaMatrix3 *result, const VmathMatrix3 *mat );

/*
 * Insert four AoS 3x3 matrices
 */
static inline void vmathSoaM3MakeFrom4Aos( VmathSoaMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1, const VmathMatrix3 *mat2, const VmathMatrix3 *mat3 );

/*
 * Extract four AoS 3x3 matrices
 */
static inline void vmathSoaM3Get4Aos( const VmathSoaMatrix3 *mat, VmathMatrix3 *result0, VmathMatrix3 *result1, VmathMatrix3 *result2, VmathMatrix3 *result3 );

/*
 * Set column 0 of a 3x3 matrix
 */
static inline void vmathSoaM3SetCol0( VmathSoaMatrix3 *result, const VmathSoaVector3 *col0 );

/*
 * Set column 1 of a 3x3 matrix
 */
static inline void vmathSoaM3SetCol1( VmathSoaMatrix3 *result, const VmathSoaVector3 *col1 );

/*
 * Set column 2 of a 3x3 matrix
 */
static inline void vmathSoaM3SetCol2( VmathSoaMatrix3 *result, const VmathSoaVector3 *col2 );

/*
 * Get column 0 of a 3x3 matrix
 */
static inline void vmathSoaM3GetCol0( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat );

/*
 * Get column 1 of a 3x3 matrix
 */
static inline void vmathSoaM3GetCol1( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat );

/*
 * Get column 2 of a 3x3 matrix
 */
static inline void vmathSoaM3GetCol2( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat );

/*
 * Set the column of a 3x3 matrix referred to by the specified index
 */
static inline void vmathSoaM3SetCol( VmathSoaMatrix3 *result, int col, const VmathSoaVector3 *vec );

/*
 * Set the row of a 3x3 matrix referred to by the specified index
 */
static inline void vmathSoaM3SetRow( VmathSoaMatrix3 *result, int row, const VmathSoaVector3 *vec );

/*
 * Get the column of a 3x3 matrix referred to by the specified index
 */
static inline void vmathSoaM3GetCol( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat, int col );

/*
 * Get the row of a 3x3 matrix referred to by the specified index
 */
static inline void vmathSoaM3GetRow( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat, int row );

/*
 * Set the element of a 3x3 matrix referred to by column and row indices
 */
static inline void vmathSoaM3SetElem( VmathSoaMatrix3 *result, int col, int row, vec_float4 val );

/*
 * Get the element of a 3x3 matrix referred to by column and row indices
 */
static inline vec_float4 vmathSoaM3GetElem( const VmathSoaMatrix3 *mat, int col, int row );

/*
 * Add two 3x3 matrices
 */
static inline void vmathSoaM3Add( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1 );

/*
 * Subtract a 3x3 matrix from another 3x3 matrix
 */
static inline void vmathSoaM3Sub( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1 );

/*
 * Negate all elements of a 3x3 matrix
 */
static inline void vmathSoaM3Neg( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat );

/*
 * Multiply a 3x3 matrix by a scalar
 */
static inline void vmathSoaM3ScalarMul( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat, vec_float4 scalar );

/*
 * Multiply a 3x3 matrix by a 3-D vector
 */
static inline void vmathSoaM3MulV3( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat, const VmathSoaVector3 *vec );

/*
 * Multiply two 3x3 matrices
 */
static inline void vmathSoaM3Mul( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1 );

/*
 * Construct an identity 3x3 matrix
 */
static inline void vmathSoaM3MakeIdentity( VmathSoaMatrix3 *result );

/*
 * Construct a 3x3 matrix to rotate around the x axis
 */
static inline void vmathSoaM3MakeRotationX( VmathSoaMatrix3 *result, vec_float4 radians );

/*
 * Construct a 3x3 matrix to rotate around the y axis
 */
static inline void vmathSoaM3MakeRotationY( VmathSoaMatrix3 *result, vec_float4 radians );

/*
 * Construct a 3x3 matrix to rotate around the z axis
 */
static inline void vmathSoaM3MakeRotationZ( VmathSoaMatrix3 *result, vec_float4 radians );

/*
 * Construct a 3x3 matrix to rotate around the x, y, and z axes
 */
static inline void vmathSoaM3MakeRotationZYX( VmathSoaMatrix3 *result, const VmathSoaVector3 *radiansXYZ );

/*
 * Construct a 3x3 matrix to rotate around a unit-length 3-D vector
 */
static inline void vmathSoaM3MakeRotationAxis( VmathSoaMatrix3 *result, vec_float4 radians, const VmathSoaVector3 *unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline void vmathSoaM3MakeRotationQ( VmathSoaMatrix3 *result, const VmathSoaQuat *unitQuat );

/*
 * Construct a 3x3 matrix to perform scaling
 */
static inline void vmathSoaM3MakeScale( VmathSoaMatrix3 *result, const VmathSoaVector3 *scaleVec );

/*
 * Append (post-multiply) a scale transformation to a 3x3 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathSoaM3AppendScale( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat, const VmathSoaVector3 *scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 3x3 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathSoaM3PrependScale( VmathSoaMatrix3 *result, const VmathSoaVector3 *scaleVec, const VmathSoaMatrix3 *mat );

/*
 * Multiply two 3x3 matrices per element
 */
static inline void vmathSoaM3MulPerElem( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1 );

/*
 * Compute the absolute value of a 3x3 matrix per element
 */
static inline void vmathSoaM3AbsPerElem( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat );

/*
 * Transpose of a 3x3 matrix
 */
static inline void vmathSoaM3Transpose( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat );

/*
 * Compute the inverse of a 3x3 matrix
 * NOTE: 
 * Result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline void vmathSoaM3Inverse( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat );

/*
 * Determinant of a 3x3 matrix
 */
static inline vec_float4 vmathSoaM3Determinant( const VmathSoaMatrix3 *mat );

/*
 * Conditionally select between two 3x3 matrices
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline void vmathSoaM3Select( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1, vec_uint4 select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3x3 matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaM3Print( const VmathSoaMatrix3 *mat );

/*
 * Print a 3x3 matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaM3Prints( const VmathSoaMatrix3 *mat, const char *name );

#endif

/*
 * Copy a 4x4 matrix
 */
static inline void vmathSoaM4Copy( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat );

/*
 * Construct a 4x4 matrix containing the specified columns
 */
static inline void vmathSoaM4MakeFromCols( VmathSoaMatrix4 *result, const VmathSoaVector4 *col0, const VmathSoaVector4 *col1, const VmathSoaVector4 *col2, const VmathSoaVector4 *col3 );

/*
 * Construct a 4x4 matrix from a 3x4 transformation matrix
 */
static inline void vmathSoaM4MakeFromT3( VmathSoaMatrix4 *result, const VmathSoaTransform3 *mat );

/*
 * Construct a 4x4 matrix from a 3x3 matrix and a 3-D vector
 */
static inline void vmathSoaM4MakeFromM3V3( VmathSoaMatrix4 *result, const VmathSoaMatrix3 *mat, const VmathSoaVector3 *translateVec );

/*
 * Construct a 4x4 matrix from a unit-length quaternion and a 3-D vector
 */
static inline void vmathSoaM4MakeFromQV3( VmathSoaMatrix4 *result, const VmathSoaQuat *unitQuat, const VmathSoaVector3 *translateVec );

/*
 * Set all elements of a 4x4 matrix to the same scalar value
 */
static inline void vmathSoaM4MakeFromScalar( VmathSoaMatrix4 *result, vec_float4 scalar );

/*
 * Replicate an AoS 4x4 matrix
 */
static inline void vmathSoaM4MakeFromAos( VmathSoaMatrix4 *result, const VmathMatrix4 *mat );

/*
 * Insert four AoS 4x4 matrices
 */
static inline void vmathSoaM4MakeFrom4Aos( VmathSoaMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1, const VmathMatrix4 *mat2, const VmathMatrix4 *mat3 );

/*
 * Extract four AoS 4x4 matrices
 */
static inline void vmathSoaM4Get4Aos( const VmathSoaMatrix4 *mat, VmathMatrix4 *result0, VmathMatrix4 *result1, VmathMatrix4 *result2, VmathMatrix4 *result3 );

/*
 * Set the upper-left 3x3 submatrix
 * NOTE: 
 * This function does not change the bottom row elements.
 */
static inline void vmathSoaM4SetUpper3x3( VmathSoaMatrix4 *result, const VmathSoaMatrix3 *mat3 );

/*
 * Get the upper-left 3x3 submatrix of a 4x4 matrix
 */
static inline void vmathSoaM4GetUpper3x3( VmathSoaMatrix3 *result, const VmathSoaMatrix4 *mat );

/*
 * Set translation component
 * NOTE: 
 * This function does not change the bottom row elements.
 */
static inline void vmathSoaM4SetTranslation( VmathSoaMatrix4 *result, const VmathSoaVector3 *translateVec );

/*
 * Get the translation component of a 4x4 matrix
 */
static inline void vmathSoaM4GetTranslation( VmathSoaVector3 *result, const VmathSoaMatrix4 *mat );

/*
 * Set column 0 of a 4x4 matrix
 */
static inline void vmathSoaM4SetCol0( VmathSoaMatrix4 *result, const VmathSoaVector4 *col0 );

/*
 * Set column 1 of a 4x4 matrix
 */
static inline void vmathSoaM4SetCol1( VmathSoaMatrix4 *result, const VmathSoaVector4 *col1 );

/*
 * Set column 2 of a 4x4 matrix
 */
static inline void vmathSoaM4SetCol2( VmathSoaMatrix4 *result, const VmathSoaVector4 *col2 );

/*
 * Set column 3 of a 4x4 matrix
 */
static inline void vmathSoaM4SetCol3( VmathSoaMatrix4 *result, const VmathSoaVector4 *col3 );

/*
 * Get column 0 of a 4x4 matrix
 */
static inline void vmathSoaM4GetCol0( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat );

/*
 * Get column 1 of a 4x4 matrix
 */
static inline void vmathSoaM4GetCol1( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat );

/*
 * Get column 2 of a 4x4 matrix
 */
static inline void vmathSoaM4GetCol2( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat );

/*
 * Get column 3 of a 4x4 matrix
 */
static inline void vmathSoaM4GetCol3( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat );

/*
 * Set the column of a 4x4 matrix referred to by the specified index
 */
static inline void vmathSoaM4SetCol( VmathSoaMatrix4 *result, int col, const VmathSoaVector4 *vec );

/*
 * Set the row of a 4x4 matrix referred to by the specified index
 */
static inline void vmathSoaM4SetRow( VmathSoaMatrix4 *result, int row, const VmathSoaVector4 *vec );

/*
 * Get the column of a 4x4 matrix referred to by the specified index
 */
static inline void vmathSoaM4GetCol( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, int col );

/*
 * Get the row of a 4x4 matrix referred to by the specified index
 */
static inline void vmathSoaM4GetRow( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, int row );

/*
 * Set the element of a 4x4 matrix referred to by column and row indices
 */
static inline void vmathSoaM4SetElem( VmathSoaMatrix4 *result, int col, int row, vec_float4 val );

/*
 * Get the element of a 4x4 matrix referred to by column and row indices
 */
static inline vec_float4 vmathSoaM4GetElem( const VmathSoaMatrix4 *mat, int col, int row );

/*
 * Add two 4x4 matrices
 */
static inline void vmathSoaM4Add( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1 );

/*
 * Subtract a 4x4 matrix from another 4x4 matrix
 */
static inline void vmathSoaM4Sub( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1 );

/*
 * Negate all elements of a 4x4 matrix
 */
static inline void vmathSoaM4Neg( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat );

/*
 * Multiply a 4x4 matrix by a scalar
 */
static inline void vmathSoaM4ScalarMul( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat, vec_float4 scalar );

/*
 * Multiply a 4x4 matrix by a 4-D vector
 */
static inline void vmathSoaM4MulV4( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, const VmathSoaVector4 *vec );

/*
 * Multiply a 4x4 matrix by a 3-D vector
 */
static inline void vmathSoaM4MulV3( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, const VmathSoaVector3 *vec );

/*
 * Multiply a 4x4 matrix by a 3-D point
 */
static inline void vmathSoaM4MulP3( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, const VmathSoaPoint3 *pnt );

/*
 * Multiply two 4x4 matrices
 */
static inline void vmathSoaM4Mul( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1 );

/*
 * Multiply a 4x4 matrix by a 3x4 transformation matrix
 */
static inline void vmathSoaM4MulT3( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat, const VmathSoaTransform3 *tfrm );

/*
 * Construct an identity 4x4 matrix
 */
static inline void vmathSoaM4MakeIdentity( VmathSoaMatrix4 *result );

/*
 * Construct a 4x4 matrix to rotate around the x axis
 */
static inline void vmathSoaM4MakeRotationX( VmathSoaMatrix4 *result, vec_float4 radians );

/*
 * Construct a 4x4 matrix to rotate around the y axis
 */
static inline void vmathSoaM4MakeRotationY( VmathSoaMatrix4 *result, vec_float4 radians );

/*
 * Construct a 4x4 matrix to rotate around the z axis
 */
static inline void vmathSoaM4MakeRotationZ( VmathSoaMatrix4 *result, vec_float4 radians );

/*
 * Construct a 4x4 matrix to rotate around the x, y, and z axes
 */
static inline void vmathSoaM4MakeRotationZYX( VmathSoaMatrix4 *result, const VmathSoaVector3 *radiansXYZ );

/*
 * Construct a 4x4 matrix to rotate around a unit-length 3-D vector
 */
static inline void vmathSoaM4MakeRotationAxis( VmathSoaMatrix4 *result, vec_float4 radians, const VmathSoaVector3 *unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline void vmathSoaM4MakeRotationQ( VmathSoaMatrix4 *result, const VmathSoaQuat *unitQuat );

/*
 * Construct a 4x4 matrix to perform scaling
 */
static inline void vmathSoaM4MakeScale( VmathSoaMatrix4 *result, const VmathSoaVector3 *scaleVec );

/*
 * Construct a 4x4 matrix to perform translation
 */
static inline void vmathSoaM4MakeTranslation( VmathSoaMatrix4 *result, const VmathSoaVector3 *translateVec );

/*
 * Construct viewing matrix based on eye position, position looked at, and up direction
 */
static inline void vmathSoaM4MakeLookAt( VmathSoaMatrix4 *result, const VmathSoaPoint3 *eyePos, const VmathSoaPoint3 *lookAtPos, const VmathSoaVector3 *upVec );

/*
 * Construct a perspective projection matrix
 */
static inline void vmathSoaM4MakePerspective( VmathSoaMatrix4 *result, vec_float4 fovyRadians, vec_float4 aspect, vec_float4 zNear, vec_float4 zFar );

/*
 * Construct a perspective projection matrix based on frustum
 */
static inline void vmathSoaM4MakeFrustum( VmathSoaMatrix4 *result, vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar );

/*
 * Construct an orthographic projection matrix
 */
static inline void vmathSoaM4MakeOrthographic( VmathSoaMatrix4 *result, vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar );

/*
 * Append (post-multiply) a scale transformation to a 4x4 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathSoaM4AppendScale( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat, const VmathSoaVector3 *scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 4x4 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathSoaM4PrependScale( VmathSoaMatrix4 *result, const VmathSoaVector3 *scaleVec, const VmathSoaMatrix4 *mat );

/*
 * Multiply two 4x4 matrices per element
 */
static inline void vmathSoaM4MulPerElem( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1 );

/*
 * Compute the absolute value of a 4x4 matrix per element
 */
static inline void vmathSoaM4AbsPerElem( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat );

/*
 * Transpose of a 4x4 matrix
 */
static inline void vmathSoaM4Transpose( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat );

/*
 * Compute the inverse of a 4x4 matrix
 * NOTE: 
 * Result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline void vmathSoaM4Inverse( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat );

/*
 * Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.  The result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline void vmathSoaM4AffineInverse( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat );

/*
 * Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix with an orthogonal upper-left 3x3 submatrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.
 */
static inline void vmathSoaM4OrthoInverse( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat );

/*
 * Determinant of a 4x4 matrix
 */
static inline vec_float4 vmathSoaM4Determinant( const VmathSoaMatrix4 *mat );

/*
 * Conditionally select between two 4x4 matrices
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline void vmathSoaM4Select( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1, vec_uint4 select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 4x4 matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaM4Print( const VmathSoaMatrix4 *mat );

/*
 * Print a 4x4 matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaM4Prints( const VmathSoaMatrix4 *mat, const char *name );

#endif

/*
 * Copy a 3x4 transformation matrix
 */
static inline void vmathSoaT3Copy( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Construct a 3x4 transformation matrix containing the specified columns
 */
static inline void vmathSoaT3MakeFromCols( VmathSoaTransform3 *result, const VmathSoaVector3 *col0, const VmathSoaVector3 *col1, const VmathSoaVector3 *col2, const VmathSoaVector3 *col3 );

/*
 * Construct a 3x4 transformation matrix from a 3x3 matrix and a 3-D vector
 */
static inline void vmathSoaT3MakeFromM3V3( VmathSoaTransform3 *result, const VmathSoaMatrix3 *tfrm, const VmathSoaVector3 *translateVec );

/*
 * Construct a 3x4 transformation matrix from a unit-length quaternion and a 3-D vector
 */
static inline void vmathSoaT3MakeFromQV3( VmathSoaTransform3 *result, const VmathSoaQuat *unitQuat, const VmathSoaVector3 *translateVec );

/*
 * Set all elements of a 3x4 transformation matrix to the same scalar value
 */
static inline void vmathSoaT3MakeFromScalar( VmathSoaTransform3 *result, vec_float4 scalar );

/*
 * Replicate an AoS 3x4 transformation matrix
 */
static inline void vmathSoaT3MakeFromAos( VmathSoaTransform3 *result, const VmathTransform3 *tfrm );

/*
 * Insert four AoS 3x4 transformation matrices
 */
static inline void vmathSoaT3MakeFrom4Aos( VmathSoaTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1, const VmathTransform3 *tfrm2, const VmathTransform3 *tfrm3 );

/*
 * Extract four AoS 3x4 transformation matrices
 */
static inline void vmathSoaT3Get4Aos( const VmathSoaTransform3 *tfrm, VmathTransform3 *result0, VmathTransform3 *result1, VmathTransform3 *result2, VmathTransform3 *result3 );

/*
 * Set the upper-left 3x3 submatrix
 */
static inline void vmathSoaT3SetUpper3x3( VmathSoaTransform3 *result, const VmathSoaMatrix3 *mat3 );

/*
 * Get the upper-left 3x3 submatrix of a 3x4 transformation matrix
 */
static inline void vmathSoaT3GetUpper3x3( VmathSoaMatrix3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Set translation component
 */
static inline void vmathSoaT3SetTranslation( VmathSoaTransform3 *result, const VmathSoaVector3 *translateVec );

/*
 * Get the translation component of a 3x4 transformation matrix
 */
static inline void vmathSoaT3GetTranslation( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Set column 0 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3SetCol0( VmathSoaTransform3 *result, const VmathSoaVector3 *col0 );

/*
 * Set column 1 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3SetCol1( VmathSoaTransform3 *result, const VmathSoaVector3 *col1 );

/*
 * Set column 2 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3SetCol2( VmathSoaTransform3 *result, const VmathSoaVector3 *col2 );

/*
 * Set column 3 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3SetCol3( VmathSoaTransform3 *result, const VmathSoaVector3 *col3 );

/*
 * Get column 0 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3GetCol0( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Get column 1 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3GetCol1( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Get column 2 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3GetCol2( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Get column 3 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3GetCol3( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Set the column of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathSoaT3SetCol( VmathSoaTransform3 *result, int col, const VmathSoaVector3 *vec );

/*
 * Set the row of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathSoaT3SetRow( VmathSoaTransform3 *result, int row, const VmathSoaVector4 *vec );

/*
 * Get the column of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathSoaT3GetCol( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm, int col );

/*
 * Get the row of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathSoaT3GetRow( VmathSoaVector4 *result, const VmathSoaTransform3 *tfrm, int row );

/*
 * Set the element of a 3x4 transformation matrix referred to by column and row indices
 */
static inline void vmathSoaT3SetElem( VmathSoaTransform3 *result, int col, int row, vec_float4 val );

/*
 * Get the element of a 3x4 transformation matrix referred to by column and row indices
 */
static inline vec_float4 vmathSoaT3GetElem( const VmathSoaTransform3 *tfrm, int col, int row );

/*
 * Multiply a 3x4 transformation matrix by a 3-D vector
 */
static inline void vmathSoaT3MulV3( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm, const VmathSoaVector3 *vec );

/*
 * Multiply a 3x4 transformation matrix by a 3-D point
 */
static inline void vmathSoaT3MulP3( VmathSoaPoint3 *result, const VmathSoaTransform3 *tfrm, const VmathSoaPoint3 *pnt );

/*
 * Multiply two 3x4 transformation matrices
 */
static inline void vmathSoaT3Mul( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm0, const VmathSoaTransform3 *tfrm1 );

/*
 * Construct an identity 3x4 transformation matrix
 */
static inline void vmathSoaT3MakeIdentity( VmathSoaTransform3 *result );

/*
 * Construct a 3x4 transformation matrix to rotate around the x axis
 */
static inline void vmathSoaT3MakeRotationX( VmathSoaTransform3 *result, vec_float4 radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the y axis
 */
static inline void vmathSoaT3MakeRotationY( VmathSoaTransform3 *result, vec_float4 radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the z axis
 */
static inline void vmathSoaT3MakeRotationZ( VmathSoaTransform3 *result, vec_float4 radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the x, y, and z axes
 */
static inline void vmathSoaT3MakeRotationZYX( VmathSoaTransform3 *result, const VmathSoaVector3 *radiansXYZ );

/*
 * Construct a 3x4 transformation matrix to rotate around a unit-length 3-D vector
 */
static inline void vmathSoaT3MakeRotationAxis( VmathSoaTransform3 *result, vec_float4 radians, const VmathSoaVector3 *unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline void vmathSoaT3MakeRotationQ( VmathSoaTransform3 *result, const VmathSoaQuat *unitQuat );

/*
 * Construct a 3x4 transformation matrix to perform scaling
 */
static inline void vmathSoaT3MakeScale( VmathSoaTransform3 *result, const VmathSoaVector3 *scaleVec );

/*
 * Construct a 3x4 transformation matrix to perform translation
 */
static inline void vmathSoaT3MakeTranslation( VmathSoaTransform3 *result, const VmathSoaVector3 *translateVec );

/*
 * Append (post-multiply) a scale transformation to a 3x4 transformation matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathSoaT3AppendScale( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm, const VmathSoaVector3 *scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 3x4 transformation matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline void vmathSoaT3PrependScale( VmathSoaTransform3 *result, const VmathSoaVector3 *scaleVec, const VmathSoaTransform3 *tfrm );

/*
 * Multiply two 3x4 transformation matrices per element
 */
static inline void vmathSoaT3MulPerElem( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm0, const VmathSoaTransform3 *tfrm1 );

/*
 * Compute the absolute value of a 3x4 transformation matrix per element
 */
static inline void vmathSoaT3AbsPerElem( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Inverse of a 3x4 transformation matrix
 * NOTE: 
 * Result is unpredictable when the determinant of the left 3x3 submatrix is equal to or near 0.
 */
static inline void vmathSoaT3Inverse( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Compute the inverse of a 3x4 transformation matrix, expected to have an orthogonal upper-left 3x3 submatrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 3x4 transformation matrix meets the given restrictions.
 */
static inline void vmathSoaT3OrthoInverse( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm );

/*
 * Conditionally select between two 3x4 transformation matrices
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline void vmathSoaT3Select( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm0, const VmathSoaTransform3 *tfrm1, vec_uint4 select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3x4 transformation matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaT3Print( const VmathSoaTransform3 *tfrm );

/*
 * Print a 3x4 transformation matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaT3Prints( const VmathSoaTransform3 *tfrm, const char *name );

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#include "vec_soa.h"
#include "quat_soa.h"
#include "mat_soa.h"

#endif
