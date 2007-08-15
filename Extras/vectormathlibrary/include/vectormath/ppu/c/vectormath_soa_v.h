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

#ifndef _VECTORMATH_SOA_C_V_PPU_H
#define _VECTORMATH_SOA_C_V_PPU_H

#include <math.h>
#include <altivec.h>
#include "vectormath_aos_v.h"

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
 * Construct a 3-D vector from x, y, and z elements
 */
static inline VmathSoaVector3 vmathSoaV3MakeFromElems_V( vec_float4 x, vec_float4 y, vec_float4 z );

/*
 * Copy elements from a 3-D point into a 3-D vector
 */
static inline VmathSoaVector3 vmathSoaV3MakeFromP3_V( VmathSoaPoint3 pnt );

/*
 * Set all elements of a 3-D vector to the same scalar value
 */
static inline VmathSoaVector3 vmathSoaV3MakeFromScalar_V( vec_float4 scalar );

/*
 * Replicate an AoS 3-D vector
 */
static inline VmathSoaVector3 vmathSoaV3MakeFromAos_V( VmathVector3 vec );

/*
 * Insert four AoS 3-D vectors
 */
static inline VmathSoaVector3 vmathSoaV3MakeFrom4Aos_V( VmathVector3 vec0, VmathVector3 vec1, VmathVector3 vec2, VmathVector3 vec3 );

/*
 * Extract four AoS 3-D vectors
 */
static inline void vmathSoaV3Get4Aos_V( VmathSoaVector3 vec, VmathVector3 *result0, VmathVector3 *result1, VmathVector3 *result2, VmathVector3 *result3 );

/*
 * Set the x element of a 3-D vector
 */
static inline void vmathSoaV3SetX_V( VmathSoaVector3 *result, vec_float4 x );

/*
 * Set the y element of a 3-D vector
 */
static inline void vmathSoaV3SetY_V( VmathSoaVector3 *result, vec_float4 y );

/*
 * Set the z element of a 3-D vector
 */
static inline void vmathSoaV3SetZ_V( VmathSoaVector3 *result, vec_float4 z );

/*
 * Get the x element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3GetX_V( VmathSoaVector3 vec );

/*
 * Get the y element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3GetY_V( VmathSoaVector3 vec );

/*
 * Get the z element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3GetZ_V( VmathSoaVector3 vec );

/*
 * Set an x, y, or z element of a 3-D vector by index
 */
static inline void vmathSoaV3SetElem_V( VmathSoaVector3 *result, int idx, vec_float4 value );

/*
 * Get an x, y, or z element of a 3-D vector by index
 */
static inline vec_float4 vmathSoaV3GetElem_V( VmathSoaVector3 vec, int idx );

/*
 * Add two 3-D vectors
 */
static inline VmathSoaVector3 vmathSoaV3Add_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Subtract a 3-D vector from another 3-D vector
 */
static inline VmathSoaVector3 vmathSoaV3Sub_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Add a 3-D vector to a 3-D point
 */
static inline VmathSoaPoint3 vmathSoaV3AddP3_V( VmathSoaVector3 vec, VmathSoaPoint3 pnt );

/*
 * Multiply a 3-D vector by a scalar
 */
static inline VmathSoaVector3 vmathSoaV3ScalarMul_V( VmathSoaVector3 vec, vec_float4 scalar );

/*
 * Divide a 3-D vector by a scalar
 */
static inline VmathSoaVector3 vmathSoaV3ScalarDiv_V( VmathSoaVector3 vec, vec_float4 scalar );

/*
 * Negate all elements of a 3-D vector
 */
static inline VmathSoaVector3 vmathSoaV3Neg_V( VmathSoaVector3 vec );

/*
 * Construct x axis
 */
static inline VmathSoaVector3 vmathSoaV3MakeXAxis_V( );

/*
 * Construct y axis
 */
static inline VmathSoaVector3 vmathSoaV3MakeYAxis_V( );

/*
 * Construct z axis
 */
static inline VmathSoaVector3 vmathSoaV3MakeZAxis_V( );

/*
 * Multiply two 3-D vectors per element
 */
static inline VmathSoaVector3 vmathSoaV3MulPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Divide two 3-D vectors per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline VmathSoaVector3 vmathSoaV3DivPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Compute the reciprocal of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline VmathSoaVector3 vmathSoaV3RecipPerElem_V( VmathSoaVector3 vec );

/*
 * Compute the square root of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline VmathSoaVector3 vmathSoaV3SqrtPerElem_V( VmathSoaVector3 vec );

/*
 * Compute the reciprocal square root of a 3-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline VmathSoaVector3 vmathSoaV3RsqrtPerElem_V( VmathSoaVector3 vec );

/*
 * Compute the absolute value of a 3-D vector per element
 */
static inline VmathSoaVector3 vmathSoaV3AbsPerElem_V( VmathSoaVector3 vec );

/*
 * Copy sign from one 3-D vector to another, per element
 */
static inline VmathSoaVector3 vmathSoaV3CopySignPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Maximum of two 3-D vectors per element
 */
static inline VmathSoaVector3 vmathSoaV3MaxPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Minimum of two 3-D vectors per element
 */
static inline VmathSoaVector3 vmathSoaV3MinPerElem_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Maximum element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3MaxElem_V( VmathSoaVector3 vec );

/*
 * Minimum element of a 3-D vector
 */
static inline vec_float4 vmathSoaV3MinElem_V( VmathSoaVector3 vec );

/*
 * Compute the sum of all elements of a 3-D vector
 */
static inline vec_float4 vmathSoaV3Sum_V( VmathSoaVector3 vec );

/*
 * Compute the dot product of two 3-D vectors
 */
static inline vec_float4 vmathSoaV3Dot_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Compute the square of the length of a 3-D vector
 */
static inline vec_float4 vmathSoaV3LengthSqr_V( VmathSoaVector3 vec );

/*
 * Compute the length of a 3-D vector
 */
static inline vec_float4 vmathSoaV3Length_V( VmathSoaVector3 vec );

/*
 * Normalize a 3-D vector
 * NOTE: 
 * The result is unpredictable when all elements of vec are at or near zero.
 */
static inline VmathSoaVector3 vmathSoaV3Normalize_V( VmathSoaVector3 vec );

/*
 * Compute cross product of two 3-D vectors
 */
static inline VmathSoaVector3 vmathSoaV3Cross_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Outer product of two 3-D vectors
 */
static inline VmathSoaMatrix3 vmathSoaV3Outer_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Pre-multiply a row vector by a 3x3 matrix
 */
static inline VmathSoaVector3 vmathSoaV3RowMul_V( VmathSoaVector3 vec, VmathSoaMatrix3 mat );

/*
 * Cross-product matrix of a 3-D vector
 */
static inline VmathSoaMatrix3 vmathSoaV3CrossMatrix_V( VmathSoaVector3 vec );

/*
 * Create cross-product matrix and multiply
 * NOTE: 
 * Faster than separately creating a cross-product matrix and multiplying.
 */
static inline VmathSoaMatrix3 vmathSoaV3CrossMatrixMul_V( VmathSoaVector3 vec, VmathSoaMatrix3 mat );

/*
 * Linear interpolation between two 3-D vectors
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline VmathSoaVector3 vmathSoaV3Lerp_V( vec_float4 t, VmathSoaVector3 vec0, VmathSoaVector3 vec1 );

/*
 * Spherical linear interpolation between two 3-D vectors
 * NOTE: 
 * The result is unpredictable if the vectors point in opposite directions.
 * Does not clamp t between 0 and 1.
 */
static inline VmathSoaVector3 vmathSoaV3Slerp_V( vec_float4 t, VmathSoaVector3 unitVec0, VmathSoaVector3 unitVec1 );

/*
 * Conditionally select between two 3-D vectors
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline VmathSoaVector3 vmathSoaV3Select_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1, vec_uint4 select1 );

/*
 * Load four three-float 3-D vectors, stored in three quadwords
 */
static inline void vmathSoaV3LoadXYZArray_V( VmathSoaVector3 *vec, const vec_float4 *threeQuads );

/*
 * Store four slots of an SoA 3-D vector in three quadwords
 */
static inline void vmathSoaV3StoreXYZArray_V( VmathSoaVector3 vec, vec_float4 *threeQuads );

/*
 * Store eight slots of two SoA 3-D vectors as half-floats
 */
static inline void vmathSoaV3StoreHalfFloats_V( VmathSoaVector3 vec0, VmathSoaVector3 vec1, vec_ushort8 *threeQuads );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3-D vector
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaV3Print_V( VmathSoaVector3 vec );

/*
 * Print a 3-D vector and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaV3Prints_V( VmathSoaVector3 vec, const char *name );

#endif

/*
 * Construct a 4-D vector from x, y, z, and w elements
 */
static inline VmathSoaVector4 vmathSoaV4MakeFromElems_V( vec_float4 x, vec_float4 y, vec_float4 z, vec_float4 w );

/*
 * Construct a 4-D vector from a 3-D vector and a scalar
 */
static inline VmathSoaVector4 vmathSoaV4MakeFromV3Scalar_V( VmathSoaVector3 xyz, vec_float4 w );

/*
 * Copy x, y, and z from a 3-D vector into a 4-D vector, and set w to 0
 */
static inline VmathSoaVector4 vmathSoaV4MakeFromV3_V( VmathSoaVector3 vec );

/*
 * Copy x, y, and z from a 3-D point into a 4-D vector, and set w to 1
 */
static inline VmathSoaVector4 vmathSoaV4MakeFromP3_V( VmathSoaPoint3 pnt );

/*
 * Copy elements from a quaternion into a 4-D vector
 */
static inline VmathSoaVector4 vmathSoaV4MakeFromQ_V( VmathSoaQuat quat );

/*
 * Set all elements of a 4-D vector to the same scalar value
 */
static inline VmathSoaVector4 vmathSoaV4MakeFromScalar_V( vec_float4 scalar );

/*
 * Replicate an AoS 4-D vector
 */
static inline VmathSoaVector4 vmathSoaV4MakeFromAos_V( VmathVector4 vec );

/*
 * Insert four AoS 4-D vectors
 */
static inline VmathSoaVector4 vmathSoaV4MakeFrom4Aos_V( VmathVector4 vec0, VmathVector4 vec1, VmathVector4 vec2, VmathVector4 vec3 );

/*
 * Extract four AoS 4-D vectors
 */
static inline void vmathSoaV4Get4Aos_V( VmathSoaVector4 vec, VmathVector4 *result0, VmathVector4 *result1, VmathVector4 *result2, VmathVector4 *result3 );

/*
 * Set the x, y, and z elements of a 4-D vector
 * NOTE: 
 * This function does not change the w element.
 */
static inline void vmathSoaV4SetXYZ_V( VmathSoaVector4 *result, VmathSoaVector3 vec );

/*
 * Get the x, y, and z elements of a 4-D vector
 */
static inline VmathSoaVector3 vmathSoaV4GetXYZ_V( VmathSoaVector4 vec );

/*
 * Set the x element of a 4-D vector
 */
static inline void vmathSoaV4SetX_V( VmathSoaVector4 *result, vec_float4 x );

/*
 * Set the y element of a 4-D vector
 */
static inline void vmathSoaV4SetY_V( VmathSoaVector4 *result, vec_float4 y );

/*
 * Set the z element of a 4-D vector
 */
static inline void vmathSoaV4SetZ_V( VmathSoaVector4 *result, vec_float4 z );

/*
 * Set the w element of a 4-D vector
 */
static inline void vmathSoaV4SetW_V( VmathSoaVector4 *result, vec_float4 w );

/*
 * Get the x element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4GetX_V( VmathSoaVector4 vec );

/*
 * Get the y element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4GetY_V( VmathSoaVector4 vec );

/*
 * Get the z element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4GetZ_V( VmathSoaVector4 vec );

/*
 * Get the w element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4GetW_V( VmathSoaVector4 vec );

/*
 * Set an x, y, z, or w element of a 4-D vector by index
 */
static inline void vmathSoaV4SetElem_V( VmathSoaVector4 *result, int idx, vec_float4 value );

/*
 * Get an x, y, z, or w element of a 4-D vector by index
 */
static inline vec_float4 vmathSoaV4GetElem_V( VmathSoaVector4 vec, int idx );

/*
 * Add two 4-D vectors
 */
static inline VmathSoaVector4 vmathSoaV4Add_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Subtract a 4-D vector from another 4-D vector
 */
static inline VmathSoaVector4 vmathSoaV4Sub_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Multiply a 4-D vector by a scalar
 */
static inline VmathSoaVector4 vmathSoaV4ScalarMul_V( VmathSoaVector4 vec, vec_float4 scalar );

/*
 * Divide a 4-D vector by a scalar
 */
static inline VmathSoaVector4 vmathSoaV4ScalarDiv_V( VmathSoaVector4 vec, vec_float4 scalar );

/*
 * Negate all elements of a 4-D vector
 */
static inline VmathSoaVector4 vmathSoaV4Neg_V( VmathSoaVector4 vec );

/*
 * Construct x axis
 */
static inline VmathSoaVector4 vmathSoaV4MakeXAxis_V( );

/*
 * Construct y axis
 */
static inline VmathSoaVector4 vmathSoaV4MakeYAxis_V( );

/*
 * Construct z axis
 */
static inline VmathSoaVector4 vmathSoaV4MakeZAxis_V( );

/*
 * Construct w axis
 */
static inline VmathSoaVector4 vmathSoaV4MakeWAxis_V( );

/*
 * Multiply two 4-D vectors per element
 */
static inline VmathSoaVector4 vmathSoaV4MulPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Divide two 4-D vectors per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline VmathSoaVector4 vmathSoaV4DivPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Compute the reciprocal of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline VmathSoaVector4 vmathSoaV4RecipPerElem_V( VmathSoaVector4 vec );

/*
 * Compute the square root of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline VmathSoaVector4 vmathSoaV4SqrtPerElem_V( VmathSoaVector4 vec );

/*
 * Compute the reciprocal square root of a 4-D vector per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline VmathSoaVector4 vmathSoaV4RsqrtPerElem_V( VmathSoaVector4 vec );

/*
 * Compute the absolute value of a 4-D vector per element
 */
static inline VmathSoaVector4 vmathSoaV4AbsPerElem_V( VmathSoaVector4 vec );

/*
 * Copy sign from one 4-D vector to another, per element
 */
static inline VmathSoaVector4 vmathSoaV4CopySignPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Maximum of two 4-D vectors per element
 */
static inline VmathSoaVector4 vmathSoaV4MaxPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Minimum of two 4-D vectors per element
 */
static inline VmathSoaVector4 vmathSoaV4MinPerElem_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Maximum element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4MaxElem_V( VmathSoaVector4 vec );

/*
 * Minimum element of a 4-D vector
 */
static inline vec_float4 vmathSoaV4MinElem_V( VmathSoaVector4 vec );

/*
 * Compute the sum of all elements of a 4-D vector
 */
static inline vec_float4 vmathSoaV4Sum_V( VmathSoaVector4 vec );

/*
 * Compute the dot product of two 4-D vectors
 */
static inline vec_float4 vmathSoaV4Dot_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Compute the square of the length of a 4-D vector
 */
static inline vec_float4 vmathSoaV4LengthSqr_V( VmathSoaVector4 vec );

/*
 * Compute the length of a 4-D vector
 */
static inline vec_float4 vmathSoaV4Length_V( VmathSoaVector4 vec );

/*
 * Normalize a 4-D vector
 * NOTE: 
 * The result is unpredictable when all elements of vec are at or near zero.
 */
static inline VmathSoaVector4 vmathSoaV4Normalize_V( VmathSoaVector4 vec );

/*
 * Outer product of two 4-D vectors
 */
static inline VmathSoaMatrix4 vmathSoaV4Outer_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Linear interpolation between two 4-D vectors
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline VmathSoaVector4 vmathSoaV4Lerp_V( vec_float4 t, VmathSoaVector4 vec0, VmathSoaVector4 vec1 );

/*
 * Spherical linear interpolation between two 4-D vectors
 * NOTE: 
 * The result is unpredictable if the vectors point in opposite directions.
 * Does not clamp t between 0 and 1.
 */
static inline VmathSoaVector4 vmathSoaV4Slerp_V( vec_float4 t, VmathSoaVector4 unitVec0, VmathSoaVector4 unitVec1 );

/*
 * Conditionally select between two 4-D vectors
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline VmathSoaVector4 vmathSoaV4Select_V( VmathSoaVector4 vec0, VmathSoaVector4 vec1, vec_uint4 select1 );

/*
 * Store four slots of an SoA 4-D vector as half-floats
 */
static inline void vmathSoaV4StoreHalfFloats_V( VmathSoaVector4 vec, vec_ushort8 *twoQuads );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 4-D vector
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaV4Print_V( VmathSoaVector4 vec );

/*
 * Print a 4-D vector and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaV4Prints_V( VmathSoaVector4 vec, const char *name );

#endif

/*
 * Construct a 3-D point from x, y, and z elements
 */
static inline VmathSoaPoint3 vmathSoaP3MakeFromElems_V( vec_float4 x, vec_float4 y, vec_float4 z );

/*
 * Copy elements from a 3-D vector into a 3-D point
 */
static inline VmathSoaPoint3 vmathSoaP3MakeFromV3_V( VmathSoaVector3 vec );

/*
 * Set all elements of a 3-D point to the same scalar value
 */
static inline VmathSoaPoint3 vmathSoaP3MakeFromScalar_V( vec_float4 scalar );

/*
 * Replicate an AoS 3-D point
 */
static inline VmathSoaPoint3 vmathSoaP3MakeFromAos_V( VmathPoint3 pnt );

/*
 * Insert four AoS 3-D points
 */
static inline VmathSoaPoint3 vmathSoaP3MakeFrom4Aos_V( VmathPoint3 pnt0, VmathPoint3 pnt1, VmathPoint3 pnt2, VmathPoint3 pnt3 );

/*
 * Extract four AoS 3-D points
 */
static inline void vmathSoaP3Get4Aos_V( VmathSoaPoint3 pnt, VmathPoint3 *result0, VmathPoint3 *result1, VmathPoint3 *result2, VmathPoint3 *result3 );

/*
 * Set the x element of a 3-D point
 */
static inline void vmathSoaP3SetX_V( VmathSoaPoint3 *result, vec_float4 x );

/*
 * Set the y element of a 3-D point
 */
static inline void vmathSoaP3SetY_V( VmathSoaPoint3 *result, vec_float4 y );

/*
 * Set the z element of a 3-D point
 */
static inline void vmathSoaP3SetZ_V( VmathSoaPoint3 *result, vec_float4 z );

/*
 * Get the x element of a 3-D point
 */
static inline vec_float4 vmathSoaP3GetX_V( VmathSoaPoint3 pnt );

/*
 * Get the y element of a 3-D point
 */
static inline vec_float4 vmathSoaP3GetY_V( VmathSoaPoint3 pnt );

/*
 * Get the z element of a 3-D point
 */
static inline vec_float4 vmathSoaP3GetZ_V( VmathSoaPoint3 pnt );

/*
 * Set an x, y, or z element of a 3-D point by index
 */
static inline void vmathSoaP3SetElem_V( VmathSoaPoint3 *result, int idx, vec_float4 value );

/*
 * Get an x, y, or z element of a 3-D point by index
 */
static inline vec_float4 vmathSoaP3GetElem_V( VmathSoaPoint3 pnt, int idx );

/*
 * Subtract a 3-D point from another 3-D point
 */
static inline VmathSoaVector3 vmathSoaP3Sub_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Add a 3-D point to a 3-D vector
 */
static inline VmathSoaPoint3 vmathSoaP3AddV3_V( VmathSoaPoint3 pnt, VmathSoaVector3 vec );

/*
 * Subtract a 3-D vector from a 3-D point
 */
static inline VmathSoaPoint3 vmathSoaP3SubV3_V( VmathSoaPoint3 pnt, VmathSoaVector3 vec );

/*
 * Multiply two 3-D points per element
 */
static inline VmathSoaPoint3 vmathSoaP3MulPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Divide two 3-D points per element
 * NOTE: 
 * Floating-point behavior matches standard library function divf4.
 */
static inline VmathSoaPoint3 vmathSoaP3DivPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Compute the reciprocal of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function recipf4.
 */
static inline VmathSoaPoint3 vmathSoaP3RecipPerElem_V( VmathSoaPoint3 pnt );

/*
 * Compute the square root of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function sqrtf4.
 */
static inline VmathSoaPoint3 vmathSoaP3SqrtPerElem_V( VmathSoaPoint3 pnt );

/*
 * Compute the reciprocal square root of a 3-D point per element
 * NOTE: 
 * Floating-point behavior matches standard library function rsqrtf4.
 */
static inline VmathSoaPoint3 vmathSoaP3RsqrtPerElem_V( VmathSoaPoint3 pnt );

/*
 * Compute the absolute value of a 3-D point per element
 */
static inline VmathSoaPoint3 vmathSoaP3AbsPerElem_V( VmathSoaPoint3 pnt );

/*
 * Copy sign from one 3-D point to another, per element
 */
static inline VmathSoaPoint3 vmathSoaP3CopySignPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Maximum of two 3-D points per element
 */
static inline VmathSoaPoint3 vmathSoaP3MaxPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Minimum of two 3-D points per element
 */
static inline VmathSoaPoint3 vmathSoaP3MinPerElem_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Maximum element of a 3-D point
 */
static inline vec_float4 vmathSoaP3MaxElem_V( VmathSoaPoint3 pnt );

/*
 * Minimum element of a 3-D point
 */
static inline vec_float4 vmathSoaP3MinElem_V( VmathSoaPoint3 pnt );

/*
 * Compute the sum of all elements of a 3-D point
 */
static inline vec_float4 vmathSoaP3Sum_V( VmathSoaPoint3 pnt );

/*
 * Apply uniform scale to a 3-D point
 */
static inline VmathSoaPoint3 vmathSoaP3Scale_V( VmathSoaPoint3 pnt, vec_float4 scaleVal );

/*
 * Apply non-uniform scale to a 3-D point
 */
static inline VmathSoaPoint3 vmathSoaP3NonUniformScale_V( VmathSoaPoint3 pnt, VmathSoaVector3 scaleVec );

/*
 * Scalar projection of a 3-D point on a unit-length 3-D vector
 */
static inline vec_float4 vmathSoaP3Projection_V( VmathSoaPoint3 pnt, VmathSoaVector3 unitVec );

/*
 * Compute the square of the distance of a 3-D point from the coordinate-system origin
 */
static inline vec_float4 vmathSoaP3DistSqrFromOrigin_V( VmathSoaPoint3 pnt );

/*
 * Compute the distance of a 3-D point from the coordinate-system origin
 */
static inline vec_float4 vmathSoaP3DistFromOrigin_V( VmathSoaPoint3 pnt );

/*
 * Compute the square of the distance between two 3-D points
 */
static inline vec_float4 vmathSoaP3DistSqr_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Compute the distance between two 3-D points
 */
static inline vec_float4 vmathSoaP3Dist_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Linear interpolation between two 3-D points
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline VmathSoaPoint3 vmathSoaP3Lerp_V( vec_float4 t, VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1 );

/*
 * Conditionally select between two 3-D points
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline VmathSoaPoint3 vmathSoaP3Select_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1, vec_uint4 select1 );

/*
 * Load four three-float 3-D points, stored in three quadwords
 */
static inline void vmathSoaP3LoadXYZArray_V( VmathSoaPoint3 *pnt, const vec_float4 *threeQuads );

/*
 * Store four slots of an SoA 3-D point in three quadwords
 */
static inline void vmathSoaP3StoreXYZArray_V( VmathSoaPoint3 pnt, vec_float4 *threeQuads );

/*
 * Store eight slots of two SoA 3-D points as half-floats
 */
static inline void vmathSoaP3StoreHalfFloats_V( VmathSoaPoint3 pnt0, VmathSoaPoint3 pnt1, vec_ushort8 *threeQuads );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3-D point
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaP3Print_V( VmathSoaPoint3 pnt );

/*
 * Print a 3-D point and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaP3Prints_V( VmathSoaPoint3 pnt, const char *name );

#endif

/*
 * Construct a quaternion from x, y, z, and w elements
 */
static inline VmathSoaQuat vmathSoaQMakeFromElems_V( vec_float4 x, vec_float4 y, vec_float4 z, vec_float4 w );

/*
 * Construct a quaternion from a 3-D vector and a scalar
 */
static inline VmathSoaQuat vmathSoaQMakeFromV3Scalar_V( VmathSoaVector3 xyz, vec_float4 w );

/*
 * Copy elements from a 4-D vector into a quaternion
 */
static inline VmathSoaQuat vmathSoaQMakeFromV4_V( VmathSoaVector4 vec );

/*
 * Convert a rotation matrix to a unit-length quaternion
 */
static inline VmathSoaQuat vmathSoaQMakeFromM3_V( VmathSoaMatrix3 rotMat );

/*
 * Set all elements of a quaternion to the same scalar value
 */
static inline VmathSoaQuat vmathSoaQMakeFromScalar_V( vec_float4 scalar );

/*
 * Replicate an AoS quaternion
 */
static inline VmathSoaQuat vmathSoaQMakeFromAos_V( VmathQuat quat );

/*
 * Insert four AoS quaternions
 */
static inline VmathSoaQuat vmathSoaQMakeFrom4Aos_V( VmathQuat quat0, VmathQuat quat1, VmathQuat quat2, VmathQuat quat3 );

/*
 * Extract four AoS quaternions
 */
static inline void vmathSoaQGet4Aos_V( VmathSoaQuat quat, VmathQuat *result0, VmathQuat *result1, VmathQuat *result2, VmathQuat *result3 );

/*
 * Set the x, y, and z elements of a quaternion
 * NOTE: 
 * This function does not change the w element.
 */
static inline void vmathSoaQSetXYZ_V( VmathSoaQuat *result, VmathSoaVector3 vec );

/*
 * Get the x, y, and z elements of a quaternion
 */
static inline VmathSoaVector3 vmathSoaQGetXYZ_V( VmathSoaQuat quat );

/*
 * Set the x element of a quaternion
 */
static inline void vmathSoaQSetX_V( VmathSoaQuat *result, vec_float4 x );

/*
 * Set the y element of a quaternion
 */
static inline void vmathSoaQSetY_V( VmathSoaQuat *result, vec_float4 y );

/*
 * Set the z element of a quaternion
 */
static inline void vmathSoaQSetZ_V( VmathSoaQuat *result, vec_float4 z );

/*
 * Set the w element of a quaternion
 */
static inline void vmathSoaQSetW_V( VmathSoaQuat *result, vec_float4 w );

/*
 * Get the x element of a quaternion
 */
static inline vec_float4 vmathSoaQGetX_V( VmathSoaQuat quat );

/*
 * Get the y element of a quaternion
 */
static inline vec_float4 vmathSoaQGetY_V( VmathSoaQuat quat );

/*
 * Get the z element of a quaternion
 */
static inline vec_float4 vmathSoaQGetZ_V( VmathSoaQuat quat );

/*
 * Get the w element of a quaternion
 */
static inline vec_float4 vmathSoaQGetW_V( VmathSoaQuat quat );

/*
 * Set an x, y, z, or w element of a quaternion by index
 */
static inline void vmathSoaQSetElem_V( VmathSoaQuat *result, int idx, vec_float4 value );

/*
 * Get an x, y, z, or w element of a quaternion by index
 */
static inline vec_float4 vmathSoaQGetElem_V( VmathSoaQuat quat, int idx );

/*
 * Add two quaternions
 */
static inline VmathSoaQuat vmathSoaQAdd_V( VmathSoaQuat quat0, VmathSoaQuat quat1 );

/*
 * Subtract a quaternion from another quaternion
 */
static inline VmathSoaQuat vmathSoaQSub_V( VmathSoaQuat quat0, VmathSoaQuat quat1 );

/*
 * Multiply two quaternions
 */
static inline VmathSoaQuat vmathSoaQMul_V( VmathSoaQuat quat0, VmathSoaQuat quat1 );

/*
 * Multiply a quaternion by a scalar
 */
static inline VmathSoaQuat vmathSoaQScalarMul_V( VmathSoaQuat quat, vec_float4 scalar );

/*
 * Divide a quaternion by a scalar
 */
static inline VmathSoaQuat vmathSoaQScalarDiv_V( VmathSoaQuat quat, vec_float4 scalar );

/*
 * Negate all elements of a quaternion
 */
static inline VmathSoaQuat vmathSoaQNeg_V( VmathSoaQuat quat );

/*
 * Construct an identity quaternion
 */
static inline VmathSoaQuat vmathSoaQMakeIdentity_V( );

/*
 * Construct a quaternion to rotate between two unit-length 3-D vectors
 * NOTE: 
 * The result is unpredictable if unitVec0 and unitVec1 point in opposite directions.
 */
static inline VmathSoaQuat vmathSoaQMakeRotationArc_V( VmathSoaVector3 unitVec0, VmathSoaVector3 unitVec1 );

/*
 * Construct a quaternion to rotate around a unit-length 3-D vector
 */
static inline VmathSoaQuat vmathSoaQMakeRotationAxis_V( vec_float4 radians, VmathSoaVector3 unitVec );

/*
 * Construct a quaternion to rotate around the x axis
 */
static inline VmathSoaQuat vmathSoaQMakeRotationX_V( vec_float4 radians );

/*
 * Construct a quaternion to rotate around the y axis
 */
static inline VmathSoaQuat vmathSoaQMakeRotationY_V( vec_float4 radians );

/*
 * Construct a quaternion to rotate around the z axis
 */
static inline VmathSoaQuat vmathSoaQMakeRotationZ_V( vec_float4 radians );

/*
 * Compute the conjugate of a quaternion
 */
static inline VmathSoaQuat vmathSoaQConj_V( VmathSoaQuat quat );

/*
 * Use a unit-length quaternion to rotate a 3-D vector
 */
static inline VmathSoaVector3 vmathSoaQRotate_V( VmathSoaQuat unitQuat, VmathSoaVector3 vec );

/*
 * Compute the dot product of two quaternions
 */
static inline vec_float4 vmathSoaQDot_V( VmathSoaQuat quat0, VmathSoaQuat quat1 );

/*
 * Compute the norm of a quaternion
 */
static inline vec_float4 vmathSoaQNorm_V( VmathSoaQuat quat );

/*
 * Compute the length of a quaternion
 */
static inline vec_float4 vmathSoaQLength_V( VmathSoaQuat quat );

/*
 * Normalize a quaternion
 * NOTE: 
 * The result is unpredictable when all elements of quat are at or near zero.
 */
static inline VmathSoaQuat vmathSoaQNormalize_V( VmathSoaQuat quat );

/*
 * Linear interpolation between two quaternions
 * NOTE: 
 * Does not clamp t between 0 and 1.
 */
static inline VmathSoaQuat vmathSoaQLerp_V( vec_float4 t, VmathSoaQuat quat0, VmathSoaQuat quat1 );

/*
 * Spherical linear interpolation between two quaternions
 * NOTE: 
 * Interpolates along the shortest path between orientations.
 * Does not clamp t between 0 and 1.
 */
static inline VmathSoaQuat vmathSoaQSlerp_V( vec_float4 t, VmathSoaQuat unitQuat0, VmathSoaQuat unitQuat1 );

/*
 * Spherical quadrangle interpolation
 */
static inline VmathSoaQuat vmathSoaQSquad_V( vec_float4 t, VmathSoaQuat unitQuat0, VmathSoaQuat unitQuat1, VmathSoaQuat unitQuat2, VmathSoaQuat unitQuat3 );

/*
 * Conditionally select between two quaternions
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline VmathSoaQuat vmathSoaQSelect_V( VmathSoaQuat quat0, VmathSoaQuat quat1, vec_uint4 select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a quaternion
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaQPrint_V( VmathSoaQuat quat );

/*
 * Print a quaternion and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaQPrints_V( VmathSoaQuat quat, const char *name );

#endif

/*
 * Construct a 3x3 matrix containing the specified columns
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeFromCols_V( VmathSoaVector3 col0, VmathSoaVector3 col1, VmathSoaVector3 col2 );

/*
 * Construct a 3x3 rotation matrix from a unit-length quaternion
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeFromQ_V( VmathSoaQuat unitQuat );

/*
 * Set all elements of a 3x3 matrix to the same scalar value
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeFromScalar_V( vec_float4 scalar );

/*
 * Replicate an AoS 3x3 matrix
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeFromAos_V( VmathMatrix3 mat );

/*
 * Insert four AoS 3x3 matrices
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeFrom4Aos_V( VmathMatrix3 mat0, VmathMatrix3 mat1, VmathMatrix3 mat2, VmathMatrix3 mat3 );

/*
 * Extract four AoS 3x3 matrices
 */
static inline void vmathSoaM3Get4Aos_V( VmathSoaMatrix3 mat, VmathMatrix3 *result0, VmathMatrix3 *result1, VmathMatrix3 *result2, VmathMatrix3 *result3 );

/*
 * Set column 0 of a 3x3 matrix
 */
static inline void vmathSoaM3SetCol0_V( VmathSoaMatrix3 *result, VmathSoaVector3 col0 );

/*
 * Set column 1 of a 3x3 matrix
 */
static inline void vmathSoaM3SetCol1_V( VmathSoaMatrix3 *result, VmathSoaVector3 col1 );

/*
 * Set column 2 of a 3x3 matrix
 */
static inline void vmathSoaM3SetCol2_V( VmathSoaMatrix3 *result, VmathSoaVector3 col2 );

/*
 * Get column 0 of a 3x3 matrix
 */
static inline VmathSoaVector3 vmathSoaM3GetCol0_V( VmathSoaMatrix3 mat );

/*
 * Get column 1 of a 3x3 matrix
 */
static inline VmathSoaVector3 vmathSoaM3GetCol1_V( VmathSoaMatrix3 mat );

/*
 * Get column 2 of a 3x3 matrix
 */
static inline VmathSoaVector3 vmathSoaM3GetCol2_V( VmathSoaMatrix3 mat );

/*
 * Set the column of a 3x3 matrix referred to by the specified index
 */
static inline void vmathSoaM3SetCol_V( VmathSoaMatrix3 *result, int col, VmathSoaVector3 vec );

/*
 * Set the row of a 3x3 matrix referred to by the specified index
 */
static inline void vmathSoaM3SetRow_V( VmathSoaMatrix3 *result, int row, VmathSoaVector3 vec );

/*
 * Get the column of a 3x3 matrix referred to by the specified index
 */
static inline VmathSoaVector3 vmathSoaM3GetCol_V( VmathSoaMatrix3 mat, int col );

/*
 * Get the row of a 3x3 matrix referred to by the specified index
 */
static inline VmathSoaVector3 vmathSoaM3GetRow_V( VmathSoaMatrix3 mat, int row );

/*
 * Set the element of a 3x3 matrix referred to by column and row indices
 */
static inline void vmathSoaM3SetElem_V( VmathSoaMatrix3 *result, int col, int row, vec_float4 val );

/*
 * Get the element of a 3x3 matrix referred to by column and row indices
 */
static inline vec_float4 vmathSoaM3GetElem_V( VmathSoaMatrix3 mat, int col, int row );

/*
 * Add two 3x3 matrices
 */
static inline VmathSoaMatrix3 vmathSoaM3Add_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1 );

/*
 * Subtract a 3x3 matrix from another 3x3 matrix
 */
static inline VmathSoaMatrix3 vmathSoaM3Sub_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1 );

/*
 * Negate all elements of a 3x3 matrix
 */
static inline VmathSoaMatrix3 vmathSoaM3Neg_V( VmathSoaMatrix3 mat );

/*
 * Multiply a 3x3 matrix by a scalar
 */
static inline VmathSoaMatrix3 vmathSoaM3ScalarMul_V( VmathSoaMatrix3 mat, vec_float4 scalar );

/*
 * Multiply a 3x3 matrix by a 3-D vector
 */
static inline VmathSoaVector3 vmathSoaM3MulV3_V( VmathSoaMatrix3 mat, VmathSoaVector3 vec );

/*
 * Multiply two 3x3 matrices
 */
static inline VmathSoaMatrix3 vmathSoaM3Mul_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1 );

/*
 * Construct an identity 3x3 matrix
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeIdentity_V( );

/*
 * Construct a 3x3 matrix to rotate around the x axis
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeRotationX_V( vec_float4 radians );

/*
 * Construct a 3x3 matrix to rotate around the y axis
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeRotationY_V( vec_float4 radians );

/*
 * Construct a 3x3 matrix to rotate around the z axis
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeRotationZ_V( vec_float4 radians );

/*
 * Construct a 3x3 matrix to rotate around the x, y, and z axes
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeRotationZYX_V( VmathSoaVector3 radiansXYZ );

/*
 * Construct a 3x3 matrix to rotate around a unit-length 3-D vector
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeRotationAxis_V( vec_float4 radians, VmathSoaVector3 unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeRotationQ_V( VmathSoaQuat unitQuat );

/*
 * Construct a 3x3 matrix to perform scaling
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeScale_V( VmathSoaVector3 scaleVec );

/*
 * Append (post-multiply) a scale transformation to a 3x3 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathSoaMatrix3 vmathSoaM3AppendScale_V( VmathSoaMatrix3 mat, VmathSoaVector3 scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 3x3 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathSoaMatrix3 vmathSoaM3PrependScale_V( VmathSoaVector3 scaleVec, VmathSoaMatrix3 mat );

/*
 * Multiply two 3x3 matrices per element
 */
static inline VmathSoaMatrix3 vmathSoaM3MulPerElem_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1 );

/*
 * Compute the absolute value of a 3x3 matrix per element
 */
static inline VmathSoaMatrix3 vmathSoaM3AbsPerElem_V( VmathSoaMatrix3 mat );

/*
 * Transpose of a 3x3 matrix
 */
static inline VmathSoaMatrix3 vmathSoaM3Transpose_V( VmathSoaMatrix3 mat );

/*
 * Compute the inverse of a 3x3 matrix
 * NOTE: 
 * Result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline VmathSoaMatrix3 vmathSoaM3Inverse_V( VmathSoaMatrix3 mat );

/*
 * Determinant of a 3x3 matrix
 */
static inline vec_float4 vmathSoaM3Determinant_V( VmathSoaMatrix3 mat );

/*
 * Conditionally select between two 3x3 matrices
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline VmathSoaMatrix3 vmathSoaM3Select_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1, vec_uint4 select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3x3 matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaM3Print_V( VmathSoaMatrix3 mat );

/*
 * Print a 3x3 matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaM3Prints_V( VmathSoaMatrix3 mat, const char *name );

#endif

/*
 * Construct a 4x4 matrix containing the specified columns
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeFromCols_V( VmathSoaVector4 col0, VmathSoaVector4 col1, VmathSoaVector4 col2, VmathSoaVector4 col3 );

/*
 * Construct a 4x4 matrix from a 3x4 transformation matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeFromT3_V( VmathSoaTransform3 mat );

/*
 * Construct a 4x4 matrix from a 3x3 matrix and a 3-D vector
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeFromM3V3_V( VmathSoaMatrix3 mat, VmathSoaVector3 translateVec );

/*
 * Construct a 4x4 matrix from a unit-length quaternion and a 3-D vector
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeFromQV3_V( VmathSoaQuat unitQuat, VmathSoaVector3 translateVec );

/*
 * Set all elements of a 4x4 matrix to the same scalar value
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeFromScalar_V( vec_float4 scalar );

/*
 * Replicate an AoS 4x4 matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeFromAos_V( VmathMatrix4 mat );

/*
 * Insert four AoS 4x4 matrices
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeFrom4Aos_V( VmathMatrix4 mat0, VmathMatrix4 mat1, VmathMatrix4 mat2, VmathMatrix4 mat3 );

/*
 * Extract four AoS 4x4 matrices
 */
static inline void vmathSoaM4Get4Aos_V( VmathSoaMatrix4 mat, VmathMatrix4 *result0, VmathMatrix4 *result1, VmathMatrix4 *result2, VmathMatrix4 *result3 );

/*
 * Set the upper-left 3x3 submatrix
 * NOTE: 
 * This function does not change the bottom row elements.
 */
static inline void vmathSoaM4SetUpper3x3_V( VmathSoaMatrix4 *result, VmathSoaMatrix3 mat3 );

/*
 * Get the upper-left 3x3 submatrix of a 4x4 matrix
 */
static inline VmathSoaMatrix3 vmathSoaM4GetUpper3x3_V( VmathSoaMatrix4 mat );

/*
 * Set translation component
 * NOTE: 
 * This function does not change the bottom row elements.
 */
static inline void vmathSoaM4SetTranslation_V( VmathSoaMatrix4 *result, VmathSoaVector3 translateVec );

/*
 * Get the translation component of a 4x4 matrix
 */
static inline VmathSoaVector3 vmathSoaM4GetTranslation_V( VmathSoaMatrix4 mat );

/*
 * Set column 0 of a 4x4 matrix
 */
static inline void vmathSoaM4SetCol0_V( VmathSoaMatrix4 *result, VmathSoaVector4 col0 );

/*
 * Set column 1 of a 4x4 matrix
 */
static inline void vmathSoaM4SetCol1_V( VmathSoaMatrix4 *result, VmathSoaVector4 col1 );

/*
 * Set column 2 of a 4x4 matrix
 */
static inline void vmathSoaM4SetCol2_V( VmathSoaMatrix4 *result, VmathSoaVector4 col2 );

/*
 * Set column 3 of a 4x4 matrix
 */
static inline void vmathSoaM4SetCol3_V( VmathSoaMatrix4 *result, VmathSoaVector4 col3 );

/*
 * Get column 0 of a 4x4 matrix
 */
static inline VmathSoaVector4 vmathSoaM4GetCol0_V( VmathSoaMatrix4 mat );

/*
 * Get column 1 of a 4x4 matrix
 */
static inline VmathSoaVector4 vmathSoaM4GetCol1_V( VmathSoaMatrix4 mat );

/*
 * Get column 2 of a 4x4 matrix
 */
static inline VmathSoaVector4 vmathSoaM4GetCol2_V( VmathSoaMatrix4 mat );

/*
 * Get column 3 of a 4x4 matrix
 */
static inline VmathSoaVector4 vmathSoaM4GetCol3_V( VmathSoaMatrix4 mat );

/*
 * Set the column of a 4x4 matrix referred to by the specified index
 */
static inline void vmathSoaM4SetCol_V( VmathSoaMatrix4 *result, int col, VmathSoaVector4 vec );

/*
 * Set the row of a 4x4 matrix referred to by the specified index
 */
static inline void vmathSoaM4SetRow_V( VmathSoaMatrix4 *result, int row, VmathSoaVector4 vec );

/*
 * Get the column of a 4x4 matrix referred to by the specified index
 */
static inline VmathSoaVector4 vmathSoaM4GetCol_V( VmathSoaMatrix4 mat, int col );

/*
 * Get the row of a 4x4 matrix referred to by the specified index
 */
static inline VmathSoaVector4 vmathSoaM4GetRow_V( VmathSoaMatrix4 mat, int row );

/*
 * Set the element of a 4x4 matrix referred to by column and row indices
 */
static inline void vmathSoaM4SetElem_V( VmathSoaMatrix4 *result, int col, int row, vec_float4 val );

/*
 * Get the element of a 4x4 matrix referred to by column and row indices
 */
static inline vec_float4 vmathSoaM4GetElem_V( VmathSoaMatrix4 mat, int col, int row );

/*
 * Add two 4x4 matrices
 */
static inline VmathSoaMatrix4 vmathSoaM4Add_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1 );

/*
 * Subtract a 4x4 matrix from another 4x4 matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4Sub_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1 );

/*
 * Negate all elements of a 4x4 matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4Neg_V( VmathSoaMatrix4 mat );

/*
 * Multiply a 4x4 matrix by a scalar
 */
static inline VmathSoaMatrix4 vmathSoaM4ScalarMul_V( VmathSoaMatrix4 mat, vec_float4 scalar );

/*
 * Multiply a 4x4 matrix by a 4-D vector
 */
static inline VmathSoaVector4 vmathSoaM4MulV4_V( VmathSoaMatrix4 mat, VmathSoaVector4 vec );

/*
 * Multiply a 4x4 matrix by a 3-D vector
 */
static inline VmathSoaVector4 vmathSoaM4MulV3_V( VmathSoaMatrix4 mat, VmathSoaVector3 vec );

/*
 * Multiply a 4x4 matrix by a 3-D point
 */
static inline VmathSoaVector4 vmathSoaM4MulP3_V( VmathSoaMatrix4 mat, VmathSoaPoint3 pnt );

/*
 * Multiply two 4x4 matrices
 */
static inline VmathSoaMatrix4 vmathSoaM4Mul_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1 );

/*
 * Multiply a 4x4 matrix by a 3x4 transformation matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4MulT3_V( VmathSoaMatrix4 mat, VmathSoaTransform3 tfrm );

/*
 * Construct an identity 4x4 matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeIdentity_V( );

/*
 * Construct a 4x4 matrix to rotate around the x axis
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeRotationX_V( vec_float4 radians );

/*
 * Construct a 4x4 matrix to rotate around the y axis
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeRotationY_V( vec_float4 radians );

/*
 * Construct a 4x4 matrix to rotate around the z axis
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeRotationZ_V( vec_float4 radians );

/*
 * Construct a 4x4 matrix to rotate around the x, y, and z axes
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeRotationZYX_V( VmathSoaVector3 radiansXYZ );

/*
 * Construct a 4x4 matrix to rotate around a unit-length 3-D vector
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeRotationAxis_V( vec_float4 radians, VmathSoaVector3 unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeRotationQ_V( VmathSoaQuat unitQuat );

/*
 * Construct a 4x4 matrix to perform scaling
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeScale_V( VmathSoaVector3 scaleVec );

/*
 * Construct a 4x4 matrix to perform translation
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeTranslation_V( VmathSoaVector3 translateVec );

/*
 * Construct viewing matrix based on eye position, position looked at, and up direction
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeLookAt_V( VmathSoaPoint3 eyePos, VmathSoaPoint3 lookAtPos, VmathSoaVector3 upVec );

/*
 * Construct a perspective projection matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4MakePerspective_V( vec_float4 fovyRadians, vec_float4 aspect, vec_float4 zNear, vec_float4 zFar );

/*
 * Construct a perspective projection matrix based on frustum
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeFrustum_V( vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar );

/*
 * Construct an orthographic projection matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4MakeOrthographic_V( vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar );

/*
 * Append (post-multiply) a scale transformation to a 4x4 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathSoaMatrix4 vmathSoaM4AppendScale_V( VmathSoaMatrix4 mat, VmathSoaVector3 scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 4x4 matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathSoaMatrix4 vmathSoaM4PrependScale_V( VmathSoaVector3 scaleVec, VmathSoaMatrix4 mat );

/*
 * Multiply two 4x4 matrices per element
 */
static inline VmathSoaMatrix4 vmathSoaM4MulPerElem_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1 );

/*
 * Compute the absolute value of a 4x4 matrix per element
 */
static inline VmathSoaMatrix4 vmathSoaM4AbsPerElem_V( VmathSoaMatrix4 mat );

/*
 * Transpose of a 4x4 matrix
 */
static inline VmathSoaMatrix4 vmathSoaM4Transpose_V( VmathSoaMatrix4 mat );

/*
 * Compute the inverse of a 4x4 matrix
 * NOTE: 
 * Result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline VmathSoaMatrix4 vmathSoaM4Inverse_V( VmathSoaMatrix4 mat );

/*
 * Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.  The result is unpredictable when the determinant of mat is equal to or near 0.
 */
static inline VmathSoaMatrix4 vmathSoaM4AffineInverse_V( VmathSoaMatrix4 mat );

/*
 * Compute the inverse of a 4x4 matrix, which is expected to be an affine matrix with an orthogonal upper-left 3x3 submatrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 4x4 matrix meets the given restrictions.
 */
static inline VmathSoaMatrix4 vmathSoaM4OrthoInverse_V( VmathSoaMatrix4 mat );

/*
 * Determinant of a 4x4 matrix
 */
static inline vec_float4 vmathSoaM4Determinant_V( VmathSoaMatrix4 mat );

/*
 * Conditionally select between two 4x4 matrices
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline VmathSoaMatrix4 vmathSoaM4Select_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1, vec_uint4 select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 4x4 matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaM4Print_V( VmathSoaMatrix4 mat );

/*
 * Print a 4x4 matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaM4Prints_V( VmathSoaMatrix4 mat, const char *name );

#endif

/*
 * Construct a 3x4 transformation matrix containing the specified columns
 */
static inline VmathSoaTransform3 vmathSoaT3MakeFromCols_V( VmathSoaVector3 col0, VmathSoaVector3 col1, VmathSoaVector3 col2, VmathSoaVector3 col3 );

/*
 * Construct a 3x4 transformation matrix from a 3x3 matrix and a 3-D vector
 */
static inline VmathSoaTransform3 vmathSoaT3MakeFromM3V3_V( VmathSoaMatrix3 tfrm, VmathSoaVector3 translateVec );

/*
 * Construct a 3x4 transformation matrix from a unit-length quaternion and a 3-D vector
 */
static inline VmathSoaTransform3 vmathSoaT3MakeFromQV3_V( VmathSoaQuat unitQuat, VmathSoaVector3 translateVec );

/*
 * Set all elements of a 3x4 transformation matrix to the same scalar value
 */
static inline VmathSoaTransform3 vmathSoaT3MakeFromScalar_V( vec_float4 scalar );

/*
 * Replicate an AoS 3x4 transformation matrix
 */
static inline VmathSoaTransform3 vmathSoaT3MakeFromAos_V( VmathTransform3 tfrm );

/*
 * Insert four AoS 3x4 transformation matrices
 */
static inline VmathSoaTransform3 vmathSoaT3MakeFrom4Aos_V( VmathTransform3 tfrm0, VmathTransform3 tfrm1, VmathTransform3 tfrm2, VmathTransform3 tfrm3 );

/*
 * Extract four AoS 3x4 transformation matrices
 */
static inline void vmathSoaT3Get4Aos_V( VmathSoaTransform3 tfrm, VmathTransform3 *result0, VmathTransform3 *result1, VmathTransform3 *result2, VmathTransform3 *result3 );

/*
 * Set the upper-left 3x3 submatrix
 */
static inline void vmathSoaT3SetUpper3x3_V( VmathSoaTransform3 *result, VmathSoaMatrix3 mat3 );

/*
 * Get the upper-left 3x3 submatrix of a 3x4 transformation matrix
 */
static inline VmathSoaMatrix3 vmathSoaT3GetUpper3x3_V( VmathSoaTransform3 tfrm );

/*
 * Set translation component
 */
static inline void vmathSoaT3SetTranslation_V( VmathSoaTransform3 *result, VmathSoaVector3 translateVec );

/*
 * Get the translation component of a 3x4 transformation matrix
 */
static inline VmathSoaVector3 vmathSoaT3GetTranslation_V( VmathSoaTransform3 tfrm );

/*
 * Set column 0 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3SetCol0_V( VmathSoaTransform3 *result, VmathSoaVector3 col0 );

/*
 * Set column 1 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3SetCol1_V( VmathSoaTransform3 *result, VmathSoaVector3 col1 );

/*
 * Set column 2 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3SetCol2_V( VmathSoaTransform3 *result, VmathSoaVector3 col2 );

/*
 * Set column 3 of a 3x4 transformation matrix
 */
static inline void vmathSoaT3SetCol3_V( VmathSoaTransform3 *result, VmathSoaVector3 col3 );

/*
 * Get column 0 of a 3x4 transformation matrix
 */
static inline VmathSoaVector3 vmathSoaT3GetCol0_V( VmathSoaTransform3 tfrm );

/*
 * Get column 1 of a 3x4 transformation matrix
 */
static inline VmathSoaVector3 vmathSoaT3GetCol1_V( VmathSoaTransform3 tfrm );

/*
 * Get column 2 of a 3x4 transformation matrix
 */
static inline VmathSoaVector3 vmathSoaT3GetCol2_V( VmathSoaTransform3 tfrm );

/*
 * Get column 3 of a 3x4 transformation matrix
 */
static inline VmathSoaVector3 vmathSoaT3GetCol3_V( VmathSoaTransform3 tfrm );

/*
 * Set the column of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathSoaT3SetCol_V( VmathSoaTransform3 *result, int col, VmathSoaVector3 vec );

/*
 * Set the row of a 3x4 transformation matrix referred to by the specified index
 */
static inline void vmathSoaT3SetRow_V( VmathSoaTransform3 *result, int row, VmathSoaVector4 vec );

/*
 * Get the column of a 3x4 transformation matrix referred to by the specified index
 */
static inline VmathSoaVector3 vmathSoaT3GetCol_V( VmathSoaTransform3 tfrm, int col );

/*
 * Get the row of a 3x4 transformation matrix referred to by the specified index
 */
static inline VmathSoaVector4 vmathSoaT3GetRow_V( VmathSoaTransform3 tfrm, int row );

/*
 * Set the element of a 3x4 transformation matrix referred to by column and row indices
 */
static inline void vmathSoaT3SetElem_V( VmathSoaTransform3 *result, int col, int row, vec_float4 val );

/*
 * Get the element of a 3x4 transformation matrix referred to by column and row indices
 */
static inline vec_float4 vmathSoaT3GetElem_V( VmathSoaTransform3 tfrm, int col, int row );

/*
 * Multiply a 3x4 transformation matrix by a 3-D vector
 */
static inline VmathSoaVector3 vmathSoaT3MulV3_V( VmathSoaTransform3 tfrm, VmathSoaVector3 vec );

/*
 * Multiply a 3x4 transformation matrix by a 3-D point
 */
static inline VmathSoaPoint3 vmathSoaT3MulP3_V( VmathSoaTransform3 tfrm, VmathSoaPoint3 pnt );

/*
 * Multiply two 3x4 transformation matrices
 */
static inline VmathSoaTransform3 vmathSoaT3Mul_V( VmathSoaTransform3 tfrm0, VmathSoaTransform3 tfrm1 );

/*
 * Construct an identity 3x4 transformation matrix
 */
static inline VmathSoaTransform3 vmathSoaT3MakeIdentity_V( );

/*
 * Construct a 3x4 transformation matrix to rotate around the x axis
 */
static inline VmathSoaTransform3 vmathSoaT3MakeRotationX_V( vec_float4 radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the y axis
 */
static inline VmathSoaTransform3 vmathSoaT3MakeRotationY_V( vec_float4 radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the z axis
 */
static inline VmathSoaTransform3 vmathSoaT3MakeRotationZ_V( vec_float4 radians );

/*
 * Construct a 3x4 transformation matrix to rotate around the x, y, and z axes
 */
static inline VmathSoaTransform3 vmathSoaT3MakeRotationZYX_V( VmathSoaVector3 radiansXYZ );

/*
 * Construct a 3x4 transformation matrix to rotate around a unit-length 3-D vector
 */
static inline VmathSoaTransform3 vmathSoaT3MakeRotationAxis_V( vec_float4 radians, VmathSoaVector3 unitVec );

/*
 * Construct a rotation matrix from a unit-length quaternion
 */
static inline VmathSoaTransform3 vmathSoaT3MakeRotationQ_V( VmathSoaQuat unitQuat );

/*
 * Construct a 3x4 transformation matrix to perform scaling
 */
static inline VmathSoaTransform3 vmathSoaT3MakeScale_V( VmathSoaVector3 scaleVec );

/*
 * Construct a 3x4 transformation matrix to perform translation
 */
static inline VmathSoaTransform3 vmathSoaT3MakeTranslation_V( VmathSoaVector3 translateVec );

/*
 * Append (post-multiply) a scale transformation to a 3x4 transformation matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathSoaTransform3 vmathSoaT3AppendScale_V( VmathSoaTransform3 tfrm, VmathSoaVector3 scaleVec );

/*
 * Prepend (pre-multiply) a scale transformation to a 3x4 transformation matrix
 * NOTE: 
 * Faster than creating and multiplying a scale transformation matrix.
 */
static inline VmathSoaTransform3 vmathSoaT3PrependScale_V( VmathSoaVector3 scaleVec, VmathSoaTransform3 tfrm );

/*
 * Multiply two 3x4 transformation matrices per element
 */
static inline VmathSoaTransform3 vmathSoaT3MulPerElem_V( VmathSoaTransform3 tfrm0, VmathSoaTransform3 tfrm1 );

/*
 * Compute the absolute value of a 3x4 transformation matrix per element
 */
static inline VmathSoaTransform3 vmathSoaT3AbsPerElem_V( VmathSoaTransform3 tfrm );

/*
 * Inverse of a 3x4 transformation matrix
 * NOTE: 
 * Result is unpredictable when the determinant of the left 3x3 submatrix is equal to or near 0.
 */
static inline VmathSoaTransform3 vmathSoaT3Inverse_V( VmathSoaTransform3 tfrm );

/*
 * Compute the inverse of a 3x4 transformation matrix, expected to have an orthogonal upper-left 3x3 submatrix
 * NOTE: 
 * This can be used to achieve better performance than a general inverse when the specified 3x4 transformation matrix meets the given restrictions.
 */
static inline VmathSoaTransform3 vmathSoaT3OrthoInverse_V( VmathSoaTransform3 tfrm );

/*
 * Conditionally select between two 3x4 transformation matrices
 * NOTE: 
 * This function uses a conditional select instruction to avoid a branch.
 */
static inline VmathSoaTransform3 vmathSoaT3Select_V( VmathSoaTransform3 tfrm0, VmathSoaTransform3 tfrm1, vec_uint4 select1 );

#ifdef _VECTORMATH_DEBUG

/*
 * Print a 3x4 transformation matrix
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaT3Print_V( VmathSoaTransform3 tfrm );

/*
 * Print a 3x4 transformation matrix and an associated string identifier
 * NOTE: 
 * Function is only defined when _VECTORMATH_DEBUG is defined.
 */
static inline void vmathSoaT3Prints_V( VmathSoaTransform3 tfrm, const char *name );

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#include "vectormath_soa.h"
#include "vec_soa_v.h"
#include "quat_soa_v.h"
#include "mat_soa_v.h"

#endif
