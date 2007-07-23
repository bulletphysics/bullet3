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

#ifndef _VECTORMATH_MAT_AOS_V_C_H
#define _VECTORMATH_MAT_AOS_V_C_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Constants
 * for shuffles, words are labeled [x,y,z,w] [a,b,c,d]
 */
#define _VECTORMATH_PERM_ZBWX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_B, _VECTORMATH_PERM_W, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_XCYX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_C, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_XYAB ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_A, _VECTORMATH_PERM_B })
#define _VECTORMATH_PERM_ZWCD ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_W, _VECTORMATH_PERM_C, _VECTORMATH_PERM_D })
#define _VECTORMATH_PERM_XZBX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_B, _VECTORMATH_PERM_X })     
#define _VECTORMATH_PERM_CXXX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_C, _VECTORMATH_PERM_X, _VECTORMATH_PERM_X, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_YAXX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_A, _VECTORMATH_PERM_X, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_XAZC ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_A, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_C })
#define _VECTORMATH_PERM_YXWZ ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_X, _VECTORMATH_PERM_W, _VECTORMATH_PERM_Z })
#define _VECTORMATH_PERM_YBWD ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_B, _VECTORMATH_PERM_W, _VECTORMATH_PERM_D })
#define _VECTORMATH_PERM_XYCX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_C, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_YCXY ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_C, _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y })
#define _VECTORMATH_PERM_CXYC ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_C, _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_C })
#define _VECTORMATH_PERM_ZAYX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_A, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_BZXX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_B, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_X, _VECTORMATH_PERM_X })
#define _VECTORMATH_PERM_XZYA ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_A })
#define _VECTORMATH_PERM_ZXXB ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_X, _VECTORMATH_PERM_X, _VECTORMATH_PERM_B })
#define _VECTORMATH_PERM_YXXC ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_X, _VECTORMATH_PERM_X, _VECTORMATH_PERM_C })
#define _VECTORMATH_PERM_BBYX ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_B, _VECTORMATH_PERM_B, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_X })
#define _VECTORMATH_PI_OVER_2 1.570796327f

/*-----------------------------------------------------------------------------
 * Definitions
 */
static inline VmathMatrix3 vmathM3MakeFromScalar_V( float scalar )
{
    VmathMatrix3 result;
    vmathM3MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathMatrix3 vmathM3MakeFromQ_V( VmathQuat unitQuat )
{
    VmathMatrix3 result;
    vmathM3MakeFromQ(&result, &unitQuat);
    return result;
}

static inline VmathMatrix3 vmathM3MakeFromCols_V( VmathVector3 _col0, VmathVector3 _col1, VmathVector3 _col2 )
{
    VmathMatrix3 result;
    vmathM3MakeFromCols(&result, &_col0, &_col1, &_col2);
    return result;
}

static inline void vmathM3SetCol0_V( VmathMatrix3 *result, VmathVector3 _col0 )
{
    vmathM3SetCol0(result, &_col0);
}

static inline void vmathM3SetCol1_V( VmathMatrix3 *result, VmathVector3 _col1 )
{
    vmathM3SetCol1(result, &_col1);
}

static inline void vmathM3SetCol2_V( VmathMatrix3 *result, VmathVector3 _col2 )
{
    vmathM3SetCol2(result, &_col2);
}

static inline void vmathM3SetCol_V( VmathMatrix3 *result, int col, VmathVector3 vec )
{
    vmathM3SetCol(result, col, &vec);
}

static inline void vmathM3SetRow_V( VmathMatrix3 *result, int row, VmathVector3 vec )
{
    vmathM3SetRow(result, row, &vec);
}

static inline void vmathM3SetElem_V( VmathMatrix3 *result, int col, int row, float val )
{
    vmathM3SetElem(result, col, row, val);
}

static inline float vmathM3GetElem_V( VmathMatrix3 mat, int col, int row )
{
    return vmathM3GetElem(&mat, col, row);
}

static inline VmathVector3 vmathM3GetCol0_V( VmathMatrix3 mat )
{
    VmathVector3 result;
    vmathM3GetCol0(&result, &mat);
    return result;
}

static inline VmathVector3 vmathM3GetCol1_V( VmathMatrix3 mat )
{
    VmathVector3 result;
    vmathM3GetCol1(&result, &mat);
    return result;
}

static inline VmathVector3 vmathM3GetCol2_V( VmathMatrix3 mat )
{
    VmathVector3 result;
    vmathM3GetCol2(&result, &mat);
    return result;
}

static inline VmathVector3 vmathM3GetCol_V( VmathMatrix3 mat, int col )
{
    VmathVector3 result;
    vmathM3GetCol(&result, &mat, col);
    return result;
}

static inline VmathVector3 vmathM3GetRow_V( VmathMatrix3 mat, int row )
{
    VmathVector3 result;
    vmathM3GetRow(&result, &mat, row);
    return result;
}

static inline VmathMatrix3 vmathM3Transpose_V( VmathMatrix3 mat )
{
    VmathMatrix3 result;
    vmathM3Transpose(&result, &mat);
    return result;
}

static inline VmathMatrix3 vmathM3Inverse_V( VmathMatrix3 mat )
{
    VmathMatrix3 result;
    vmathM3Inverse(&result, &mat);
    return result;
}

static inline float vmathM3Determinant_V( VmathMatrix3 mat )
{
    return vmathM3Determinant(&mat);
}

static inline VmathMatrix3 vmathM3Add_V( VmathMatrix3 mat0, VmathMatrix3 mat1 )
{
    VmathMatrix3 result;
    vmathM3Add(&result, &mat0, &mat1);
    return result;
}

static inline VmathMatrix3 vmathM3Sub_V( VmathMatrix3 mat0, VmathMatrix3 mat1 )
{
    VmathMatrix3 result;
    vmathM3Sub(&result, &mat0, &mat1);
    return result;
}

static inline VmathMatrix3 vmathM3Neg_V( VmathMatrix3 mat )
{
    VmathMatrix3 result;
    vmathM3Neg(&result, &mat);
    return result;
}

static inline VmathMatrix3 vmathM3AbsPerElem_V( VmathMatrix3 mat )
{
    VmathMatrix3 result;
    vmathM3AbsPerElem(&result, &mat);
    return result;
}

static inline VmathMatrix3 vmathM3ScalarMul_V( VmathMatrix3 mat, float scalar )
{
    VmathMatrix3 result;
    vmathM3ScalarMul(&result, &mat, scalar);
    return result;
}

static inline VmathVector3 vmathM3MulV3_V( VmathMatrix3 mat, VmathVector3 vec )
{
    VmathVector3 result;
    vmathM3MulV3(&result, &mat, &vec);
    return result;
}

static inline VmathMatrix3 vmathM3Mul_V( VmathMatrix3 mat0, VmathMatrix3 mat1 )
{
    VmathMatrix3 result;
    vmathM3Mul(&result, &mat0, &mat1);
    return result;
}

static inline VmathMatrix3 vmathM3MulPerElem_V( VmathMatrix3 mat0, VmathMatrix3 mat1 )
{
    VmathMatrix3 result;
    vmathM3MulPerElem(&result, &mat0, &mat1);
    return result;
}

static inline VmathMatrix3 vmathM3MakeIdentity_V( )
{
    VmathMatrix3 result;
    vmathM3MakeIdentity(&result);
    return result;
}

static inline VmathMatrix3 vmathM3MakeRotationX_V( float radians )
{
    VmathMatrix3 result;
    vmathM3MakeRotationX(&result, radians);
    return result;
}

static inline VmathMatrix3 vmathM3MakeRotationY_V( float radians )
{
    VmathMatrix3 result;
    vmathM3MakeRotationY(&result, radians);
    return result;
}

static inline VmathMatrix3 vmathM3MakeRotationZ_V( float radians )
{
    VmathMatrix3 result;
    vmathM3MakeRotationZ(&result, radians);
    return result;
}

static inline VmathMatrix3 vmathM3MakeRotationZYX_V( VmathVector3 radiansXYZ )
{
    VmathMatrix3 result;
    vmathM3MakeRotationZYX(&result, &radiansXYZ);
    return result;
}

static inline VmathMatrix3 vmathM3MakeRotationAxis_V( float radians, VmathVector3 unitVec )
{
    VmathMatrix3 result;
    vmathM3MakeRotationAxis(&result, radians, &unitVec);
    return result;
}

static inline VmathMatrix3 vmathM3MakeRotationQ_V( VmathQuat unitQuat )
{
    VmathMatrix3 result;
    vmathM3MakeRotationQ(&result, &unitQuat);
    return result;
}

static inline VmathMatrix3 vmathM3MakeScale_V( VmathVector3 scaleVec )
{
    VmathMatrix3 result;
    vmathM3MakeScale(&result, &scaleVec);
    return result;
}

static inline VmathMatrix3 vmathM3AppendScale_V( VmathMatrix3 mat, VmathVector3 scaleVec )
{
    VmathMatrix3 result;
    vmathM3AppendScale(&result, &mat, &scaleVec);
    return result;
}

static inline VmathMatrix3 vmathM3PrependScale_V( VmathVector3 scaleVec, VmathMatrix3 mat )
{
    VmathMatrix3 result;
    vmathM3PrependScale(&result, &scaleVec, &mat);
    return result;
}

static inline VmathMatrix3 vmathM3Select_V( VmathMatrix3 mat0, VmathMatrix3 mat1, unsigned int select1 )
{
    VmathMatrix3 result;
    vmathM3Select(&result, &mat0, &mat1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathM3Print_V( VmathMatrix3 mat )
{
    vmathM3Print(&mat);
}

static inline void vmathM3Prints_V( VmathMatrix3 mat, const char *name )
{
    vmathM3Prints(&mat, name);
}

#endif

static inline VmathMatrix4 vmathM4MakeFromScalar_V( float scalar )
{
    VmathMatrix4 result;
    vmathM4MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathMatrix4 vmathM4MakeFromT3_V( VmathTransform3 mat )
{
    VmathMatrix4 result;
    vmathM4MakeFromT3(&result, &mat);
    return result;
}

static inline VmathMatrix4 vmathM4MakeFromCols_V( VmathVector4 _col0, VmathVector4 _col1, VmathVector4 _col2, VmathVector4 _col3 )
{
    VmathMatrix4 result;
    vmathM4MakeFromCols(&result, &_col0, &_col1, &_col2, &_col3);
    return result;
}

static inline VmathMatrix4 vmathM4MakeFromM3V3_V( VmathMatrix3 mat, VmathVector3 translateVec )
{
    VmathMatrix4 result;
    vmathM4MakeFromM3V3(&result, &mat, &translateVec);
    return result;
}

static inline VmathMatrix4 vmathM4MakeFromQV3_V( VmathQuat unitQuat, VmathVector3 translateVec )
{
    VmathMatrix4 result;
    vmathM4MakeFromQV3(&result, &unitQuat, &translateVec);
    return result;
}

static inline void vmathM4SetCol0_V( VmathMatrix4 *result, VmathVector4 _col0 )
{
    vmathM4SetCol0(result, &_col0);
}

static inline void vmathM4SetCol1_V( VmathMatrix4 *result, VmathVector4 _col1 )
{
    vmathM4SetCol1(result, &_col1);
}

static inline void vmathM4SetCol2_V( VmathMatrix4 *result, VmathVector4 _col2 )
{
    vmathM4SetCol2(result, &_col2);
}

static inline void vmathM4SetCol3_V( VmathMatrix4 *result, VmathVector4 _col3 )
{
    vmathM4SetCol3(result, &_col3);
}

static inline void vmathM4SetCol_V( VmathMatrix4 *result, int col, VmathVector4 vec )
{
    vmathM4SetCol(result, col, &vec);
}

static inline void vmathM4SetRow_V( VmathMatrix4 *result, int row, VmathVector4 vec )
{
    vmathM4SetRow(result, row, &vec);
}

static inline void vmathM4SetElem_V( VmathMatrix4 *result, int col, int row, float val )
{
    vmathM4SetElem(result, col, row, val);
}

static inline float vmathM4GetElem_V( VmathMatrix4 mat, int col, int row )
{
    return vmathM4GetElem(&mat, col, row);
}

static inline VmathVector4 vmathM4GetCol0_V( VmathMatrix4 mat )
{
    VmathVector4 result;
    vmathM4GetCol0(&result, &mat);
    return result;
}

static inline VmathVector4 vmathM4GetCol1_V( VmathMatrix4 mat )
{
    VmathVector4 result;
    vmathM4GetCol1(&result, &mat);
    return result;
}

static inline VmathVector4 vmathM4GetCol2_V( VmathMatrix4 mat )
{
    VmathVector4 result;
    vmathM4GetCol2(&result, &mat);
    return result;
}

static inline VmathVector4 vmathM4GetCol3_V( VmathMatrix4 mat )
{
    VmathVector4 result;
    vmathM4GetCol3(&result, &mat);
    return result;
}

static inline VmathVector4 vmathM4GetCol_V( VmathMatrix4 mat, int col )
{
    VmathVector4 result;
    vmathM4GetCol(&result, &mat, col);
    return result;
}

static inline VmathVector4 vmathM4GetRow_V( VmathMatrix4 mat, int row )
{
    VmathVector4 result;
    vmathM4GetRow(&result, &mat, row);
    return result;
}

static inline VmathMatrix4 vmathM4Transpose_V( VmathMatrix4 mat )
{
    VmathMatrix4 result;
    vmathM4Transpose(&result, &mat);
    return result;
}

static inline VmathMatrix4 vmathM4Inverse_V( VmathMatrix4 mat )
{
    VmathMatrix4 result;
    vmathM4Inverse(&result, &mat);
    return result;
}

static inline VmathMatrix4 vmathM4AffineInverse_V( VmathMatrix4 mat )
{
    VmathMatrix4 result;
    vmathM4AffineInverse(&result, &mat);
    return result;
}

static inline VmathMatrix4 vmathM4OrthoInverse_V( VmathMatrix4 mat )
{
    VmathMatrix4 result;
    vmathM4OrthoInverse(&result, &mat);
    return result;
}

static inline float vmathM4Determinant_V( VmathMatrix4 mat )
{
    return vmathM4Determinant(&mat);
}

static inline VmathMatrix4 vmathM4Add_V( VmathMatrix4 mat0, VmathMatrix4 mat1 )
{
    VmathMatrix4 result;
    vmathM4Add(&result, &mat0, &mat1);
    return result;
}

static inline VmathMatrix4 vmathM4Sub_V( VmathMatrix4 mat0, VmathMatrix4 mat1 )
{
    VmathMatrix4 result;
    vmathM4Sub(&result, &mat0, &mat1);
    return result;
}

static inline VmathMatrix4 vmathM4Neg_V( VmathMatrix4 mat )
{
    VmathMatrix4 result;
    vmathM4Neg(&result, &mat);
    return result;
}

static inline VmathMatrix4 vmathM4AbsPerElem_V( VmathMatrix4 mat )
{
    VmathMatrix4 result;
    vmathM4AbsPerElem(&result, &mat);
    return result;
}

static inline VmathMatrix4 vmathM4ScalarMul_V( VmathMatrix4 mat, float scalar )
{
    VmathMatrix4 result;
    vmathM4ScalarMul(&result, &mat, scalar);
    return result;
}

static inline VmathVector4 vmathM4MulV4_V( VmathMatrix4 mat, VmathVector4 vec )
{
    VmathVector4 result;
    vmathM4MulV4(&result, &mat, &vec);
    return result;
}

static inline VmathVector4 vmathM4MulV3_V( VmathMatrix4 mat, VmathVector3 vec )
{
    VmathVector4 result;
    vmathM4MulV3(&result, &mat, &vec);
    return result;
}

static inline VmathVector4 vmathM4MulP3_V( VmathMatrix4 mat, VmathPoint3 pnt )
{
    VmathVector4 result;
    vmathM4MulP3(&result, &mat, &pnt);
    return result;
}

static inline VmathMatrix4 vmathM4Mul_V( VmathMatrix4 mat0, VmathMatrix4 mat1 )
{
    VmathMatrix4 result;
    vmathM4Mul(&result, &mat0, &mat1);
    return result;
}

static inline VmathMatrix4 vmathM4MulT3_V( VmathMatrix4 mat, VmathTransform3 tfrm1 )
{
    VmathMatrix4 result;
    vmathM4MulT3(&result, &mat, &tfrm1);
    return result;
}

static inline VmathMatrix4 vmathM4MulPerElem_V( VmathMatrix4 mat0, VmathMatrix4 mat1 )
{
    VmathMatrix4 result;
    vmathM4MulPerElem(&result, &mat0, &mat1);
    return result;
}

static inline VmathMatrix4 vmathM4MakeIdentity_V( )
{
    VmathMatrix4 result;
    vmathM4MakeIdentity(&result);
    return result;
}

static inline void vmathM4SetUpper3x3_V( VmathMatrix4 *result, VmathMatrix3 mat3 )
{
    vmathM4SetUpper3x3(result, &mat3);
}

static inline VmathMatrix3 vmathM4GetUpper3x3_V( VmathMatrix4 mat )
{
    VmathMatrix3 result;
    vmathM4GetUpper3x3(&result, &mat);
    return result;
}

static inline void vmathM4SetTranslation_V( VmathMatrix4 *result, VmathVector3 translateVec )
{
    vmathM4SetTranslation(result, &translateVec);
}

static inline VmathVector3 vmathM4GetTranslation_V( VmathMatrix4 mat )
{
    VmathVector3 result;
    vmathM4GetTranslation(&result, &mat);
    return result;
}

static inline VmathMatrix4 vmathM4MakeRotationX_V( float radians )
{
    VmathMatrix4 result;
    vmathM4MakeRotationX(&result, radians);
    return result;
}

static inline VmathMatrix4 vmathM4MakeRotationY_V( float radians )
{
    VmathMatrix4 result;
    vmathM4MakeRotationY(&result, radians);
    return result;
}

static inline VmathMatrix4 vmathM4MakeRotationZ_V( float radians )
{
    VmathMatrix4 result;
    vmathM4MakeRotationZ(&result, radians);
    return result;
}

static inline VmathMatrix4 vmathM4MakeRotationZYX_V( VmathVector3 radiansXYZ )
{
    VmathMatrix4 result;
    vmathM4MakeRotationZYX(&result, &radiansXYZ);
    return result;
}

static inline VmathMatrix4 vmathM4MakeRotationAxis_V( float radians, VmathVector3 unitVec )
{
    VmathMatrix4 result;
    vmathM4MakeRotationAxis(&result, radians, &unitVec);
    return result;
}

static inline VmathMatrix4 vmathM4MakeRotationQ_V( VmathQuat unitQuat )
{
    VmathMatrix4 result;
    vmathM4MakeRotationQ(&result, &unitQuat);
    return result;
}

static inline VmathMatrix4 vmathM4MakeScale_V( VmathVector3 scaleVec )
{
    VmathMatrix4 result;
    vmathM4MakeScale(&result, &scaleVec);
    return result;
}

static inline VmathMatrix4 vmathM4AppendScale_V( VmathMatrix4 mat, VmathVector3 scaleVec )
{
    VmathMatrix4 result;
    vmathM4AppendScale(&result, &mat, &scaleVec);
    return result;
}

static inline VmathMatrix4 vmathM4PrependScale_V( VmathVector3 scaleVec, VmathMatrix4 mat )
{
    VmathMatrix4 result;
    vmathM4PrependScale(&result, &scaleVec, &mat);
    return result;
}

static inline VmathMatrix4 vmathM4MakeTranslation_V( VmathVector3 translateVec )
{
    VmathMatrix4 result;
    vmathM4MakeTranslation(&result, &translateVec);
    return result;
}

static inline VmathMatrix4 vmathM4MakeLookAt_V( VmathPoint3 eyePos, VmathPoint3 lookAtPos, VmathVector3 upVec )
{
    VmathMatrix4 result;
    vmathM4MakeLookAt(&result, &eyePos, &lookAtPos, &upVec);
    return result;
}

static inline VmathMatrix4 vmathM4MakePerspective_V( float fovyRadians, float aspect, float zNear, float zFar )
{
    VmathMatrix4 result;
    vmathM4MakePerspective(&result, fovyRadians, aspect, zNear, zFar);
    return result;
}

static inline VmathMatrix4 vmathM4MakeFrustum_V( float left, float right, float bottom, float top, float zNear, float zFar )
{
    VmathMatrix4 result;
    vmathM4MakeFrustum(&result, left, right, bottom, top, zNear, zFar);
    return result;
}

static inline VmathMatrix4 vmathM4MakeOrthographic_V( float left, float right, float bottom, float top, float zNear, float zFar )
{
    VmathMatrix4 result;
    vmathM4MakeOrthographic(&result, left, right, bottom, top, zNear, zFar);
    return result;
}

static inline VmathMatrix4 vmathM4Select_V( VmathMatrix4 mat0, VmathMatrix4 mat1, unsigned int select1 )
{
    VmathMatrix4 result;
    vmathM4Select(&result, &mat0, &mat1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathM4Print_V( VmathMatrix4 mat )
{
    vmathM4Print(&mat);
}

static inline void vmathM4Prints_V( VmathMatrix4 mat, const char *name )
{
    vmathM4Prints(&mat, name);
}

#endif

static inline VmathTransform3 vmathT3MakeFromScalar_V( float scalar )
{
    VmathTransform3 result;
    vmathT3MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathTransform3 vmathT3MakeFromCols_V( VmathVector3 _col0, VmathVector3 _col1, VmathVector3 _col2, VmathVector3 _col3 )
{
    VmathTransform3 result;
    vmathT3MakeFromCols(&result, &_col0, &_col1, &_col2, &_col3);
    return result;
}

static inline VmathTransform3 vmathT3MakeFromM3V3_V( VmathMatrix3 tfrm, VmathVector3 translateVec )
{
    VmathTransform3 result;
    vmathT3MakeFromM3V3(&result, &tfrm, &translateVec);
    return result;
}

static inline VmathTransform3 vmathT3MakeFromQV3_V( VmathQuat unitQuat, VmathVector3 translateVec )
{
    VmathTransform3 result;
    vmathT3MakeFromQV3(&result, &unitQuat, &translateVec);
    return result;
}

static inline void vmathT3SetCol0_V( VmathTransform3 *result, VmathVector3 _col0 )
{
    vmathT3SetCol0(result, &_col0);
}

static inline void vmathT3SetCol1_V( VmathTransform3 *result, VmathVector3 _col1 )
{
    vmathT3SetCol1(result, &_col1);
}

static inline void vmathT3SetCol2_V( VmathTransform3 *result, VmathVector3 _col2 )
{
    vmathT3SetCol2(result, &_col2);
}

static inline void vmathT3SetCol3_V( VmathTransform3 *result, VmathVector3 _col3 )
{
    vmathT3SetCol3(result, &_col3);
}

static inline void vmathT3SetCol_V( VmathTransform3 *result, int col, VmathVector3 vec )
{
    vmathT3SetCol(result, col, &vec);
}

static inline void vmathT3SetRow_V( VmathTransform3 *result, int row, VmathVector4 vec )
{
    vmathT3SetRow(result, row, &vec);
}

static inline void vmathT3SetElem_V( VmathTransform3 *result, int col, int row, float val )
{
    vmathT3SetElem(result, col, row, val);
}

static inline float vmathT3GetElem_V( VmathTransform3 tfrm, int col, int row )
{
    return vmathT3GetElem(&tfrm, col, row);
}

static inline VmathVector3 vmathT3GetCol0_V( VmathTransform3 tfrm )
{
    VmathVector3 result;
    vmathT3GetCol0(&result, &tfrm);
    return result;
}

static inline VmathVector3 vmathT3GetCol1_V( VmathTransform3 tfrm )
{
    VmathVector3 result;
    vmathT3GetCol1(&result, &tfrm);
    return result;
}

static inline VmathVector3 vmathT3GetCol2_V( VmathTransform3 tfrm )
{
    VmathVector3 result;
    vmathT3GetCol2(&result, &tfrm);
    return result;
}

static inline VmathVector3 vmathT3GetCol3_V( VmathTransform3 tfrm )
{
    VmathVector3 result;
    vmathT3GetCol3(&result, &tfrm);
    return result;
}

static inline VmathVector3 vmathT3GetCol_V( VmathTransform3 tfrm, int col )
{
    VmathVector3 result;
    vmathT3GetCol(&result, &tfrm, col);
    return result;
}

static inline VmathVector4 vmathT3GetRow_V( VmathTransform3 tfrm, int row )
{
    VmathVector4 result;
    vmathT3GetRow(&result, &tfrm, row);
    return result;
}

static inline VmathTransform3 vmathT3Inverse_V( VmathTransform3 tfrm )
{
    VmathTransform3 result;
    vmathT3Inverse(&result, &tfrm);
    return result;
}

static inline VmathTransform3 vmathT3OrthoInverse_V( VmathTransform3 tfrm )
{
    VmathTransform3 result;
    vmathT3OrthoInverse(&result, &tfrm);
    return result;
}

static inline VmathTransform3 vmathT3AbsPerElem_V( VmathTransform3 tfrm )
{
    VmathTransform3 result;
    vmathT3AbsPerElem(&result, &tfrm);
    return result;
}

static inline VmathVector3 vmathT3MulV3_V( VmathTransform3 tfrm, VmathVector3 vec )
{
    VmathVector3 result;
    vmathT3MulV3(&result, &tfrm, &vec);
    return result;
}

static inline VmathPoint3 vmathT3MulP3_V( VmathTransform3 tfrm, VmathPoint3 pnt )
{
    VmathPoint3 result;
    vmathT3MulP3(&result, &tfrm, &pnt);
    return result;
}

static inline VmathTransform3 vmathT3Mul_V( VmathTransform3 tfrm0, VmathTransform3 tfrm1 )
{
    VmathTransform3 result;
    vmathT3Mul(&result, &tfrm0, &tfrm1);
    return result;
}

static inline VmathTransform3 vmathT3MulPerElem_V( VmathTransform3 tfrm0, VmathTransform3 tfrm1 )
{
    VmathTransform3 result;
    vmathT3MulPerElem(&result, &tfrm0, &tfrm1);
    return result;
}

static inline VmathTransform3 vmathT3MakeIdentity_V( )
{
    VmathTransform3 result;
    vmathT3MakeIdentity(&result);
    return result;
}

static inline void vmathT3SetUpper3x3_V( VmathTransform3 *result, VmathMatrix3 tfrm )
{
    vmathT3SetUpper3x3(result, &tfrm);
}

static inline VmathMatrix3 vmathT3GetUpper3x3_V( VmathTransform3 tfrm )
{
    VmathMatrix3 result;
    vmathT3GetUpper3x3(&result, &tfrm);
    return result;
}

static inline void vmathT3SetTranslation_V( VmathTransform3 *result, VmathVector3 translateVec )
{
    vmathT3SetTranslation(result, &translateVec);
}

static inline VmathVector3 vmathT3GetTranslation_V( VmathTransform3 tfrm )
{
    VmathVector3 result;
    vmathT3GetTranslation(&result, &tfrm);
    return result;
}

static inline VmathTransform3 vmathT3MakeRotationX_V( float radians )
{
    VmathTransform3 result;
    vmathT3MakeRotationX(&result, radians);
    return result;
}

static inline VmathTransform3 vmathT3MakeRotationY_V( float radians )
{
    VmathTransform3 result;
    vmathT3MakeRotationY(&result, radians);
    return result;
}

static inline VmathTransform3 vmathT3MakeRotationZ_V( float radians )
{
    VmathTransform3 result;
    vmathT3MakeRotationZ(&result, radians);
    return result;
}

static inline VmathTransform3 vmathT3MakeRotationZYX_V( VmathVector3 radiansXYZ )
{
    VmathTransform3 result;
    vmathT3MakeRotationZYX(&result, &radiansXYZ);
    return result;
}

static inline VmathTransform3 vmathT3MakeRotationAxis_V( float radians, VmathVector3 unitVec )
{
    VmathTransform3 result;
    vmathT3MakeRotationAxis(&result, radians, &unitVec);
    return result;
}

static inline VmathTransform3 vmathT3MakeRotationQ_V( VmathQuat unitQuat )
{
    VmathTransform3 result;
    vmathT3MakeRotationQ(&result, &unitQuat);
    return result;
}

static inline VmathTransform3 vmathT3MakeScale_V( VmathVector3 scaleVec )
{
    VmathTransform3 result;
    vmathT3MakeScale(&result, &scaleVec);
    return result;
}

static inline VmathTransform3 vmathT3AppendScale_V( VmathTransform3 tfrm, VmathVector3 scaleVec )
{
    VmathTransform3 result;
    vmathT3AppendScale(&result, &tfrm, &scaleVec);
    return result;
}

static inline VmathTransform3 vmathT3PrependScale_V( VmathVector3 scaleVec, VmathTransform3 tfrm )
{
    VmathTransform3 result;
    vmathT3PrependScale(&result, &scaleVec, &tfrm);
    return result;
}

static inline VmathTransform3 vmathT3MakeTranslation_V( VmathVector3 translateVec )
{
    VmathTransform3 result;
    vmathT3MakeTranslation(&result, &translateVec);
    return result;
}

static inline VmathTransform3 vmathT3Select_V( VmathTransform3 tfrm0, VmathTransform3 tfrm1, unsigned int select1 )
{
    VmathTransform3 result;
    vmathT3Select(&result, &tfrm0, &tfrm1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathT3Print_V( VmathTransform3 tfrm )
{
    vmathT3Print(&tfrm);
}

static inline void vmathT3Prints_V( VmathTransform3 tfrm, const char *name )
{
    vmathT3Prints(&tfrm, name);
}

#endif

static inline VmathQuat vmathQMakeFromM3_V( VmathMatrix3 tfrm )
{
    VmathQuat result;
    vmathQMakeFromM3(&result, &tfrm);
    return result;
}

static inline VmathMatrix3 vmathV3Outer_V( VmathVector3 tfrm0, VmathVector3 tfrm1 )
{
    VmathMatrix3 result;
    vmathV3Outer(&result, &tfrm0, &tfrm1);
    return result;
}

static inline VmathMatrix4 vmathV4Outer_V( VmathVector4 tfrm0, VmathVector4 tfrm1 )
{
    VmathMatrix4 result;
    vmathV4Outer(&result, &tfrm0, &tfrm1);
    return result;
}

static inline VmathVector3 vmathV3RowMul_V( VmathVector3 vec, VmathMatrix3 mat )
{
    VmathVector3 result;
    vmathV3RowMul(&result, &vec, &mat);
    return result;
}

static inline VmathMatrix3 vmathV3CrossMatrix_V( VmathVector3 vec )
{
    VmathMatrix3 result;
    vmathV3CrossMatrix(&result, &vec);
    return result;
}

static inline VmathMatrix3 vmathV3CrossMatrixMul_V( VmathVector3 vec, VmathMatrix3 mat )
{
    VmathMatrix3 result;
    vmathV3CrossMatrixMul(&result, &vec, &mat);
    return result;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
