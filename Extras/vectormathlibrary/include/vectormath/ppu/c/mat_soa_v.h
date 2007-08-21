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

#ifndef _VECTORMATH_MAT_SOA_V_C_H
#define _VECTORMATH_MAT_SOA_V_C_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Constants
 */
#define _VECTORMATH_PI_OVER_2 1.570796327f

/*-----------------------------------------------------------------------------
 * Definitions
 */
static inline VmathSoaMatrix3 vmathSoaM3MakeFromScalar_V( vec_float4 scalar )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeFromQ_V( VmathSoaQuat unitQuat )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeFromQ(&result, &unitQuat);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeFromCols_V( VmathSoaVector3 _col0, VmathSoaVector3 _col1, VmathSoaVector3 _col2 )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeFromCols(&result, &_col0, &_col1, &_col2);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeFromAos_V( VmathMatrix3 mat )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeFromAos(&result, &mat);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeFrom4Aos_V( VmathMatrix3 mat0, VmathMatrix3 mat1, VmathMatrix3 mat2, VmathMatrix3 mat3 )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeFrom4Aos(&result, &mat0, &mat1, &mat2, &mat3);
    return result;
}

static inline void vmathSoaM3Get4Aos_V( VmathSoaMatrix3 mat, VmathMatrix3 *result0, VmathMatrix3 *result1, VmathMatrix3 *result2, VmathMatrix3 *result3 )
{
    vmathSoaM3Get4Aos(&mat, result0, result1, result2, result3);
}

static inline void vmathSoaM3SetCol0_V( VmathSoaMatrix3 *result, VmathSoaVector3 _col0 )
{
    vmathSoaM3SetCol0(result, &_col0);
}

static inline void vmathSoaM3SetCol1_V( VmathSoaMatrix3 *result, VmathSoaVector3 _col1 )
{
    vmathSoaM3SetCol1(result, &_col1);
}

static inline void vmathSoaM3SetCol2_V( VmathSoaMatrix3 *result, VmathSoaVector3 _col2 )
{
    vmathSoaM3SetCol2(result, &_col2);
}

static inline void vmathSoaM3SetCol_V( VmathSoaMatrix3 *result, int col, VmathSoaVector3 vec )
{
    vmathSoaM3SetCol(result, col, &vec);
}

static inline void vmathSoaM3SetRow_V( VmathSoaMatrix3 *result, int row, VmathSoaVector3 vec )
{
    vmathSoaM3SetRow(result, row, &vec);
}

static inline void vmathSoaM3SetElem_V( VmathSoaMatrix3 *result, int col, int row, vec_float4 val )
{
    vmathSoaM3SetElem(result, col, row, val);
}

static inline vec_float4 vmathSoaM3GetElem_V( VmathSoaMatrix3 mat, int col, int row )
{
    return vmathSoaM3GetElem(&mat, col, row);
}

static inline VmathSoaVector3 vmathSoaM3GetCol0_V( VmathSoaMatrix3 mat )
{
    VmathSoaVector3 result;
    vmathSoaM3GetCol0(&result, &mat);
    return result;
}

static inline VmathSoaVector3 vmathSoaM3GetCol1_V( VmathSoaMatrix3 mat )
{
    VmathSoaVector3 result;
    vmathSoaM3GetCol1(&result, &mat);
    return result;
}

static inline VmathSoaVector3 vmathSoaM3GetCol2_V( VmathSoaMatrix3 mat )
{
    VmathSoaVector3 result;
    vmathSoaM3GetCol2(&result, &mat);
    return result;
}

static inline VmathSoaVector3 vmathSoaM3GetCol_V( VmathSoaMatrix3 mat, int col )
{
    VmathSoaVector3 result;
    vmathSoaM3GetCol(&result, &mat, col);
    return result;
}

static inline VmathSoaVector3 vmathSoaM3GetRow_V( VmathSoaMatrix3 mat, int row )
{
    VmathSoaVector3 result;
    vmathSoaM3GetRow(&result, &mat, row);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3Transpose_V( VmathSoaMatrix3 mat )
{
    VmathSoaMatrix3 result;
    vmathSoaM3Transpose(&result, &mat);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3Inverse_V( VmathSoaMatrix3 mat )
{
    VmathSoaMatrix3 result;
    vmathSoaM3Inverse(&result, &mat);
    return result;
}

static inline vec_float4 vmathSoaM3Determinant_V( VmathSoaMatrix3 mat )
{
    return vmathSoaM3Determinant(&mat);
}

static inline VmathSoaMatrix3 vmathSoaM3Add_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1 )
{
    VmathSoaMatrix3 result;
    vmathSoaM3Add(&result, &mat0, &mat1);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3Sub_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1 )
{
    VmathSoaMatrix3 result;
    vmathSoaM3Sub(&result, &mat0, &mat1);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3Neg_V( VmathSoaMatrix3 mat )
{
    VmathSoaMatrix3 result;
    vmathSoaM3Neg(&result, &mat);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3AbsPerElem_V( VmathSoaMatrix3 mat )
{
    VmathSoaMatrix3 result;
    vmathSoaM3AbsPerElem(&result, &mat);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3ScalarMul_V( VmathSoaMatrix3 mat, vec_float4 scalar )
{
    VmathSoaMatrix3 result;
    vmathSoaM3ScalarMul(&result, &mat, scalar);
    return result;
}

static inline VmathSoaVector3 vmathSoaM3MulV3_V( VmathSoaMatrix3 mat, VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaM3MulV3(&result, &mat, &vec);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3Mul_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1 )
{
    VmathSoaMatrix3 result;
    vmathSoaM3Mul(&result, &mat0, &mat1);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MulPerElem_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1 )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MulPerElem(&result, &mat0, &mat1);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeIdentity_V( )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeIdentity(&result);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeRotationX_V( vec_float4 radians )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeRotationX(&result, radians);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeRotationY_V( vec_float4 radians )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeRotationY(&result, radians);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeRotationZ_V( vec_float4 radians )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeRotationZ(&result, radians);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeRotationZYX_V( VmathSoaVector3 radiansXYZ )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeRotationZYX(&result, &radiansXYZ);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeRotationAxis_V( vec_float4 radians, VmathSoaVector3 unitVec )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeRotationAxis(&result, radians, &unitVec);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeRotationQ_V( VmathSoaQuat unitQuat )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeRotationQ(&result, &unitQuat);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3MakeScale_V( VmathSoaVector3 scaleVec )
{
    VmathSoaMatrix3 result;
    vmathSoaM3MakeScale(&result, &scaleVec);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3AppendScale_V( VmathSoaMatrix3 mat, VmathSoaVector3 scaleVec )
{
    VmathSoaMatrix3 result;
    vmathSoaM3AppendScale(&result, &mat, &scaleVec);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3PrependScale_V( VmathSoaVector3 scaleVec, VmathSoaMatrix3 mat )
{
    VmathSoaMatrix3 result;
    vmathSoaM3PrependScale(&result, &scaleVec, &mat);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaM3Select_V( VmathSoaMatrix3 mat0, VmathSoaMatrix3 mat1, vec_uint4 select1 )
{
    VmathSoaMatrix3 result;
    vmathSoaM3Select(&result, &mat0, &mat1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaM3Print_V( VmathSoaMatrix3 mat )
{
    vmathSoaM3Print(&mat);
}

static inline void vmathSoaM3Prints_V( VmathSoaMatrix3 mat, const char *name )
{
    vmathSoaM3Prints(&mat, name);
}

#endif

static inline VmathSoaMatrix4 vmathSoaM4MakeFromScalar_V( vec_float4 scalar )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeFromT3_V( VmathSoaTransform3 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeFromT3(&result, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeFromCols_V( VmathSoaVector4 _col0, VmathSoaVector4 _col1, VmathSoaVector4 _col2, VmathSoaVector4 _col3 )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeFromCols(&result, &_col0, &_col1, &_col2, &_col3);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeFromM3V3_V( VmathSoaMatrix3 mat, VmathSoaVector3 translateVec )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeFromM3V3(&result, &mat, &translateVec);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeFromQV3_V( VmathSoaQuat unitQuat, VmathSoaVector3 translateVec )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeFromQV3(&result, &unitQuat, &translateVec);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeFromAos_V( VmathMatrix4 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeFromAos(&result, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeFrom4Aos_V( VmathMatrix4 mat0, VmathMatrix4 mat1, VmathMatrix4 mat2, VmathMatrix4 mat3 )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeFrom4Aos(&result, &mat0, &mat1, &mat2, &mat3);
    return result;
}

static inline void vmathSoaM4Get4Aos_V( VmathSoaMatrix4 mat, VmathMatrix4 *result0, VmathMatrix4 *result1, VmathMatrix4 *result2, VmathMatrix4 *result3 )
{
    vmathSoaM4Get4Aos(&mat, result0, result1, result2, result3);
}

static inline void vmathSoaM4SetCol0_V( VmathSoaMatrix4 *result, VmathSoaVector4 _col0 )
{
    vmathSoaM4SetCol0(result, &_col0);
}

static inline void vmathSoaM4SetCol1_V( VmathSoaMatrix4 *result, VmathSoaVector4 _col1 )
{
    vmathSoaM4SetCol1(result, &_col1);
}

static inline void vmathSoaM4SetCol2_V( VmathSoaMatrix4 *result, VmathSoaVector4 _col2 )
{
    vmathSoaM4SetCol2(result, &_col2);
}

static inline void vmathSoaM4SetCol3_V( VmathSoaMatrix4 *result, VmathSoaVector4 _col3 )
{
    vmathSoaM4SetCol3(result, &_col3);
}

static inline void vmathSoaM4SetCol_V( VmathSoaMatrix4 *result, int col, VmathSoaVector4 vec )
{
    vmathSoaM4SetCol(result, col, &vec);
}

static inline void vmathSoaM4SetRow_V( VmathSoaMatrix4 *result, int row, VmathSoaVector4 vec )
{
    vmathSoaM4SetRow(result, row, &vec);
}

static inline void vmathSoaM4SetElem_V( VmathSoaMatrix4 *result, int col, int row, vec_float4 val )
{
    vmathSoaM4SetElem(result, col, row, val);
}

static inline vec_float4 vmathSoaM4GetElem_V( VmathSoaMatrix4 mat, int col, int row )
{
    return vmathSoaM4GetElem(&mat, col, row);
}

static inline VmathSoaVector4 vmathSoaM4GetCol0_V( VmathSoaMatrix4 mat )
{
    VmathSoaVector4 result;
    vmathSoaM4GetCol0(&result, &mat);
    return result;
}

static inline VmathSoaVector4 vmathSoaM4GetCol1_V( VmathSoaMatrix4 mat )
{
    VmathSoaVector4 result;
    vmathSoaM4GetCol1(&result, &mat);
    return result;
}

static inline VmathSoaVector4 vmathSoaM4GetCol2_V( VmathSoaMatrix4 mat )
{
    VmathSoaVector4 result;
    vmathSoaM4GetCol2(&result, &mat);
    return result;
}

static inline VmathSoaVector4 vmathSoaM4GetCol3_V( VmathSoaMatrix4 mat )
{
    VmathSoaVector4 result;
    vmathSoaM4GetCol3(&result, &mat);
    return result;
}

static inline VmathSoaVector4 vmathSoaM4GetCol_V( VmathSoaMatrix4 mat, int col )
{
    VmathSoaVector4 result;
    vmathSoaM4GetCol(&result, &mat, col);
    return result;
}

static inline VmathSoaVector4 vmathSoaM4GetRow_V( VmathSoaMatrix4 mat, int row )
{
    VmathSoaVector4 result;
    vmathSoaM4GetRow(&result, &mat, row);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4Transpose_V( VmathSoaMatrix4 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4Transpose(&result, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4Inverse_V( VmathSoaMatrix4 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4Inverse(&result, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4AffineInverse_V( VmathSoaMatrix4 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4AffineInverse(&result, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4OrthoInverse_V( VmathSoaMatrix4 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4OrthoInverse(&result, &mat);
    return result;
}

static inline vec_float4 vmathSoaM4Determinant_V( VmathSoaMatrix4 mat )
{
    return vmathSoaM4Determinant(&mat);
}

static inline VmathSoaMatrix4 vmathSoaM4Add_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1 )
{
    VmathSoaMatrix4 result;
    vmathSoaM4Add(&result, &mat0, &mat1);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4Sub_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1 )
{
    VmathSoaMatrix4 result;
    vmathSoaM4Sub(&result, &mat0, &mat1);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4Neg_V( VmathSoaMatrix4 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4Neg(&result, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4AbsPerElem_V( VmathSoaMatrix4 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4AbsPerElem(&result, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4ScalarMul_V( VmathSoaMatrix4 mat, vec_float4 scalar )
{
    VmathSoaMatrix4 result;
    vmathSoaM4ScalarMul(&result, &mat, scalar);
    return result;
}

static inline VmathSoaVector4 vmathSoaM4MulV4_V( VmathSoaMatrix4 mat, VmathSoaVector4 vec )
{
    VmathSoaVector4 result;
    vmathSoaM4MulV4(&result, &mat, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaM4MulV3_V( VmathSoaMatrix4 mat, VmathSoaVector3 vec )
{
    VmathSoaVector4 result;
    vmathSoaM4MulV3(&result, &mat, &vec);
    return result;
}

static inline VmathSoaVector4 vmathSoaM4MulP3_V( VmathSoaMatrix4 mat, VmathSoaPoint3 pnt )
{
    VmathSoaVector4 result;
    vmathSoaM4MulP3(&result, &mat, &pnt);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4Mul_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1 )
{
    VmathSoaMatrix4 result;
    vmathSoaM4Mul(&result, &mat0, &mat1);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MulT3_V( VmathSoaMatrix4 mat, VmathSoaTransform3 tfrm1 )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MulT3(&result, &mat, &tfrm1);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MulPerElem_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1 )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MulPerElem(&result, &mat0, &mat1);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeIdentity_V( )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeIdentity(&result);
    return result;
}

static inline void vmathSoaM4SetUpper3x3_V( VmathSoaMatrix4 *result, VmathSoaMatrix3 mat3 )
{
    vmathSoaM4SetUpper3x3(result, &mat3);
}

static inline VmathSoaMatrix3 vmathSoaM4GetUpper3x3_V( VmathSoaMatrix4 mat )
{
    VmathSoaMatrix3 result;
    vmathSoaM4GetUpper3x3(&result, &mat);
    return result;
}

static inline void vmathSoaM4SetTranslation_V( VmathSoaMatrix4 *result, VmathSoaVector3 translateVec )
{
    vmathSoaM4SetTranslation(result, &translateVec);
}

static inline VmathSoaVector3 vmathSoaM4GetTranslation_V( VmathSoaMatrix4 mat )
{
    VmathSoaVector3 result;
    vmathSoaM4GetTranslation(&result, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeRotationX_V( vec_float4 radians )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeRotationX(&result, radians);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeRotationY_V( vec_float4 radians )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeRotationY(&result, radians);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeRotationZ_V( vec_float4 radians )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeRotationZ(&result, radians);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeRotationZYX_V( VmathSoaVector3 radiansXYZ )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeRotationZYX(&result, &radiansXYZ);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeRotationAxis_V( vec_float4 radians, VmathSoaVector3 unitVec )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeRotationAxis(&result, radians, &unitVec);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeRotationQ_V( VmathSoaQuat unitQuat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeRotationQ(&result, &unitQuat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeScale_V( VmathSoaVector3 scaleVec )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeScale(&result, &scaleVec);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4AppendScale_V( VmathSoaMatrix4 mat, VmathSoaVector3 scaleVec )
{
    VmathSoaMatrix4 result;
    vmathSoaM4AppendScale(&result, &mat, &scaleVec);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4PrependScale_V( VmathSoaVector3 scaleVec, VmathSoaMatrix4 mat )
{
    VmathSoaMatrix4 result;
    vmathSoaM4PrependScale(&result, &scaleVec, &mat);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeTranslation_V( VmathSoaVector3 translateVec )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeTranslation(&result, &translateVec);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeLookAt_V( VmathSoaPoint3 eyePos, VmathSoaPoint3 lookAtPos, VmathSoaVector3 upVec )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeLookAt(&result, &eyePos, &lookAtPos, &upVec);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakePerspective_V( vec_float4 fovyRadians, vec_float4 aspect, vec_float4 zNear, vec_float4 zFar )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakePerspective(&result, fovyRadians, aspect, zNear, zFar);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeFrustum_V( vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeFrustum(&result, left, right, bottom, top, zNear, zFar);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4MakeOrthographic_V( vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar )
{
    VmathSoaMatrix4 result;
    vmathSoaM4MakeOrthographic(&result, left, right, bottom, top, zNear, zFar);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaM4Select_V( VmathSoaMatrix4 mat0, VmathSoaMatrix4 mat1, vec_uint4 select1 )
{
    VmathSoaMatrix4 result;
    vmathSoaM4Select(&result, &mat0, &mat1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaM4Print_V( VmathSoaMatrix4 mat )
{
    vmathSoaM4Print(&mat);
}

static inline void vmathSoaM4Prints_V( VmathSoaMatrix4 mat, const char *name )
{
    vmathSoaM4Prints(&mat, name);
}

#endif

static inline VmathSoaTransform3 vmathSoaT3MakeFromScalar_V( vec_float4 scalar )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeFromScalar(&result, scalar);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeFromCols_V( VmathSoaVector3 _col0, VmathSoaVector3 _col1, VmathSoaVector3 _col2, VmathSoaVector3 _col3 )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeFromCols(&result, &_col0, &_col1, &_col2, &_col3);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeFromM3V3_V( VmathSoaMatrix3 tfrm, VmathSoaVector3 translateVec )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeFromM3V3(&result, &tfrm, &translateVec);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeFromQV3_V( VmathSoaQuat unitQuat, VmathSoaVector3 translateVec )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeFromQV3(&result, &unitQuat, &translateVec);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeFromAos_V( VmathTransform3 tfrm )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeFromAos(&result, &tfrm);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeFrom4Aos_V( VmathTransform3 tfrm0, VmathTransform3 tfrm1, VmathTransform3 tfrm2, VmathTransform3 tfrm3 )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeFrom4Aos(&result, &tfrm0, &tfrm1, &tfrm2, &tfrm3);
    return result;
}

static inline void vmathSoaT3Get4Aos_V( VmathSoaTransform3 tfrm, VmathTransform3 *result0, VmathTransform3 *result1, VmathTransform3 *result2, VmathTransform3 *result3 )
{
    vmathSoaT3Get4Aos(&tfrm, result0, result1, result2, result3);
}

static inline void vmathSoaT3SetCol0_V( VmathSoaTransform3 *result, VmathSoaVector3 _col0 )
{
    vmathSoaT3SetCol0(result, &_col0);
}

static inline void vmathSoaT3SetCol1_V( VmathSoaTransform3 *result, VmathSoaVector3 _col1 )
{
    vmathSoaT3SetCol1(result, &_col1);
}

static inline void vmathSoaT3SetCol2_V( VmathSoaTransform3 *result, VmathSoaVector3 _col2 )
{
    vmathSoaT3SetCol2(result, &_col2);
}

static inline void vmathSoaT3SetCol3_V( VmathSoaTransform3 *result, VmathSoaVector3 _col3 )
{
    vmathSoaT3SetCol3(result, &_col3);
}

static inline void vmathSoaT3SetCol_V( VmathSoaTransform3 *result, int col, VmathSoaVector3 vec )
{
    vmathSoaT3SetCol(result, col, &vec);
}

static inline void vmathSoaT3SetRow_V( VmathSoaTransform3 *result, int row, VmathSoaVector4 vec )
{
    vmathSoaT3SetRow(result, row, &vec);
}

static inline void vmathSoaT3SetElem_V( VmathSoaTransform3 *result, int col, int row, vec_float4 val )
{
    vmathSoaT3SetElem(result, col, row, val);
}

static inline vec_float4 vmathSoaT3GetElem_V( VmathSoaTransform3 tfrm, int col, int row )
{
    return vmathSoaT3GetElem(&tfrm, col, row);
}

static inline VmathSoaVector3 vmathSoaT3GetCol0_V( VmathSoaTransform3 tfrm )
{
    VmathSoaVector3 result;
    vmathSoaT3GetCol0(&result, &tfrm);
    return result;
}

static inline VmathSoaVector3 vmathSoaT3GetCol1_V( VmathSoaTransform3 tfrm )
{
    VmathSoaVector3 result;
    vmathSoaT3GetCol1(&result, &tfrm);
    return result;
}

static inline VmathSoaVector3 vmathSoaT3GetCol2_V( VmathSoaTransform3 tfrm )
{
    VmathSoaVector3 result;
    vmathSoaT3GetCol2(&result, &tfrm);
    return result;
}

static inline VmathSoaVector3 vmathSoaT3GetCol3_V( VmathSoaTransform3 tfrm )
{
    VmathSoaVector3 result;
    vmathSoaT3GetCol3(&result, &tfrm);
    return result;
}

static inline VmathSoaVector3 vmathSoaT3GetCol_V( VmathSoaTransform3 tfrm, int col )
{
    VmathSoaVector3 result;
    vmathSoaT3GetCol(&result, &tfrm, col);
    return result;
}

static inline VmathSoaVector4 vmathSoaT3GetRow_V( VmathSoaTransform3 tfrm, int row )
{
    VmathSoaVector4 result;
    vmathSoaT3GetRow(&result, &tfrm, row);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3Inverse_V( VmathSoaTransform3 tfrm )
{
    VmathSoaTransform3 result;
    vmathSoaT3Inverse(&result, &tfrm);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3OrthoInverse_V( VmathSoaTransform3 tfrm )
{
    VmathSoaTransform3 result;
    vmathSoaT3OrthoInverse(&result, &tfrm);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3AbsPerElem_V( VmathSoaTransform3 tfrm )
{
    VmathSoaTransform3 result;
    vmathSoaT3AbsPerElem(&result, &tfrm);
    return result;
}

static inline VmathSoaVector3 vmathSoaT3MulV3_V( VmathSoaTransform3 tfrm, VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaT3MulV3(&result, &tfrm, &vec);
    return result;
}

static inline VmathSoaPoint3 vmathSoaT3MulP3_V( VmathSoaTransform3 tfrm, VmathSoaPoint3 pnt )
{
    VmathSoaPoint3 result;
    vmathSoaT3MulP3(&result, &tfrm, &pnt);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3Mul_V( VmathSoaTransform3 tfrm0, VmathSoaTransform3 tfrm1 )
{
    VmathSoaTransform3 result;
    vmathSoaT3Mul(&result, &tfrm0, &tfrm1);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MulPerElem_V( VmathSoaTransform3 tfrm0, VmathSoaTransform3 tfrm1 )
{
    VmathSoaTransform3 result;
    vmathSoaT3MulPerElem(&result, &tfrm0, &tfrm1);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeIdentity_V( )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeIdentity(&result);
    return result;
}

static inline void vmathSoaT3SetUpper3x3_V( VmathSoaTransform3 *result, VmathSoaMatrix3 tfrm )
{
    vmathSoaT3SetUpper3x3(result, &tfrm);
}

static inline VmathSoaMatrix3 vmathSoaT3GetUpper3x3_V( VmathSoaTransform3 tfrm )
{
    VmathSoaMatrix3 result;
    vmathSoaT3GetUpper3x3(&result, &tfrm);
    return result;
}

static inline void vmathSoaT3SetTranslation_V( VmathSoaTransform3 *result, VmathSoaVector3 translateVec )
{
    vmathSoaT3SetTranslation(result, &translateVec);
}

static inline VmathSoaVector3 vmathSoaT3GetTranslation_V( VmathSoaTransform3 tfrm )
{
    VmathSoaVector3 result;
    vmathSoaT3GetTranslation(&result, &tfrm);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeRotationX_V( vec_float4 radians )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeRotationX(&result, radians);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeRotationY_V( vec_float4 radians )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeRotationY(&result, radians);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeRotationZ_V( vec_float4 radians )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeRotationZ(&result, radians);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeRotationZYX_V( VmathSoaVector3 radiansXYZ )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeRotationZYX(&result, &radiansXYZ);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeRotationAxis_V( vec_float4 radians, VmathSoaVector3 unitVec )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeRotationAxis(&result, radians, &unitVec);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeRotationQ_V( VmathSoaQuat unitQuat )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeRotationQ(&result, &unitQuat);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeScale_V( VmathSoaVector3 scaleVec )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeScale(&result, &scaleVec);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3AppendScale_V( VmathSoaTransform3 tfrm, VmathSoaVector3 scaleVec )
{
    VmathSoaTransform3 result;
    vmathSoaT3AppendScale(&result, &tfrm, &scaleVec);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3PrependScale_V( VmathSoaVector3 scaleVec, VmathSoaTransform3 tfrm )
{
    VmathSoaTransform3 result;
    vmathSoaT3PrependScale(&result, &scaleVec, &tfrm);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3MakeTranslation_V( VmathSoaVector3 translateVec )
{
    VmathSoaTransform3 result;
    vmathSoaT3MakeTranslation(&result, &translateVec);
    return result;
}

static inline VmathSoaTransform3 vmathSoaT3Select_V( VmathSoaTransform3 tfrm0, VmathSoaTransform3 tfrm1, vec_uint4 select1 )
{
    VmathSoaTransform3 result;
    vmathSoaT3Select(&result, &tfrm0, &tfrm1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaT3Print_V( VmathSoaTransform3 tfrm )
{
    vmathSoaT3Print(&tfrm);
}

static inline void vmathSoaT3Prints_V( VmathSoaTransform3 tfrm, const char *name )
{
    vmathSoaT3Prints(&tfrm, name);
}

#endif

static inline VmathSoaQuat vmathSoaQMakeFromM3_V( VmathSoaMatrix3 tfrm )
{
    VmathSoaQuat result;
    vmathSoaQMakeFromM3(&result, &tfrm);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaV3Outer_V( VmathSoaVector3 tfrm0, VmathSoaVector3 tfrm1 )
{
    VmathSoaMatrix3 result;
    vmathSoaV3Outer(&result, &tfrm0, &tfrm1);
    return result;
}

static inline VmathSoaMatrix4 vmathSoaV4Outer_V( VmathSoaVector4 tfrm0, VmathSoaVector4 tfrm1 )
{
    VmathSoaMatrix4 result;
    vmathSoaV4Outer(&result, &tfrm0, &tfrm1);
    return result;
}

static inline VmathSoaVector3 vmathSoaV3RowMul_V( VmathSoaVector3 vec, VmathSoaMatrix3 mat )
{
    VmathSoaVector3 result;
    vmathSoaV3RowMul(&result, &vec, &mat);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaV3CrossMatrix_V( VmathSoaVector3 vec )
{
    VmathSoaMatrix3 result;
    vmathSoaV3CrossMatrix(&result, &vec);
    return result;
}

static inline VmathSoaMatrix3 vmathSoaV3CrossMatrixMul_V( VmathSoaVector3 vec, VmathSoaMatrix3 mat )
{
    VmathSoaMatrix3 result;
    vmathSoaV3CrossMatrixMul(&result, &vec, &mat);
    return result;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
