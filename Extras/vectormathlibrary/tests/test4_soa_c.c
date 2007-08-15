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

#define _VECTORMATH_SOA_TEST

#include "vectormath_soa.h"
#include "test.h"

int iteration = 0;

void
Matrix3_methods_test()
{
    VmathSoaMatrix3 a_Matrix3, b_Matrix3;
    VmathSoaMatrix4 a_Matrix4, b_Matrix4;
    VmathSoaTransform3 a_Transform3, b_Transform3;
    VmathSoaMatrix3 tmpM3_0, tmpM3_1, tmpM3_2, tmpM3_3, tmpM3_4, tmpM3_5, tmpM3_6, tmpM3_7, tmpM3_8, tmpM3_9, tmpM3_10;
    VmathSoaVector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3;
    VmathSoaVector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4;
    VmathSoaPoint3 a_Point3, b_Point3, c_Point3, d_Point3;
    VmathSoaQuat a_Quat, b_Quat, c_Quat, d_Quat;
    VmathSoaVector3 tmpV3_0;
    vec_float4 rndflt1, rndflt2, rndflt3, rndflt4, rndflt5, rndflt6;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &a_Vector3, rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaV3MakeFromElems( &b_Vector3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &c_Vector3, rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &d_Vector3, rndflt1, rndflt2, rndflt3 );
    vmathSoaV3Prints( &a_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &b_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &c_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &d_Vector3, "set Vector3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &a_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaV4MakeFromElems( &b_Vector4, rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &c_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &d_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    vmathSoaV4Prints( &a_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &b_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &c_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &d_Vector4, "set Vector4 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &a_Point3, rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaP3MakeFromElems( &b_Point3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &c_Point3, rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &d_Point3, rndflt1, rndflt2, rndflt3 );
    vmathSoaP3Prints( &a_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &b_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &c_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &d_Point3, "set Point3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &a_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaQMakeFromElems( &b_Quat, rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &c_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &d_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    vmathSoaQPrints( &a_Quat, "set Quat with floats" );
    vmathSoaQPrints( &b_Quat, "set Quat with floats" );
    vmathSoaQPrints( &c_Quat, "set Quat with floats" );
    vmathSoaQPrints( &d_Quat, "set Quat with floats" );
    vmathSoaM3MakeFromCols( &a_Matrix3, &a_Vector3, &b_Vector3, &c_Vector3 );
    vmathSoaM3MakeFromCols( &b_Matrix3, &d_Vector3, &a_Vector3, &b_Vector3 );
    vmathSoaM3Prints( &a_Matrix3, "set Matrix3 columns" );
    vmathSoaM3Prints( &b_Matrix3, "set Matrix3 columns" );
    vmathSoaM4MakeFromCols( &a_Matrix4, &a_Vector4, &b_Vector4, &c_Vector4, &d_Vector4 );
    vmathSoaM4MakeFromCols( &b_Matrix4, &d_Vector4, &a_Vector4, &b_Vector4, &c_Vector4 );
    vmathSoaM4Prints( &a_Matrix4, "set Matrix4 columns" );
    vmathSoaM4Prints( &b_Matrix4, "set Matrix4 columns" );
    vmathSoaT3MakeFromCols( &a_Transform3, &a_Vector3, &b_Vector3, &c_Vector3, &d_Vector3 );
    vmathSoaT3MakeFromCols( &b_Transform3, &d_Vector3, &a_Vector3, &b_Vector3, &c_Vector3 );
    vmathSoaT3Prints( &a_Transform3, "set Transform3 columns" );
    vmathSoaT3Prints( &b_Transform3, "set Transform3 columns" );
    vmathSoaM3AppendScale( &tmpM3_0, &a_Matrix3, &a_Vector3 );
    vmathSoaM3Prints( &tmpM3_0, "appendScale Matrix3 Vector3" );
    vmathSoaM3PrependScale( &tmpM3_1, &a_Vector3, &a_Matrix3 );
    vmathSoaM3Prints( &tmpM3_1, "prependScale Vector3 Matrix3" );
    vmathSoaM3MulPerElem( &tmpM3_2, &a_Matrix3, &b_Matrix3 );
    vmathSoaM3Prints( &tmpM3_2, "mulPerElem Matrix3" );
    vmathSoaM3AbsPerElem( &tmpM3_3, &a_Matrix3 );
    vmathSoaM3Prints( &tmpM3_3, "absPerElem Matrix3" );
    vmathSoaM3Transpose( &tmpM3_4, &a_Matrix3 );
    vmathSoaM3Prints( &tmpM3_4, "transpose Matrix3" );
    vmathSoaM3Inverse( &tmpM3_5, &a_Matrix3 );
    vmathSoaM3Prints( &tmpM3_5, "inverse Matrix3" );
    vmathSoaM3Inverse( &tmpM3_6, &a_Matrix3 );
    vmathSoaM3Mul( &tmpM3_7, &tmpM3_6, &a_Matrix3 );
    vmathSoaM3Prints( &tmpM3_7, "inverse(Matrix3) * Matrix3" );
    printf("%f\n", getfloat(vmathSoaM3Determinant( &a_Matrix3 )) );
    vmathSoaV3Outer( &tmpM3_8, &a_Vector3, &b_Vector3 );
    vmathSoaM3Prints( &tmpM3_8, "outer Vector3" );
    vmathSoaV3RowMul( &tmpV3_0, &a_Vector3, &a_Matrix3 );
    vmathSoaV3Prints( &tmpV3_0, "rowMul Vector3" );
    vmathSoaV3CrossMatrix( &tmpM3_9, &a_Vector3 );
    vmathSoaM3Prints( &tmpM3_9, "crossMatrix" );
    vmathSoaV3CrossMatrixMul( &tmpM3_10, &a_Vector3, &a_Matrix3 );
    vmathSoaM3Prints( &tmpM3_10, "crossMatrixMul" );
}

void
Matrix4_methods_test()
{
    VmathSoaMatrix3 a_Matrix3, b_Matrix3;
    VmathSoaMatrix4 a_Matrix4, b_Matrix4;
    VmathSoaTransform3 a_Transform3, b_Transform3;
    VmathSoaMatrix4 tmpM4_0, tmpM4_1, tmpM4_2, tmpM4_3, tmpM4_4, tmpM4_5, tmpM4_6, tmpM4_7;
    VmathSoaMatrix3 tmpM3_0;
    VmathSoaMatrix4 tmpM4_8, tmpM4_9, tmpM4_10, tmpM4_11, tmpM4_12, tmpM4_13, tmpM4_14;
    VmathSoaVector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3;
    VmathSoaVector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4;
    VmathSoaPoint3 a_Point3, b_Point3, c_Point3, d_Point3;
    VmathSoaQuat a_Quat, b_Quat, c_Quat, d_Quat;
    VmathSoaVector4 tmpV4_0;
    VmathSoaQuat tmpQ_0;
    vec_float4 rndflt1, rndflt2, rndflt3, rndflt4, rndflt5, rndflt6;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &a_Vector3, rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaV3MakeFromElems( &b_Vector3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &c_Vector3, rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &d_Vector3, rndflt1, rndflt2, rndflt3 );
    vmathSoaV3Prints( &a_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &b_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &c_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &d_Vector3, "set Vector3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &a_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaV4MakeFromElems( &b_Vector4, rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &c_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &d_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    vmathSoaV4Prints( &a_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &b_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &c_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &d_Vector4, "set Vector4 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &a_Point3, rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaP3MakeFromElems( &b_Point3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &c_Point3, rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &d_Point3, rndflt1, rndflt2, rndflt3 );
    vmathSoaP3Prints( &a_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &b_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &c_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &d_Point3, "set Point3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &a_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaQMakeFromElems( &b_Quat, rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &c_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &d_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    vmathSoaQPrints( &a_Quat, "set Quat with floats" );
    vmathSoaQPrints( &b_Quat, "set Quat with floats" );
    vmathSoaQPrints( &c_Quat, "set Quat with floats" );
    vmathSoaQPrints( &d_Quat, "set Quat with floats" );
    vmathSoaM3MakeFromCols( &a_Matrix3, &a_Vector3, &b_Vector3, &c_Vector3 );
    vmathSoaM3MakeFromCols( &b_Matrix3, &d_Vector3, &a_Vector3, &b_Vector3 );
    vmathSoaM3Prints( &a_Matrix3, "set Matrix3 columns" );
    vmathSoaM3Prints( &b_Matrix3, "set Matrix3 columns" );
    vmathSoaM4MakeFromCols( &a_Matrix4, &a_Vector4, &b_Vector4, &c_Vector4, &d_Vector4 );
    vmathSoaM4MakeFromCols( &b_Matrix4, &d_Vector4, &a_Vector4, &b_Vector4, &c_Vector4 );
    vmathSoaM4Prints( &a_Matrix4, "set Matrix4 columns" );
    vmathSoaM4Prints( &b_Matrix4, "set Matrix4 columns" );
    vmathSoaT3MakeFromCols( &a_Transform3, &a_Vector3, &b_Vector3, &c_Vector3, &d_Vector3 );
    vmathSoaT3MakeFromCols( &b_Transform3, &d_Vector3, &a_Vector3, &b_Vector3, &c_Vector3 );
    vmathSoaT3Prints( &a_Transform3, "set Transform3 columns" );
    vmathSoaT3Prints( &b_Transform3, "set Transform3 columns" );
    vmathSoaM4AppendScale( &tmpM4_0, &a_Matrix4, &a_Vector3 );
    vmathSoaM4Prints( &tmpM4_0, "appendScale Matrix4 Vector3" );
    vmathSoaM4PrependScale( &tmpM4_1, &a_Vector3, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_1, "prependScale Vector3 Matrix4" );
    vmathSoaM4MulPerElem( &tmpM4_2, &a_Matrix4, &b_Matrix4 );
    vmathSoaM4Prints( &tmpM4_2, "mulPerElem Matrix4" );
    vmathSoaM4AbsPerElem( &tmpM4_3, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_3, "absPerElem Matrix4" );
    vmathSoaM4Transpose( &tmpM4_4, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_4, "transpose Matrix4" );
    vmathSoaM4Inverse( &tmpM4_5, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_5, "inverse Matrix4" );
    vmathSoaM4Inverse( &tmpM4_6, &a_Matrix4 );
    vmathSoaM4Mul( &tmpM4_7, &tmpM4_6, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_7, "inverse(Matrix4) * Matrix4" );
    vmathSoaV4MakeFromElems( &tmpV4_0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
    vmathSoaM4SetRow( &a_Matrix4, 3, &tmpV4_0 );
    vmathSoaQNormalize( &tmpQ_0, &a_Quat );
    vmathSoaM3MakeFromQ( &tmpM3_0, &tmpQ_0 );
    vmathSoaM4SetUpper3x3( &a_Matrix4, &tmpM3_0 );
    vmathSoaM4AffineInverse( &tmpM4_8, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_8, "affineInverse Matrix4" );
    vmathSoaM4AffineInverse( &tmpM4_9, &a_Matrix4 );
    vmathSoaM4Mul( &tmpM4_10, &tmpM4_9, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_10, "affineInverse(Matrix4) * Matrix4" );
    vmathSoaM4OrthoInverse( &tmpM4_11, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_11, "orthoInverse Matrix4" );
    vmathSoaM4OrthoInverse( &tmpM4_12, &a_Matrix4 );
    vmathSoaM4Mul( &tmpM4_13, &tmpM4_12, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_13, "orthoInverse(Matrix4) * Matrix4" );
    printf("%f\n", getfloat(vmathSoaM4Determinant( &a_Matrix4 )) );
    vmathSoaV4Outer( &tmpM4_14, &a_Vector4, &b_Vector4 );
    vmathSoaM4Prints( &tmpM4_14, "outer Vector4" );
}

void
Transform3_methods_test()
{
    VmathSoaMatrix3 a_Matrix3, b_Matrix3;
    VmathSoaMatrix4 a_Matrix4, b_Matrix4;
    VmathSoaTransform3 a_Transform3, b_Transform3, tmpT3_0, tmpT3_1, tmpT3_2, tmpT3_3, tmpT3_4, tmpT3_5, tmpT3_6;
    VmathSoaMatrix3 tmpM3_0;
    VmathSoaTransform3 tmpT3_7, tmpT3_8, tmpT3_9;
    VmathSoaVector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3;
    VmathSoaVector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4;
    VmathSoaPoint3 a_Point3, b_Point3, c_Point3, d_Point3;
    VmathSoaQuat a_Quat, b_Quat, c_Quat, d_Quat, tmpQ_0;
    vec_float4 rndflt1, rndflt2, rndflt3, rndflt4, rndflt5, rndflt6;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &a_Vector3, rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaV3MakeFromElems( &b_Vector3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &c_Vector3, rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaV3MakeFromElems( &d_Vector3, rndflt1, rndflt2, rndflt3 );
    vmathSoaV3Prints( &a_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &b_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &c_Vector3, "set Vector3 with floats" );
    vmathSoaV3Prints( &d_Vector3, "set Vector3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &a_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaV4MakeFromElems( &b_Vector4, rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &c_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaV4MakeFromElems( &d_Vector4, rndflt1, rndflt2, rndflt3, rndflt4 );
    vmathSoaV4Prints( &a_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &b_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &c_Vector4, "set Vector4 with floats" );
    vmathSoaV4Prints( &d_Vector4, "set Vector4 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &a_Point3, rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaP3MakeFromElems( &b_Point3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &c_Point3, rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    vmathSoaP3MakeFromElems( &d_Point3, rndflt1, rndflt2, rndflt3 );
    vmathSoaP3Prints( &a_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &b_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &c_Point3, "set Point3 with floats" );
    vmathSoaP3Prints( &d_Point3, "set Point3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &a_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    vmathSoaQMakeFromElems( &b_Quat, rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &c_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    vmathSoaQMakeFromElems( &d_Quat, rndflt1, rndflt2, rndflt3, rndflt4 );
    vmathSoaQPrints( &a_Quat, "set Quat with floats" );
    vmathSoaQPrints( &b_Quat, "set Quat with floats" );
    vmathSoaQPrints( &c_Quat, "set Quat with floats" );
    vmathSoaQPrints( &d_Quat, "set Quat with floats" );
    vmathSoaM3MakeFromCols( &a_Matrix3, &a_Vector3, &b_Vector3, &c_Vector3 );
    vmathSoaM3MakeFromCols( &b_Matrix3, &d_Vector3, &a_Vector3, &b_Vector3 );
    vmathSoaM3Prints( &a_Matrix3, "set Matrix3 columns" );
    vmathSoaM3Prints( &b_Matrix3, "set Matrix3 columns" );
    vmathSoaM4MakeFromCols( &a_Matrix4, &a_Vector4, &b_Vector4, &c_Vector4, &d_Vector4 );
    vmathSoaM4MakeFromCols( &b_Matrix4, &d_Vector4, &a_Vector4, &b_Vector4, &c_Vector4 );
    vmathSoaM4Prints( &a_Matrix4, "set Matrix4 columns" );
    vmathSoaM4Prints( &b_Matrix4, "set Matrix4 columns" );
    vmathSoaT3MakeFromCols( &a_Transform3, &a_Vector3, &b_Vector3, &c_Vector3, &d_Vector3 );
    vmathSoaT3MakeFromCols( &b_Transform3, &d_Vector3, &a_Vector3, &b_Vector3, &c_Vector3 );
    vmathSoaT3Prints( &a_Transform3, "set Transform3 columns" );
    vmathSoaT3Prints( &b_Transform3, "set Transform3 columns" );
    vmathSoaT3AppendScale( &tmpT3_0, &a_Transform3, &a_Vector3 );
    vmathSoaT3Prints( &tmpT3_0, "appendScale Transform3 Vector3" );
    vmathSoaT3PrependScale( &tmpT3_1, &a_Vector3, &a_Transform3 );
    vmathSoaT3Prints( &tmpT3_1, "prependScale Vector3 Transform3" );
    vmathSoaT3MulPerElem( &tmpT3_2, &a_Transform3, &b_Transform3 );
    vmathSoaT3Prints( &tmpT3_2, "mulPerElem Transform3" );
    vmathSoaT3AbsPerElem( &tmpT3_3, &a_Transform3 );
    vmathSoaT3Prints( &tmpT3_3, "absPerElem Transform3" );
    vmathSoaT3Inverse( &tmpT3_4, &a_Transform3 );
    vmathSoaT3Prints( &tmpT3_4, "inverse Transform3" );
    vmathSoaT3Inverse( &tmpT3_5, &a_Transform3 );
    vmathSoaT3Mul( &tmpT3_6, &tmpT3_5, &a_Transform3 );
    vmathSoaT3Prints( &tmpT3_6, "inverse(Transform3) * Transform3" );
    vmathSoaQNormalize( &tmpQ_0, &a_Quat );
    vmathSoaM3MakeFromQ( &tmpM3_0, &tmpQ_0 );
    vmathSoaT3SetUpper3x3( &a_Transform3, &tmpM3_0 );
    vmathSoaT3OrthoInverse( &tmpT3_7, &a_Transform3 );
    vmathSoaT3Prints( &tmpT3_7, "orthoInverse Transform3" );
    vmathSoaT3OrthoInverse( &tmpT3_8, &a_Transform3 );
    vmathSoaT3Mul( &tmpT3_9, &tmpT3_8, &a_Transform3 );
    vmathSoaT3Prints( &tmpT3_9, "orthoInverse(Transform3) * Transform3" );
}

int main()
{
    int i;
    printf("\n __begin__ \n");
    for ( i = 0; i < 2; i++ ) {
        Matrix3_methods_test();
        Matrix4_methods_test();
        Transform3_methods_test();
    }
    printf("\n __end__ \n");
    return 0;
}
