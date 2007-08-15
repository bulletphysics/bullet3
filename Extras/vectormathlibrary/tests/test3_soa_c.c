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
    VmathSoaMatrix3 tmpM3_0, tmpM3_1, tmpM3_2, tmpM3_3, tmpM3_4, tmpM3_5;
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
    vmathSoaM3Add( &tmpM3_0, &a_Matrix3, &b_Matrix3 );
    vmathSoaM3Prints( &tmpM3_0, "Matrix3 + Matrix3" );
    vmathSoaM3Sub( &tmpM3_1, &a_Matrix3, &b_Matrix3 );
    vmathSoaM3Prints( &tmpM3_1, "Matrix3 - Matrix3" );
    vmathSoaM3Neg( &tmpM3_2, &a_Matrix3 );
    vmathSoaM3Prints( &tmpM3_2, "-Matrix3" );
    vmathSoaM3ScalarMul( &tmpM3_3, &a_Matrix3, randfloat() );
    vmathSoaM3Prints( &tmpM3_3, "Matrix3 * float" );
    vmathSoaM3ScalarMul( &tmpM3_4, &a_Matrix3, randfloat() );
    vmathSoaM3Prints( &tmpM3_4, "float * Matrix3" );
    vmathSoaM3MulV3( &tmpV3_0, &a_Matrix3, &a_Vector3 );
    vmathSoaV3Prints( &tmpV3_0, "Matrix3 * Vector3" );
    vmathSoaM3Mul( &tmpM3_5, &a_Matrix3, &b_Matrix3 );
    vmathSoaM3Prints( &tmpM3_5, "Matrix3 * Matrix3" );
}

void
Matrix4_methods_test()
{
    VmathSoaMatrix3 a_Matrix3, b_Matrix3;
    VmathSoaMatrix4 a_Matrix4, b_Matrix4;
    VmathSoaTransform3 a_Transform3, b_Transform3;
    VmathSoaMatrix4 tmpM4_0, tmpM4_1, tmpM4_2, tmpM4_3, tmpM4_4, tmpM4_5, tmpM4_6;
    VmathSoaVector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3;
    VmathSoaVector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4;
    VmathSoaPoint3 a_Point3, b_Point3, c_Point3, d_Point3;
    VmathSoaQuat a_Quat, b_Quat, c_Quat, d_Quat;
    VmathSoaVector4 tmpV4_0, tmpV4_1, tmpV4_2;
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
    vmathSoaM4Add( &tmpM4_0, &a_Matrix4, &b_Matrix4 );
    vmathSoaM4Prints( &tmpM4_0, "Matrix4 + Matrix4" );
    vmathSoaM4Sub( &tmpM4_1, &a_Matrix4, &b_Matrix4 );
    vmathSoaM4Prints( &tmpM4_1, "Matrix4 - Matrix4" );
    vmathSoaM4Neg( &tmpM4_2, &a_Matrix4 );
    vmathSoaM4Prints( &tmpM4_2, "-Matrix4" );
    vmathSoaM4ScalarMul( &tmpM4_3, &a_Matrix4, randfloat() );
    vmathSoaM4Prints( &tmpM4_3, "Matrix4 * float" );
    vmathSoaM4ScalarMul( &tmpM4_4, &a_Matrix4, randfloat() );
    vmathSoaM4Prints( &tmpM4_4, "float * Matrix4" );
    vmathSoaM4MulV4( &tmpV4_0, &a_Matrix4, &a_Vector4 );
    vmathSoaV4Prints( &tmpV4_0, "Matrix4 * Vector4" );
    vmathSoaM4MulV3( &tmpV4_1, &a_Matrix4, &a_Vector3 );
    vmathSoaV4Prints( &tmpV4_1, "Matrix4 * Vector3" );
    vmathSoaM4MulP3( &tmpV4_2, &a_Matrix4, &a_Point3 );
    vmathSoaV4Prints( &tmpV4_2, "Matrix4 * Point3" );
    vmathSoaM4Mul( &tmpM4_5, &a_Matrix4, &b_Matrix4 );
    vmathSoaM4Prints( &tmpM4_5, "Matrix4 * Matrix4" );
    vmathSoaM4MulT3( &tmpM4_6, &a_Matrix4, &b_Transform3 );
    vmathSoaM4Prints( &tmpM4_6, "Matrix4 * Transform3" );
}

void
Transform3_methods_test()
{
    VmathSoaMatrix3 a_Matrix3, b_Matrix3;
    VmathSoaMatrix4 a_Matrix4, b_Matrix4;
    VmathSoaTransform3 a_Transform3, b_Transform3, tmpT3_0;
    VmathSoaVector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3;
    VmathSoaVector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4;
    VmathSoaPoint3 a_Point3, b_Point3, c_Point3, d_Point3;
    VmathSoaQuat a_Quat, b_Quat, c_Quat, d_Quat;
    VmathSoaVector3 tmpV3_0;
    VmathSoaPoint3 tmpP3_0;
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
    vmathSoaT3MulV3( &tmpV3_0, &a_Transform3, &a_Vector3 );
    vmathSoaV3Prints( &tmpV3_0, "Transform3 * Vector3" );
    vmathSoaT3MulP3( &tmpP3_0, &a_Transform3, &a_Point3 );
    vmathSoaP3Prints( &tmpP3_0, "Transform3 * Point3" );
    vmathSoaT3Mul( &tmpT3_0, &a_Transform3, &b_Transform3 );
    vmathSoaT3Prints( &tmpT3_0, "Transform3 * Transform3" );
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
