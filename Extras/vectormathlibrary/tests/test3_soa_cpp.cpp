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

using namespace Vectormath;
using namespace Vectormath::Soa;

void
Matrix3_methods_test()
{
    Matrix3 a_Matrix3, b_Matrix3;
    Matrix4 a_Matrix4, b_Matrix4;
    Transform3 a_Transform3, b_Transform3;
    Vector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3;
    Vector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4;
    Point3 a_Point3, b_Point3, c_Point3, d_Point3;
    Quat a_Quat, b_Quat, c_Quat, d_Quat;
    vec_float4 rndflt1, rndflt2, rndflt3, rndflt4, rndflt5, rndflt6;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Vector3 = Vector3( rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    c_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    d_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    print( a_Vector3, "set Vector3 with floats" );
    print( b_Vector3, "set Vector3 with floats" );
    print( c_Vector3, "set Vector3 with floats" );
    print( d_Vector3, "set Vector3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Vector4 = Vector4( rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    c_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    d_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    print( a_Vector4, "set Vector4 with floats" );
    print( b_Vector4, "set Vector4 with floats" );
    print( c_Vector4, "set Vector4 with floats" );
    print( d_Vector4, "set Vector4 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Point3 = Point3( rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    c_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    d_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    print( a_Point3, "set Point3 with floats" );
    print( b_Point3, "set Point3 with floats" );
    print( c_Point3, "set Point3 with floats" );
    print( d_Point3, "set Point3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Quat = Quat( rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    c_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    d_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    print( a_Quat, "set Quat with floats" );
    print( b_Quat, "set Quat with floats" );
    print( c_Quat, "set Quat with floats" );
    print( d_Quat, "set Quat with floats" );
    a_Matrix3 = Matrix3( a_Vector3, b_Vector3, c_Vector3 );
    b_Matrix3 = Matrix3( d_Vector3, a_Vector3, b_Vector3 );
    print( a_Matrix3, "set Matrix3 columns" );
    print( b_Matrix3, "set Matrix3 columns" );
    a_Matrix4 = Matrix4( a_Vector4, b_Vector4, c_Vector4, d_Vector4 );
    b_Matrix4 = Matrix4( d_Vector4, a_Vector4, b_Vector4, c_Vector4 );
    print( a_Matrix4, "set Matrix4 columns" );
    print( b_Matrix4, "set Matrix4 columns" );
    a_Transform3 = Transform3( a_Vector3, b_Vector3, c_Vector3, d_Vector3 );
    b_Transform3 = Transform3( d_Vector3, a_Vector3, b_Vector3, c_Vector3 );
    print( a_Transform3, "set Transform3 columns" );
    print( b_Transform3, "set Transform3 columns" );
    print( ( a_Matrix3 + b_Matrix3 ), "Matrix3 + Matrix3" );
    print( ( a_Matrix3 - b_Matrix3 ), "Matrix3 - Matrix3" );
    print( ( -a_Matrix3 ), "-Matrix3" );
    print( ( a_Matrix3 * randfloat() ), "Matrix3 * float" );
    print( ( randfloat() * a_Matrix3 ), "float * Matrix3" );
    print( ( a_Matrix3 * a_Vector3 ), "Matrix3 * Vector3" );
    print( ( a_Matrix3 * b_Matrix3 ), "Matrix3 * Matrix3" );
}

void
Matrix4_methods_test()
{
    Matrix3 a_Matrix3, b_Matrix3;
    Matrix4 a_Matrix4, b_Matrix4;
    Transform3 a_Transform3, b_Transform3;
    Vector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3;
    Vector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4;
    Point3 a_Point3, b_Point3, c_Point3, d_Point3;
    Quat a_Quat, b_Quat, c_Quat, d_Quat;
    vec_float4 rndflt1, rndflt2, rndflt3, rndflt4, rndflt5, rndflt6;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Vector3 = Vector3( rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    c_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    d_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    print( a_Vector3, "set Vector3 with floats" );
    print( b_Vector3, "set Vector3 with floats" );
    print( c_Vector3, "set Vector3 with floats" );
    print( d_Vector3, "set Vector3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Vector4 = Vector4( rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    c_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    d_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    print( a_Vector4, "set Vector4 with floats" );
    print( b_Vector4, "set Vector4 with floats" );
    print( c_Vector4, "set Vector4 with floats" );
    print( d_Vector4, "set Vector4 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Point3 = Point3( rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    c_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    d_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    print( a_Point3, "set Point3 with floats" );
    print( b_Point3, "set Point3 with floats" );
    print( c_Point3, "set Point3 with floats" );
    print( d_Point3, "set Point3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Quat = Quat( rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    c_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    d_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    print( a_Quat, "set Quat with floats" );
    print( b_Quat, "set Quat with floats" );
    print( c_Quat, "set Quat with floats" );
    print( d_Quat, "set Quat with floats" );
    a_Matrix3 = Matrix3( a_Vector3, b_Vector3, c_Vector3 );
    b_Matrix3 = Matrix3( d_Vector3, a_Vector3, b_Vector3 );
    print( a_Matrix3, "set Matrix3 columns" );
    print( b_Matrix3, "set Matrix3 columns" );
    a_Matrix4 = Matrix4( a_Vector4, b_Vector4, c_Vector4, d_Vector4 );
    b_Matrix4 = Matrix4( d_Vector4, a_Vector4, b_Vector4, c_Vector4 );
    print( a_Matrix4, "set Matrix4 columns" );
    print( b_Matrix4, "set Matrix4 columns" );
    a_Transform3 = Transform3( a_Vector3, b_Vector3, c_Vector3, d_Vector3 );
    b_Transform3 = Transform3( d_Vector3, a_Vector3, b_Vector3, c_Vector3 );
    print( a_Transform3, "set Transform3 columns" );
    print( b_Transform3, "set Transform3 columns" );
    print( ( a_Matrix4 + b_Matrix4 ), "Matrix4 + Matrix4" );
    print( ( a_Matrix4 - b_Matrix4 ), "Matrix4 - Matrix4" );
    print( ( -a_Matrix4 ), "-Matrix4" );
    print( ( a_Matrix4 * randfloat() ), "Matrix4 * float" );
    print( ( randfloat() * a_Matrix4 ), "float * Matrix4" );
    print( ( a_Matrix4 * a_Vector4 ), "Matrix4 * Vector4" );
    print( ( a_Matrix4 * a_Vector3 ), "Matrix4 * Vector3" );
    print( ( a_Matrix4 * a_Point3 ), "Matrix4 * Point3" );
    print( ( a_Matrix4 * b_Matrix4 ), "Matrix4 * Matrix4" );
    print( ( a_Matrix4 * b_Transform3 ), "Matrix4 * Transform3" );
}

void
Transform3_methods_test()
{
    Matrix3 a_Matrix3, b_Matrix3;
    Matrix4 a_Matrix4, b_Matrix4;
    Transform3 a_Transform3, b_Transform3;
    Vector3 a_Vector3, b_Vector3, c_Vector3, d_Vector3;
    Vector4 a_Vector4, b_Vector4, c_Vector4, d_Vector4;
    Point3 a_Point3, b_Point3, c_Point3, d_Point3;
    Quat a_Quat, b_Quat, c_Quat, d_Quat;
    vec_float4 rndflt1, rndflt2, rndflt3, rndflt4, rndflt5, rndflt6;
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Vector3 = Vector3( rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    c_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    d_Vector3 = Vector3( rndflt1, rndflt2, rndflt3 );
    print( a_Vector3, "set Vector3 with floats" );
    print( b_Vector3, "set Vector3 with floats" );
    print( c_Vector3, "set Vector3 with floats" );
    print( d_Vector3, "set Vector3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Vector4 = Vector4( rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    c_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    d_Vector4 = Vector4( rndflt1, rndflt2, rndflt3, rndflt4 );
    print( a_Vector4, "set Vector4 with floats" );
    print( b_Vector4, "set Vector4 with floats" );
    print( c_Vector4, "set Vector4 with floats" );
    print( d_Vector4, "set Vector4 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    a_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Point3 = Point3( rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    c_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    d_Point3 = Point3( rndflt1, rndflt2, rndflt3 );
    print( a_Point3, "set Point3 with floats" );
    print( b_Point3, "set Point3 with floats" );
    print( c_Point3, "set Point3 with floats" );
    print( d_Point3, "set Point3 with floats" );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    a_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    rndflt5 = randfloat();
    rndflt6 = randfloat();
    b_Quat = Quat( rndflt3, rndflt4, rndflt5, rndflt6 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    c_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    rndflt1 = randfloat();
    rndflt2 = randfloat();
    rndflt3 = randfloat();
    rndflt4 = randfloat();
    d_Quat = Quat( rndflt1, rndflt2, rndflt3, rndflt4 );
    print( a_Quat, "set Quat with floats" );
    print( b_Quat, "set Quat with floats" );
    print( c_Quat, "set Quat with floats" );
    print( d_Quat, "set Quat with floats" );
    a_Matrix3 = Matrix3( a_Vector3, b_Vector3, c_Vector3 );
    b_Matrix3 = Matrix3( d_Vector3, a_Vector3, b_Vector3 );
    print( a_Matrix3, "set Matrix3 columns" );
    print( b_Matrix3, "set Matrix3 columns" );
    a_Matrix4 = Matrix4( a_Vector4, b_Vector4, c_Vector4, d_Vector4 );
    b_Matrix4 = Matrix4( d_Vector4, a_Vector4, b_Vector4, c_Vector4 );
    print( a_Matrix4, "set Matrix4 columns" );
    print( b_Matrix4, "set Matrix4 columns" );
    a_Transform3 = Transform3( a_Vector3, b_Vector3, c_Vector3, d_Vector3 );
    b_Transform3 = Transform3( d_Vector3, a_Vector3, b_Vector3, c_Vector3 );
    print( a_Transform3, "set Transform3 columns" );
    print( b_Transform3, "set Transform3 columns" );
    print( ( a_Transform3 * a_Vector3 ), "Transform3 * Vector3" );
    print( ( a_Transform3 * a_Point3 ), "Transform3 * Point3" );
    print( ( a_Transform3 * b_Transform3 ), "Transform3 * Transform3" );
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
