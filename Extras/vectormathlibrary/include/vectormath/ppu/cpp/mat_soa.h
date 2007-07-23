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

#ifndef _VECTORMATH_MAT_SOA_CPP_H
#define _VECTORMATH_MAT_SOA_CPP_H

namespace Vectormath {
namespace Soa {

//-----------------------------------------------------------------------------
// Constants

#define _VECTORMATH_PI_OVER_2 1.570796327f

//-----------------------------------------------------------------------------
// Definitions

inline Matrix3::Matrix3( const Matrix3 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
}

inline Matrix3::Matrix3( vec_float4 scalar )
{
    mCol0 = Vector3( scalar );
    mCol1 = Vector3( scalar );
    mCol2 = Vector3( scalar );
}

inline Matrix3::Matrix3( const Quat & unitQuat )
{
    vec_float4 qx, qy, qz, qw, qx2, qy2, qz2, qxqx2, qyqy2, qzqz2, qxqy2, qyqz2, qzqw2, qxqz2, qyqw2, qxqw2;
    qx = unitQuat.getX();
    qy = unitQuat.getY();
    qz = unitQuat.getZ();
    qw = unitQuat.getW();
    qx2 = vec_add( qx, qx );
    qy2 = vec_add( qy, qy );
    qz2 = vec_add( qz, qz );
    qxqx2 = vec_madd( qx, qx2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qxqy2 = vec_madd( qx, qy2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qxqz2 = vec_madd( qx, qz2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qxqw2 = vec_madd( qw, qx2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qyqy2 = vec_madd( qy, qy2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qyqz2 = vec_madd( qy, qz2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qyqw2 = vec_madd( qw, qy2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qzqz2 = vec_madd( qz, qz2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qzqw2 = vec_madd( qw, qz2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol0 = Vector3( vec_sub( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), qyqy2 ), qzqz2 ), vec_add( qxqy2, qzqw2 ), vec_sub( qxqz2, qyqw2 ) );
    mCol1 = Vector3( vec_sub( qxqy2, qzqw2 ), vec_sub( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), qxqx2 ), qzqz2 ), vec_add( qyqz2, qxqw2 ) );
    mCol2 = Vector3( vec_add( qxqz2, qyqw2 ), vec_sub( qyqz2, qxqw2 ), vec_sub( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), qxqx2 ), qyqy2 ) );
}

inline Matrix3::Matrix3( const Vector3 & _col0, const Vector3 & _col1, const Vector3 & _col2 )
{
    mCol0 = _col0;
    mCol1 = _col1;
    mCol2 = _col2;
}

inline Matrix3::Matrix3( const Aos::Matrix3 & mat )
{
    mCol0 = Vector3( mat.getCol0() );
    mCol1 = Vector3( mat.getCol1() );
    mCol2 = Vector3( mat.getCol2() );
}

inline Matrix3::Matrix3( const Aos::Matrix3 & mat0, const Aos::Matrix3 & mat1, const Aos::Matrix3 & mat2, const Aos::Matrix3 & mat3 )
{
    mCol0 = Vector3( mat0.getCol0(), mat1.getCol0(), mat2.getCol0(), mat3.getCol0() );
    mCol1 = Vector3( mat0.getCol1(), mat1.getCol1(), mat2.getCol1(), mat3.getCol1() );
    mCol2 = Vector3( mat0.getCol2(), mat1.getCol2(), mat2.getCol2(), mat3.getCol2() );
}

inline void Matrix3::get4Aos( Aos::Matrix3 & result0, Aos::Matrix3 & result1, Aos::Matrix3 & result2, Aos::Matrix3 & result3 ) const
{
    Aos::Vector3 tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3;
    mCol0.get4Aos( tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3 );
    result0.setCol0( tmpV3_0 );
    result1.setCol0( tmpV3_1 );
    result2.setCol0( tmpV3_2 );
    result3.setCol0( tmpV3_3 );
    mCol1.get4Aos( tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3 );
    result0.setCol1( tmpV3_0 );
    result1.setCol1( tmpV3_1 );
    result2.setCol1( tmpV3_2 );
    result3.setCol1( tmpV3_3 );
    mCol2.get4Aos( tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3 );
    result0.setCol2( tmpV3_0 );
    result1.setCol2( tmpV3_1 );
    result2.setCol2( tmpV3_2 );
    result3.setCol2( tmpV3_3 );
}

inline Matrix3 & Matrix3::setCol0( const Vector3 & _col0 )
{
    mCol0 = _col0;
    return *this;
}

inline Matrix3 & Matrix3::setCol1( const Vector3 & _col1 )
{
    mCol1 = _col1;
    return *this;
}

inline Matrix3 & Matrix3::setCol2( const Vector3 & _col2 )
{
    mCol2 = _col2;
    return *this;
}

inline Matrix3 & Matrix3::setCol( int col, const Vector3 & vec )
{
    *(&mCol0 + col) = vec;
    return *this;
}

inline Matrix3 & Matrix3::setRow( int row, const Vector3 & vec )
{
    mCol0.setElem( row, vec.getElem( 0 ) );
    mCol1.setElem( row, vec.getElem( 1 ) );
    mCol2.setElem( row, vec.getElem( 2 ) );
    return *this;
}

inline Matrix3 & Matrix3::setElem( int col, int row, vec_float4 val )
{
    Vector3 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

inline vec_float4 Matrix3::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

inline const Vector3 Matrix3::getCol0( ) const
{
    return mCol0;
}

inline const Vector3 Matrix3::getCol1( ) const
{
    return mCol1;
}

inline const Vector3 Matrix3::getCol2( ) const
{
    return mCol2;
}

inline const Vector3 Matrix3::getCol( int col ) const
{
    return *(&mCol0 + col);
}

inline const Vector3 Matrix3::getRow( int row ) const
{
    return Vector3( mCol0.getElem( row ), mCol1.getElem( row ), mCol2.getElem( row ) );
}

inline Vector3 & Matrix3::operator []( int col )
{
    return *(&mCol0 + col);
}

inline const Vector3 Matrix3::operator []( int col ) const
{
    return *(&mCol0 + col);
}

inline Matrix3 & Matrix3::operator =( const Matrix3 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
    return *this;
}

inline const Matrix3 transpose( const Matrix3 & mat )
{
    return Matrix3(
        Vector3( mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX() ),
        Vector3( mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY() ),
        Vector3( mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ() )
    );
}

inline const Matrix3 inverse( const Matrix3 & mat )
{
    Vector3 tmp0, tmp1, tmp2;
    vec_float4 detinv;
    tmp0 = cross( mat.getCol1(), mat.getCol2() );
    tmp1 = cross( mat.getCol2(), mat.getCol0() );
    tmp2 = cross( mat.getCol0(), mat.getCol1() );
    detinv = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), dot( mat.getCol2(), tmp2 ) );
    return Matrix3(
        Vector3( vec_madd( tmp0.getX(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp1.getX(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp2.getX(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        Vector3( vec_madd( tmp0.getY(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp1.getY(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp2.getY(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        Vector3( vec_madd( tmp0.getZ(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp1.getZ(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp2.getZ(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
    );
}

inline vec_float4 determinant( const Matrix3 & mat )
{
    return dot( mat.getCol2(), cross( mat.getCol0(), mat.getCol1() ) );
}

inline const Matrix3 Matrix3::operator +( const Matrix3 & mat ) const
{
    return Matrix3(
        ( mCol0 + mat.mCol0 ),
        ( mCol1 + mat.mCol1 ),
        ( mCol2 + mat.mCol2 )
    );
}

inline const Matrix3 Matrix3::operator -( const Matrix3 & mat ) const
{
    return Matrix3(
        ( mCol0 - mat.mCol0 ),
        ( mCol1 - mat.mCol1 ),
        ( mCol2 - mat.mCol2 )
    );
}

inline Matrix3 & Matrix3::operator +=( const Matrix3 & mat )
{
    *this = *this + mat;
    return *this;
}

inline Matrix3 & Matrix3::operator -=( const Matrix3 & mat )
{
    *this = *this - mat;
    return *this;
}

inline const Matrix3 Matrix3::operator -( ) const
{
    return Matrix3(
        ( -mCol0 ),
        ( -mCol1 ),
        ( -mCol2 )
    );
}

inline const Matrix3 absPerElem( const Matrix3 & mat )
{
    return Matrix3(
        absPerElem( mat.getCol0() ),
        absPerElem( mat.getCol1() ),
        absPerElem( mat.getCol2() )
    );
}

inline const Matrix3 Matrix3::operator *( vec_float4 scalar ) const
{
    return Matrix3(
        ( mCol0 * scalar ),
        ( mCol1 * scalar ),
        ( mCol2 * scalar )
    );
}

inline Matrix3 & Matrix3::operator *=( vec_float4 scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const Matrix3 operator *( vec_float4 scalar, const Matrix3 & mat )
{
    return mat * scalar;
}

inline const Vector3 Matrix3::operator *( const Vector3 & vec ) const
{
    return Vector3(
        vec_add( vec_add( vec_madd( mCol0.getX(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getX(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getX(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( mCol0.getY(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getY(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getY(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( mCol0.getZ(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getZ(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getZ(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
    );
}

inline const Matrix3 Matrix3::operator *( const Matrix3 & mat ) const
{
    return Matrix3(
        ( *this * mat.mCol0 ),
        ( *this * mat.mCol1 ),
        ( *this * mat.mCol2 )
    );
}

inline Matrix3 & Matrix3::operator *=( const Matrix3 & mat )
{
    *this = *this * mat;
    return *this;
}

inline const Matrix3 mulPerElem( const Matrix3 & mat0, const Matrix3 & mat1 )
{
    return Matrix3(
        mulPerElem( mat0.getCol0(), mat1.getCol0() ),
        mulPerElem( mat0.getCol1(), mat1.getCol1() ),
        mulPerElem( mat0.getCol2(), mat1.getCol2() )
    );
}

inline const Matrix3 Matrix3::identity( )
{
    return Matrix3(
        Vector3::xAxis( ),
        Vector3::yAxis( ),
        Vector3::zAxis( )
    );
}

inline const Matrix3 Matrix3::rotationX( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix3(
        Vector3::xAxis( ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c, s ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), negatef4( s ), c )
    );
}

inline const Matrix3 Matrix3::rotationY( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix3(
        Vector3( c, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), negatef4( s ) ),
        Vector3::yAxis( ),
        Vector3( s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c )
    );
}

inline const Matrix3 Matrix3::rotationZ( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix3(
        Vector3( c, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector3( negatef4( s ), c, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector3::zAxis( )
    );
}

inline const Matrix3 Matrix3::rotationZYX( const Vector3 & radiansXYZ )
{
    vec_float4 sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sincosf4( radiansXYZ.getX(), &sX, &cX );
    sincosf4( radiansXYZ.getY(), &sY, &cY );
    sincosf4( radiansXYZ.getZ(), &sZ, &cZ );
    tmp0 = vec_madd( cZ, sY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    tmp1 = vec_madd( sZ, sY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    return Matrix3(
        Vector3( vec_madd( cZ, cY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, cY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), negatef4( sY ) ),
        Vector3( vec_sub( vec_madd( tmp0, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_add( vec_madd( tmp1, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( cZ, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( cY, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        Vector3( vec_add( vec_madd( tmp0, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_sub( vec_madd( tmp1, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( cZ, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( cY, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
    );
}

inline const Matrix3 Matrix3::rotation( vec_float4 radians, const Vector3 & unitVec )
{
    vec_float4 x, y, z, s, c, oneMinusC, xy, yz, zx;
    sincosf4( radians, &s, &c );
    x = unitVec.getX();
    y = unitVec.getY();
    z = unitVec.getZ();
    xy = vec_madd( x, y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    yz = vec_madd( y, z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    zx = vec_madd( z, x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    oneMinusC = vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), c );
    return Matrix3(
        Vector3( vec_add( vec_madd( vec_madd( x, x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), c ), vec_add( vec_madd( xy, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( z, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_sub( vec_madd( zx, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( y, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) ),
        Vector3( vec_sub( vec_madd( xy, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( z, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_add( vec_madd( vec_madd( y, y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), c ), vec_add( vec_madd( yz, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( x, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) ),
        Vector3( vec_add( vec_madd( zx, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( y, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_sub( vec_madd( yz, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( x, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_add( vec_madd( vec_madd( z, z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), c ) )
    );
}

inline const Matrix3 Matrix3::rotation( const Quat & unitQuat )
{
    return Matrix3( unitQuat );
}

inline const Matrix3 Matrix3::scale( const Vector3 & scaleVec )
{
    return Matrix3(
        Vector3( scaleVec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), scaleVec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), scaleVec.getZ() )
    );
}

inline const Matrix3 appendScale( const Matrix3 & mat, const Vector3 & scaleVec )
{
    return Matrix3(
        ( mat.getCol0() * scaleVec.getX( ) ),
        ( mat.getCol1() * scaleVec.getY( ) ),
        ( mat.getCol2() * scaleVec.getZ( ) )
    );
}

inline const Matrix3 prependScale( const Vector3 & scaleVec, const Matrix3 & mat )
{
    return Matrix3(
        mulPerElem( mat.getCol0(), scaleVec ),
        mulPerElem( mat.getCol1(), scaleVec ),
        mulPerElem( mat.getCol2(), scaleVec )
    );
}

inline const Matrix3 select( const Matrix3 & mat0, const Matrix3 & mat1, vec_uint4 select1 )
{
    return Matrix3(
        select( mat0.getCol0(), mat1.getCol0(), select1 ),
        select( mat0.getCol1(), mat1.getCol1(), select1 ),
        select( mat0.getCol2(), mat1.getCol2(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

inline void print( const Matrix3 & mat )
{
    Aos::Matrix3 mat0, mat1, mat2, mat3;
    mat.get4Aos( mat0, mat1, mat2, mat3 );
    printf("slot 0:\n");
    print( mat0 );
    printf("slot 1:\n");
    print( mat1 );
    printf("slot 2:\n");
    print( mat2 );
    printf("slot 3:\n");
    print( mat3 );
}

inline void print( const Matrix3 & mat, const char * name )
{
    printf("%s:\n", name);
    print( mat );
}

#endif

inline Matrix4::Matrix4( const Matrix4 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
    mCol3 = mat.mCol3;
}

inline Matrix4::Matrix4( vec_float4 scalar )
{
    mCol0 = Vector4( scalar );
    mCol1 = Vector4( scalar );
    mCol2 = Vector4( scalar );
    mCol3 = Vector4( scalar );
}

inline Matrix4::Matrix4( const Transform3 & mat )
{
    mCol0 = Vector4( mat.getCol0(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol1 = Vector4( mat.getCol1(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol2 = Vector4( mat.getCol2(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol3 = Vector4( mat.getCol3(), ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
}

inline Matrix4::Matrix4( const Vector4 & _col0, const Vector4 & _col1, const Vector4 & _col2, const Vector4 & _col3 )
{
    mCol0 = _col0;
    mCol1 = _col1;
    mCol2 = _col2;
    mCol3 = _col3;
}

inline Matrix4::Matrix4( const Matrix3 & mat, const Vector3 & translateVec )
{
    mCol0 = Vector4( mat.getCol0(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol1 = Vector4( mat.getCol1(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol2 = Vector4( mat.getCol2(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol3 = Vector4( translateVec, ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
}

inline Matrix4::Matrix4( const Quat & unitQuat, const Vector3 & translateVec )
{
    Matrix3 mat;
    mat = Matrix3( unitQuat );
    mCol0 = Vector4( mat.getCol0(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol1 = Vector4( mat.getCol1(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol2 = Vector4( mat.getCol2(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    mCol3 = Vector4( translateVec, ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
}

inline Matrix4::Matrix4( const Aos::Matrix4 & mat )
{
    mCol0 = Vector4( mat.getCol0() );
    mCol1 = Vector4( mat.getCol1() );
    mCol2 = Vector4( mat.getCol2() );
    mCol3 = Vector4( mat.getCol3() );
}

inline Matrix4::Matrix4( const Aos::Matrix4 & mat0, const Aos::Matrix4 & mat1, const Aos::Matrix4 & mat2, const Aos::Matrix4 & mat3 )
{
    mCol0 = Vector4( mat0.getCol0(), mat1.getCol0(), mat2.getCol0(), mat3.getCol0() );
    mCol1 = Vector4( mat0.getCol1(), mat1.getCol1(), mat2.getCol1(), mat3.getCol1() );
    mCol2 = Vector4( mat0.getCol2(), mat1.getCol2(), mat2.getCol2(), mat3.getCol2() );
    mCol3 = Vector4( mat0.getCol3(), mat1.getCol3(), mat2.getCol3(), mat3.getCol3() );
}

inline void Matrix4::get4Aos( Aos::Matrix4 & result0, Aos::Matrix4 & result1, Aos::Matrix4 & result2, Aos::Matrix4 & result3 ) const
{
    Aos::Vector4 tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3;
    mCol0.get4Aos( tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3 );
    result0.setCol0( tmpV4_0 );
    result1.setCol0( tmpV4_1 );
    result2.setCol0( tmpV4_2 );
    result3.setCol0( tmpV4_3 );
    mCol1.get4Aos( tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3 );
    result0.setCol1( tmpV4_0 );
    result1.setCol1( tmpV4_1 );
    result2.setCol1( tmpV4_2 );
    result3.setCol1( tmpV4_3 );
    mCol2.get4Aos( tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3 );
    result0.setCol2( tmpV4_0 );
    result1.setCol2( tmpV4_1 );
    result2.setCol2( tmpV4_2 );
    result3.setCol2( tmpV4_3 );
    mCol3.get4Aos( tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3 );
    result0.setCol3( tmpV4_0 );
    result1.setCol3( tmpV4_1 );
    result2.setCol3( tmpV4_2 );
    result3.setCol3( tmpV4_3 );
}

inline Matrix4 & Matrix4::setCol0( const Vector4 & _col0 )
{
    mCol0 = _col0;
    return *this;
}

inline Matrix4 & Matrix4::setCol1( const Vector4 & _col1 )
{
    mCol1 = _col1;
    return *this;
}

inline Matrix4 & Matrix4::setCol2( const Vector4 & _col2 )
{
    mCol2 = _col2;
    return *this;
}

inline Matrix4 & Matrix4::setCol3( const Vector4 & _col3 )
{
    mCol3 = _col3;
    return *this;
}

inline Matrix4 & Matrix4::setCol( int col, const Vector4 & vec )
{
    *(&mCol0 + col) = vec;
    return *this;
}

inline Matrix4 & Matrix4::setRow( int row, const Vector4 & vec )
{
    mCol0.setElem( row, vec.getElem( 0 ) );
    mCol1.setElem( row, vec.getElem( 1 ) );
    mCol2.setElem( row, vec.getElem( 2 ) );
    mCol3.setElem( row, vec.getElem( 3 ) );
    return *this;
}

inline Matrix4 & Matrix4::setElem( int col, int row, vec_float4 val )
{
    Vector4 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

inline vec_float4 Matrix4::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

inline const Vector4 Matrix4::getCol0( ) const
{
    return mCol0;
}

inline const Vector4 Matrix4::getCol1( ) const
{
    return mCol1;
}

inline const Vector4 Matrix4::getCol2( ) const
{
    return mCol2;
}

inline const Vector4 Matrix4::getCol3( ) const
{
    return mCol3;
}

inline const Vector4 Matrix4::getCol( int col ) const
{
    return *(&mCol0 + col);
}

inline const Vector4 Matrix4::getRow( int row ) const
{
    return Vector4( mCol0.getElem( row ), mCol1.getElem( row ), mCol2.getElem( row ), mCol3.getElem( row ) );
}

inline Vector4 & Matrix4::operator []( int col )
{
    return *(&mCol0 + col);
}

inline const Vector4 Matrix4::operator []( int col ) const
{
    return *(&mCol0 + col);
}

inline Matrix4 & Matrix4::operator =( const Matrix4 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
    mCol3 = mat.mCol3;
    return *this;
}

inline const Matrix4 transpose( const Matrix4 & mat )
{
    return Matrix4(
        Vector4( mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX(), mat.getCol3().getX() ),
        Vector4( mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY(), mat.getCol3().getY() ),
        Vector4( mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ(), mat.getCol3().getZ() ),
        Vector4( mat.getCol0().getW(), mat.getCol1().getW(), mat.getCol2().getW(), mat.getCol3().getW() )
    );
}

inline const Matrix4 inverse( const Matrix4 & mat )
{
    Vector4 res0, res1, res2, res3;
    vec_float4 mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, detInv;
    mA = mat.getCol0().getX();
    mB = mat.getCol0().getY();
    mC = mat.getCol0().getZ();
    mD = mat.getCol0().getW();
    mE = mat.getCol1().getX();
    mF = mat.getCol1().getY();
    mG = mat.getCol1().getZ();
    mH = mat.getCol1().getW();
    mI = mat.getCol2().getX();
    mJ = mat.getCol2().getY();
    mK = mat.getCol2().getZ();
    mL = mat.getCol2().getW();
    mM = mat.getCol3().getX();
    mN = mat.getCol3().getY();
    mO = mat.getCol3().getZ();
    mP = mat.getCol3().getW();
    tmp0 = vec_sub( vec_madd( mK, mD, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mC, mL, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp1 = vec_sub( vec_madd( mO, mH, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mG, mP, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp2 = vec_sub( vec_madd( mB, mK, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mJ, mC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp3 = vec_sub( vec_madd( mF, mO, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mN, mG, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp4 = vec_sub( vec_madd( mJ, mD, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mB, mL, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp5 = vec_sub( vec_madd( mN, mH, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mF, mP, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res0.setX( vec_sub( vec_sub( vec_madd( mJ, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mL, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mK, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) );
    res0.setY( vec_sub( vec_sub( vec_madd( mN, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mP, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mO, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) );
    res0.setZ( vec_sub( vec_add( vec_madd( mD, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mC, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mB, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) );
    res0.setW( vec_sub( vec_add( vec_madd( mH, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mG, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mF, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) );
    detInv = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec_add( vec_add( vec_add( vec_madd( mA, res0.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mE, res0.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mI, res0.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mM, res0.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) );
    res1.setX( vec_madd( mI, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res1.setY( vec_madd( mM, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res1.setZ( vec_madd( mA, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res1.setW( vec_madd( mE, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res3.setX( vec_madd( mI, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res3.setY( vec_madd( mM, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res3.setZ( vec_madd( mA, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res3.setW( vec_madd( mE, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res2.setX( vec_madd( mI, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res2.setY( vec_madd( mM, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res2.setZ( vec_madd( mA, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res2.setW( vec_madd( mE, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp0 = vec_sub( vec_madd( mI, mB, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mA, mJ, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp1 = vec_sub( vec_madd( mM, mF, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mE, mN, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp2 = vec_sub( vec_madd( mI, mD, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mA, mL, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp3 = vec_sub( vec_madd( mM, mH, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mE, mP, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp4 = vec_sub( vec_madd( mI, mC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mA, mK, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp5 = vec_sub( vec_madd( mM, mG, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mE, mO, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    res2.setX( vec_add( vec_sub( vec_madd( mL, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mJ, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res2.getX() ) );
    res2.setY( vec_add( vec_sub( vec_madd( mP, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mN, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res2.getY() ) );
    res2.setZ( vec_sub( vec_sub( vec_madd( mB, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mD, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res2.getZ() ) );
    res2.setW( vec_sub( vec_sub( vec_madd( mF, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mH, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res2.getW() ) );
    res3.setX( vec_add( vec_sub( vec_madd( mJ, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mK, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res3.getX() ) );
    res3.setY( vec_add( vec_sub( vec_madd( mN, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mO, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res3.getY() ) );
    res3.setZ( vec_sub( vec_sub( vec_madd( mC, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mB, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res3.getZ() ) );
    res3.setW( vec_sub( vec_sub( vec_madd( mG, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mF, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res3.getW() ) );
    res1.setX( vec_sub( vec_sub( vec_madd( mK, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mL, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res1.getX() ) );
    res1.setY( vec_sub( vec_sub( vec_madd( mO, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mP, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res1.getY() ) );
    res1.setZ( vec_add( vec_sub( vec_madd( mD, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mC, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res1.getZ() ) );
    res1.setW( vec_add( vec_sub( vec_madd( mH, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mG, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), res1.getW() ) );
    return Matrix4(
        ( res0 * detInv ),
        ( res1 * detInv ),
        ( res2 * detInv ),
        ( res3 * detInv )
    );
}

inline const Matrix4 affineInverse( const Matrix4 & mat )
{
    Transform3 affineMat;
    affineMat.setCol0( mat.getCol0().getXYZ( ) );
    affineMat.setCol1( mat.getCol1().getXYZ( ) );
    affineMat.setCol2( mat.getCol2().getXYZ( ) );
    affineMat.setCol3( mat.getCol3().getXYZ( ) );
    return Matrix4( inverse( affineMat ) );
}

inline const Matrix4 orthoInverse( const Matrix4 & mat )
{
    Transform3 affineMat;
    affineMat.setCol0( mat.getCol0().getXYZ( ) );
    affineMat.setCol1( mat.getCol1().getXYZ( ) );
    affineMat.setCol2( mat.getCol2().getXYZ( ) );
    affineMat.setCol3( mat.getCol3().getXYZ( ) );
    return Matrix4( orthoInverse( affineMat ) );
}

inline vec_float4 determinant( const Matrix4 & mat )
{
    vec_float4 dx, dy, dz, dw, mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5;
    mA = mat.getCol0().getX();
    mB = mat.getCol0().getY();
    mC = mat.getCol0().getZ();
    mD = mat.getCol0().getW();
    mE = mat.getCol1().getX();
    mF = mat.getCol1().getY();
    mG = mat.getCol1().getZ();
    mH = mat.getCol1().getW();
    mI = mat.getCol2().getX();
    mJ = mat.getCol2().getY();
    mK = mat.getCol2().getZ();
    mL = mat.getCol2().getW();
    mM = mat.getCol3().getX();
    mN = mat.getCol3().getY();
    mO = mat.getCol3().getZ();
    mP = mat.getCol3().getW();
    tmp0 = vec_sub( vec_madd( mK, mD, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mC, mL, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp1 = vec_sub( vec_madd( mO, mH, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mG, mP, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp2 = vec_sub( vec_madd( mB, mK, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mJ, mC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp3 = vec_sub( vec_madd( mF, mO, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mN, mG, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp4 = vec_sub( vec_madd( mJ, mD, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mB, mL, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmp5 = vec_sub( vec_madd( mN, mH, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mF, mP, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    dx = vec_sub( vec_sub( vec_madd( mJ, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mL, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mK, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    dy = vec_sub( vec_sub( vec_madd( mN, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mP, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mO, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    dz = vec_sub( vec_add( vec_madd( mD, tmp3, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mC, tmp5, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mB, tmp1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    dw = vec_sub( vec_add( vec_madd( mH, tmp2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mG, tmp4, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mF, tmp0, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    return vec_add( vec_add( vec_add( vec_madd( mA, dx, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mE, dy, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mI, dz, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mM, dw, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

inline const Matrix4 Matrix4::operator +( const Matrix4 & mat ) const
{
    return Matrix4(
        ( mCol0 + mat.mCol0 ),
        ( mCol1 + mat.mCol1 ),
        ( mCol2 + mat.mCol2 ),
        ( mCol3 + mat.mCol3 )
    );
}

inline const Matrix4 Matrix4::operator -( const Matrix4 & mat ) const
{
    return Matrix4(
        ( mCol0 - mat.mCol0 ),
        ( mCol1 - mat.mCol1 ),
        ( mCol2 - mat.mCol2 ),
        ( mCol3 - mat.mCol3 )
    );
}

inline Matrix4 & Matrix4::operator +=( const Matrix4 & mat )
{
    *this = *this + mat;
    return *this;
}

inline Matrix4 & Matrix4::operator -=( const Matrix4 & mat )
{
    *this = *this - mat;
    return *this;
}

inline const Matrix4 Matrix4::operator -( ) const
{
    return Matrix4(
        ( -mCol0 ),
        ( -mCol1 ),
        ( -mCol2 ),
        ( -mCol3 )
    );
}

inline const Matrix4 absPerElem( const Matrix4 & mat )
{
    return Matrix4(
        absPerElem( mat.getCol0() ),
        absPerElem( mat.getCol1() ),
        absPerElem( mat.getCol2() ),
        absPerElem( mat.getCol3() )
    );
}

inline const Matrix4 Matrix4::operator *( vec_float4 scalar ) const
{
    return Matrix4(
        ( mCol0 * scalar ),
        ( mCol1 * scalar ),
        ( mCol2 * scalar ),
        ( mCol3 * scalar )
    );
}

inline Matrix4 & Matrix4::operator *=( vec_float4 scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const Matrix4 operator *( vec_float4 scalar, const Matrix4 & mat )
{
    return mat * scalar;
}

inline const Vector4 Matrix4::operator *( const Vector4 & vec ) const
{
    return Vector4(
        vec_add( vec_add( vec_add( vec_madd( mCol0.getX(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getX(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getX(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol3.getX(), vec.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_add( vec_madd( mCol0.getY(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getY(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getY(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol3.getY(), vec.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_add( vec_madd( mCol0.getZ(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getZ(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getZ(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol3.getZ(), vec.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_add( vec_madd( mCol0.getW(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getW(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getW(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol3.getW(), vec.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
    );
}

inline const Vector4 Matrix4::operator *( const Vector3 & vec ) const
{
    return Vector4(
        vec_add( vec_add( vec_madd( mCol0.getX(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getX(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getX(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( mCol0.getY(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getY(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getY(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( mCol0.getZ(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getZ(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getZ(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( mCol0.getW(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getW(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getW(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
    );
}

inline const Vector4 Matrix4::operator *( const Point3 & pnt ) const
{
    return Vector4(
        vec_add( vec_add( vec_add( vec_madd( mCol0.getX(), pnt.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getX(), pnt.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getX(), pnt.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), mCol3.getX() ),
        vec_add( vec_add( vec_add( vec_madd( mCol0.getY(), pnt.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getY(), pnt.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getY(), pnt.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), mCol3.getY() ),
        vec_add( vec_add( vec_add( vec_madd( mCol0.getZ(), pnt.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getZ(), pnt.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getZ(), pnt.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), mCol3.getZ() ),
        vec_add( vec_add( vec_add( vec_madd( mCol0.getW(), pnt.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getW(), pnt.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getW(), pnt.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), mCol3.getW() )
    );
}

inline const Matrix4 Matrix4::operator *( const Matrix4 & mat ) const
{
    return Matrix4(
        ( *this * mat.mCol0 ),
        ( *this * mat.mCol1 ),
        ( *this * mat.mCol2 ),
        ( *this * mat.mCol3 )
    );
}

inline Matrix4 & Matrix4::operator *=( const Matrix4 & mat )
{
    *this = *this * mat;
    return *this;
}

inline const Matrix4 Matrix4::operator *( const Transform3 & tfrm ) const
{
    return Matrix4(
        ( *this * tfrm.getCol0() ),
        ( *this * tfrm.getCol1() ),
        ( *this * tfrm.getCol2() ),
        ( *this * Point3( tfrm.getCol3() ) )
    );
}

inline Matrix4 & Matrix4::operator *=( const Transform3 & tfrm )
{
    *this = *this * tfrm;
    return *this;
}

inline const Matrix4 mulPerElem( const Matrix4 & mat0, const Matrix4 & mat1 )
{
    return Matrix4(
        mulPerElem( mat0.getCol0(), mat1.getCol0() ),
        mulPerElem( mat0.getCol1(), mat1.getCol1() ),
        mulPerElem( mat0.getCol2(), mat1.getCol2() ),
        mulPerElem( mat0.getCol3(), mat1.getCol3() )
    );
}

inline const Matrix4 Matrix4::identity( )
{
    return Matrix4(
        Vector4::xAxis( ),
        Vector4::yAxis( ),
        Vector4::zAxis( ),
        Vector4::wAxis( )
    );
}

inline Matrix4 & Matrix4::setUpper3x3( const Matrix3 & mat3 )
{
    mCol0.setXYZ( mat3.getCol0() );
    mCol1.setXYZ( mat3.getCol1() );
    mCol2.setXYZ( mat3.getCol2() );
    return *this;
}

inline const Matrix3 Matrix4::getUpper3x3( ) const
{
    return Matrix3(
        mCol0.getXYZ( ),
        mCol1.getXYZ( ),
        mCol2.getXYZ( )
    );
}

inline Matrix4 & Matrix4::setTranslation( const Vector3 & translateVec )
{
    mCol3.setXYZ( translateVec );
    return *this;
}

inline const Vector3 Matrix4::getTranslation( ) const
{
    return mCol3.getXYZ( );
}

inline const Matrix4 Matrix4::rotationX( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix4(
        Vector4::xAxis( ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), negatef4( s ), c, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4::wAxis( )
    );
}

inline const Matrix4 Matrix4::rotationY( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix4(
        Vector4( c, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), negatef4( s ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4::yAxis( ),
        Vector4( s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4::wAxis( )
    );
}

inline const Matrix4 Matrix4::rotationZ( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix4(
        Vector4( c, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( negatef4( s ), c, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4::zAxis( ),
        Vector4::wAxis( )
    );
}

inline const Matrix4 Matrix4::rotationZYX( const Vector3 & radiansXYZ )
{
    vec_float4 sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sincosf4( radiansXYZ.getX(), &sX, &cX );
    sincosf4( radiansXYZ.getY(), &sY, &cY );
    sincosf4( radiansXYZ.getZ(), &sZ, &cZ );
    tmp0 = vec_madd( cZ, sY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    tmp1 = vec_madd( sZ, sY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    return Matrix4(
        Vector4( vec_madd( cZ, cY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, cY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), negatef4( sY ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( vec_sub( vec_madd( tmp0, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_add( vec_madd( tmp1, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( cZ, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( cY, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( vec_add( vec_madd( tmp0, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_sub( vec_madd( tmp1, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( cZ, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( cY, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4::wAxis( )
    );
}

inline const Matrix4 Matrix4::rotation( vec_float4 radians, const Vector3 & unitVec )
{
    vec_float4 x, y, z, s, c, oneMinusC, xy, yz, zx;
    sincosf4( radians, &s, &c );
    x = unitVec.getX();
    y = unitVec.getY();
    z = unitVec.getZ();
    xy = vec_madd( x, y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    yz = vec_madd( y, z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    zx = vec_madd( z, x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    oneMinusC = vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), c );
    return Matrix4(
        Vector4( vec_add( vec_madd( vec_madd( x, x, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), c ), vec_add( vec_madd( xy, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( z, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_sub( vec_madd( zx, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( y, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( vec_sub( vec_madd( xy, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( z, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_add( vec_madd( vec_madd( y, y, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), c ), vec_add( vec_madd( yz, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( x, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( vec_add( vec_madd( zx, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( y, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_sub( vec_madd( yz, oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( x, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_add( vec_madd( vec_madd( z, z, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), oneMinusC, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), c ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4::wAxis( )
    );
}

inline const Matrix4 Matrix4::rotation( const Quat & unitQuat )
{
    return Matrix4( Transform3::rotation( unitQuat ) );
}

inline const Matrix4 Matrix4::scale( const Vector3 & scaleVec )
{
    return Matrix4(
        Vector4( scaleVec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), scaleVec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), scaleVec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4::wAxis( )
    );
}

inline const Matrix4 appendScale( const Matrix4 & mat, const Vector3 & scaleVec )
{
    return Matrix4(
        ( mat.getCol0() * scaleVec.getX( ) ),
        ( mat.getCol1() * scaleVec.getY( ) ),
        ( mat.getCol2() * scaleVec.getZ( ) ),
        mat.getCol3()
    );
}

inline const Matrix4 prependScale( const Vector3 & scaleVec, const Matrix4 & mat )
{
    Vector4 scale4;
    scale4 = Vector4( scaleVec, ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
    return Matrix4(
        mulPerElem( mat.getCol0(), scale4 ),
        mulPerElem( mat.getCol1(), scale4 ),
        mulPerElem( mat.getCol2(), scale4 ),
        mulPerElem( mat.getCol3(), scale4 )
    );
}

inline const Matrix4 Matrix4::translation( const Vector3 & translateVec )
{
    return Matrix4(
        Vector4::xAxis( ),
        Vector4::yAxis( ),
        Vector4::zAxis( ),
        Vector4( translateVec, ((vec_float4){1.0f,1.0f,1.0f,1.0f}) )
    );
}

inline const Matrix4 Matrix4::lookAt( const Point3 & eyePos, const Point3 & lookAtPos, const Vector3 & upVec )
{
    Matrix4 m4EyeFrame;
    Vector3 v3X, v3Y, v3Z;
    v3Y = normalize( upVec );
    v3Z = normalize( ( eyePos - lookAtPos ) );
    v3X = normalize( cross( v3Y, v3Z ) );
    v3Y = cross( v3Z, v3X );
    m4EyeFrame = Matrix4( Vector4( v3X ), Vector4( v3Y ), Vector4( v3Z ), Vector4( eyePos ) );
    return orthoInverse( m4EyeFrame );
}

inline const Matrix4 Matrix4::perspective( vec_float4 fovyRadians, vec_float4 aspect, vec_float4 zNear, vec_float4 zFar )
{
    vec_float4 f, rangeInv;
    f = tanf4( vec_sub( ((vec_float4){_VECTORMATH_PI_OVER_2,_VECTORMATH_PI_OVER_2,_VECTORMATH_PI_OVER_2,_VECTORMATH_PI_OVER_2}), vec_madd( ((vec_float4){0.5f,0.5f,0.5f,0.5f}), fovyRadians, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) );
    rangeInv = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec_sub( zNear, zFar ) );
    return Matrix4(
        Vector4( divf4( f, aspect ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), f, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), vec_madd( vec_add( zNear, zFar ), rangeInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){-1.0f,-1.0f,-1.0f,-1.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), vec_madd( vec_madd( vec_madd( zNear, zFar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), rangeInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){2.0f,2.0f,2.0f,2.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Matrix4 Matrix4::frustum( vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar )
{
    vec_float4 sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf, n2;
    sum_rl = vec_add( right, left );
    sum_tb = vec_add( top, bottom );
    sum_nf = vec_add( zNear, zFar );
    inv_rl = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec_sub( right, left ) );
    inv_tb = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec_sub( top, bottom ) );
    inv_nf = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec_sub( zNear, zFar ) );
    n2 = vec_add( zNear, zNear );
    return Matrix4(
        Vector4( vec_madd( n2, inv_rl, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), vec_madd( n2, inv_tb, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( vec_madd( sum_rl, inv_rl, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sum_tb, inv_tb, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sum_nf, inv_nf, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){-1.0f,-1.0f,-1.0f,-1.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), vec_madd( vec_madd( n2, inv_nf, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), zFar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Matrix4 Matrix4::orthographic( vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar )
{
    vec_float4 sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf;
    sum_rl = vec_add( right, left );
    sum_tb = vec_add( top, bottom );
    sum_nf = vec_add( zNear, zFar );
    inv_rl = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec_sub( right, left ) );
    inv_tb = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec_sub( top, bottom ) );
    inv_nf = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), vec_sub( zNear, zFar ) );
    return Matrix4(
        Vector4( vec_add( inv_rl, inv_rl ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), vec_add( inv_tb, inv_tb ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), vec_add( inv_nf, inv_nf ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector4( vec_madd( negatef4( sum_rl ), inv_rl, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( negatef4( sum_tb ), inv_tb, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sum_nf, inv_nf, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), ((vec_float4){1.0f,1.0f,1.0f,1.0f}) )
    );
}

inline const Matrix4 select( const Matrix4 & mat0, const Matrix4 & mat1, vec_uint4 select1 )
{
    return Matrix4(
        select( mat0.getCol0(), mat1.getCol0(), select1 ),
        select( mat0.getCol1(), mat1.getCol1(), select1 ),
        select( mat0.getCol2(), mat1.getCol2(), select1 ),
        select( mat0.getCol3(), mat1.getCol3(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

inline void print( const Matrix4 & mat )
{
    Aos::Matrix4 mat0, mat1, mat2, mat3;
    mat.get4Aos( mat0, mat1, mat2, mat3 );
    printf("slot 0:\n");
    print( mat0 );
    printf("slot 1:\n");
    print( mat1 );
    printf("slot 2:\n");
    print( mat2 );
    printf("slot 3:\n");
    print( mat3 );
}

inline void print( const Matrix4 & mat, const char * name )
{
    printf("%s:\n", name);
    print( mat );
}

#endif

inline Transform3::Transform3( const Transform3 & tfrm )
{
    mCol0 = tfrm.mCol0;
    mCol1 = tfrm.mCol1;
    mCol2 = tfrm.mCol2;
    mCol3 = tfrm.mCol3;
}

inline Transform3::Transform3( vec_float4 scalar )
{
    mCol0 = Vector3( scalar );
    mCol1 = Vector3( scalar );
    mCol2 = Vector3( scalar );
    mCol3 = Vector3( scalar );
}

inline Transform3::Transform3( const Vector3 & _col0, const Vector3 & _col1, const Vector3 & _col2, const Vector3 & _col3 )
{
    mCol0 = _col0;
    mCol1 = _col1;
    mCol2 = _col2;
    mCol3 = _col3;
}

inline Transform3::Transform3( const Matrix3 & tfrm, const Vector3 & translateVec )
{
    this->setUpper3x3( tfrm );
    this->setTranslation( translateVec );
}

inline Transform3::Transform3( const Quat & unitQuat, const Vector3 & translateVec )
{
    this->setUpper3x3( Matrix3( unitQuat ) );
    this->setTranslation( translateVec );
}

inline Transform3::Transform3( const Aos::Transform3 & tfrm )
{
    mCol0 = Vector3( tfrm.getCol0() );
    mCol1 = Vector3( tfrm.getCol1() );
    mCol2 = Vector3( tfrm.getCol2() );
    mCol3 = Vector3( tfrm.getCol3() );
}

inline Transform3::Transform3( const Aos::Transform3 & tfrm0, const Aos::Transform3 & tfrm1, const Aos::Transform3 & tfrm2, const Aos::Transform3 & tfrm3 )
{
    mCol0 = Vector3( tfrm0.getCol0(), tfrm1.getCol0(), tfrm2.getCol0(), tfrm3.getCol0() );
    mCol1 = Vector3( tfrm0.getCol1(), tfrm1.getCol1(), tfrm2.getCol1(), tfrm3.getCol1() );
    mCol2 = Vector3( tfrm0.getCol2(), tfrm1.getCol2(), tfrm2.getCol2(), tfrm3.getCol2() );
    mCol3 = Vector3( tfrm0.getCol3(), tfrm1.getCol3(), tfrm2.getCol3(), tfrm3.getCol3() );
}

inline void Transform3::get4Aos( Aos::Transform3 & result0, Aos::Transform3 & result1, Aos::Transform3 & result2, Aos::Transform3 & result3 ) const
{
    Aos::Vector3 tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3;
    mCol0.get4Aos( tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3 );
    result0.setCol0( tmpV3_0 );
    result1.setCol0( tmpV3_1 );
    result2.setCol0( tmpV3_2 );
    result3.setCol0( tmpV3_3 );
    mCol1.get4Aos( tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3 );
    result0.setCol1( tmpV3_0 );
    result1.setCol1( tmpV3_1 );
    result2.setCol1( tmpV3_2 );
    result3.setCol1( tmpV3_3 );
    mCol2.get4Aos( tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3 );
    result0.setCol2( tmpV3_0 );
    result1.setCol2( tmpV3_1 );
    result2.setCol2( tmpV3_2 );
    result3.setCol2( tmpV3_3 );
    mCol3.get4Aos( tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3 );
    result0.setCol3( tmpV3_0 );
    result1.setCol3( tmpV3_1 );
    result2.setCol3( tmpV3_2 );
    result3.setCol3( tmpV3_3 );
}

inline Transform3 & Transform3::setCol0( const Vector3 & _col0 )
{
    mCol0 = _col0;
    return *this;
}

inline Transform3 & Transform3::setCol1( const Vector3 & _col1 )
{
    mCol1 = _col1;
    return *this;
}

inline Transform3 & Transform3::setCol2( const Vector3 & _col2 )
{
    mCol2 = _col2;
    return *this;
}

inline Transform3 & Transform3::setCol3( const Vector3 & _col3 )
{
    mCol3 = _col3;
    return *this;
}

inline Transform3 & Transform3::setCol( int col, const Vector3 & vec )
{
    *(&mCol0 + col) = vec;
    return *this;
}

inline Transform3 & Transform3::setRow( int row, const Vector4 & vec )
{
    mCol0.setElem( row, vec.getElem( 0 ) );
    mCol1.setElem( row, vec.getElem( 1 ) );
    mCol2.setElem( row, vec.getElem( 2 ) );
    mCol3.setElem( row, vec.getElem( 3 ) );
    return *this;
}

inline Transform3 & Transform3::setElem( int col, int row, vec_float4 val )
{
    Vector3 tmpV3_0;
    tmpV3_0 = this->getCol( col );
    tmpV3_0.setElem( row, val );
    this->setCol( col, tmpV3_0 );
    return *this;
}

inline vec_float4 Transform3::getElem( int col, int row ) const
{
    return this->getCol( col ).getElem( row );
}

inline const Vector3 Transform3::getCol0( ) const
{
    return mCol0;
}

inline const Vector3 Transform3::getCol1( ) const
{
    return mCol1;
}

inline const Vector3 Transform3::getCol2( ) const
{
    return mCol2;
}

inline const Vector3 Transform3::getCol3( ) const
{
    return mCol3;
}

inline const Vector3 Transform3::getCol( int col ) const
{
    return *(&mCol0 + col);
}

inline const Vector4 Transform3::getRow( int row ) const
{
    return Vector4( mCol0.getElem( row ), mCol1.getElem( row ), mCol2.getElem( row ), mCol3.getElem( row ) );
}

inline Vector3 & Transform3::operator []( int col )
{
    return *(&mCol0 + col);
}

inline const Vector3 Transform3::operator []( int col ) const
{
    return *(&mCol0 + col);
}

inline Transform3 & Transform3::operator =( const Transform3 & tfrm )
{
    mCol0 = tfrm.mCol0;
    mCol1 = tfrm.mCol1;
    mCol2 = tfrm.mCol2;
    mCol3 = tfrm.mCol3;
    return *this;
}

inline const Transform3 inverse( const Transform3 & tfrm )
{
    Vector3 tmp0, tmp1, tmp2, inv0, inv1, inv2;
    vec_float4 detinv;
    tmp0 = cross( tfrm.getCol1(), tfrm.getCol2() );
    tmp1 = cross( tfrm.getCol2(), tfrm.getCol0() );
    tmp2 = cross( tfrm.getCol0(), tfrm.getCol1() );
    detinv = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), dot( tfrm.getCol2(), tmp2 ) );
    inv0 = Vector3( vec_madd( tmp0.getX(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp1.getX(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp2.getX(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    inv1 = Vector3( vec_madd( tmp0.getY(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp1.getY(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp2.getY(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    inv2 = Vector3( vec_madd( tmp0.getZ(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp1.getZ(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmp2.getZ(), detinv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    return Transform3(
        inv0,
        inv1,
        inv2,
        Vector3( ( -( ( inv0 * tfrm.getCol3().getX() ) + ( ( inv1 * tfrm.getCol3().getY() ) + ( inv2 * tfrm.getCol3().getZ() ) ) ) ) )
    );
}

inline const Transform3 orthoInverse( const Transform3 & tfrm )
{
    Vector3 inv0, inv1, inv2;
    inv0 = Vector3( tfrm.getCol0().getX(), tfrm.getCol1().getX(), tfrm.getCol2().getX() );
    inv1 = Vector3( tfrm.getCol0().getY(), tfrm.getCol1().getY(), tfrm.getCol2().getY() );
    inv2 = Vector3( tfrm.getCol0().getZ(), tfrm.getCol1().getZ(), tfrm.getCol2().getZ() );
    return Transform3(
        inv0,
        inv1,
        inv2,
        Vector3( ( -( ( inv0 * tfrm.getCol3().getX() ) + ( ( inv1 * tfrm.getCol3().getY() ) + ( inv2 * tfrm.getCol3().getZ() ) ) ) ) )
    );
}

inline const Transform3 absPerElem( const Transform3 & tfrm )
{
    return Transform3(
        absPerElem( tfrm.getCol0() ),
        absPerElem( tfrm.getCol1() ),
        absPerElem( tfrm.getCol2() ),
        absPerElem( tfrm.getCol3() )
    );
}

inline const Vector3 Transform3::operator *( const Vector3 & vec ) const
{
    return Vector3(
        vec_add( vec_add( vec_madd( mCol0.getX(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getX(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getX(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( mCol0.getY(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getY(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getY(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( mCol0.getZ(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getZ(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getZ(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
    );
}

inline const Point3 Transform3::operator *( const Point3 & pnt ) const
{
    return Point3(
        vec_add( vec_add( vec_add( vec_madd( mCol0.getX(), pnt.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getX(), pnt.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getX(), pnt.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), mCol3.getX() ),
        vec_add( vec_add( vec_add( vec_madd( mCol0.getY(), pnt.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getY(), pnt.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getY(), pnt.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), mCol3.getY() ),
        vec_add( vec_add( vec_add( vec_madd( mCol0.getZ(), pnt.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mCol1.getZ(), pnt.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mCol2.getZ(), pnt.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), mCol3.getZ() )
    );
}

inline const Transform3 Transform3::operator *( const Transform3 & tfrm ) const
{
    return Transform3(
        ( *this * tfrm.mCol0 ),
        ( *this * tfrm.mCol1 ),
        ( *this * tfrm.mCol2 ),
        Vector3( ( *this * Point3( tfrm.mCol3 ) ) )
    );
}

inline Transform3 & Transform3::operator *=( const Transform3 & tfrm )
{
    *this = *this * tfrm;
    return *this;
}

inline const Transform3 mulPerElem( const Transform3 & tfrm0, const Transform3 & tfrm1 )
{
    return Transform3(
        mulPerElem( tfrm0.getCol0(), tfrm1.getCol0() ),
        mulPerElem( tfrm0.getCol1(), tfrm1.getCol1() ),
        mulPerElem( tfrm0.getCol2(), tfrm1.getCol2() ),
        mulPerElem( tfrm0.getCol3(), tfrm1.getCol3() )
    );
}

inline const Transform3 Transform3::identity( )
{
    return Transform3(
        Vector3::xAxis( ),
        Vector3::yAxis( ),
        Vector3::zAxis( ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline Transform3 & Transform3::setUpper3x3( const Matrix3 & tfrm )
{
    mCol0 = tfrm.getCol0();
    mCol1 = tfrm.getCol1();
    mCol2 = tfrm.getCol2();
    return *this;
}

inline const Matrix3 Transform3::getUpper3x3( ) const
{
    return Matrix3( mCol0, mCol1, mCol2 );
}

inline Transform3 & Transform3::setTranslation( const Vector3 & translateVec )
{
    mCol3 = translateVec;
    return *this;
}

inline const Vector3 Transform3::getTranslation( ) const
{
    return mCol3;
}

inline const Transform3 Transform3::rotationX( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Transform3(
        Vector3::xAxis( ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c, s ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), negatef4( s ), c ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Transform3 Transform3::rotationY( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Transform3(
        Vector3( c, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), negatef4( s ) ),
        Vector3::yAxis( ),
        Vector3( s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Transform3 Transform3::rotationZ( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Transform3(
        Vector3( c, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector3( negatef4( s ), c, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector3::zAxis( ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Transform3 Transform3::rotationZYX( const Vector3 & radiansXYZ )
{
    vec_float4 sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sincosf4( radiansXYZ.getX(), &sX, &cX );
    sincosf4( radiansXYZ.getY(), &sY, &cY );
    sincosf4( radiansXYZ.getZ(), &sZ, &cZ );
    tmp0 = vec_madd( cZ, sY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    tmp1 = vec_madd( sZ, sY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    return Transform3(
        Vector3( vec_madd( cZ, cY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, cY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), negatef4( sY ) ),
        Vector3( vec_sub( vec_madd( tmp0, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_add( vec_madd( tmp1, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( cZ, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( cY, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        Vector3( vec_add( vec_madd( tmp0, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( sZ, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_sub( vec_madd( tmp1, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( cZ, sX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( cY, cX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Transform3 Transform3::rotation( vec_float4 radians, const Vector3 & unitVec )
{
    return Transform3( Matrix3::rotation( radians, unitVec ), Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

inline const Transform3 Transform3::rotation( const Quat & unitQuat )
{
    return Transform3( Matrix3( unitQuat ), Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

inline const Transform3 Transform3::scale( const Vector3 & scaleVec )
{
    return Transform3(
        Vector3( scaleVec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), scaleVec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), scaleVec.getZ() ),
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Transform3 appendScale( const Transform3 & tfrm, const Vector3 & scaleVec )
{
    return Transform3(
        ( tfrm.getCol0() * scaleVec.getX( ) ),
        ( tfrm.getCol1() * scaleVec.getY( ) ),
        ( tfrm.getCol2() * scaleVec.getZ( ) ),
        tfrm.getCol3()
    );
}

inline const Transform3 prependScale( const Vector3 & scaleVec, const Transform3 & tfrm )
{
    return Transform3(
        mulPerElem( tfrm.getCol0(), scaleVec ),
        mulPerElem( tfrm.getCol1(), scaleVec ),
        mulPerElem( tfrm.getCol2(), scaleVec ),
        mulPerElem( tfrm.getCol3(), scaleVec )
    );
}

inline const Transform3 Transform3::translation( const Vector3 & translateVec )
{
    return Transform3(
        Vector3::xAxis( ),
        Vector3::yAxis( ),
        Vector3::zAxis( ),
        translateVec
    );
}

inline const Transform3 select( const Transform3 & tfrm0, const Transform3 & tfrm1, vec_uint4 select1 )
{
    return Transform3(
        select( tfrm0.getCol0(), tfrm1.getCol0(), select1 ),
        select( tfrm0.getCol1(), tfrm1.getCol1(), select1 ),
        select( tfrm0.getCol2(), tfrm1.getCol2(), select1 ),
        select( tfrm0.getCol3(), tfrm1.getCol3(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

inline void print( const Transform3 & tfrm )
{
    Aos::Transform3 mat0, mat1, mat2, mat3;
    tfrm.get4Aos( mat0, mat1, mat2, mat3 );
    printf("slot 0:\n");
    print( mat0 );
    printf("slot 1:\n");
    print( mat1 );
    printf("slot 2:\n");
    print( mat2 );
    printf("slot 3:\n");
    print( mat3 );
}

inline void print( const Transform3 & tfrm, const char * name )
{
    printf("%s:\n", name);
    print( tfrm );
}

#endif

inline Quat::Quat( const Matrix3 & tfrm )
{
    vec_float4 trace, radicand, scale, xx, yx, zx, xy, yy, zy, xz, yz, zz, tmpx, tmpy, tmpz, tmpw, qx, qy, qz, qw;
    vec_uint4 negTrace, ZgtX, ZgtY, YgtX;
    vec_uint4 largestXorY, largestYorZ, largestZorX;

    xx = tfrm.getCol0().getX();
    yx = tfrm.getCol0().getY();
    zx = tfrm.getCol0().getZ();
    xy = tfrm.getCol1().getX();
    yy = tfrm.getCol1().getY();
    zy = tfrm.getCol1().getZ();
    xz = tfrm.getCol2().getX();
    yz = tfrm.getCol2().getY();
    zz = tfrm.getCol2().getZ();

    trace = vec_add( vec_add( xx, yy ), zz );

    negTrace = (vec_uint4)vec_cmpgt( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), trace );
    ZgtX = (vec_uint4)vec_cmpgt( zz, xx );
    ZgtY = (vec_uint4)vec_cmpgt( zz, yy );
    YgtX = (vec_uint4)vec_cmpgt( yy, xx );
    largestXorY = vec_andc( negTrace, vec_and( ZgtX, ZgtY ) );
    largestYorZ = vec_and( negTrace, vec_or( YgtX, ZgtX ) );
    largestZorX = vec_andc( negTrace, vec_andc( YgtX, ZgtY ) );
    
    zz = vec_sel( zz, negatef4(zz), largestXorY );
    xy = vec_sel( xy, negatef4(xy), largestXorY );
    xx = vec_sel( xx, negatef4(xx), largestYorZ );
    yz = vec_sel( yz, negatef4(yz), largestYorZ );
    yy = vec_sel( yy, negatef4(yy), largestZorX );
    zx = vec_sel( zx, negatef4(zx), largestZorX );

    radicand = vec_add( vec_add( vec_add( xx, yy ), zz ), ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
    scale = vec_madd( ((vec_float4){0.5f,0.5f,0.5f,0.5f}), divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( radicand ) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );

    tmpx = vec_madd( vec_sub( zy, yz ), scale, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    tmpy = vec_madd( vec_sub( xz, zx ), scale, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    tmpz = vec_madd( vec_sub( yx, xy ), scale, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    tmpw = vec_madd( radicand, scale, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qx = tmpx;
    qy = tmpy;
    qz = tmpz;
    qw = tmpw;

    qx = vec_sel( qx, tmpw, largestXorY );
    qy = vec_sel( qy, tmpz, largestXorY );
    qz = vec_sel( qz, tmpy, largestXorY );
    qw = vec_sel( qw, tmpx, largestXorY );
    tmpx = qx;
    tmpz = qz;
    qx = vec_sel( qx, qy, largestYorZ );
    qy = vec_sel( qy, tmpx, largestYorZ );
    qz = vec_sel( qz, qw, largestYorZ );
    qw = vec_sel( qw, tmpz, largestYorZ );

    mX = qx;
    mY = qy;
    mZ = qz;
    mW = qw;
}

inline const Matrix3 outer( const Vector3 & tfrm0, const Vector3 & tfrm1 )
{
    return Matrix3(
        ( tfrm0 * tfrm1.getX( ) ),
        ( tfrm0 * tfrm1.getY( ) ),
        ( tfrm0 * tfrm1.getZ( ) )
    );
}

inline const Matrix4 outer( const Vector4 & tfrm0, const Vector4 & tfrm1 )
{
    return Matrix4(
        ( tfrm0 * tfrm1.getX( ) ),
        ( tfrm0 * tfrm1.getY( ) ),
        ( tfrm0 * tfrm1.getZ( ) ),
        ( tfrm0 * tfrm1.getW( ) )
    );
}

inline const Vector3 rowMul( const Vector3 & vec, const Matrix3 & mat )
{
    return Vector3(
        vec_add( vec_add( vec_madd( vec.getX(), mat.getCol0().getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( vec.getY(), mat.getCol0().getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( vec.getZ(), mat.getCol0().getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( vec.getX(), mat.getCol1().getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( vec.getY(), mat.getCol1().getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( vec.getZ(), mat.getCol1().getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_add( vec_madd( vec.getX(), mat.getCol2().getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( vec.getY(), mat.getCol2().getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( vec.getZ(), mat.getCol2().getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
    );
}

inline const Matrix3 crossMatrix( const Vector3 & vec )
{
    return Matrix3(
        Vector3( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), vec.getZ(), negatef4( vec.getY() ) ),
        Vector3( negatef4( vec.getZ() ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), vec.getX() ),
        Vector3( vec.getY(), negatef4( vec.getX() ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Matrix3 crossMatrixMul( const Vector3 & vec, const Matrix3 & mat )
{
    return Matrix3( cross( vec, mat.getCol0() ), cross( vec, mat.getCol1() ), cross( vec, mat.getCol2() ) );
}

} // namespace Soa
} // namespace Vectormath

#endif
