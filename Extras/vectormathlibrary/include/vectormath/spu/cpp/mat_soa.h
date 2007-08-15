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
    qx2 = spu_add( qx, qx );
    qy2 = spu_add( qy, qy );
    qz2 = spu_add( qz, qz );
    qxqx2 = spu_mul( qx, qx2 );
    qxqy2 = spu_mul( qx, qy2 );
    qxqz2 = spu_mul( qx, qz2 );
    qxqw2 = spu_mul( qw, qx2 );
    qyqy2 = spu_mul( qy, qy2 );
    qyqz2 = spu_mul( qy, qz2 );
    qyqw2 = spu_mul( qw, qy2 );
    qzqz2 = spu_mul( qz, qz2 );
    qzqw2 = spu_mul( qw, qz2 );
    mCol0 = Vector3( spu_sub( spu_sub( spu_splats(1.0f), qyqy2 ), qzqz2 ), spu_add( qxqy2, qzqw2 ), spu_sub( qxqz2, qyqw2 ) );
    mCol1 = Vector3( spu_sub( qxqy2, qzqw2 ), spu_sub( spu_sub( spu_splats(1.0f), qxqx2 ), qzqz2 ), spu_add( qyqz2, qxqw2 ) );
    mCol2 = Vector3( spu_add( qxqz2, qyqw2 ), spu_sub( qyqz2, qxqw2 ), spu_sub( spu_sub( spu_splats(1.0f), qxqx2 ), qyqy2 ) );
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
    detinv = recipf4( dot( mat.getCol2(), tmp2 ) );
    return Matrix3(
        Vector3( spu_mul( tmp0.getX(), detinv ), spu_mul( tmp1.getX(), detinv ), spu_mul( tmp2.getX(), detinv ) ),
        Vector3( spu_mul( tmp0.getY(), detinv ), spu_mul( tmp1.getY(), detinv ), spu_mul( tmp2.getY(), detinv ) ),
        Vector3( spu_mul( tmp0.getZ(), detinv ), spu_mul( tmp1.getZ(), detinv ), spu_mul( tmp2.getZ(), detinv ) )
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
        spu_add( spu_add( spu_mul( mCol0.getX(), vec.getX() ), spu_mul( mCol1.getX(), vec.getY() ) ), spu_mul( mCol2.getX(), vec.getZ() ) ),
        spu_add( spu_add( spu_mul( mCol0.getY(), vec.getX() ), spu_mul( mCol1.getY(), vec.getY() ) ), spu_mul( mCol2.getY(), vec.getZ() ) ),
        spu_add( spu_add( spu_mul( mCol0.getZ(), vec.getX() ), spu_mul( mCol1.getZ(), vec.getY() ) ), spu_mul( mCol2.getZ(), vec.getZ() ) )
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
        Vector3( spu_splats(0.0f), c, s ),
        Vector3( spu_splats(0.0f), negatef4( s ), c )
    );
}

inline const Matrix3 Matrix3::rotationY( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix3(
        Vector3( c, spu_splats(0.0f), negatef4( s ) ),
        Vector3::yAxis( ),
        Vector3( s, spu_splats(0.0f), c )
    );
}

inline const Matrix3 Matrix3::rotationZ( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix3(
        Vector3( c, s, spu_splats(0.0f) ),
        Vector3( negatef4( s ), c, spu_splats(0.0f) ),
        Vector3::zAxis( )
    );
}

inline const Matrix3 Matrix3::rotationZYX( const Vector3 & radiansXYZ )
{
    vec_float4 sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sincosf4( radiansXYZ.getX(), &sX, &cX );
    sincosf4( radiansXYZ.getY(), &sY, &cY );
    sincosf4( radiansXYZ.getZ(), &sZ, &cZ );
    tmp0 = spu_mul( cZ, sY );
    tmp1 = spu_mul( sZ, sY );
    return Matrix3(
        Vector3( spu_mul( cZ, cY ), spu_mul( sZ, cY ), negatef4( sY ) ),
        Vector3( spu_sub( spu_mul( tmp0, sX ), spu_mul( sZ, cX ) ), spu_add( spu_mul( tmp1, sX ), spu_mul( cZ, cX ) ), spu_mul( cY, sX ) ),
        Vector3( spu_add( spu_mul( tmp0, cX ), spu_mul( sZ, sX ) ), spu_sub( spu_mul( tmp1, cX ), spu_mul( cZ, sX ) ), spu_mul( cY, cX ) )
    );
}

inline const Matrix3 Matrix3::rotation( vec_float4 radians, const Vector3 & unitVec )
{
    vec_float4 x, y, z, s, c, oneMinusC, xy, yz, zx;
    sincosf4( radians, &s, &c );
    x = unitVec.getX();
    y = unitVec.getY();
    z = unitVec.getZ();
    xy = spu_mul( x, y );
    yz = spu_mul( y, z );
    zx = spu_mul( z, x );
    oneMinusC = spu_sub( spu_splats(1.0f), c );
    return Matrix3(
        Vector3( spu_add( spu_mul( spu_mul( x, x ), oneMinusC ), c ), spu_add( spu_mul( xy, oneMinusC ), spu_mul( z, s ) ), spu_sub( spu_mul( zx, oneMinusC ), spu_mul( y, s ) ) ),
        Vector3( spu_sub( spu_mul( xy, oneMinusC ), spu_mul( z, s ) ), spu_add( spu_mul( spu_mul( y, y ), oneMinusC ), c ), spu_add( spu_mul( yz, oneMinusC ), spu_mul( x, s ) ) ),
        Vector3( spu_add( spu_mul( zx, oneMinusC ), spu_mul( y, s ) ), spu_sub( spu_mul( yz, oneMinusC ), spu_mul( x, s ) ), spu_add( spu_mul( spu_mul( z, z ), oneMinusC ), c ) )
    );
}

inline const Matrix3 Matrix3::rotation( const Quat & unitQuat )
{
    return Matrix3( unitQuat );
}

inline const Matrix3 Matrix3::scale( const Vector3 & scaleVec )
{
    return Matrix3(
        Vector3( scaleVec.getX(), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector3( spu_splats(0.0f), scaleVec.getY(), spu_splats(0.0f) ),
        Vector3( spu_splats(0.0f), spu_splats(0.0f), scaleVec.getZ() )
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
    mCol0 = Vector4( mat.getCol0(), spu_splats(0.0f) );
    mCol1 = Vector4( mat.getCol1(), spu_splats(0.0f) );
    mCol2 = Vector4( mat.getCol2(), spu_splats(0.0f) );
    mCol3 = Vector4( mat.getCol3(), spu_splats(1.0f) );
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
    mCol0 = Vector4( mat.getCol0(), spu_splats(0.0f) );
    mCol1 = Vector4( mat.getCol1(), spu_splats(0.0f) );
    mCol2 = Vector4( mat.getCol2(), spu_splats(0.0f) );
    mCol3 = Vector4( translateVec, spu_splats(1.0f) );
}

inline Matrix4::Matrix4( const Quat & unitQuat, const Vector3 & translateVec )
{
    Matrix3 mat;
    mat = Matrix3( unitQuat );
    mCol0 = Vector4( mat.getCol0(), spu_splats(0.0f) );
    mCol1 = Vector4( mat.getCol1(), spu_splats(0.0f) );
    mCol2 = Vector4( mat.getCol2(), spu_splats(0.0f) );
    mCol3 = Vector4( translateVec, spu_splats(1.0f) );
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
    tmp0 = spu_sub( spu_mul( mK, mD ), spu_mul( mC, mL ) );
    tmp1 = spu_sub( spu_mul( mO, mH ), spu_mul( mG, mP ) );
    tmp2 = spu_sub( spu_mul( mB, mK ), spu_mul( mJ, mC ) );
    tmp3 = spu_sub( spu_mul( mF, mO ), spu_mul( mN, mG ) );
    tmp4 = spu_sub( spu_mul( mJ, mD ), spu_mul( mB, mL ) );
    tmp5 = spu_sub( spu_mul( mN, mH ), spu_mul( mF, mP ) );
    res0.setX( spu_sub( spu_sub( spu_mul( mJ, tmp1 ), spu_mul( mL, tmp3 ) ), spu_mul( mK, tmp5 ) ) );
    res0.setY( spu_sub( spu_sub( spu_mul( mN, tmp0 ), spu_mul( mP, tmp2 ) ), spu_mul( mO, tmp4 ) ) );
    res0.setZ( spu_sub( spu_add( spu_mul( mD, tmp3 ), spu_mul( mC, tmp5 ) ), spu_mul( mB, tmp1 ) ) );
    res0.setW( spu_sub( spu_add( spu_mul( mH, tmp2 ), spu_mul( mG, tmp4 ) ), spu_mul( mF, tmp0 ) ) );
    detInv = recipf4( spu_add( spu_add( spu_add( spu_mul( mA, res0.getX() ), spu_mul( mE, res0.getY() ) ), spu_mul( mI, res0.getZ() ) ), spu_mul( mM, res0.getW() ) ) );
    res1.setX( spu_mul( mI, tmp1 ) );
    res1.setY( spu_mul( mM, tmp0 ) );
    res1.setZ( spu_mul( mA, tmp1 ) );
    res1.setW( spu_mul( mE, tmp0 ) );
    res3.setX( spu_mul( mI, tmp3 ) );
    res3.setY( spu_mul( mM, tmp2 ) );
    res3.setZ( spu_mul( mA, tmp3 ) );
    res3.setW( spu_mul( mE, tmp2 ) );
    res2.setX( spu_mul( mI, tmp5 ) );
    res2.setY( spu_mul( mM, tmp4 ) );
    res2.setZ( spu_mul( mA, tmp5 ) );
    res2.setW( spu_mul( mE, tmp4 ) );
    tmp0 = spu_sub( spu_mul( mI, mB ), spu_mul( mA, mJ ) );
    tmp1 = spu_sub( spu_mul( mM, mF ), spu_mul( mE, mN ) );
    tmp2 = spu_sub( spu_mul( mI, mD ), spu_mul( mA, mL ) );
    tmp3 = spu_sub( spu_mul( mM, mH ), spu_mul( mE, mP ) );
    tmp4 = spu_sub( spu_mul( mI, mC ), spu_mul( mA, mK ) );
    tmp5 = spu_sub( spu_mul( mM, mG ), spu_mul( mE, mO ) );
    res2.setX( spu_add( spu_sub( spu_mul( mL, tmp1 ), spu_mul( mJ, tmp3 ) ), res2.getX() ) );
    res2.setY( spu_add( spu_sub( spu_mul( mP, tmp0 ), spu_mul( mN, tmp2 ) ), res2.getY() ) );
    res2.setZ( spu_sub( spu_sub( spu_mul( mB, tmp3 ), spu_mul( mD, tmp1 ) ), res2.getZ() ) );
    res2.setW( spu_sub( spu_sub( spu_mul( mF, tmp2 ), spu_mul( mH, tmp0 ) ), res2.getW() ) );
    res3.setX( spu_add( spu_sub( spu_mul( mJ, tmp5 ), spu_mul( mK, tmp1 ) ), res3.getX() ) );
    res3.setY( spu_add( spu_sub( spu_mul( mN, tmp4 ), spu_mul( mO, tmp0 ) ), res3.getY() ) );
    res3.setZ( spu_sub( spu_sub( spu_mul( mC, tmp1 ), spu_mul( mB, tmp5 ) ), res3.getZ() ) );
    res3.setW( spu_sub( spu_sub( spu_mul( mG, tmp0 ), spu_mul( mF, tmp4 ) ), res3.getW() ) );
    res1.setX( spu_sub( spu_sub( spu_mul( mK, tmp3 ), spu_mul( mL, tmp5 ) ), res1.getX() ) );
    res1.setY( spu_sub( spu_sub( spu_mul( mO, tmp2 ), spu_mul( mP, tmp4 ) ), res1.getY() ) );
    res1.setZ( spu_add( spu_sub( spu_mul( mD, tmp5 ), spu_mul( mC, tmp3 ) ), res1.getZ() ) );
    res1.setW( spu_add( spu_sub( spu_mul( mH, tmp4 ), spu_mul( mG, tmp2 ) ), res1.getW() ) );
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
    tmp0 = spu_sub( spu_mul( mK, mD ), spu_mul( mC, mL ) );
    tmp1 = spu_sub( spu_mul( mO, mH ), spu_mul( mG, mP ) );
    tmp2 = spu_sub( spu_mul( mB, mK ), spu_mul( mJ, mC ) );
    tmp3 = spu_sub( spu_mul( mF, mO ), spu_mul( mN, mG ) );
    tmp4 = spu_sub( spu_mul( mJ, mD ), spu_mul( mB, mL ) );
    tmp5 = spu_sub( spu_mul( mN, mH ), spu_mul( mF, mP ) );
    dx = spu_sub( spu_sub( spu_mul( mJ, tmp1 ), spu_mul( mL, tmp3 ) ), spu_mul( mK, tmp5 ) );
    dy = spu_sub( spu_sub( spu_mul( mN, tmp0 ), spu_mul( mP, tmp2 ) ), spu_mul( mO, tmp4 ) );
    dz = spu_sub( spu_add( spu_mul( mD, tmp3 ), spu_mul( mC, tmp5 ) ), spu_mul( mB, tmp1 ) );
    dw = spu_sub( spu_add( spu_mul( mH, tmp2 ), spu_mul( mG, tmp4 ) ), spu_mul( mF, tmp0 ) );
    return spu_add( spu_add( spu_add( spu_mul( mA, dx ), spu_mul( mE, dy ) ), spu_mul( mI, dz ) ), spu_mul( mM, dw ) );
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
        spu_add( spu_add( spu_add( spu_mul( mCol0.getX(), vec.getX() ), spu_mul( mCol1.getX(), vec.getY() ) ), spu_mul( mCol2.getX(), vec.getZ() ) ), spu_mul( mCol3.getX(), vec.getW() ) ),
        spu_add( spu_add( spu_add( spu_mul( mCol0.getY(), vec.getX() ), spu_mul( mCol1.getY(), vec.getY() ) ), spu_mul( mCol2.getY(), vec.getZ() ) ), spu_mul( mCol3.getY(), vec.getW() ) ),
        spu_add( spu_add( spu_add( spu_mul( mCol0.getZ(), vec.getX() ), spu_mul( mCol1.getZ(), vec.getY() ) ), spu_mul( mCol2.getZ(), vec.getZ() ) ), spu_mul( mCol3.getZ(), vec.getW() ) ),
        spu_add( spu_add( spu_add( spu_mul( mCol0.getW(), vec.getX() ), spu_mul( mCol1.getW(), vec.getY() ) ), spu_mul( mCol2.getW(), vec.getZ() ) ), spu_mul( mCol3.getW(), vec.getW() ) )
    );
}

inline const Vector4 Matrix4::operator *( const Vector3 & vec ) const
{
    return Vector4(
        spu_add( spu_add( spu_mul( mCol0.getX(), vec.getX() ), spu_mul( mCol1.getX(), vec.getY() ) ), spu_mul( mCol2.getX(), vec.getZ() ) ),
        spu_add( spu_add( spu_mul( mCol0.getY(), vec.getX() ), spu_mul( mCol1.getY(), vec.getY() ) ), spu_mul( mCol2.getY(), vec.getZ() ) ),
        spu_add( spu_add( spu_mul( mCol0.getZ(), vec.getX() ), spu_mul( mCol1.getZ(), vec.getY() ) ), spu_mul( mCol2.getZ(), vec.getZ() ) ),
        spu_add( spu_add( spu_mul( mCol0.getW(), vec.getX() ), spu_mul( mCol1.getW(), vec.getY() ) ), spu_mul( mCol2.getW(), vec.getZ() ) )
    );
}

inline const Vector4 Matrix4::operator *( const Point3 & pnt ) const
{
    return Vector4(
        spu_add( spu_add( spu_add( spu_mul( mCol0.getX(), pnt.getX() ), spu_mul( mCol1.getX(), pnt.getY() ) ), spu_mul( mCol2.getX(), pnt.getZ() ) ), mCol3.getX() ),
        spu_add( spu_add( spu_add( spu_mul( mCol0.getY(), pnt.getX() ), spu_mul( mCol1.getY(), pnt.getY() ) ), spu_mul( mCol2.getY(), pnt.getZ() ) ), mCol3.getY() ),
        spu_add( spu_add( spu_add( spu_mul( mCol0.getZ(), pnt.getX() ), spu_mul( mCol1.getZ(), pnt.getY() ) ), spu_mul( mCol2.getZ(), pnt.getZ() ) ), mCol3.getZ() ),
        spu_add( spu_add( spu_add( spu_mul( mCol0.getW(), pnt.getX() ), spu_mul( mCol1.getW(), pnt.getY() ) ), spu_mul( mCol2.getW(), pnt.getZ() ) ), mCol3.getW() )
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
        Vector4( spu_splats(0.0f), c, s, spu_splats(0.0f) ),
        Vector4( spu_splats(0.0f), negatef4( s ), c, spu_splats(0.0f) ),
        Vector4::wAxis( )
    );
}

inline const Matrix4 Matrix4::rotationY( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix4(
        Vector4( c, spu_splats(0.0f), negatef4( s ), spu_splats(0.0f) ),
        Vector4::yAxis( ),
        Vector4( s, spu_splats(0.0f), c, spu_splats(0.0f) ),
        Vector4::wAxis( )
    );
}

inline const Matrix4 Matrix4::rotationZ( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Matrix4(
        Vector4( c, s, spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( negatef4( s ), c, spu_splats(0.0f), spu_splats(0.0f) ),
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
    tmp0 = spu_mul( cZ, sY );
    tmp1 = spu_mul( sZ, sY );
    return Matrix4(
        Vector4( spu_mul( cZ, cY ), spu_mul( sZ, cY ), negatef4( sY ), spu_splats(0.0f) ),
        Vector4( spu_sub( spu_mul( tmp0, sX ), spu_mul( sZ, cX ) ), spu_add( spu_mul( tmp1, sX ), spu_mul( cZ, cX ) ), spu_mul( cY, sX ), spu_splats(0.0f) ),
        Vector4( spu_add( spu_mul( tmp0, cX ), spu_mul( sZ, sX ) ), spu_sub( spu_mul( tmp1, cX ), spu_mul( cZ, sX ) ), spu_mul( cY, cX ), spu_splats(0.0f) ),
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
    xy = spu_mul( x, y );
    yz = spu_mul( y, z );
    zx = spu_mul( z, x );
    oneMinusC = spu_sub( spu_splats(1.0f), c );
    return Matrix4(
        Vector4( spu_add( spu_mul( spu_mul( x, x ), oneMinusC ), c ), spu_add( spu_mul( xy, oneMinusC ), spu_mul( z, s ) ), spu_sub( spu_mul( zx, oneMinusC ), spu_mul( y, s ) ), spu_splats(0.0f) ),
        Vector4( spu_sub( spu_mul( xy, oneMinusC ), spu_mul( z, s ) ), spu_add( spu_mul( spu_mul( y, y ), oneMinusC ), c ), spu_add( spu_mul( yz, oneMinusC ), spu_mul( x, s ) ), spu_splats(0.0f) ),
        Vector4( spu_add( spu_mul( zx, oneMinusC ), spu_mul( y, s ) ), spu_sub( spu_mul( yz, oneMinusC ), spu_mul( x, s ) ), spu_add( spu_mul( spu_mul( z, z ), oneMinusC ), c ), spu_splats(0.0f) ),
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
        Vector4( scaleVec.getX(), spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( spu_splats(0.0f), scaleVec.getY(), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( spu_splats(0.0f), spu_splats(0.0f), scaleVec.getZ(), spu_splats(0.0f) ),
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
    scale4 = Vector4( scaleVec, spu_splats(1.0f) );
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
        Vector4( translateVec, spu_splats(1.0f) )
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
    f = tanf4( spu_sub( spu_splats( _VECTORMATH_PI_OVER_2 ), spu_mul( spu_splats(0.5f), fovyRadians ) ) );
    rangeInv = recipf4( spu_sub( zNear, zFar ) );
    return Matrix4(
        Vector4( divf4( f, aspect ), spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( spu_splats(0.0f), f, spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( spu_splats(0.0f), spu_splats(0.0f), spu_mul( spu_add( zNear, zFar ), rangeInv ), spu_splats(-1.0f) ),
        Vector4( spu_splats(0.0f), spu_splats(0.0f), spu_mul( spu_mul( spu_mul( zNear, zFar ), rangeInv ), spu_splats(2.0f) ), spu_splats(0.0f) )
    );
}

inline const Matrix4 Matrix4::frustum( vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar )
{
    vec_float4 sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf, n2;
    sum_rl = spu_add( right, left );
    sum_tb = spu_add( top, bottom );
    sum_nf = spu_add( zNear, zFar );
    inv_rl = recipf4( spu_sub( right, left ) );
    inv_tb = recipf4( spu_sub( top, bottom ) );
    inv_nf = recipf4( spu_sub( zNear, zFar ) );
    n2 = spu_add( zNear, zNear );
    return Matrix4(
        Vector4( spu_mul( n2, inv_rl ), spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( spu_splats(0.0f), spu_mul( n2, inv_tb ), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( spu_mul( sum_rl, inv_rl ), spu_mul( sum_tb, inv_tb ), spu_mul( sum_nf, inv_nf ), spu_splats(-1.0f) ),
        Vector4( spu_splats(0.0f), spu_splats(0.0f), spu_mul( spu_mul( n2, inv_nf ), zFar ), spu_splats(0.0f) )
    );
}

inline const Matrix4 Matrix4::orthographic( vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar )
{
    vec_float4 sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf;
    sum_rl = spu_add( right, left );
    sum_tb = spu_add( top, bottom );
    sum_nf = spu_add( zNear, zFar );
    inv_rl = recipf4( spu_sub( right, left ) );
    inv_tb = recipf4( spu_sub( top, bottom ) );
    inv_nf = recipf4( spu_sub( zNear, zFar ) );
    return Matrix4(
        Vector4( spu_add( inv_rl, inv_rl ), spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( spu_splats(0.0f), spu_add( inv_tb, inv_tb ), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector4( spu_splats(0.0f), spu_splats(0.0f), spu_add( inv_nf, inv_nf ), spu_splats(0.0f) ),
        Vector4( spu_mul( negatef4( sum_rl ), inv_rl ), spu_mul( negatef4( sum_tb ), inv_tb ), spu_mul( sum_nf, inv_nf ), spu_splats(1.0f) )
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
    detinv = recipf4( dot( tfrm.getCol2(), tmp2 ) );
    inv0 = Vector3( spu_mul( tmp0.getX(), detinv ), spu_mul( tmp1.getX(), detinv ), spu_mul( tmp2.getX(), detinv ) );
    inv1 = Vector3( spu_mul( tmp0.getY(), detinv ), spu_mul( tmp1.getY(), detinv ), spu_mul( tmp2.getY(), detinv ) );
    inv2 = Vector3( spu_mul( tmp0.getZ(), detinv ), spu_mul( tmp1.getZ(), detinv ), spu_mul( tmp2.getZ(), detinv ) );
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
        spu_add( spu_add( spu_mul( mCol0.getX(), vec.getX() ), spu_mul( mCol1.getX(), vec.getY() ) ), spu_mul( mCol2.getX(), vec.getZ() ) ),
        spu_add( spu_add( spu_mul( mCol0.getY(), vec.getX() ), spu_mul( mCol1.getY(), vec.getY() ) ), spu_mul( mCol2.getY(), vec.getZ() ) ),
        spu_add( spu_add( spu_mul( mCol0.getZ(), vec.getX() ), spu_mul( mCol1.getZ(), vec.getY() ) ), spu_mul( mCol2.getZ(), vec.getZ() ) )
    );
}

inline const Point3 Transform3::operator *( const Point3 & pnt ) const
{
    return Point3(
        spu_add( spu_add( spu_add( spu_mul( mCol0.getX(), pnt.getX() ), spu_mul( mCol1.getX(), pnt.getY() ) ), spu_mul( mCol2.getX(), pnt.getZ() ) ), mCol3.getX() ),
        spu_add( spu_add( spu_add( spu_mul( mCol0.getY(), pnt.getX() ), spu_mul( mCol1.getY(), pnt.getY() ) ), spu_mul( mCol2.getY(), pnt.getZ() ) ), mCol3.getY() ),
        spu_add( spu_add( spu_add( spu_mul( mCol0.getZ(), pnt.getX() ), spu_mul( mCol1.getZ(), pnt.getY() ) ), spu_mul( mCol2.getZ(), pnt.getZ() ) ), mCol3.getZ() )
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
        Vector3( spu_splats(0.0f) )
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
        Vector3( spu_splats(0.0f), c, s ),
        Vector3( spu_splats(0.0f), negatef4( s ), c ),
        Vector3( spu_splats(0.0f) )
    );
}

inline const Transform3 Transform3::rotationY( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Transform3(
        Vector3( c, spu_splats(0.0f), negatef4( s ) ),
        Vector3::yAxis( ),
        Vector3( s, spu_splats(0.0f), c ),
        Vector3( spu_splats(0.0f) )
    );
}

inline const Transform3 Transform3::rotationZ( vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    return Transform3(
        Vector3( c, s, spu_splats(0.0f) ),
        Vector3( negatef4( s ), c, spu_splats(0.0f) ),
        Vector3::zAxis( ),
        Vector3( spu_splats(0.0f) )
    );
}

inline const Transform3 Transform3::rotationZYX( const Vector3 & radiansXYZ )
{
    vec_float4 sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sincosf4( radiansXYZ.getX(), &sX, &cX );
    sincosf4( radiansXYZ.getY(), &sY, &cY );
    sincosf4( radiansXYZ.getZ(), &sZ, &cZ );
    tmp0 = spu_mul( cZ, sY );
    tmp1 = spu_mul( sZ, sY );
    return Transform3(
        Vector3( spu_mul( cZ, cY ), spu_mul( sZ, cY ), negatef4( sY ) ),
        Vector3( spu_sub( spu_mul( tmp0, sX ), spu_mul( sZ, cX ) ), spu_add( spu_mul( tmp1, sX ), spu_mul( cZ, cX ) ), spu_mul( cY, sX ) ),
        Vector3( spu_add( spu_mul( tmp0, cX ), spu_mul( sZ, sX ) ), spu_sub( spu_mul( tmp1, cX ), spu_mul( cZ, sX ) ), spu_mul( cY, cX ) ),
        Vector3( spu_splats(0.0f) )
    );
}

inline const Transform3 Transform3::rotation( vec_float4 radians, const Vector3 & unitVec )
{
    return Transform3( Matrix3::rotation( radians, unitVec ), Vector3( spu_splats(0.0f) ) );
}

inline const Transform3 Transform3::rotation( const Quat & unitQuat )
{
    return Transform3( Matrix3( unitQuat ), Vector3( spu_splats(0.0f) ) );
}

inline const Transform3 Transform3::scale( const Vector3 & scaleVec )
{
    return Transform3(
        Vector3( scaleVec.getX(), spu_splats(0.0f), spu_splats(0.0f) ),
        Vector3( spu_splats(0.0f), scaleVec.getY(), spu_splats(0.0f) ),
        Vector3( spu_splats(0.0f), spu_splats(0.0f), scaleVec.getZ() ),
        Vector3( spu_splats(0.0f) )
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

    trace = spu_add( spu_add( xx, yy ), zz );

    negTrace = spu_cmpgt( spu_splats(0.0f), trace );
    ZgtX = spu_cmpgt( zz, xx );
    ZgtY = spu_cmpgt( zz, yy );
    YgtX = spu_cmpgt( yy, xx );
    largestXorY = spu_and( negTrace, spu_nand( ZgtX, ZgtY ) );
    largestYorZ = spu_and( negTrace, spu_or( YgtX, ZgtX ) );
    largestZorX = spu_and( negTrace, spu_orc( ZgtY, YgtX ) );
    
    zz = spu_sel( zz, negatef4(zz), largestXorY );
    xy = spu_sel( xy, negatef4(xy), largestXorY );
    xx = spu_sel( xx, negatef4(xx), largestYorZ );
    yz = spu_sel( yz, negatef4(yz), largestYorZ );
    yy = spu_sel( yy, negatef4(yy), largestZorX );
    zx = spu_sel( zx, negatef4(zx), largestZorX );

    radicand = spu_add( spu_add( spu_add( xx, yy ), zz ), spu_splats(1.0f) );
    scale = spu_mul( spu_splats(0.5f), rsqrtf4( radicand ) );

    tmpx = spu_mul( spu_sub( zy, yz ), scale );
    tmpy = spu_mul( spu_sub( xz, zx ), scale );
    tmpz = spu_mul( spu_sub( yx, xy ), scale );
    tmpw = spu_mul( radicand, scale );
    qx = tmpx;
    qy = tmpy;
    qz = tmpz;
    qw = tmpw;

    qx = spu_sel( qx, tmpw, largestXorY );
    qy = spu_sel( qy, tmpz, largestXorY );
    qz = spu_sel( qz, tmpy, largestXorY );
    qw = spu_sel( qw, tmpx, largestXorY );
    tmpx = qx;
    tmpz = qz;
    qx = spu_sel( qx, qy, largestYorZ );
    qy = spu_sel( qy, tmpx, largestYorZ );
    qz = spu_sel( qz, qw, largestYorZ );
    qw = spu_sel( qw, tmpz, largestYorZ );

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
        spu_add( spu_add( spu_mul( vec.getX(), mat.getCol0().getX() ), spu_mul( vec.getY(), mat.getCol0().getY() ) ), spu_mul( vec.getZ(), mat.getCol0().getZ() ) ),
        spu_add( spu_add( spu_mul( vec.getX(), mat.getCol1().getX() ), spu_mul( vec.getY(), mat.getCol1().getY() ) ), spu_mul( vec.getZ(), mat.getCol1().getZ() ) ),
        spu_add( spu_add( spu_mul( vec.getX(), mat.getCol2().getX() ), spu_mul( vec.getY(), mat.getCol2().getY() ) ), spu_mul( vec.getZ(), mat.getCol2().getZ() ) )
    );
}

inline const Matrix3 crossMatrix( const Vector3 & vec )
{
    return Matrix3(
        Vector3( spu_splats(0.0f), vec.getZ(), negatef4( vec.getY() ) ),
        Vector3( negatef4( vec.getZ() ), spu_splats(0.0f), vec.getX() ),
        Vector3( vec.getY(), negatef4( vec.getX() ), spu_splats(0.0f) )
    );
}

inline const Matrix3 crossMatrixMul( const Vector3 & vec, const Matrix3 & mat )
{
    return Matrix3( cross( vec, mat.getCol0() ), cross( vec, mat.getCol1() ), cross( vec, mat.getCol2() ) );
}

} // namespace Soa
} // namespace Vectormath

#endif
