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

#ifndef _VECTORMATH_QUAT_SOA_CPP_H
#define _VECTORMATH_QUAT_SOA_CPP_H
//-----------------------------------------------------------------------------
// Definitions

#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

namespace Vectormath {
namespace Soa {

inline Quat::Quat( const Quat & quat )
{
    mX = quat.mX;
    mY = quat.mY;
    mZ = quat.mZ;
    mW = quat.mW;
}

inline Quat::Quat( vec_float4 _x, vec_float4 _y, vec_float4 _z, vec_float4 _w )
{
    mX = _x;
    mY = _y;
    mZ = _z;
    mW = _w;
}

inline Quat::Quat( const Vector3 & xyz, vec_float4 _w )
{
    this->setXYZ( xyz );
    this->setW( _w );
}

inline Quat::Quat( const Vector4 & vec )
{
    mX = vec.getX();
    mY = vec.getY();
    mZ = vec.getZ();
    mW = vec.getW();
}

inline Quat::Quat( vec_float4 scalar )
{
    mX = scalar;
    mY = scalar;
    mZ = scalar;
    mW = scalar;
}

inline Quat::Quat( Aos::Quat quat )
{
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    vec_uchar16 shuffle_wwww = (vec_uchar16)spu_splats((int)0x0c0d0e0f);
    vec_float4 vec128 = quat.get128();
    mX = spu_shuffle( vec128, vec128, shuffle_xxxx );
    mY = spu_shuffle( vec128, vec128, shuffle_yyyy );
    mZ = spu_shuffle( vec128, vec128, shuffle_zzzz );
    mW = spu_shuffle( vec128, vec128, shuffle_wwww );
}

inline Quat::Quat( Aos::Quat quat0, Aos::Quat quat1, Aos::Quat quat2, Aos::Quat quat3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = spu_shuffle( quat0.get128(), quat2.get128(), _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( quat1.get128(), quat3.get128(), _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( quat0.get128(), quat2.get128(), _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( quat1.get128(), quat3.get128(), _VECTORMATH_SHUF_ZCWD );
    mX = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB );
    mY = spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD );
    mZ = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB );
    mW = spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_ZCWD );
}

inline const Quat Quat::identity( )
{
    return Quat( spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f), spu_splats(1.0f) );
}

inline const Quat lerp( vec_float4 t, const Quat & quat0, const Quat & quat1 )
{
    return ( quat0 + ( ( quat1 - quat0 ) * t ) );
}

inline const Quat slerp( vec_float4 t, const Quat & unitQuat0, const Quat & unitQuat1 )
{
    Quat start;
    vec_float4 recipSinAngle, scale0, scale1, cosAngle, angle;
    vec_uint4 selectMask;
    cosAngle = dot( unitQuat0, unitQuat1 );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(0.0f), cosAngle );
    cosAngle = spu_sel( cosAngle, negatef4( cosAngle ), selectMask );
    start.setX( spu_sel( unitQuat0.getX(), negatef4( unitQuat0.getX() ), selectMask ) );
    start.setY( spu_sel( unitQuat0.getY(), negatef4( unitQuat0.getY() ), selectMask ) );
    start.setZ( spu_sel( unitQuat0.getZ(), negatef4( unitQuat0.getZ() ), selectMask ) );
    start.setW( spu_sel( unitQuat0.getW(), negatef4( unitQuat0.getW() ), selectMask ) );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(_VECTORMATH_SLERP_TOL), cosAngle );
    angle = acosf4( cosAngle );
    recipSinAngle = recipf4( sinf4( angle ) );
    scale0 = spu_sel( spu_sub( spu_splats(1.0f), t ), spu_mul( sinf4( spu_mul( spu_sub( spu_splats(1.0f), t ), angle ) ), recipSinAngle ), selectMask );
    scale1 = spu_sel( t, spu_mul( sinf4( spu_mul( t, angle ) ), recipSinAngle ), selectMask );
    return ( ( start * scale0 ) + ( unitQuat1 * scale1 ) );
}

inline const Quat squad( vec_float4 t, const Quat & unitQuat0, const Quat & unitQuat1, const Quat & unitQuat2, const Quat & unitQuat3 )
{
    Quat tmp0, tmp1;
    tmp0 = slerp( t, unitQuat0, unitQuat3 );
    tmp1 = slerp( t, unitQuat1, unitQuat2 );
    return slerp( spu_mul( spu_mul( spu_splats(2.0f), t ), spu_sub( spu_splats(1.0f), t ) ), tmp0, tmp1 );
}

inline void Quat::get4Aos( Aos::Quat & result0, Aos::Quat & result1, Aos::Quat & result2, Aos::Quat & result3 ) const
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = spu_shuffle( mX, mZ, _VECTORMATH_SHUF_XAYB );
    tmp1 = spu_shuffle( mY, mW, _VECTORMATH_SHUF_XAYB );
    tmp2 = spu_shuffle( mX, mZ, _VECTORMATH_SHUF_ZCWD );
    tmp3 = spu_shuffle( mY, mW, _VECTORMATH_SHUF_ZCWD );
    result0 = Aos::Quat( spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_XAYB ) );
    result1 = Aos::Quat( spu_shuffle( tmp0, tmp1, _VECTORMATH_SHUF_ZCWD ) );
    result2 = Aos::Quat( spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_XAYB ) );
    result3 = Aos::Quat( spu_shuffle( tmp2, tmp3, _VECTORMATH_SHUF_ZCWD ) );
}

inline Quat & Quat::operator =( const Quat & quat )
{
    mX = quat.mX;
    mY = quat.mY;
    mZ = quat.mZ;
    mW = quat.mW;
    return *this;
}

inline Quat & Quat::setXYZ( const Vector3 & vec )
{
    mX = vec.getX();
    mY = vec.getY();
    mZ = vec.getZ();
    return *this;
}

inline const Vector3 Quat::getXYZ( ) const
{
    return Vector3( mX, mY, mZ );
}

inline Quat & Quat::setX( vec_float4 _x )
{
    mX = _x;
    return *this;
}

inline vec_float4 Quat::getX( ) const
{
    return mX;
}

inline Quat & Quat::setY( vec_float4 _y )
{
    mY = _y;
    return *this;
}

inline vec_float4 Quat::getY( ) const
{
    return mY;
}

inline Quat & Quat::setZ( vec_float4 _z )
{
    mZ = _z;
    return *this;
}

inline vec_float4 Quat::getZ( ) const
{
    return mZ;
}

inline Quat & Quat::setW( vec_float4 _w )
{
    mW = _w;
    return *this;
}

inline vec_float4 Quat::getW( ) const
{
    return mW;
}

inline Quat & Quat::setElem( int idx, vec_float4 value )
{
    *(&mX + idx) = value;
    return *this;
}

inline vec_float4 Quat::getElem( int idx ) const
{
    return *(&mX + idx);
}

inline Quat::vec_float4_t & Quat::operator []( int idx )
{
    return *(&mX + idx);
}

inline vec_float4 Quat::operator []( int idx ) const
{
    return *(&mX + idx);
}

inline const Quat Quat::operator +( const Quat & quat ) const
{
    return Quat(
        spu_add( mX, quat.mX ),
        spu_add( mY, quat.mY ),
        spu_add( mZ, quat.mZ ),
        spu_add( mW, quat.mW )
    );
}

inline const Quat Quat::operator -( const Quat & quat ) const
{
    return Quat(
        spu_sub( mX, quat.mX ),
        spu_sub( mY, quat.mY ),
        spu_sub( mZ, quat.mZ ),
        spu_sub( mW, quat.mW )
    );
}

inline const Quat Quat::operator *( vec_float4 scalar ) const
{
    return Quat(
        spu_mul( mX, scalar ),
        spu_mul( mY, scalar ),
        spu_mul( mZ, scalar ),
        spu_mul( mW, scalar )
    );
}

inline Quat & Quat::operator +=( const Quat & quat )
{
    *this = *this + quat;
    return *this;
}

inline Quat & Quat::operator -=( const Quat & quat )
{
    *this = *this - quat;
    return *this;
}

inline Quat & Quat::operator *=( vec_float4 scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const Quat Quat::operator /( vec_float4 scalar ) const
{
    return Quat(
        divf4( mX, scalar ),
        divf4( mY, scalar ),
        divf4( mZ, scalar ),
        divf4( mW, scalar )
    );
}

inline Quat & Quat::operator /=( vec_float4 scalar )
{
    *this = *this / scalar;
    return *this;
}

inline const Quat Quat::operator -( ) const
{
    return Quat(
        negatef4( mX ),
        negatef4( mY ),
        negatef4( mZ ),
        negatef4( mW )
    );
}

inline const Quat operator *( vec_float4 scalar, const Quat & quat )
{
    return quat * scalar;
}

inline vec_float4 dot( const Quat & quat0, const Quat & quat1 )
{
    vec_float4 result;
    result = spu_mul( quat0.getX(), quat1.getX() );
    result = spu_add( result, spu_mul( quat0.getY(), quat1.getY() ) );
    result = spu_add( result, spu_mul( quat0.getZ(), quat1.getZ() ) );
    result = spu_add( result, spu_mul( quat0.getW(), quat1.getW() ) );
    return result;
}

inline vec_float4 norm( const Quat & quat )
{
    vec_float4 result;
    result = spu_mul( quat.getX(), quat.getX() );
    result = spu_add( result, spu_mul( quat.getY(), quat.getY() ) );
    result = spu_add( result, spu_mul( quat.getZ(), quat.getZ() ) );
    result = spu_add( result, spu_mul( quat.getW(), quat.getW() ) );
    return result;
}

inline vec_float4 length( const Quat & quat )
{
    return sqrtf4( norm( quat ) );
}

inline const Quat normalize( const Quat & quat )
{
    vec_float4 lenSqr, lenInv;
    lenSqr = norm( quat );
    lenInv = rsqrtf4( lenSqr );
    return Quat(
        spu_mul( quat.getX(), lenInv ),
        spu_mul( quat.getY(), lenInv ),
        spu_mul( quat.getZ(), lenInv ),
        spu_mul( quat.getW(), lenInv )
    );
}

inline const Quat Quat::rotation( const Vector3 & unitVec0, const Vector3 & unitVec1 )
{
    vec_float4 cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf4( spu_mul( spu_splats(2.0f), spu_add( spu_splats(1.0f), dot( unitVec0, unitVec1 ) ) ) );
    recipCosHalfAngleX2 = recipf4( cosHalfAngleX2 );
    return Quat( ( cross( unitVec0, unitVec1 ) * recipCosHalfAngleX2 ), spu_mul( cosHalfAngleX2, spu_splats(0.5f) ) );
}

inline const Quat Quat::rotation( vec_float4 radians, const Vector3 & unitVec )
{
    vec_float4 s, c, angle;
    angle = spu_mul( radians, spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    return Quat( ( unitVec * s ), c );
}

inline const Quat Quat::rotationX( vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = spu_mul( radians, spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    return Quat( s, spu_splats(0.0f), spu_splats(0.0f), c );
}

inline const Quat Quat::rotationY( vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = spu_mul( radians, spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    return Quat( spu_splats(0.0f), s, spu_splats(0.0f), c );
}

inline const Quat Quat::rotationZ( vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = spu_mul( radians, spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    return Quat( spu_splats(0.0f), spu_splats(0.0f), s, c );
}

inline const Quat Quat::operator *( const Quat & quat ) const
{
    return Quat(
        spu_sub( spu_add( spu_add( spu_mul( mW, quat.mX ), spu_mul( mX, quat.mW ) ), spu_mul( mY, quat.mZ ) ), spu_mul( mZ, quat.mY ) ),
        spu_sub( spu_add( spu_add( spu_mul( mW, quat.mY ), spu_mul( mY, quat.mW ) ), spu_mul( mZ, quat.mX ) ), spu_mul( mX, quat.mZ ) ),
        spu_sub( spu_add( spu_add( spu_mul( mW, quat.mZ ), spu_mul( mZ, quat.mW ) ), spu_mul( mX, quat.mY ) ), spu_mul( mY, quat.mX ) ),
        spu_sub( spu_sub( spu_sub( spu_mul( mW, quat.mW ), spu_mul( mX, quat.mX ) ), spu_mul( mY, quat.mY ) ), spu_mul( mZ, quat.mZ ) )
    );
}

inline Quat & Quat::operator *=( const Quat & quat )
{
    *this = *this * quat;
    return *this;
}

inline const Vector3 rotate( const Quat & quat, const Vector3 & vec )
{
    vec_float4 tmpX, tmpY, tmpZ, tmpW;
    tmpX = spu_sub( spu_add( spu_mul( quat.getW(), vec.getX() ), spu_mul( quat.getY(), vec.getZ() ) ), spu_mul( quat.getZ(), vec.getY() ) );
    tmpY = spu_sub( spu_add( spu_mul( quat.getW(), vec.getY() ), spu_mul( quat.getZ(), vec.getX() ) ), spu_mul( quat.getX(), vec.getZ() ) );
    tmpZ = spu_sub( spu_add( spu_mul( quat.getW(), vec.getZ() ), spu_mul( quat.getX(), vec.getY() ) ), spu_mul( quat.getY(), vec.getX() ) );
    tmpW = spu_add( spu_add( spu_mul( quat.getX(), vec.getX() ), spu_mul( quat.getY(), vec.getY() ) ), spu_mul( quat.getZ(), vec.getZ() ) );
    return Vector3(
        spu_add( spu_sub( spu_add( spu_mul( tmpW, quat.getX() ), spu_mul( tmpX, quat.getW() ) ), spu_mul( tmpY, quat.getZ() ) ), spu_mul( tmpZ, quat.getY() ) ),
        spu_add( spu_sub( spu_add( spu_mul( tmpW, quat.getY() ), spu_mul( tmpY, quat.getW() ) ), spu_mul( tmpZ, quat.getX() ) ), spu_mul( tmpX, quat.getZ() ) ),
        spu_add( spu_sub( spu_add( spu_mul( tmpW, quat.getZ() ), spu_mul( tmpZ, quat.getW() ) ), spu_mul( tmpX, quat.getY() ) ), spu_mul( tmpY, quat.getX() ) )
    );
}

inline const Quat conj( const Quat & quat )
{
    return Quat( negatef4( quat.getX() ), negatef4( quat.getY() ), negatef4( quat.getZ() ), quat.getW() );
}

inline const Quat select( const Quat & quat0, const Quat & quat1, vec_uint4 select1 )
{
    return Quat(
        spu_sel( quat0.getX(), quat1.getX(), select1 ),
        spu_sel( quat0.getY(), quat1.getY(), select1 ),
        spu_sel( quat0.getZ(), quat1.getZ(), select1 ),
        spu_sel( quat0.getW(), quat1.getW(), select1 )
    );
}

#ifdef _VECTORMATH_DEBUG

inline void print( const Quat & quat )
{
    Aos::Quat vec0, vec1, vec2, vec3;
    quat.get4Aos( vec0, vec1, vec2, vec3 );
    printf("slot 0:\n");
    print( vec0 );
    printf("slot 1:\n");
    print( vec1 );
    printf("slot 2:\n");
    print( vec2 );
    printf("slot 3:\n");
    print( vec3 );
}

inline void print( const Quat & quat, const char * name )
{
    Aos::Quat vec0, vec1, vec2, vec3;
    printf( "%s:\n", name );
    quat.get4Aos( vec0, vec1, vec2, vec3 );
    printf("slot 0:\n");
    print( vec0 );
    printf("slot 1:\n");
    print( vec1 );
    printf("slot 2:\n");
    print( vec2 );
    printf("slot 3:\n");
    print( vec3 );
}

#endif

} // namespace Soa
} // namespace Vectormath

#endif
