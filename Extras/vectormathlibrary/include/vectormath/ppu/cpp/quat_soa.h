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
    vec_float4 vec128 = quat.get128();
    mX = vec_splat( vec128, 0 );
    mY = vec_splat( vec128, 1 );
    mZ = vec_splat( vec128, 2 );
    mW = vec_splat( vec128, 3 );
}

inline Quat::Quat( Aos::Quat quat0, Aos::Quat quat1, Aos::Quat quat2, Aos::Quat quat3 )
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = vec_mergeh( quat0.get128(), quat2.get128() );
    tmp1 = vec_mergeh( quat1.get128(), quat3.get128() );
    tmp2 = vec_mergel( quat0.get128(), quat2.get128() );
    tmp3 = vec_mergel( quat1.get128(), quat3.get128() );
    mX = vec_mergeh( tmp0, tmp1 );
    mY = vec_mergel( tmp0, tmp1 );
    mZ = vec_mergeh( tmp2, tmp3 );
    mW = vec_mergel( tmp2, tmp3 );
}

inline const Quat Quat::identity( )
{
    return Quat( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){1.0f,1.0f,1.0f,1.0f}) );
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
    selectMask = (vec_uint4)vec_cmpgt( (vec_float4){0.0f,0.0f,0.0f,0.0f}, cosAngle );
    cosAngle = vec_sel( cosAngle, negatef4( cosAngle ), selectMask );
    start.setX( vec_sel( unitQuat0.getX(), negatef4( unitQuat0.getX() ), selectMask ) );
    start.setY( vec_sel( unitQuat0.getY(), negatef4( unitQuat0.getY() ), selectMask ) );
    start.setZ( vec_sel( unitQuat0.getZ(), negatef4( unitQuat0.getZ() ), selectMask ) );
    start.setW( vec_sel( unitQuat0.getW(), negatef4( unitQuat0.getW() ), selectMask ) );
    selectMask = (vec_uint4)vec_cmpgt( (vec_float4){_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL}, cosAngle );
    angle = acosf4( cosAngle );
    recipSinAngle = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sinf4( angle ) );
    scale0 = vec_sel( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), vec_madd( sinf4( vec_madd( vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), recipSinAngle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), selectMask );
    scale1 = vec_sel( t, vec_madd( sinf4( vec_madd( t, angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), recipSinAngle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), selectMask );
    return ( ( start * scale0 ) + ( unitQuat1 * scale1 ) );
}

inline const Quat squad( vec_float4 t, const Quat & unitQuat0, const Quat & unitQuat1, const Quat & unitQuat2, const Quat & unitQuat3 )
{
    Quat tmp0, tmp1;
    tmp0 = slerp( t, unitQuat0, unitQuat3 );
    tmp1 = slerp( t, unitQuat1, unitQuat2 );
    return slerp( vec_madd( vec_madd( ((vec_float4){2.0f,2.0f,2.0f,2.0f}), t, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), t ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), tmp0, tmp1 );
}

inline void Quat::get4Aos( Aos::Quat & result0, Aos::Quat & result1, Aos::Quat & result2, Aos::Quat & result3 ) const
{
    vec_float4 tmp0, tmp1, tmp2, tmp3;
    tmp0 = vec_mergeh( mX, mZ );
    tmp1 = vec_mergeh( mY, mW );
    tmp2 = vec_mergel( mX, mZ );
    tmp3 = vec_mergel( mY, mW );
    result0 = Aos::Quat( vec_mergeh( tmp0, tmp1 ) );
    result1 = Aos::Quat( vec_mergel( tmp0, tmp1 ) );
    result2 = Aos::Quat( vec_mergeh( tmp2, tmp3 ) );
    result3 = Aos::Quat( vec_mergel( tmp2, tmp3 ) );
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
        vec_add( mX, quat.mX ),
        vec_add( mY, quat.mY ),
        vec_add( mZ, quat.mZ ),
        vec_add( mW, quat.mW )
    );
}

inline const Quat Quat::operator -( const Quat & quat ) const
{
    return Quat(
        vec_sub( mX, quat.mX ),
        vec_sub( mY, quat.mY ),
        vec_sub( mZ, quat.mZ ),
        vec_sub( mW, quat.mW )
    );
}

inline const Quat Quat::operator *( vec_float4 scalar ) const
{
    return Quat(
        vec_madd( mX, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        vec_madd( mY, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        vec_madd( mZ, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        vec_madd( mW, scalar, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
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
    result = vec_madd( quat0.getX(), quat1.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( quat0.getY(), quat1.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( quat0.getZ(), quat1.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( quat0.getW(), quat1.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    return result;
}

inline vec_float4 norm( const Quat & quat )
{
    vec_float4 result;
    result = vec_madd( quat.getX(), quat.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    result = vec_add( result, vec_madd( quat.getY(), quat.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( quat.getZ(), quat.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    result = vec_add( result, vec_madd( quat.getW(), quat.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
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
    lenInv = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), sqrtf4( lenSqr ) );
    return Quat(
        vec_madd( quat.getX(), lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        vec_madd( quat.getY(), lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        vec_madd( quat.getZ(), lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ),
        vec_madd( quat.getW(), lenInv, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) )
    );
}

inline const Quat Quat::rotation( const Vector3 & unitVec0, const Vector3 & unitVec1 )
{
    vec_float4 cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf4( vec_madd( ((vec_float4){2.0f,2.0f,2.0f,2.0f}), vec_add( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), dot( unitVec0, unitVec1 ) ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    recipCosHalfAngleX2 = divf4( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), cosHalfAngleX2 );
    return Quat( ( cross( unitVec0, unitVec1 ) * recipCosHalfAngleX2 ), vec_madd( cosHalfAngleX2, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

inline const Quat Quat::rotation( vec_float4 radians, const Vector3 & unitVec )
{
    vec_float4 s, c, angle;
    angle = vec_madd( radians, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    return Quat( ( unitVec * s ), c );
}

inline const Quat Quat::rotationX( vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = vec_madd( radians, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    return Quat( s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c );
}

inline const Quat Quat::rotationY( vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = vec_madd( radians, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    return Quat( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}), c );
}

inline const Quat Quat::rotationZ( vec_float4 radians )
{
    vec_float4 s, c, angle;
    angle = vec_madd( radians, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    return Quat( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, c );
}

inline const Quat Quat::operator *( const Quat & quat ) const
{
    return Quat(
        vec_sub( vec_add( vec_add( vec_madd( mW, quat.mX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mX, quat.mW, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mY, quat.mZ, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mZ, quat.mY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_sub( vec_add( vec_add( vec_madd( mW, quat.mY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mY, quat.mW, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mZ, quat.mX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mX, quat.mZ, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_sub( vec_add( vec_add( vec_madd( mW, quat.mZ, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mZ, quat.mW, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mX, quat.mY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mY, quat.mX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_sub( vec_sub( vec_sub( vec_madd( mW, quat.mW, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( mX, quat.mX, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mY, quat.mY, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( mZ, quat.mZ, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
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
    tmpX = vec_sub( vec_add( vec_madd( quat.getW(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat.getY(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat.getZ(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpY = vec_sub( vec_add( vec_madd( quat.getW(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat.getZ(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat.getX(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpZ = vec_sub( vec_add( vec_madd( quat.getW(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat.getX(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat.getY(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    tmpW = vec_add( vec_add( vec_madd( quat.getX(), vec.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( quat.getY(), vec.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( quat.getZ(), vec.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
    return Vector3(
        vec_add( vec_sub( vec_add( vec_madd( tmpW, quat.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmpX, quat.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpY, quat.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpZ, quat.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_sub( vec_add( vec_madd( tmpW, quat.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmpY, quat.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpZ, quat.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpX, quat.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ),
        vec_add( vec_sub( vec_add( vec_madd( tmpW, quat.getZ(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), vec_madd( tmpZ, quat.getW(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpX, quat.getY(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ), vec_madd( tmpY, quat.getX(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) )
    );
}

inline const Quat conj( const Quat & quat )
{
    return Quat( negatef4( quat.getX() ), negatef4( quat.getY() ), negatef4( quat.getZ() ), quat.getW() );
}

inline const Quat select( const Quat & quat0, const Quat & quat1, vec_uint4 select1 )
{
    return Quat(
        vec_sel( quat0.getX(), quat1.getX(), select1 ),
        vec_sel( quat0.getY(), quat1.getY(), select1 ),
        vec_sel( quat0.getZ(), quat1.getZ(), select1 ),
        vec_sel( quat0.getW(), quat1.getW(), select1 )
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
