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

#ifndef _VECTORMATH_QUAT_AOS_CPP_H
#define _VECTORMATH_QUAT_AOS_CPP_H
//-----------------------------------------------------------------------------
// Definitions

#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

namespace Vectormath {
namespace Aos {

inline Quat::Quat( float _x, float _y, float _z, float _w )
{
    mVec128 = (vec_float4){ _x, _y, _z, _w };
}

inline Quat::Quat( Vector3 xyz, float _w )
{
    mVec128 = spu_shuffle( xyz.get128(), spu_promote( _w, 0 ), _VECTORMATH_SHUF_XYZA );
}

inline Quat::Quat( Vector4 vec )
{
    mVec128 = vec.get128();
}

inline Quat::Quat( float scalar )
{
    mVec128 = spu_splats( scalar );
}

inline Quat::Quat( vec_float4 vf4 )
{
    mVec128 = vf4;
}

inline const Quat Quat::identity( )
{
    return Quat( _VECTORMATH_UNIT_0001 );
}

inline const Quat lerp( float t, Quat quat0, Quat quat1 )
{
    return ( quat0 + ( ( quat1 - quat0 ) * t ) );
}

inline const Quat slerp( float t, Quat unitQuat0, Quat unitQuat1 )
{
    Quat start;
    vec_float4 scales, scale0, scale1, cosAngle, angle, tttt, oneMinusT, angles, sines;
    vec_uint4 selectMask;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    cosAngle = _vmathVfDot4( unitQuat0.get128(), unitQuat1.get128() );
    cosAngle = spu_shuffle( cosAngle, cosAngle, shuffle_xxxx );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(0.0f), cosAngle );
    cosAngle = spu_sel( cosAngle, negatef4( cosAngle ), selectMask );
    start = Quat( spu_sel( unitQuat0.get128(), negatef4( unitQuat0.get128() ), selectMask ) );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(_VECTORMATH_SLERP_TOL), cosAngle );
    angle = acosf4( cosAngle );
    tttt = spu_splats(t);
    oneMinusT = spu_sub( spu_splats(1.0f), tttt );
    angles = spu_sel( spu_splats(1.0f), oneMinusT, (vec_uint4)spu_maskb(0x0f00) );
    angles = spu_sel( angles, tttt, (vec_uint4)spu_maskb(0x00f0) );
    angles = spu_mul( angles, angle );
    sines = sinf4( angles );
    scales = divf4( sines, spu_shuffle( sines, sines, shuffle_xxxx ) );
    scale0 = spu_sel( oneMinusT, spu_shuffle( scales, scales, shuffle_yyyy ), selectMask );
    scale1 = spu_sel( tttt, spu_shuffle( scales, scales, shuffle_zzzz ), selectMask );
    return Quat( spu_madd( start.get128(), scale0, spu_mul( unitQuat1.get128(), scale1 ) ) );
}

inline const Quat squad( float t, Quat unitQuat0, Quat unitQuat1, Quat unitQuat2, Quat unitQuat3 )
{
    Quat tmp0, tmp1;
    tmp0 = slerp( t, unitQuat0, unitQuat3 );
    tmp1 = slerp( t, unitQuat1, unitQuat2 );
    return slerp( ( ( 2.0f * t ) * ( 1.0f - t ) ), tmp0, tmp1 );
}

inline vec_float4 Quat::get128( ) const
{
    return mVec128;
}

inline Quat & Quat::operator =( Quat quat )
{
    mVec128 = quat.mVec128;
    return *this;
}

inline Quat & Quat::setXYZ( Vector3 vec )
{
    mVec128 = spu_sel( vec.get128(), mVec128, (vec_uint4)spu_maskb(0x000f) );
    return *this;
}

inline const Vector3 Quat::getXYZ( ) const
{
    return Vector3( mVec128 );
}

inline Quat & Quat::setX( float _x )
{
    mVec128 = spu_insert( _x, mVec128, 0 );
    return *this;
}

inline float Quat::getX( ) const
{
    return spu_extract( mVec128, 0 );
}

inline Quat & Quat::setY( float _y )
{
    mVec128 = spu_insert( _y, mVec128, 1 );
    return *this;
}

inline float Quat::getY( ) const
{
    return spu_extract( mVec128, 1 );
}

inline Quat & Quat::setZ( float _z )
{
    mVec128 = spu_insert( _z, mVec128, 2 );
    return *this;
}

inline float Quat::getZ( ) const
{
    return spu_extract( mVec128, 2 );
}

inline Quat & Quat::setW( float _w )
{
    mVec128 = spu_insert( _w, mVec128, 3 );
    return *this;
}

inline float Quat::getW( ) const
{
    return spu_extract( mVec128, 3 );
}

inline Quat & Quat::setElem( int idx, float value )
{
    mVec128 = spu_insert( value, mVec128, idx );
    return *this;
}

inline float Quat::getElem( int idx ) const
{
    return spu_extract( mVec128, idx );
}

inline VecIdx Quat::operator []( int idx )
{
    return VecIdx( mVec128, idx );
}

inline float Quat::operator []( int idx ) const
{
    return spu_extract( mVec128, idx );
}

inline const Quat Quat::operator +( Quat quat ) const
{
    return Quat( spu_add( mVec128, quat.mVec128 ) );
}

inline const Quat Quat::operator -( Quat quat ) const
{
    return Quat( spu_sub( mVec128, quat.mVec128 ) );
}

inline const Quat Quat::operator *( float scalar ) const
{
    return Quat( spu_mul( mVec128, spu_splats(scalar) ) );
}

inline Quat & Quat::operator +=( Quat quat )
{
    *this = *this + quat;
    return *this;
}

inline Quat & Quat::operator -=( Quat quat )
{
    *this = *this - quat;
    return *this;
}

inline Quat & Quat::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const Quat Quat::operator /( float scalar ) const
{
    return Quat( divf4( mVec128, spu_splats(scalar) ) );
}

inline Quat & Quat::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

inline const Quat Quat::operator -( ) const
{
    return Quat( negatef4( mVec128 ) );
}

inline const Quat operator *( float scalar, Quat quat )
{
    return quat * scalar;
}

inline float dot( Quat quat0, Quat quat1 )
{
    return spu_extract( _vmathVfDot4( quat0.get128(), quat1.get128() ), 0 );
}

inline float norm( Quat quat )
{
    return spu_extract( _vmathVfDot4( quat.get128(), quat.get128() ), 0 );
}

inline float length( Quat quat )
{
    return sqrtf( norm( quat ) );
}

inline const Quat normalize( Quat quat )
{
    vec_float4 dot = _vmathVfDot4( quat.get128(), quat.get128() );
    return Quat( spu_mul( quat.get128(), rsqrtf4( dot ) ) );
}

inline const Quat Quat::rotation( Vector3 unitVec0, Vector3 unitVec1 )
{
    Vector3 crossVec;
    vec_float4 cosAngle, cosAngleX2Plus2, recipCosHalfAngleX2, cosHalfAngleX2, res;
    cosAngle = _vmathVfDot3( unitVec0.get128(), unitVec1.get128() );
    cosAngle = spu_shuffle( cosAngle, cosAngle, (vec_uchar16)spu_splats(0x00010203) );
    cosAngleX2Plus2 = spu_madd( cosAngle, spu_splats(2.0f), spu_splats(2.0f) );
    recipCosHalfAngleX2 = rsqrtf4( cosAngleX2Plus2 );
    cosHalfAngleX2 = spu_mul( recipCosHalfAngleX2, cosAngleX2Plus2 );
    crossVec = cross( unitVec0, unitVec1 );
    res = spu_mul( crossVec.get128(), recipCosHalfAngleX2 );
    res = spu_sel( res, spu_mul( cosHalfAngleX2, spu_splats(0.5f) ), (vec_uint4)spu_maskb(0x000f) );
    return Quat( res );
}

inline const Quat Quat::rotation( float radians, Vector3 unitVec )
{
    vec_float4 s, c, angle, res;
    angle = spu_mul( spu_splats(radians), spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    res = spu_sel( spu_mul( unitVec.get128(), s ), c, (vec_uint4)spu_maskb(0x000f) );
    return Quat( res );
}

inline const Quat Quat::rotationX( float radians )
{
    vec_float4 s, c, angle, res;
    angle = spu_mul( spu_splats(radians), spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    res = spu_sel( spu_splats(0.0f), s, (vec_uint4)spu_maskb(0xf000) );
    res = spu_sel( res, c, (vec_uint4)spu_maskb(0x000f) );
    return Quat( res );
}

inline const Quat Quat::rotationY( float radians )
{
    vec_float4 s, c, angle, res;
    angle = spu_mul( spu_splats(radians), spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    res = spu_sel( spu_splats(0.0f), s, (vec_uint4)spu_maskb(0x0f00) );
    res = spu_sel( res, c, (vec_uint4)spu_maskb(0x000f) );
    return Quat( res );
}

inline const Quat Quat::rotationZ( float radians )
{
    vec_float4 s, c, angle, res;
    angle = spu_mul( spu_splats(radians), spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    res = spu_sel( spu_splats(0.0f), s, (vec_uint4)spu_maskb(0x00f0) );
    res = spu_sel( res, c, (vec_uint4)spu_maskb(0x000f) );
    return Quat( res );
}

inline const Quat Quat::operator *( Quat quat ) const
{
    vec_float4 ldata, rdata, qv, tmp0, tmp1, tmp2, tmp3;
    vec_float4 product, l_wxyz, r_wxyz, xy, qw;
    ldata = mVec128;
    rdata = quat.mVec128;
    vec_uchar16 shuffle_wwww = (vec_uchar16)spu_splats((int)0x0c0d0e0f);
    tmp0 = spu_shuffle( ldata, ldata, _VECTORMATH_SHUF_YZXW );
    tmp1 = spu_shuffle( rdata, rdata, _VECTORMATH_SHUF_ZXYW );
    tmp2 = spu_shuffle( ldata, ldata, _VECTORMATH_SHUF_ZXYW );
    tmp3 = spu_shuffle( rdata, rdata, _VECTORMATH_SHUF_YZXW );
    qv = spu_mul( spu_shuffle( ldata, ldata, shuffle_wwww ), rdata );
    qv = spu_madd( spu_shuffle( rdata, rdata, shuffle_wwww ), ldata, qv );
    qv = spu_madd( tmp0, tmp1, qv );
    qv = spu_nmsub( tmp2, tmp3, qv );
    product = spu_mul( ldata, rdata );
    l_wxyz = spu_rlqwbyte( ldata, 12 );
    r_wxyz = spu_rlqwbyte( rdata, 12 );
    qw = spu_nmsub( l_wxyz, r_wxyz, product );
    xy = spu_madd( l_wxyz, r_wxyz, product );
    qw = spu_sub( qw, spu_rlqwbyte( xy, 8 ) );
    return Quat( spu_sel( qv, qw, (vec_uint4)spu_maskb( 0x000f ) ) );
}

inline Quat & Quat::operator *=( Quat quat )
{
    *this = *this * quat;
    return *this;
}

inline const Vector3 rotate( Quat quat, Vector3 vec )
{
    vec_float4 qdata, vdata, product, tmp0, tmp1, tmp2, tmp3, wwww, qv, qw, res;
    qdata = quat.get128();
    vdata = vec.get128();
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_wwww = (vec_uchar16)spu_splats((int)0x0c0d0e0f);
    tmp0 = spu_shuffle( qdata, qdata, _VECTORMATH_SHUF_YZXW );
    tmp1 = spu_shuffle( vdata, vdata, _VECTORMATH_SHUF_ZXYW );
    tmp2 = spu_shuffle( qdata, qdata, _VECTORMATH_SHUF_ZXYW );
    tmp3 = spu_shuffle( vdata, vdata, _VECTORMATH_SHUF_YZXW );
    wwww = spu_shuffle( qdata, qdata, shuffle_wwww );
    qv = spu_mul( wwww, vdata );
    qv = spu_madd( tmp0, tmp1, qv );
    qv = spu_nmsub( tmp2, tmp3, qv );
    product = spu_mul( qdata, vdata );
    qw = spu_madd( spu_rlqwbyte( qdata, 4 ), spu_rlqwbyte( vdata, 4 ), product );
    qw = spu_add( spu_rlqwbyte( product, 8 ), qw );
    tmp1 = spu_shuffle( qv, qv, _VECTORMATH_SHUF_ZXYW );
    tmp3 = spu_shuffle( qv, qv, _VECTORMATH_SHUF_YZXW );
    res = spu_mul( spu_shuffle( qw, qw, shuffle_xxxx ), qdata );
    res = spu_madd( wwww, qv, res );
    res = spu_madd( tmp0, tmp1, res );
    res = spu_nmsub( tmp2, tmp3, res );
    return Vector3( res );
}

inline const Quat conj( Quat quat )
{
    return Quat( spu_xor( quat.get128(), ((vec_float4)(vec_int4){0x80000000,0x80000000,0x80000000,0}) ) );
}

inline const Quat select( Quat quat0, Quat quat1, bool select1 )
{
    return Quat( spu_sel( quat0.get128(), quat1.get128(), spu_splats( (unsigned int)-(select1 > 0) ) ) );
}

#ifdef _VECTORMATH_DEBUG

inline void print( Quat quat )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = quat.get128();
    printf( "( %f %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

inline void print( Quat quat, const char * name )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = quat.get128();
    printf( "%s: ( %f %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

#endif

} // namespace Aos
} // namespace Vectormath

#endif
