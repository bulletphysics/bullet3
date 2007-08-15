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
    if (__builtin_constant_p(_x) & __builtin_constant_p(_y) &
        __builtin_constant_p(_z) & __builtin_constant_p(_w)) {
        mVec128 = (vec_float4){_x, _y, _z, _w};
    } else {
        float *pf = (float *)&mVec128;
        pf[0] = _x;
        pf[1] = _y;
        pf[2] = _z;
        pf[3] = _w;
    }
}

inline Quat::Quat( floatInVec _x, floatInVec _y, floatInVec _z, floatInVec _w )
{
    vec_float4 xz = vec_mergeh( _x.get128(), _z.get128() );
    vec_float4 yw = vec_mergeh( _y.get128(), _w.get128() );
    mVec128 = vec_mergeh( xz, yw );
}

inline Quat::Quat( Vector3 xyz, float _w )
{
    mVec128 = xyz.get128();
    _vmathVfSetElement(mVec128, _w, 3);
}

inline Quat::Quat( Vector3 xyz, floatInVec _w )
{
    mVec128 = xyz.get128();
    mVec128 = _vmathVfInsert(mVec128, _w.get128(), 3);
}

inline Quat::Quat( Vector4 vec )
{
    mVec128 = vec.get128();
}

inline Quat::Quat( float scalar )
{
    mVec128 = floatInVec(scalar).get128();
}

inline Quat::Quat( floatInVec scalar )
{
    mVec128 = scalar.get128();
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
    return lerp( floatInVec(t), quat0, quat1 );
}

inline const Quat lerp( floatInVec t, Quat quat0, Quat quat1 )
{
    return ( quat0 + ( ( quat1 - quat0 ) * t ) );
}

inline const Quat slerp( float t, Quat unitQuat0, Quat unitQuat1 )
{
    return slerp( floatInVec(t), unitQuat0, unitQuat1 );
}

inline const Quat slerp( floatInVec t, Quat unitQuat0, Quat unitQuat1 )
{
    Quat start;
    vec_float4 scales, scale0, scale1, cosAngle, angle, tttt, oneMinusT, angles, sines;
    vec_uint4 selectMask;
    cosAngle = _vmathVfDot4( unitQuat0.get128(), unitQuat1.get128() );
    cosAngle = vec_splat( cosAngle, 0 );
    selectMask = (vec_uint4)vec_cmpgt( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), cosAngle );
    cosAngle = vec_sel( cosAngle, negatef4( cosAngle ), selectMask );
    start = Quat( vec_sel( unitQuat0.get128(), negatef4( unitQuat0.get128() ), selectMask ) );
    selectMask = (vec_uint4)vec_cmpgt( ((vec_float4){_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL}), cosAngle );
    angle = acosf4( cosAngle );
    tttt = t.get128();
    oneMinusT = vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), tttt );
    angles = vec_mergeh( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), tttt );
    angles = vec_mergeh( angles, oneMinusT );
    angles = vec_madd( angles, angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sines = sinf4( angles );
    scales = divf4( sines, vec_splat( sines, 0 ) );
    scale0 = vec_sel( oneMinusT, vec_splat( scales, 1 ), selectMask );
    scale1 = vec_sel( tttt, vec_splat( scales, 2 ), selectMask );
    return Quat( vec_madd( start.get128(), scale0, vec_madd( unitQuat1.get128(), scale1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) ) );
}

inline const Quat squad( float t, Quat unitQuat0, Quat unitQuat1, Quat unitQuat2, Quat unitQuat3 )
{
    return squad( floatInVec(t), unitQuat0, unitQuat1, unitQuat2, unitQuat3 );
}

inline const Quat squad( floatInVec t, Quat unitQuat0, Quat unitQuat1, Quat unitQuat2, Quat unitQuat3 )
{
    Quat tmp0, tmp1;
    tmp0 = slerp( t, unitQuat0, unitQuat3 );
    tmp1 = slerp( t, unitQuat1, unitQuat2 );
    return slerp( ( ( floatInVec(2.0f) * t ) * ( floatInVec(1.0f) - t ) ), tmp0, tmp1 );
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
    mVec128 = vec_sel( vec.get128(), mVec128, _VECTORMATH_MASK_0x000F );
    return *this;
}

inline const Vector3 Quat::getXYZ( ) const
{
    return Vector3( mVec128 );
}

inline Quat & Quat::setX( float _x )
{
    _vmathVfSetElement(mVec128, _x, 0);
    return *this;
}

inline Quat & Quat::setX( floatInVec _x )
{
    mVec128 = _vmathVfInsert(mVec128, _x.get128(), 0);
    return *this;
}

inline const floatInVec Quat::getX( ) const
{
    return floatInVec( mVec128, 0 );
}

inline Quat & Quat::setY( float _y )
{
    _vmathVfSetElement(mVec128, _y, 1);
    return *this;
}

inline Quat & Quat::setY( floatInVec _y )
{
    mVec128 = _vmathVfInsert(mVec128, _y.get128(), 1);
    return *this;
}

inline const floatInVec Quat::getY( ) const
{
    return floatInVec( mVec128, 1 );
}

inline Quat & Quat::setZ( float _z )
{
    _vmathVfSetElement(mVec128, _z, 2);
    return *this;
}

inline Quat & Quat::setZ( floatInVec _z )
{
    mVec128 = _vmathVfInsert(mVec128, _z.get128(), 2);
    return *this;
}

inline const floatInVec Quat::getZ( ) const
{
    return floatInVec( mVec128, 2 );
}

inline Quat & Quat::setW( float _w )
{
    _vmathVfSetElement(mVec128, _w, 3);
    return *this;
}

inline Quat & Quat::setW( floatInVec _w )
{
    mVec128 = _vmathVfInsert(mVec128, _w.get128(), 3);
    return *this;
}

inline const floatInVec Quat::getW( ) const
{
    return floatInVec( mVec128, 3 );
}

inline Quat & Quat::setElem( int idx, float value )
{
    _vmathVfSetElement(mVec128, value, idx);
    return *this;
}

inline Quat & Quat::setElem( int idx, floatInVec value )
{
    mVec128 = _vmathVfInsert(mVec128, value.get128(), idx);
    return *this;
}

inline const floatInVec Quat::getElem( int idx ) const
{
    return floatInVec( mVec128, idx );
}

inline VecIdx Quat::operator []( int idx )
{
    return VecIdx( mVec128, idx );
}

inline const floatInVec Quat::operator []( int idx ) const
{
    return floatInVec( mVec128, idx );
}

inline const Quat Quat::operator +( Quat quat ) const
{
    return Quat( vec_add( mVec128, quat.mVec128 ) );
}

inline const Quat Quat::operator -( Quat quat ) const
{
    return Quat( vec_sub( mVec128, quat.mVec128 ) );
}

inline const Quat Quat::operator *( float scalar ) const
{
    return *this * floatInVec(scalar);
}

inline const Quat Quat::operator *( floatInVec scalar ) const
{
    return Quat( vec_madd( mVec128, scalar.get128(), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
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

inline Quat & Quat::operator *=( floatInVec scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const Quat Quat::operator /( float scalar ) const
{
    return *this / floatInVec(scalar);
}

inline const Quat Quat::operator /( floatInVec scalar ) const
{
    return Quat( divf4( mVec128, scalar.get128() ) );
}

inline Quat & Quat::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

inline Quat & Quat::operator /=( floatInVec scalar )
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
    return floatInVec(scalar) * quat;
}

inline const Quat operator *( floatInVec scalar, Quat quat )
{
    return quat * scalar;
}

inline const floatInVec dot( Quat quat0, Quat quat1 )
{
    return floatInVec( _vmathVfDot4( quat0.get128(), quat1.get128() ), 0 );
}

inline const floatInVec norm( Quat quat )
{
    return floatInVec(  _vmathVfDot4( quat.get128(), quat.get128() ), 0 );
}

inline const floatInVec length( Quat quat )
{
    return floatInVec(  sqrtf4(_vmathVfDot4( quat.get128(), quat.get128() )), 0 );
}

inline const Quat normalize( Quat quat )
{
    vec_float4 dot = _vmathVfDot4( quat.get128(), quat.get128() );
    return Quat( vec_madd( quat.get128(), rsqrtf4( dot ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
}

inline const Quat Quat::rotation( Vector3 unitVec0, Vector3 unitVec1 )
{
    Vector3 crossVec;
    vec_float4 cosAngle, cosAngleX2Plus2, recipCosHalfAngleX2, cosHalfAngleX2, res;
    cosAngle = _vmathVfDot3( unitVec0.get128(), unitVec1.get128() );
    cosAngle = vec_splat( cosAngle, 0 );
    cosAngleX2Plus2 = vec_madd( cosAngle, ((vec_float4){2.0f,2.0f,2.0f,2.0f}), ((vec_float4){2.0f,2.0f,2.0f,2.0f}) );
    recipCosHalfAngleX2 = rsqrtf4( cosAngleX2Plus2 );
    cosHalfAngleX2 = vec_madd( recipCosHalfAngleX2, cosAngleX2Plus2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    crossVec = cross( unitVec0, unitVec1 );
    res = vec_madd( crossVec.get128(), recipCosHalfAngleX2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    res = vec_sel( res, vec_madd( cosHalfAngleX2, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), _VECTORMATH_MASK_0x000F );
    return Quat( res );
}

inline const Quat Quat::rotation( float radians, Vector3 unitVec )
{
    return rotation( floatInVec(radians), unitVec );
}

inline const Quat Quat::rotation( floatInVec radians, Vector3 unitVec )
{
    vec_float4 s, c, angle, res;
    angle = vec_madd( radians.get128(), ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    res = vec_sel( vec_madd( unitVec.get128(), s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), c, _VECTORMATH_MASK_0x000F );
    return Quat( res );
}

inline const Quat Quat::rotationX( float radians )
{
    return rotationX( floatInVec(radians) );
}

inline const Quat Quat::rotationX( floatInVec radians )
{
    vec_float4 s, c, angle, res;
    angle = vec_madd( radians.get128(), ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    res = vec_sel( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, _VECTORMATH_MASK_0xF000 );
    res = vec_sel( res, c, _VECTORMATH_MASK_0x000F );
    return Quat( res );
}

inline const Quat Quat::rotationY( float radians )
{
    return rotationY( floatInVec(radians) );
}

inline const Quat Quat::rotationY( floatInVec radians )
{
    vec_float4 s, c, angle, res;
    angle = vec_madd( radians.get128(), ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    res = vec_sel( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, _VECTORMATH_MASK_0x0F00 );
    res = vec_sel( res, c, _VECTORMATH_MASK_0x000F );
    return Quat( res );
}

inline const Quat Quat::rotationZ( float radians )
{
    return rotationZ( floatInVec(radians) );
}

inline const Quat Quat::rotationZ( floatInVec radians )
{
    vec_float4 s, c, angle, res;
    angle = vec_madd( radians.get128(), ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    res = vec_sel( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, _VECTORMATH_MASK_0x00F0 );
    res = vec_sel( res, c, _VECTORMATH_MASK_0x000F );
    return Quat( res );
}

inline const Quat Quat::operator *( Quat quat ) const
{
    vec_float4 ldata, rdata, qv, tmp0, tmp1, tmp2, tmp3;
    vec_float4 product, l_wxyz, r_wxyz, xy, qw;
    ldata = mVec128;
    rdata = quat.mVec128;
    tmp0 = vec_perm( ldata, ldata, _VECTORMATH_PERM_YZXW );
    tmp1 = vec_perm( rdata, rdata, _VECTORMATH_PERM_ZXYW );
    tmp2 = vec_perm( ldata, ldata, _VECTORMATH_PERM_ZXYW );
    tmp3 = vec_perm( rdata, rdata, _VECTORMATH_PERM_YZXW );
    qv = vec_madd( vec_splat( ldata, 3 ), rdata, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qv = vec_madd( vec_splat( rdata, 3 ), ldata, qv );
    qv = vec_madd( tmp0, tmp1, qv );
    qv = vec_nmsub( tmp2, tmp3, qv );
    product = vec_madd( ldata, rdata, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    l_wxyz = vec_sld( ldata, ldata, 12 );
    r_wxyz = vec_sld( rdata, rdata, 12 );
    qw = vec_nmsub( l_wxyz, r_wxyz, product );
    xy = vec_madd( l_wxyz, r_wxyz, product );
    qw = vec_sub( qw, vec_sld( xy, xy, 8 ) );
    return Quat( vec_sel( qv, qw, _VECTORMATH_MASK_0x000F ) );
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
    tmp0 = vec_perm( qdata, qdata, _VECTORMATH_PERM_YZXW );
    tmp1 = vec_perm( vdata, vdata, _VECTORMATH_PERM_ZXYW );
    tmp2 = vec_perm( qdata, qdata, _VECTORMATH_PERM_ZXYW );
    tmp3 = vec_perm( vdata, vdata, _VECTORMATH_PERM_YZXW );
    wwww = vec_splat( qdata, 3 );
    qv = vec_madd( wwww, vdata, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qv = vec_madd( tmp0, tmp1, qv );
    qv = vec_nmsub( tmp2, tmp3, qv );
    product = vec_madd( qdata, vdata, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    qw = vec_madd( vec_sld( qdata, qdata, 4 ), vec_sld( vdata, vdata, 4 ), product );
    qw = vec_add( vec_sld( product, product, 8 ), qw );
    tmp1 = vec_perm( qv, qv, _VECTORMATH_PERM_ZXYW );
    tmp3 = vec_perm( qv, qv, _VECTORMATH_PERM_YZXW );
    res = vec_madd( vec_splat( qw, 0 ), qdata, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    res = vec_madd( wwww, qv, res );
    res = vec_madd( tmp0, tmp1, res );
    res = vec_nmsub( tmp2, tmp3, res );
    return Vector3( res );
}

inline const Quat conj( Quat quat )
{
    return Quat( vec_xor( quat.get128(), ((vec_float4)(vec_int4){0x80000000,0x80000000,0x80000000,0}) ) );
}

inline const Quat select( Quat quat0, Quat quat1, bool select1 )
{
    return select( quat0, quat1, boolInVec(select1) );
}

inline const Quat select( Quat quat0, Quat quat1, boolInVec select1 )
{
    return Quat( vec_sel( quat0.get128(), quat1.get128(), select1.get128() ) );
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
