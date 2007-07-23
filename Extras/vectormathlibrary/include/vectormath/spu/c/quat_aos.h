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

#ifndef _VECTORMATH_QUAT_AOS_C_H
#define _VECTORMATH_QUAT_AOS_C_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Definitions
 */
#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

static inline void vmathQCopy( VmathQuat *result, const VmathQuat *quat )
{
    result->vec128 = quat->vec128;
}

static inline void vmathQMakeFromElems( VmathQuat *result, float _x, float _y, float _z, float _w )
{
    result->vec128 = (vec_float4){ _x, _y, _z, _w };
}

static inline void vmathQMakeFromV3Scalar( VmathQuat *result, const VmathVector3 *xyz, float _w )
{
    result->vec128 = spu_shuffle( xyz->vec128, spu_promote( _w, 0 ), _VECTORMATH_SHUF_XYZA );
}

static inline void vmathQMakeFromV4( VmathQuat *result, const VmathVector4 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathQMakeFromScalar( VmathQuat *result, float scalar )
{
    result->vec128 = spu_splats( scalar );
}

static inline void vmathQMakeFrom128( VmathQuat *result, vec_float4 vf4 )
{
    result->vec128 = vf4;
}

static inline void vmathQMakeIdentity( VmathQuat *result )
{
    result->vec128 = _VECTORMATH_UNIT_0001;
}

static inline void vmathQLerp( VmathQuat *result, float t, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    VmathQuat tmpQ_0, tmpQ_1;
    vmathQSub( &tmpQ_0, quat1, quat0 );
    vmathQScalarMul( &tmpQ_1, &tmpQ_0, t );
    vmathQAdd( result, quat0, &tmpQ_1 );
}

static inline void vmathQSlerp( VmathQuat *result, float t, const VmathQuat *unitQuat0, const VmathQuat *unitQuat1 )
{
    VmathQuat start;
    vec_float4 scales, scale0, scale1, cosAngle, angle, tttt, oneMinusT, angles, sines;
    vec_uint4 selectMask;
    vec_uchar16 shuffle_xxxx = (vec_uchar16)spu_splats((int)0x00010203);
    vec_uchar16 shuffle_yyyy = (vec_uchar16)spu_splats((int)0x04050607);
    vec_uchar16 shuffle_zzzz = (vec_uchar16)spu_splats((int)0x08090a0b);
    cosAngle = _vmathVfDot4( unitQuat0->vec128, unitQuat1->vec128 );
    cosAngle = spu_shuffle( cosAngle, cosAngle, shuffle_xxxx );
    selectMask = (vec_uint4)spu_cmpgt( spu_splats(0.0f), cosAngle );
    cosAngle = spu_sel( cosAngle, negatef4( cosAngle ), selectMask );
    start.vec128 = spu_sel( unitQuat0->vec128, negatef4( unitQuat0->vec128 ), selectMask );
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
    result->vec128 = spu_madd( start.vec128, scale0, spu_mul( unitQuat1->vec128, scale1 ) );
}

static inline void vmathQSquad( VmathQuat *result, float t, const VmathQuat *unitQuat0, const VmathQuat *unitQuat1, const VmathQuat *unitQuat2, const VmathQuat *unitQuat3 )
{
    VmathQuat tmp0, tmp1;
    vmathQSlerp( &tmp0, t, unitQuat0, unitQuat3 );
    vmathQSlerp( &tmp1, t, unitQuat1, unitQuat2 );
    vmathQSlerp( result, ( ( 2.0f * t ) * ( 1.0f - t ) ), &tmp0, &tmp1 );
}

static inline vec_float4 vmathQGet128( const VmathQuat *quat )
{
    return quat->vec128;
}

static inline void vmathQSetXYZ( VmathQuat *result, const VmathVector3 *vec )
{
    result->vec128 = spu_sel( vec->vec128, result->vec128, (vec_uint4)spu_maskb(0x000f) );
}

static inline void vmathQGetXYZ( VmathVector3 *result, const VmathQuat *quat )
{
    result->vec128 = quat->vec128;
}

static inline void vmathQSetX( VmathQuat *result, float _x )
{
    result->vec128 = spu_insert( _x, result->vec128, 0 );
}

static inline float vmathQGetX( const VmathQuat *quat )
{
    return spu_extract( quat->vec128, 0 );
}

static inline void vmathQSetY( VmathQuat *result, float _y )
{
    result->vec128 = spu_insert( _y, result->vec128, 1 );
}

static inline float vmathQGetY( const VmathQuat *quat )
{
    return spu_extract( quat->vec128, 1 );
}

static inline void vmathQSetZ( VmathQuat *result, float _z )
{
    result->vec128 = spu_insert( _z, result->vec128, 2 );
}

static inline float vmathQGetZ( const VmathQuat *quat )
{
    return spu_extract( quat->vec128, 2 );
}

static inline void vmathQSetW( VmathQuat *result, float _w )
{
    result->vec128 = spu_insert( _w, result->vec128, 3 );
}

static inline float vmathQGetW( const VmathQuat *quat )
{
    return spu_extract( quat->vec128, 3 );
}

static inline void vmathQSetElem( VmathQuat *result, int idx, float value )
{
    result->vec128 = spu_insert( value, result->vec128, idx );
}

static inline float vmathQGetElem( const VmathQuat *quat, int idx )
{
    return spu_extract( quat->vec128, idx );
}

static inline void vmathQAdd( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    result->vec128 = spu_add( quat0->vec128, quat1->vec128 );
}

static inline void vmathQSub( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    result->vec128 = spu_sub( quat0->vec128, quat1->vec128 );
}

static inline void vmathQScalarMul( VmathQuat *result, const VmathQuat *quat, float scalar )
{
    result->vec128 = spu_mul( quat->vec128, spu_splats(scalar) );
}

static inline void vmathQScalarDiv( VmathQuat *result, const VmathQuat *quat, float scalar )
{
    result->vec128 = divf4( quat->vec128, spu_splats(scalar) );
}

static inline void vmathQNeg( VmathQuat *result, const VmathQuat *quat )
{
    result->vec128 = negatef4( quat->vec128 );
}

static inline float vmathQDot( const VmathQuat *quat0, const VmathQuat *quat1 )
{
    return spu_extract( _vmathVfDot4( quat0->vec128, quat1->vec128 ), 0 );
}

static inline float vmathQNorm( const VmathQuat *quat )
{
    return spu_extract( _vmathVfDot4( quat->vec128, quat->vec128 ), 0 );
}

static inline float vmathQLength( const VmathQuat *quat )
{
    return sqrtf( vmathQNorm( quat ) );
}

static inline void vmathQNormalize( VmathQuat *result, const VmathQuat *quat )
{
    vec_float4 dot = _vmathVfDot4( quat->vec128, quat->vec128 );
    result->vec128 = spu_mul( quat->vec128, rsqrtf4( dot ) );
}

static inline void vmathQMakeRotationArc( VmathQuat *result, const VmathVector3 *unitVec0, const VmathVector3 *unitVec1 )
{
    VmathVector3 crossVec, tmpV3_0;
    vec_float4 cosAngle, cosAngleX2Plus2, recipCosHalfAngleX2, cosHalfAngleX2, res;
    cosAngle = _vmathVfDot3( unitVec0->vec128, unitVec1->vec128 );
    cosAngle = spu_shuffle( cosAngle, cosAngle, (vec_uchar16)spu_splats(0x00010203) );
    cosAngleX2Plus2 = spu_madd( cosAngle, spu_splats(2.0f), spu_splats(2.0f) );
    recipCosHalfAngleX2 = rsqrtf4( cosAngleX2Plus2 );
    cosHalfAngleX2 = spu_mul( recipCosHalfAngleX2, cosAngleX2Plus2 );
    vmathV3Cross( &tmpV3_0, unitVec0, unitVec1 );
    crossVec = tmpV3_0;
    res = spu_mul( crossVec.vec128, recipCosHalfAngleX2 );
    res = spu_sel( res, spu_mul( cosHalfAngleX2, spu_splats(0.5f) ), (vec_uint4)spu_maskb(0x000f) );
    result->vec128 = res;
}

static inline void vmathQMakeRotationAxis( VmathQuat *result, float radians, const VmathVector3 *unitVec )
{
    vec_float4 s, c, angle, res;
    angle = spu_mul( spu_splats(radians), spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    res = spu_sel( spu_mul( unitVec->vec128, s ), c, (vec_uint4)spu_maskb(0x000f) );
    result->vec128 = res;
}

static inline void vmathQMakeRotationX( VmathQuat *result, float radians )
{
    vec_float4 s, c, angle, res;
    angle = spu_mul( spu_splats(radians), spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    res = spu_sel( spu_splats(0.0f), s, (vec_uint4)spu_maskb(0xf000) );
    res = spu_sel( res, c, (vec_uint4)spu_maskb(0x000f) );
    result->vec128 = res;
}

static inline void vmathQMakeRotationY( VmathQuat *result, float radians )
{
    vec_float4 s, c, angle, res;
    angle = spu_mul( spu_splats(radians), spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    res = spu_sel( spu_splats(0.0f), s, (vec_uint4)spu_maskb(0x0f00) );
    res = spu_sel( res, c, (vec_uint4)spu_maskb(0x000f) );
    result->vec128 = res;
}

static inline void vmathQMakeRotationZ( VmathQuat *result, float radians )
{
    vec_float4 s, c, angle, res;
    angle = spu_mul( spu_splats(radians), spu_splats(0.5f) );
    sincosf4( angle, &s, &c );
    res = spu_sel( spu_splats(0.0f), s, (vec_uint4)spu_maskb(0x00f0) );
    res = spu_sel( res, c, (vec_uint4)spu_maskb(0x000f) );
    result->vec128 = res;
}

static inline void vmathQMul( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    vec_float4 ldata, rdata, qv, tmp0, tmp1, tmp2, tmp3;
    vec_float4 product, l_wxyz, r_wxyz, xy, qw;
    ldata = quat0->vec128;
    rdata = quat1->vec128;
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
    result->vec128 = spu_sel( qv, qw, (vec_uint4)spu_maskb( 0x000f ) );
}

static inline void vmathQRotate( VmathVector3 *result, const VmathQuat *quat, const VmathVector3 *vec )
{
    vec_float4 qdata, vdata, product, tmp0, tmp1, tmp2, tmp3, wwww, qv, qw, res;
    qdata = quat->vec128;
    vdata = vec->vec128;
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
    result->vec128 = res;
}

static inline void vmathQConj( VmathQuat *result, const VmathQuat *quat )
{
    result->vec128 = spu_xor( quat->vec128, ((vec_float4)(vec_int4){0x80000000,0x80000000,0x80000000,0}) );
}

static inline void vmathQSelect( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1, unsigned int select1 )
{
    result->vec128 = spu_sel( quat0->vec128, quat1->vec128, spu_splats( (unsigned int)-(select1 > 0) ) );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathQPrint( const VmathQuat *quat )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = quat->vec128;
    printf( "( %f %f %f %f )\n", tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

static inline void vmathQPrints( const VmathQuat *quat, const char *name )
{
    union { vec_float4 v; float s[4]; } tmp;
    tmp.v = quat->vec128;
    printf( "%s: ( %f %f %f %f )\n", name, tmp.s[0], tmp.s[1], tmp.s[2], tmp.s[3] );
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
