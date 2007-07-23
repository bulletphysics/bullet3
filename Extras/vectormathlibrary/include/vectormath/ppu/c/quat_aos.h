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
    if (__builtin_constant_p(_x) & __builtin_constant_p(_y) &
        __builtin_constant_p(_z) & __builtin_constant_p(_w)) {
        result->vec128 = (vec_float4){_x, _y, _z, _w};
    } else {
        float *pf = (float *)&result->vec128;
        pf[0] = _x;
        pf[1] = _y;
        pf[2] = _z;
        pf[3] = _w;
    }
}

static inline void vmathQMakeFromV3Scalar( VmathQuat *result, const VmathVector3 *xyz, float _w )
{
    result->vec128 = xyz->vec128;
    _vmathVfSetElement(result->vec128, _w, 3);
}

static inline void vmathQMakeFromV4( VmathQuat *result, const VmathVector4 *vec )
{
    result->vec128 = vec->vec128;
}

static inline void vmathQMakeFromScalar( VmathQuat *result, float scalar )
{
    result->vec128 = _vmathVfSplatScalar(scalar);
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
    cosAngle = _vmathVfDot4( unitQuat0->vec128, unitQuat1->vec128 );
    cosAngle = vec_splat( cosAngle, 0 );
    selectMask = (vec_uint4)vec_cmpgt( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), cosAngle );
    cosAngle = vec_sel( cosAngle, negatef4( cosAngle ), selectMask );
    start.vec128 = vec_sel( unitQuat0->vec128, negatef4( unitQuat0->vec128 ), selectMask );
    selectMask = (vec_uint4)vec_cmpgt( ((vec_float4){_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL,_VECTORMATH_SLERP_TOL}), cosAngle );
    angle = acosf4( cosAngle );
    tttt = _vmathVfSplatScalar(t);
    oneMinusT = vec_sub( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), tttt );
    angles = vec_mergeh( ((vec_float4){1.0f,1.0f,1.0f,1.0f}), tttt );
    angles = vec_mergeh( angles, oneMinusT );
    angles = vec_madd( angles, angle, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sines = sinf4( angles );
    scales = divf4( sines, vec_splat( sines, 0 ) );
    scale0 = vec_sel( oneMinusT, vec_splat( scales, 1 ), selectMask );
    scale1 = vec_sel( tttt, vec_splat( scales, 2 ), selectMask );
    result->vec128 = vec_madd( start.vec128, scale0, vec_madd( unitQuat1->vec128, scale1, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ) );
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
    result->vec128 = vec_sel( vec->vec128, result->vec128, _VECTORMATH_MASK_0x000F );
}

static inline void vmathQGetXYZ( VmathVector3 *result, const VmathQuat *quat )
{
    result->vec128 = quat->vec128;
}

static inline void vmathQSetX( VmathQuat *result, float _x )
{
    _vmathVfSetElement(result->vec128, _x, 0);
}

static inline float vmathQGetX( const VmathQuat *quat )
{
    return _vmathVfGetElement(quat->vec128, 0);
}

static inline void vmathQSetY( VmathQuat *result, float _y )
{
    _vmathVfSetElement(result->vec128, _y, 1);
}

static inline float vmathQGetY( const VmathQuat *quat )
{
    return _vmathVfGetElement(quat->vec128, 1);
}

static inline void vmathQSetZ( VmathQuat *result, float _z )
{
    _vmathVfSetElement(result->vec128, _z, 2);
}

static inline float vmathQGetZ( const VmathQuat *quat )
{
    return _vmathVfGetElement(quat->vec128, 2);
}

static inline void vmathQSetW( VmathQuat *result, float _w )
{
    _vmathVfSetElement(result->vec128, _w, 3);
}

static inline float vmathQGetW( const VmathQuat *quat )
{
    return _vmathVfGetElement(quat->vec128, 3);
}

static inline void vmathQSetElem( VmathQuat *result, int idx, float value )
{
    _vmathVfSetElement(result->vec128, value, idx);
}

static inline float vmathQGetElem( const VmathQuat *quat, int idx )
{
    return _vmathVfGetElement(quat->vec128, idx);
}

static inline void vmathQAdd( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    result->vec128 = vec_add( quat0->vec128, quat1->vec128 );
}

static inline void vmathQSub( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    result->vec128 = vec_sub( quat0->vec128, quat1->vec128 );
}

static inline void vmathQScalarMul( VmathQuat *result, const VmathQuat *quat, float scalar )
{
    result->vec128 = vec_madd( quat->vec128, _vmathVfSplatScalar(scalar), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathQScalarDiv( VmathQuat *result, const VmathQuat *quat, float scalar )
{
    result->vec128 = divf4( quat->vec128, _vmathVfSplatScalar(scalar) );
}

static inline void vmathQNeg( VmathQuat *result, const VmathQuat *quat )
{
    result->vec128 = negatef4( quat->vec128 );
}

static inline float vmathQDot( const VmathQuat *quat0, const VmathQuat *quat1 )
{
    vec_float4 result = _vmathVfDot4( quat0->vec128, quat1->vec128 );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathQNorm( const VmathQuat *quat )
{
    vec_float4 result = _vmathVfDot4( quat->vec128, quat->vec128 );
    return _vmathVfGetElement(result, 0);
}

static inline float vmathQLength( const VmathQuat *quat )
{
    return sqrtf( vmathQNorm( quat ) );
}

static inline void vmathQNormalize( VmathQuat *result, const VmathQuat *quat )
{
    vec_float4 dot = _vmathVfDot4( quat->vec128, quat->vec128 );
    result->vec128 = vec_madd( quat->vec128, rsqrtf4( dot ), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
}

static inline void vmathQMakeRotationArc( VmathQuat *result, const VmathVector3 *unitVec0, const VmathVector3 *unitVec1 )
{
    VmathVector3 crossVec, tmpV3_0;
    vec_float4 cosAngle, cosAngleX2Plus2, recipCosHalfAngleX2, cosHalfAngleX2, res;
    cosAngle = _vmathVfDot3( unitVec0->vec128, unitVec1->vec128 );
    cosAngle = vec_splat( cosAngle, 0 );
    cosAngleX2Plus2 = vec_madd( cosAngle, ((vec_float4){2.0f,2.0f,2.0f,2.0f}), ((vec_float4){2.0f,2.0f,2.0f,2.0f}) );
    recipCosHalfAngleX2 = rsqrtf4( cosAngleX2Plus2 );
    cosHalfAngleX2 = vec_madd( recipCosHalfAngleX2, cosAngleX2Plus2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    vmathV3Cross( &tmpV3_0, unitVec0, unitVec1 );
    crossVec = tmpV3_0;
    res = vec_madd( crossVec.vec128, recipCosHalfAngleX2, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    res = vec_sel( res, vec_madd( cosHalfAngleX2, ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), _VECTORMATH_MASK_0x000F );
    result->vec128 = res;
}

static inline void vmathQMakeRotationAxis( VmathQuat *result, float radians, const VmathVector3 *unitVec )
{
    vec_float4 s, c, angle, res;
    angle = vec_madd( _vmathVfSplatScalar(radians), ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    res = vec_sel( vec_madd( unitVec->vec128, s, ((vec_float4){0.0f,0.0f,0.0f,0.0f}) ), c, _VECTORMATH_MASK_0x000F );
    result->vec128 = res;
}

static inline void vmathQMakeRotationX( VmathQuat *result, float radians )
{
    vec_float4 s, c, angle, res;
    angle = vec_madd( _vmathVfSplatScalar(radians), ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    res = vec_sel( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, _VECTORMATH_MASK_0xF000 );
    res = vec_sel( res, c, _VECTORMATH_MASK_0x000F );
    result->vec128 = res;
}

static inline void vmathQMakeRotationY( VmathQuat *result, float radians )
{
    vec_float4 s, c, angle, res;
    angle = vec_madd( _vmathVfSplatScalar(radians), ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    res = vec_sel( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, _VECTORMATH_MASK_0x0F00 );
    res = vec_sel( res, c, _VECTORMATH_MASK_0x000F );
    result->vec128 = res;
}

static inline void vmathQMakeRotationZ( VmathQuat *result, float radians )
{
    vec_float4 s, c, angle, res;
    angle = vec_madd( _vmathVfSplatScalar(radians), ((vec_float4){0.5f,0.5f,0.5f,0.5f}), ((vec_float4){0.0f,0.0f,0.0f,0.0f}) );
    sincosf4( angle, &s, &c );
    res = vec_sel( ((vec_float4){0.0f,0.0f,0.0f,0.0f}), s, _VECTORMATH_MASK_0x00F0 );
    res = vec_sel( res, c, _VECTORMATH_MASK_0x000F );
    result->vec128 = res;
}

static inline void vmathQMul( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1 )
{
    vec_float4 ldata, rdata, qv, tmp0, tmp1, tmp2, tmp3;
    vec_float4 product, l_wxyz, r_wxyz, xy, qw;
    ldata = quat0->vec128;
    rdata = quat1->vec128;
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
    result->vec128 = vec_sel( qv, qw, _VECTORMATH_MASK_0x000F );
}

static inline void vmathQRotate( VmathVector3 *result, const VmathQuat *quat, const VmathVector3 *vec )
{
    vec_float4 qdata, vdata, product, tmp0, tmp1, tmp2, tmp3, wwww, qv, qw, res;
    qdata = quat->vec128;
    vdata = vec->vec128;
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
    result->vec128 = res;
}

static inline void vmathQConj( VmathQuat *result, const VmathQuat *quat )
{
    result->vec128 = vec_xor( quat->vec128, ((vec_float4)(vec_int4){0x80000000,0x80000000,0x80000000,0}) );
}

static inline void vmathQSelect( VmathQuat *result, const VmathQuat *quat0, const VmathQuat *quat1, unsigned int select1 )
{
    unsigned int tmp;
    tmp = (unsigned int)-(select1 > 0);
    result->vec128 = vec_sel( quat0->vec128, quat1->vec128, _vmathVuiSplatScalar(tmp) );
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
