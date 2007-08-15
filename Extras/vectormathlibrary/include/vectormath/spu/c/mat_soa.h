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

#ifndef _VECTORMATH_MAT_SOA_C_H
#define _VECTORMATH_MAT_SOA_C_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Constants
 */
#define _VECTORMATH_PI_OVER_2 1.570796327f

/*-----------------------------------------------------------------------------
 * Definitions
 */
static inline void vmathSoaM3Copy( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat )
{
    vmathSoaV3Copy( &result->col0, &mat->col0 );
    vmathSoaV3Copy( &result->col1, &mat->col1 );
    vmathSoaV3Copy( &result->col2, &mat->col2 );
}

static inline void vmathSoaM3MakeFromScalar( VmathSoaMatrix3 *result, vec_float4 scalar )
{
    vmathSoaV3MakeFromScalar( &result->col0, scalar );
    vmathSoaV3MakeFromScalar( &result->col1, scalar );
    vmathSoaV3MakeFromScalar( &result->col2, scalar );
}

static inline void vmathSoaM3MakeFromQ( VmathSoaMatrix3 *result, const VmathSoaQuat *unitQuat )
{
    vec_float4 qx, qy, qz, qw, qx2, qy2, qz2, qxqx2, qyqy2, qzqz2, qxqy2, qyqz2, qzqw2, qxqz2, qyqw2, qxqw2;
    qx = unitQuat->x;
    qy = unitQuat->y;
    qz = unitQuat->z;
    qw = unitQuat->w;
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
    vmathSoaV3MakeFromElems( &result->col0, spu_sub( spu_sub( spu_splats(1.0f), qyqy2 ), qzqz2 ), spu_add( qxqy2, qzqw2 ), spu_sub( qxqz2, qyqw2 ) );
    vmathSoaV3MakeFromElems( &result->col1, spu_sub( qxqy2, qzqw2 ), spu_sub( spu_sub( spu_splats(1.0f), qxqx2 ), qzqz2 ), spu_add( qyqz2, qxqw2 ) );
    vmathSoaV3MakeFromElems( &result->col2, spu_add( qxqz2, qyqw2 ), spu_sub( qyqz2, qxqw2 ), spu_sub( spu_sub( spu_splats(1.0f), qxqx2 ), qyqy2 ) );
}

static inline void vmathSoaM3MakeFromCols( VmathSoaMatrix3 *result, const VmathSoaVector3 *_col0, const VmathSoaVector3 *_col1, const VmathSoaVector3 *_col2 )
{
    vmathSoaV3Copy( &result->col0, _col0 );
    vmathSoaV3Copy( &result->col1, _col1 );
    vmathSoaV3Copy( &result->col2, _col2 );
}

static inline void vmathSoaM3MakeFromAos( VmathSoaMatrix3 *result, const VmathMatrix3 *mat )
{
    vmathSoaV3MakeFromAos( &result->col0, &mat->col0 );
    vmathSoaV3MakeFromAos( &result->col1, &mat->col1 );
    vmathSoaV3MakeFromAos( &result->col2, &mat->col2 );
}

static inline void vmathSoaM3MakeFrom4Aos( VmathSoaMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1, const VmathMatrix3 *mat2, const VmathMatrix3 *mat3 )
{
    vmathSoaV3MakeFrom4Aos( &result->col0, &mat0->col0, &mat1->col0, &mat2->col0, &mat3->col0 );
    vmathSoaV3MakeFrom4Aos( &result->col1, &mat0->col1, &mat1->col1, &mat2->col1, &mat3->col1 );
    vmathSoaV3MakeFrom4Aos( &result->col2, &mat0->col2, &mat1->col2, &mat2->col2, &mat3->col2 );
}

static inline void vmathSoaM3Get4Aos( const VmathSoaMatrix3 *mat, VmathMatrix3 *result0, VmathMatrix3 *result1, VmathMatrix3 *result2, VmathMatrix3 *result3 )
{
    vmathSoaV3Get4Aos( &mat->col0, &result0->col0, &result1->col0, &result2->col0, &result3->col0 );
    vmathSoaV3Get4Aos( &mat->col1, &result0->col1, &result1->col1, &result2->col1, &result3->col1 );
    vmathSoaV3Get4Aos( &mat->col2, &result0->col2, &result1->col2, &result2->col2, &result3->col2 );
}

static inline void vmathSoaM3SetCol0( VmathSoaMatrix3 *result, const VmathSoaVector3 *_col0 )
{
    vmathSoaV3Copy( &result->col0, _col0 );
}

static inline void vmathSoaM3SetCol1( VmathSoaMatrix3 *result, const VmathSoaVector3 *_col1 )
{
    vmathSoaV3Copy( &result->col1, _col1 );
}

static inline void vmathSoaM3SetCol2( VmathSoaMatrix3 *result, const VmathSoaVector3 *_col2 )
{
    vmathSoaV3Copy( &result->col2, _col2 );
}

static inline void vmathSoaM3SetCol( VmathSoaMatrix3 *result, int col, const VmathSoaVector3 *vec )
{
    vmathSoaV3Copy( (&result->col0 + col), vec );
}

static inline void vmathSoaM3SetRow( VmathSoaMatrix3 *result, int row, const VmathSoaVector3 *vec )
{
    vmathSoaV3SetElem( &result->col0, row, vmathSoaV3GetElem( vec, 0 ) );
    vmathSoaV3SetElem( &result->col1, row, vmathSoaV3GetElem( vec, 1 ) );
    vmathSoaV3SetElem( &result->col2, row, vmathSoaV3GetElem( vec, 2 ) );
}

static inline void vmathSoaM3SetElem( VmathSoaMatrix3 *result, int col, int row, vec_float4 val )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaM3GetCol( &tmpV3_0, result, col );
    vmathSoaV3SetElem( &tmpV3_0, row, val );
    vmathSoaM3SetCol( result, col, &tmpV3_0 );
}

static inline vec_float4 vmathSoaM3GetElem( const VmathSoaMatrix3 *mat, int col, int row )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaM3GetCol( &tmpV3_0, mat, col );
    return vmathSoaV3GetElem( &tmpV3_0, row );
}

static inline void vmathSoaM3GetCol0( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat )
{
    vmathSoaV3Copy( result, &mat->col0 );
}

static inline void vmathSoaM3GetCol1( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat )
{
    vmathSoaV3Copy( result, &mat->col1 );
}

static inline void vmathSoaM3GetCol2( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat )
{
    vmathSoaV3Copy( result, &mat->col2 );
}

static inline void vmathSoaM3GetCol( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat, int col )
{
    vmathSoaV3Copy( result, (&mat->col0 + col) );
}

static inline void vmathSoaM3GetRow( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat, int row )
{
    vmathSoaV3MakeFromElems( result, vmathSoaV3GetElem( &mat->col0, row ), vmathSoaV3GetElem( &mat->col1, row ), vmathSoaV3GetElem( &mat->col2, row ) );
}

static inline void vmathSoaM3Transpose( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat )
{
    VmathSoaMatrix3 tmpResult;
    vmathSoaV3MakeFromElems( &tmpResult.col0, mat->col0.x, mat->col1.x, mat->col2.x );
    vmathSoaV3MakeFromElems( &tmpResult.col1, mat->col0.y, mat->col1.y, mat->col2.y );
    vmathSoaV3MakeFromElems( &tmpResult.col2, mat->col0.z, mat->col1.z, mat->col2.z );
    vmathSoaM3Copy( result, &tmpResult );
}

static inline void vmathSoaM3Inverse( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat )
{
    VmathSoaVector3 tmp0, tmp1, tmp2;
    vec_float4 detinv;
    vmathSoaV3Cross( &tmp0, &mat->col1, &mat->col2 );
    vmathSoaV3Cross( &tmp1, &mat->col2, &mat->col0 );
    vmathSoaV3Cross( &tmp2, &mat->col0, &mat->col1 );
    detinv = recipf4( vmathSoaV3Dot( &mat->col2, &tmp2 ) );
    vmathSoaV3MakeFromElems( &result->col0, spu_mul( tmp0.x, detinv ), spu_mul( tmp1.x, detinv ), spu_mul( tmp2.x, detinv ) );
    vmathSoaV3MakeFromElems( &result->col1, spu_mul( tmp0.y, detinv ), spu_mul( tmp1.y, detinv ), spu_mul( tmp2.y, detinv ) );
    vmathSoaV3MakeFromElems( &result->col2, spu_mul( tmp0.z, detinv ), spu_mul( tmp1.z, detinv ), spu_mul( tmp2.z, detinv ) );
}

static inline vec_float4 vmathSoaM3Determinant( const VmathSoaMatrix3 *mat )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaV3Cross( &tmpV3_0, &mat->col0, &mat->col1 );
    return vmathSoaV3Dot( &mat->col2, &tmpV3_0 );
}

static inline void vmathSoaM3Add( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1 )
{
    vmathSoaV3Add( &result->col0, &mat0->col0, &mat1->col0 );
    vmathSoaV3Add( &result->col1, &mat0->col1, &mat1->col1 );
    vmathSoaV3Add( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathSoaM3Sub( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1 )
{
    vmathSoaV3Sub( &result->col0, &mat0->col0, &mat1->col0 );
    vmathSoaV3Sub( &result->col1, &mat0->col1, &mat1->col1 );
    vmathSoaV3Sub( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathSoaM3Neg( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat )
{
    vmathSoaV3Neg( &result->col0, &mat->col0 );
    vmathSoaV3Neg( &result->col1, &mat->col1 );
    vmathSoaV3Neg( &result->col2, &mat->col2 );
}

static inline void vmathSoaM3AbsPerElem( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat )
{
    vmathSoaV3AbsPerElem( &result->col0, &mat->col0 );
    vmathSoaV3AbsPerElem( &result->col1, &mat->col1 );
    vmathSoaV3AbsPerElem( &result->col2, &mat->col2 );
}

static inline void vmathSoaM3ScalarMul( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat, vec_float4 scalar )
{
    vmathSoaV3ScalarMul( &result->col0, &mat->col0, scalar );
    vmathSoaV3ScalarMul( &result->col1, &mat->col1, scalar );
    vmathSoaV3ScalarMul( &result->col2, &mat->col2, scalar );
}

static inline void vmathSoaM3MulV3( VmathSoaVector3 *result, const VmathSoaMatrix3 *mat, const VmathSoaVector3 *vec )
{
    vec_float4 tmpX, tmpY, tmpZ;
    tmpX = spu_add( spu_add( spu_mul( mat->col0.x, vec->x ), spu_mul( mat->col1.x, vec->y ) ), spu_mul( mat->col2.x, vec->z ) );
    tmpY = spu_add( spu_add( spu_mul( mat->col0.y, vec->x ), spu_mul( mat->col1.y, vec->y ) ), spu_mul( mat->col2.y, vec->z ) );
    tmpZ = spu_add( spu_add( spu_mul( mat->col0.z, vec->x ), spu_mul( mat->col1.z, vec->y ) ), spu_mul( mat->col2.z, vec->z ) );
    vmathSoaV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathSoaM3Mul( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1 )
{
    VmathSoaMatrix3 tmpResult;
    vmathSoaM3MulV3( &tmpResult.col0, mat0, &mat1->col0 );
    vmathSoaM3MulV3( &tmpResult.col1, mat0, &mat1->col1 );
    vmathSoaM3MulV3( &tmpResult.col2, mat0, &mat1->col2 );
    vmathSoaM3Copy( result, &tmpResult );
}

static inline void vmathSoaM3MulPerElem( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1 )
{
    vmathSoaV3MulPerElem( &result->col0, &mat0->col0, &mat1->col0 );
    vmathSoaV3MulPerElem( &result->col1, &mat0->col1, &mat1->col1 );
    vmathSoaV3MulPerElem( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathSoaM3MakeIdentity( VmathSoaMatrix3 *result )
{
    vmathSoaV3MakeXAxis( &result->col0 );
    vmathSoaV3MakeYAxis( &result->col1 );
    vmathSoaV3MakeZAxis( &result->col2 );
}

static inline void vmathSoaM3MakeRotationX( VmathSoaMatrix3 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV3MakeXAxis( &result->col0 );
    vmathSoaV3MakeFromElems( &result->col1, spu_splats(0.0f), c, s );
    vmathSoaV3MakeFromElems( &result->col2, spu_splats(0.0f), negatef4( s ), c );
}

static inline void vmathSoaM3MakeRotationY( VmathSoaMatrix3 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV3MakeFromElems( &result->col0, c, spu_splats(0.0f), negatef4( s ) );
    vmathSoaV3MakeYAxis( &result->col1 );
    vmathSoaV3MakeFromElems( &result->col2, s, spu_splats(0.0f), c );
}

static inline void vmathSoaM3MakeRotationZ( VmathSoaMatrix3 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV3MakeFromElems( &result->col0, c, s, spu_splats(0.0f) );
    vmathSoaV3MakeFromElems( &result->col1, negatef4( s ), c, spu_splats(0.0f) );
    vmathSoaV3MakeZAxis( &result->col2 );
}

static inline void vmathSoaM3MakeRotationZYX( VmathSoaMatrix3 *result, const VmathSoaVector3 *radiansXYZ )
{
    vec_float4 sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sincosf4( radiansXYZ->x, &sX, &cX );
    sincosf4( radiansXYZ->y, &sY, &cY );
    sincosf4( radiansXYZ->z, &sZ, &cZ );
    tmp0 = spu_mul( cZ, sY );
    tmp1 = spu_mul( sZ, sY );
    vmathSoaV3MakeFromElems( &result->col0, spu_mul( cZ, cY ), spu_mul( sZ, cY ), negatef4( sY ) );
    vmathSoaV3MakeFromElems( &result->col1, spu_sub( spu_mul( tmp0, sX ), spu_mul( sZ, cX ) ), spu_add( spu_mul( tmp1, sX ), spu_mul( cZ, cX ) ), spu_mul( cY, sX ) );
    vmathSoaV3MakeFromElems( &result->col2, spu_add( spu_mul( tmp0, cX ), spu_mul( sZ, sX ) ), spu_sub( spu_mul( tmp1, cX ), spu_mul( cZ, sX ) ), spu_mul( cY, cX ) );
}

static inline void vmathSoaM3MakeRotationAxis( VmathSoaMatrix3 *result, vec_float4 radians, const VmathSoaVector3 *unitVec )
{
    vec_float4 x, y, z, s, c, oneMinusC, xy, yz, zx;
    sincosf4( radians, &s, &c );
    x = unitVec->x;
    y = unitVec->y;
    z = unitVec->z;
    xy = spu_mul( x, y );
    yz = spu_mul( y, z );
    zx = spu_mul( z, x );
    oneMinusC = spu_sub( spu_splats(1.0f), c );
    vmathSoaV3MakeFromElems( &result->col0, spu_add( spu_mul( spu_mul( x, x ), oneMinusC ), c ), spu_add( spu_mul( xy, oneMinusC ), spu_mul( z, s ) ), spu_sub( spu_mul( zx, oneMinusC ), spu_mul( y, s ) ) );
    vmathSoaV3MakeFromElems( &result->col1, spu_sub( spu_mul( xy, oneMinusC ), spu_mul( z, s ) ), spu_add( spu_mul( spu_mul( y, y ), oneMinusC ), c ), spu_add( spu_mul( yz, oneMinusC ), spu_mul( x, s ) ) );
    vmathSoaV3MakeFromElems( &result->col2, spu_add( spu_mul( zx, oneMinusC ), spu_mul( y, s ) ), spu_sub( spu_mul( yz, oneMinusC ), spu_mul( x, s ) ), spu_add( spu_mul( spu_mul( z, z ), oneMinusC ), c ) );
}

static inline void vmathSoaM3MakeRotationQ( VmathSoaMatrix3 *result, const VmathSoaQuat *unitQuat )
{
    vmathSoaM3MakeFromQ( result, unitQuat );
}

static inline void vmathSoaM3MakeScale( VmathSoaMatrix3 *result, const VmathSoaVector3 *scaleVec )
{
    vmathSoaV3MakeFromElems( &result->col0, scaleVec->x, spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV3MakeFromElems( &result->col1, spu_splats(0.0f), scaleVec->y, spu_splats(0.0f) );
    vmathSoaV3MakeFromElems( &result->col2, spu_splats(0.0f), spu_splats(0.0f), scaleVec->z );
}

static inline void vmathSoaM3AppendScale( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat, const VmathSoaVector3 *scaleVec )
{
    vmathSoaV3ScalarMul( &result->col0, &mat->col0, vmathSoaV3GetX( scaleVec ) );
    vmathSoaV3ScalarMul( &result->col1, &mat->col1, vmathSoaV3GetY( scaleVec ) );
    vmathSoaV3ScalarMul( &result->col2, &mat->col2, vmathSoaV3GetZ( scaleVec ) );
}

static inline void vmathSoaM3PrependScale( VmathSoaMatrix3 *result, const VmathSoaVector3 *scaleVec, const VmathSoaMatrix3 *mat )
{
    vmathSoaV3MulPerElem( &result->col0, &mat->col0, scaleVec );
    vmathSoaV3MulPerElem( &result->col1, &mat->col1, scaleVec );
    vmathSoaV3MulPerElem( &result->col2, &mat->col2, scaleVec );
}

static inline void vmathSoaM3Select( VmathSoaMatrix3 *result, const VmathSoaMatrix3 *mat0, const VmathSoaMatrix3 *mat1, vec_uint4 select1 )
{
    vmathSoaV3Select( &result->col0, &mat0->col0, &mat1->col0, select1 );
    vmathSoaV3Select( &result->col1, &mat0->col1, &mat1->col1, select1 );
    vmathSoaV3Select( &result->col2, &mat0->col2, &mat1->col2, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaM3Print( const VmathSoaMatrix3 *mat )
{
    VmathMatrix3 mat0, mat1, mat2, mat3;
    vmathSoaM3Get4Aos( mat, &mat0, &mat1, &mat2, &mat3 );
    printf("slot 0:\n");
    vmathM3Print( &mat0 );
    printf("slot 1:\n");
    vmathM3Print( &mat1 );
    printf("slot 2:\n");
    vmathM3Print( &mat2 );
    printf("slot 3:\n");
    vmathM3Print( &mat3 );
}

static inline void vmathSoaM3Prints( const VmathSoaMatrix3 *mat, const char *name )
{
    printf("%s:\n", name);
    vmathSoaM3Print( mat );
}

#endif

static inline void vmathSoaM4Copy( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4Copy( &result->col0, &mat->col0 );
    vmathSoaV4Copy( &result->col1, &mat->col1 );
    vmathSoaV4Copy( &result->col2, &mat->col2 );
    vmathSoaV4Copy( &result->col3, &mat->col3 );
}

static inline void vmathSoaM4MakeFromScalar( VmathSoaMatrix4 *result, vec_float4 scalar )
{
    vmathSoaV4MakeFromScalar( &result->col0, scalar );
    vmathSoaV4MakeFromScalar( &result->col1, scalar );
    vmathSoaV4MakeFromScalar( &result->col2, scalar );
    vmathSoaV4MakeFromScalar( &result->col3, scalar );
}

static inline void vmathSoaM4MakeFromT3( VmathSoaMatrix4 *result, const VmathSoaTransform3 *mat )
{
    vmathSoaV4MakeFromV3Scalar( &result->col0, &mat->col0, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col1, &mat->col1, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col2, &mat->col2, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col3, &mat->col3, spu_splats(1.0f) );
}

static inline void vmathSoaM4MakeFromCols( VmathSoaMatrix4 *result, const VmathSoaVector4 *_col0, const VmathSoaVector4 *_col1, const VmathSoaVector4 *_col2, const VmathSoaVector4 *_col3 )
{
    vmathSoaV4Copy( &result->col0, _col0 );
    vmathSoaV4Copy( &result->col1, _col1 );
    vmathSoaV4Copy( &result->col2, _col2 );
    vmathSoaV4Copy( &result->col3, _col3 );
}

static inline void vmathSoaM4MakeFromM3V3( VmathSoaMatrix4 *result, const VmathSoaMatrix3 *mat, const VmathSoaVector3 *translateVec )
{
    vmathSoaV4MakeFromV3Scalar( &result->col0, &mat->col0, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col1, &mat->col1, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col2, &mat->col2, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col3, translateVec, spu_splats(1.0f) );
}

static inline void vmathSoaM4MakeFromQV3( VmathSoaMatrix4 *result, const VmathSoaQuat *unitQuat, const VmathSoaVector3 *translateVec )
{
    VmathSoaMatrix3 mat;
    vmathSoaM3MakeFromQ( &mat, unitQuat );
    vmathSoaV4MakeFromV3Scalar( &result->col0, &mat.col0, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col1, &mat.col1, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col2, &mat.col2, spu_splats(0.0f) );
    vmathSoaV4MakeFromV3Scalar( &result->col3, translateVec, spu_splats(1.0f) );
}

static inline void vmathSoaM4MakeFromAos( VmathSoaMatrix4 *result, const VmathMatrix4 *mat )
{
    vmathSoaV4MakeFromAos( &result->col0, &mat->col0 );
    vmathSoaV4MakeFromAos( &result->col1, &mat->col1 );
    vmathSoaV4MakeFromAos( &result->col2, &mat->col2 );
    vmathSoaV4MakeFromAos( &result->col3, &mat->col3 );
}

static inline void vmathSoaM4MakeFrom4Aos( VmathSoaMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1, const VmathMatrix4 *mat2, const VmathMatrix4 *mat3 )
{
    vmathSoaV4MakeFrom4Aos( &result->col0, &mat0->col0, &mat1->col0, &mat2->col0, &mat3->col0 );
    vmathSoaV4MakeFrom4Aos( &result->col1, &mat0->col1, &mat1->col1, &mat2->col1, &mat3->col1 );
    vmathSoaV4MakeFrom4Aos( &result->col2, &mat0->col2, &mat1->col2, &mat2->col2, &mat3->col2 );
    vmathSoaV4MakeFrom4Aos( &result->col3, &mat0->col3, &mat1->col3, &mat2->col3, &mat3->col3 );
}

static inline void vmathSoaM4Get4Aos( const VmathSoaMatrix4 *mat, VmathMatrix4 *result0, VmathMatrix4 *result1, VmathMatrix4 *result2, VmathMatrix4 *result3 )
{
    vmathSoaV4Get4Aos( &mat->col0, &result0->col0, &result1->col0, &result2->col0, &result3->col0 );
    vmathSoaV4Get4Aos( &mat->col1, &result0->col1, &result1->col1, &result2->col1, &result3->col1 );
    vmathSoaV4Get4Aos( &mat->col2, &result0->col2, &result1->col2, &result2->col2, &result3->col2 );
    vmathSoaV4Get4Aos( &mat->col3, &result0->col3, &result1->col3, &result2->col3, &result3->col3 );
}

static inline void vmathSoaM4SetCol0( VmathSoaMatrix4 *result, const VmathSoaVector4 *_col0 )
{
    vmathSoaV4Copy( &result->col0, _col0 );
}

static inline void vmathSoaM4SetCol1( VmathSoaMatrix4 *result, const VmathSoaVector4 *_col1 )
{
    vmathSoaV4Copy( &result->col1, _col1 );
}

static inline void vmathSoaM4SetCol2( VmathSoaMatrix4 *result, const VmathSoaVector4 *_col2 )
{
    vmathSoaV4Copy( &result->col2, _col2 );
}

static inline void vmathSoaM4SetCol3( VmathSoaMatrix4 *result, const VmathSoaVector4 *_col3 )
{
    vmathSoaV4Copy( &result->col3, _col3 );
}

static inline void vmathSoaM4SetCol( VmathSoaMatrix4 *result, int col, const VmathSoaVector4 *vec )
{
    vmathSoaV4Copy( (&result->col0 + col), vec );
}

static inline void vmathSoaM4SetRow( VmathSoaMatrix4 *result, int row, const VmathSoaVector4 *vec )
{
    vmathSoaV4SetElem( &result->col0, row, vmathSoaV4GetElem( vec, 0 ) );
    vmathSoaV4SetElem( &result->col1, row, vmathSoaV4GetElem( vec, 1 ) );
    vmathSoaV4SetElem( &result->col2, row, vmathSoaV4GetElem( vec, 2 ) );
    vmathSoaV4SetElem( &result->col3, row, vmathSoaV4GetElem( vec, 3 ) );
}

static inline void vmathSoaM4SetElem( VmathSoaMatrix4 *result, int col, int row, vec_float4 val )
{
    VmathSoaVector4 tmpV3_0;
    vmathSoaM4GetCol( &tmpV3_0, result, col );
    vmathSoaV4SetElem( &tmpV3_0, row, val );
    vmathSoaM4SetCol( result, col, &tmpV3_0 );
}

static inline vec_float4 vmathSoaM4GetElem( const VmathSoaMatrix4 *mat, int col, int row )
{
    VmathSoaVector4 tmpV4_0;
    vmathSoaM4GetCol( &tmpV4_0, mat, col );
    return vmathSoaV4GetElem( &tmpV4_0, row );
}

static inline void vmathSoaM4GetCol0( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4Copy( result, &mat->col0 );
}

static inline void vmathSoaM4GetCol1( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4Copy( result, &mat->col1 );
}

static inline void vmathSoaM4GetCol2( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4Copy( result, &mat->col2 );
}

static inline void vmathSoaM4GetCol3( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4Copy( result, &mat->col3 );
}

static inline void vmathSoaM4GetCol( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, int col )
{
    vmathSoaV4Copy( result, (&mat->col0 + col) );
}

static inline void vmathSoaM4GetRow( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, int row )
{
    vmathSoaV4MakeFromElems( result, vmathSoaV4GetElem( &mat->col0, row ), vmathSoaV4GetElem( &mat->col1, row ), vmathSoaV4GetElem( &mat->col2, row ), vmathSoaV4GetElem( &mat->col3, row ) );
}

static inline void vmathSoaM4Transpose( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat )
{
    VmathSoaMatrix4 tmpResult;
    vmathSoaV4MakeFromElems( &tmpResult.col0, mat->col0.x, mat->col1.x, mat->col2.x, mat->col3.x );
    vmathSoaV4MakeFromElems( &tmpResult.col1, mat->col0.y, mat->col1.y, mat->col2.y, mat->col3.y );
    vmathSoaV4MakeFromElems( &tmpResult.col2, mat->col0.z, mat->col1.z, mat->col2.z, mat->col3.z );
    vmathSoaV4MakeFromElems( &tmpResult.col3, mat->col0.w, mat->col1.w, mat->col2.w, mat->col3.w );
    vmathSoaM4Copy( result, &tmpResult );
}

static inline void vmathSoaM4Inverse( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat )
{
    VmathSoaVector4 res0, res1, res2, res3;
    vec_float4 mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, detInv;
    mA = mat->col0.x;
    mB = mat->col0.y;
    mC = mat->col0.z;
    mD = mat->col0.w;
    mE = mat->col1.x;
    mF = mat->col1.y;
    mG = mat->col1.z;
    mH = mat->col1.w;
    mI = mat->col2.x;
    mJ = mat->col2.y;
    mK = mat->col2.z;
    mL = mat->col2.w;
    mM = mat->col3.x;
    mN = mat->col3.y;
    mO = mat->col3.z;
    mP = mat->col3.w;
    tmp0 = spu_sub( spu_mul( mK, mD ), spu_mul( mC, mL ) );
    tmp1 = spu_sub( spu_mul( mO, mH ), spu_mul( mG, mP ) );
    tmp2 = spu_sub( spu_mul( mB, mK ), spu_mul( mJ, mC ) );
    tmp3 = spu_sub( spu_mul( mF, mO ), spu_mul( mN, mG ) );
    tmp4 = spu_sub( spu_mul( mJ, mD ), spu_mul( mB, mL ) );
    tmp5 = spu_sub( spu_mul( mN, mH ), spu_mul( mF, mP ) );
    vmathSoaV4SetX( &res0, spu_sub( spu_sub( spu_mul( mJ, tmp1 ), spu_mul( mL, tmp3 ) ), spu_mul( mK, tmp5 ) ) );
    vmathSoaV4SetY( &res0, spu_sub( spu_sub( spu_mul( mN, tmp0 ), spu_mul( mP, tmp2 ) ), spu_mul( mO, tmp4 ) ) );
    vmathSoaV4SetZ( &res0, spu_sub( spu_add( spu_mul( mD, tmp3 ), spu_mul( mC, tmp5 ) ), spu_mul( mB, tmp1 ) ) );
    vmathSoaV4SetW( &res0, spu_sub( spu_add( spu_mul( mH, tmp2 ), spu_mul( mG, tmp4 ) ), spu_mul( mF, tmp0 ) ) );
    detInv = recipf4( spu_add( spu_add( spu_add( spu_mul( mA, res0.x ), spu_mul( mE, res0.y ) ), spu_mul( mI, res0.z ) ), spu_mul( mM, res0.w ) ) );
    vmathSoaV4SetX( &res1, spu_mul( mI, tmp1 ) );
    vmathSoaV4SetY( &res1, spu_mul( mM, tmp0 ) );
    vmathSoaV4SetZ( &res1, spu_mul( mA, tmp1 ) );
    vmathSoaV4SetW( &res1, spu_mul( mE, tmp0 ) );
    vmathSoaV4SetX( &res3, spu_mul( mI, tmp3 ) );
    vmathSoaV4SetY( &res3, spu_mul( mM, tmp2 ) );
    vmathSoaV4SetZ( &res3, spu_mul( mA, tmp3 ) );
    vmathSoaV4SetW( &res3, spu_mul( mE, tmp2 ) );
    vmathSoaV4SetX( &res2, spu_mul( mI, tmp5 ) );
    vmathSoaV4SetY( &res2, spu_mul( mM, tmp4 ) );
    vmathSoaV4SetZ( &res2, spu_mul( mA, tmp5 ) );
    vmathSoaV4SetW( &res2, spu_mul( mE, tmp4 ) );
    tmp0 = spu_sub( spu_mul( mI, mB ), spu_mul( mA, mJ ) );
    tmp1 = spu_sub( spu_mul( mM, mF ), spu_mul( mE, mN ) );
    tmp2 = spu_sub( spu_mul( mI, mD ), spu_mul( mA, mL ) );
    tmp3 = spu_sub( spu_mul( mM, mH ), spu_mul( mE, mP ) );
    tmp4 = spu_sub( spu_mul( mI, mC ), spu_mul( mA, mK ) );
    tmp5 = spu_sub( spu_mul( mM, mG ), spu_mul( mE, mO ) );
    vmathSoaV4SetX( &res2, spu_add( spu_sub( spu_mul( mL, tmp1 ), spu_mul( mJ, tmp3 ) ), res2.x ) );
    vmathSoaV4SetY( &res2, spu_add( spu_sub( spu_mul( mP, tmp0 ), spu_mul( mN, tmp2 ) ), res2.y ) );
    vmathSoaV4SetZ( &res2, spu_sub( spu_sub( spu_mul( mB, tmp3 ), spu_mul( mD, tmp1 ) ), res2.z ) );
    vmathSoaV4SetW( &res2, spu_sub( spu_sub( spu_mul( mF, tmp2 ), spu_mul( mH, tmp0 ) ), res2.w ) );
    vmathSoaV4SetX( &res3, spu_add( spu_sub( spu_mul( mJ, tmp5 ), spu_mul( mK, tmp1 ) ), res3.x ) );
    vmathSoaV4SetY( &res3, spu_add( spu_sub( spu_mul( mN, tmp4 ), spu_mul( mO, tmp0 ) ), res3.y ) );
    vmathSoaV4SetZ( &res3, spu_sub( spu_sub( spu_mul( mC, tmp1 ), spu_mul( mB, tmp5 ) ), res3.z ) );
    vmathSoaV4SetW( &res3, spu_sub( spu_sub( spu_mul( mG, tmp0 ), spu_mul( mF, tmp4 ) ), res3.w ) );
    vmathSoaV4SetX( &res1, spu_sub( spu_sub( spu_mul( mK, tmp3 ), spu_mul( mL, tmp5 ) ), res1.x ) );
    vmathSoaV4SetY( &res1, spu_sub( spu_sub( spu_mul( mO, tmp2 ), spu_mul( mP, tmp4 ) ), res1.y ) );
    vmathSoaV4SetZ( &res1, spu_add( spu_sub( spu_mul( mD, tmp5 ), spu_mul( mC, tmp3 ) ), res1.z ) );
    vmathSoaV4SetW( &res1, spu_add( spu_sub( spu_mul( mH, tmp4 ), spu_mul( mG, tmp2 ) ), res1.w ) );
    vmathSoaV4ScalarMul( &result->col0, &res0, detInv );
    vmathSoaV4ScalarMul( &result->col1, &res1, detInv );
    vmathSoaV4ScalarMul( &result->col2, &res2, detInv );
    vmathSoaV4ScalarMul( &result->col3, &res3, detInv );
}

static inline void vmathSoaM4AffineInverse( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat )
{
    VmathSoaTransform3 affineMat, tmpT3_0;
    VmathSoaVector3 tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3;
    vmathSoaV4GetXYZ( &tmpV3_0, &mat->col0 );
    vmathSoaT3SetCol0( &affineMat, &tmpV3_0 );
    vmathSoaV4GetXYZ( &tmpV3_1, &mat->col1 );
    vmathSoaT3SetCol1( &affineMat, &tmpV3_1 );
    vmathSoaV4GetXYZ( &tmpV3_2, &mat->col2 );
    vmathSoaT3SetCol2( &affineMat, &tmpV3_2 );
    vmathSoaV4GetXYZ( &tmpV3_3, &mat->col3 );
    vmathSoaT3SetCol3( &affineMat, &tmpV3_3 );
    vmathSoaT3Inverse( &tmpT3_0, &affineMat );
    vmathSoaM4MakeFromT3( result, &tmpT3_0 );
}

static inline void vmathSoaM4OrthoInverse( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat )
{
    VmathSoaTransform3 affineMat, tmpT3_0;
    VmathSoaVector3 tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3;
    vmathSoaV4GetXYZ( &tmpV3_0, &mat->col0 );
    vmathSoaT3SetCol0( &affineMat, &tmpV3_0 );
    vmathSoaV4GetXYZ( &tmpV3_1, &mat->col1 );
    vmathSoaT3SetCol1( &affineMat, &tmpV3_1 );
    vmathSoaV4GetXYZ( &tmpV3_2, &mat->col2 );
    vmathSoaT3SetCol2( &affineMat, &tmpV3_2 );
    vmathSoaV4GetXYZ( &tmpV3_3, &mat->col3 );
    vmathSoaT3SetCol3( &affineMat, &tmpV3_3 );
    vmathSoaT3OrthoInverse( &tmpT3_0, &affineMat );
    vmathSoaM4MakeFromT3( result, &tmpT3_0 );
}

static inline vec_float4 vmathSoaM4Determinant( const VmathSoaMatrix4 *mat )
{
    vec_float4 dx, dy, dz, dw, mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5;
    mA = mat->col0.x;
    mB = mat->col0.y;
    mC = mat->col0.z;
    mD = mat->col0.w;
    mE = mat->col1.x;
    mF = mat->col1.y;
    mG = mat->col1.z;
    mH = mat->col1.w;
    mI = mat->col2.x;
    mJ = mat->col2.y;
    mK = mat->col2.z;
    mL = mat->col2.w;
    mM = mat->col3.x;
    mN = mat->col3.y;
    mO = mat->col3.z;
    mP = mat->col3.w;
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

static inline void vmathSoaM4Add( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1 )
{
    vmathSoaV4Add( &result->col0, &mat0->col0, &mat1->col0 );
    vmathSoaV4Add( &result->col1, &mat0->col1, &mat1->col1 );
    vmathSoaV4Add( &result->col2, &mat0->col2, &mat1->col2 );
    vmathSoaV4Add( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathSoaM4Sub( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1 )
{
    vmathSoaV4Sub( &result->col0, &mat0->col0, &mat1->col0 );
    vmathSoaV4Sub( &result->col1, &mat0->col1, &mat1->col1 );
    vmathSoaV4Sub( &result->col2, &mat0->col2, &mat1->col2 );
    vmathSoaV4Sub( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathSoaM4Neg( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4Neg( &result->col0, &mat->col0 );
    vmathSoaV4Neg( &result->col1, &mat->col1 );
    vmathSoaV4Neg( &result->col2, &mat->col2 );
    vmathSoaV4Neg( &result->col3, &mat->col3 );
}

static inline void vmathSoaM4AbsPerElem( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4AbsPerElem( &result->col0, &mat->col0 );
    vmathSoaV4AbsPerElem( &result->col1, &mat->col1 );
    vmathSoaV4AbsPerElem( &result->col2, &mat->col2 );
    vmathSoaV4AbsPerElem( &result->col3, &mat->col3 );
}

static inline void vmathSoaM4ScalarMul( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat, vec_float4 scalar )
{
    vmathSoaV4ScalarMul( &result->col0, &mat->col0, scalar );
    vmathSoaV4ScalarMul( &result->col1, &mat->col1, scalar );
    vmathSoaV4ScalarMul( &result->col2, &mat->col2, scalar );
    vmathSoaV4ScalarMul( &result->col3, &mat->col3, scalar );
}

static inline void vmathSoaM4MulV4( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, const VmathSoaVector4 *vec )
{
    vec_float4 tmpX, tmpY, tmpZ, tmpW;
    tmpX = spu_add( spu_add( spu_add( spu_mul( mat->col0.x, vec->x ), spu_mul( mat->col1.x, vec->y ) ), spu_mul( mat->col2.x, vec->z ) ), spu_mul( mat->col3.x, vec->w ) );
    tmpY = spu_add( spu_add( spu_add( spu_mul( mat->col0.y, vec->x ), spu_mul( mat->col1.y, vec->y ) ), spu_mul( mat->col2.y, vec->z ) ), spu_mul( mat->col3.y, vec->w ) );
    tmpZ = spu_add( spu_add( spu_add( spu_mul( mat->col0.z, vec->x ), spu_mul( mat->col1.z, vec->y ) ), spu_mul( mat->col2.z, vec->z ) ), spu_mul( mat->col3.z, vec->w ) );
    tmpW = spu_add( spu_add( spu_add( spu_mul( mat->col0.w, vec->x ), spu_mul( mat->col1.w, vec->y ) ), spu_mul( mat->col2.w, vec->z ) ), spu_mul( mat->col3.w, vec->w ) );
    vmathSoaV4MakeFromElems( result, tmpX, tmpY, tmpZ, tmpW );
}

static inline void vmathSoaM4MulV3( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, const VmathSoaVector3 *vec )
{
    result->x = spu_add( spu_add( spu_mul( mat->col0.x, vec->x ), spu_mul( mat->col1.x, vec->y ) ), spu_mul( mat->col2.x, vec->z ) );
    result->y = spu_add( spu_add( spu_mul( mat->col0.y, vec->x ), spu_mul( mat->col1.y, vec->y ) ), spu_mul( mat->col2.y, vec->z ) );
    result->z = spu_add( spu_add( spu_mul( mat->col0.z, vec->x ), spu_mul( mat->col1.z, vec->y ) ), spu_mul( mat->col2.z, vec->z ) );
    result->w = spu_add( spu_add( spu_mul( mat->col0.w, vec->x ), spu_mul( mat->col1.w, vec->y ) ), spu_mul( mat->col2.w, vec->z ) );
}

static inline void vmathSoaM4MulP3( VmathSoaVector4 *result, const VmathSoaMatrix4 *mat, const VmathSoaPoint3 *pnt )
{
    result->x = spu_add( spu_add( spu_add( spu_mul( mat->col0.x, pnt->x ), spu_mul( mat->col1.x, pnt->y ) ), spu_mul( mat->col2.x, pnt->z ) ), mat->col3.x );
    result->y = spu_add( spu_add( spu_add( spu_mul( mat->col0.y, pnt->x ), spu_mul( mat->col1.y, pnt->y ) ), spu_mul( mat->col2.y, pnt->z ) ), mat->col3.y );
    result->z = spu_add( spu_add( spu_add( spu_mul( mat->col0.z, pnt->x ), spu_mul( mat->col1.z, pnt->y ) ), spu_mul( mat->col2.z, pnt->z ) ), mat->col3.z );
    result->w = spu_add( spu_add( spu_add( spu_mul( mat->col0.w, pnt->x ), spu_mul( mat->col1.w, pnt->y ) ), spu_mul( mat->col2.w, pnt->z ) ), mat->col3.w );
}

static inline void vmathSoaM4Mul( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1 )
{
    VmathSoaMatrix4 tmpResult;
    vmathSoaM4MulV4( &tmpResult.col0, mat0, &mat1->col0 );
    vmathSoaM4MulV4( &tmpResult.col1, mat0, &mat1->col1 );
    vmathSoaM4MulV4( &tmpResult.col2, mat0, &mat1->col2 );
    vmathSoaM4MulV4( &tmpResult.col3, mat0, &mat1->col3 );
    vmathSoaM4Copy( result, &tmpResult );
}

static inline void vmathSoaM4MulT3( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat, const VmathSoaTransform3 *tfrm1 )
{
    VmathSoaMatrix4 tmpResult;
    VmathSoaPoint3 tmpP3_0;
    vmathSoaM4MulV3( &tmpResult.col0, mat, &tfrm1->col0 );
    vmathSoaM4MulV3( &tmpResult.col1, mat, &tfrm1->col1 );
    vmathSoaM4MulV3( &tmpResult.col2, mat, &tfrm1->col2 );
    vmathSoaP3MakeFromV3( &tmpP3_0, &tfrm1->col3 );
    vmathSoaM4MulP3( &tmpResult.col3, mat, &tmpP3_0 );
    vmathSoaM4Copy( result, &tmpResult );
}

static inline void vmathSoaM4MulPerElem( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1 )
{
    vmathSoaV4MulPerElem( &result->col0, &mat0->col0, &mat1->col0 );
    vmathSoaV4MulPerElem( &result->col1, &mat0->col1, &mat1->col1 );
    vmathSoaV4MulPerElem( &result->col2, &mat0->col2, &mat1->col2 );
    vmathSoaV4MulPerElem( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathSoaM4MakeIdentity( VmathSoaMatrix4 *result )
{
    vmathSoaV4MakeXAxis( &result->col0 );
    vmathSoaV4MakeYAxis( &result->col1 );
    vmathSoaV4MakeZAxis( &result->col2 );
    vmathSoaV4MakeWAxis( &result->col3 );
}

static inline void vmathSoaM4SetUpper3x3( VmathSoaMatrix4 *result, const VmathSoaMatrix3 *mat3 )
{
    vmathSoaV4SetXYZ( &result->col0, &mat3->col0 );
    vmathSoaV4SetXYZ( &result->col1, &mat3->col1 );
    vmathSoaV4SetXYZ( &result->col2, &mat3->col2 );
}

static inline void vmathSoaM4GetUpper3x3( VmathSoaMatrix3 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4GetXYZ( &result->col0, &mat->col0 );
    vmathSoaV4GetXYZ( &result->col1, &mat->col1 );
    vmathSoaV4GetXYZ( &result->col2, &mat->col2 );
}

static inline void vmathSoaM4SetTranslation( VmathSoaMatrix4 *result, const VmathSoaVector3 *translateVec )
{
    vmathSoaV4SetXYZ( &result->col3, translateVec );
}

static inline void vmathSoaM4GetTranslation( VmathSoaVector3 *result, const VmathSoaMatrix4 *mat )
{
    vmathSoaV4GetXYZ( result, &mat->col3 );
}

static inline void vmathSoaM4MakeRotationX( VmathSoaMatrix4 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV4MakeXAxis( &result->col0 );
    vmathSoaV4MakeFromElems( &result->col1, spu_splats(0.0f), c, s, spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col2, spu_splats(0.0f), negatef4( s ), c, spu_splats(0.0f) );
    vmathSoaV4MakeWAxis( &result->col3 );
}

static inline void vmathSoaM4MakeRotationY( VmathSoaMatrix4 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV4MakeFromElems( &result->col0, c, spu_splats(0.0f), negatef4( s ), spu_splats(0.0f) );
    vmathSoaV4MakeYAxis( &result->col1 );
    vmathSoaV4MakeFromElems( &result->col2, s, spu_splats(0.0f), c, spu_splats(0.0f) );
    vmathSoaV4MakeWAxis( &result->col3 );
}

static inline void vmathSoaM4MakeRotationZ( VmathSoaMatrix4 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV4MakeFromElems( &result->col0, c, s, spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col1, negatef4( s ), c, spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeZAxis( &result->col2 );
    vmathSoaV4MakeWAxis( &result->col3 );
}

static inline void vmathSoaM4MakeRotationZYX( VmathSoaMatrix4 *result, const VmathSoaVector3 *radiansXYZ )
{
    vec_float4 sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sincosf4( radiansXYZ->x, &sX, &cX );
    sincosf4( radiansXYZ->y, &sY, &cY );
    sincosf4( radiansXYZ->z, &sZ, &cZ );
    tmp0 = spu_mul( cZ, sY );
    tmp1 = spu_mul( sZ, sY );
    vmathSoaV4MakeFromElems( &result->col0, spu_mul( cZ, cY ), spu_mul( sZ, cY ), negatef4( sY ), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col1, spu_sub( spu_mul( tmp0, sX ), spu_mul( sZ, cX ) ), spu_add( spu_mul( tmp1, sX ), spu_mul( cZ, cX ) ), spu_mul( cY, sX ), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col2, spu_add( spu_mul( tmp0, cX ), spu_mul( sZ, sX ) ), spu_sub( spu_mul( tmp1, cX ), spu_mul( cZ, sX ) ), spu_mul( cY, cX ), spu_splats(0.0f) );
    vmathSoaV4MakeWAxis( &result->col3 );
}

static inline void vmathSoaM4MakeRotationAxis( VmathSoaMatrix4 *result, vec_float4 radians, const VmathSoaVector3 *unitVec )
{
    vec_float4 x, y, z, s, c, oneMinusC, xy, yz, zx;
    sincosf4( radians, &s, &c );
    x = unitVec->x;
    y = unitVec->y;
    z = unitVec->z;
    xy = spu_mul( x, y );
    yz = spu_mul( y, z );
    zx = spu_mul( z, x );
    oneMinusC = spu_sub( spu_splats(1.0f), c );
    vmathSoaV4MakeFromElems( &result->col0, spu_add( spu_mul( spu_mul( x, x ), oneMinusC ), c ), spu_add( spu_mul( xy, oneMinusC ), spu_mul( z, s ) ), spu_sub( spu_mul( zx, oneMinusC ), spu_mul( y, s ) ), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col1, spu_sub( spu_mul( xy, oneMinusC ), spu_mul( z, s ) ), spu_add( spu_mul( spu_mul( y, y ), oneMinusC ), c ), spu_add( spu_mul( yz, oneMinusC ), spu_mul( x, s ) ), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col2, spu_add( spu_mul( zx, oneMinusC ), spu_mul( y, s ) ), spu_sub( spu_mul( yz, oneMinusC ), spu_mul( x, s ) ), spu_add( spu_mul( spu_mul( z, z ), oneMinusC ), c ), spu_splats(0.0f) );
    vmathSoaV4MakeWAxis( &result->col3 );
}

static inline void vmathSoaM4MakeRotationQ( VmathSoaMatrix4 *result, const VmathSoaQuat *unitQuat )
{
    VmathSoaTransform3 tmpT3_0;
    vmathSoaT3MakeRotationQ( &tmpT3_0, unitQuat );
    vmathSoaM4MakeFromT3( result, &tmpT3_0 );
}

static inline void vmathSoaM4MakeScale( VmathSoaMatrix4 *result, const VmathSoaVector3 *scaleVec )
{
    vmathSoaV4MakeFromElems( &result->col0, scaleVec->x, spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col1, spu_splats(0.0f), scaleVec->y, spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col2, spu_splats(0.0f), spu_splats(0.0f), scaleVec->z, spu_splats(0.0f) );
    vmathSoaV4MakeWAxis( &result->col3 );
}

static inline void vmathSoaM4AppendScale( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat, const VmathSoaVector3 *scaleVec )
{
    vmathSoaV4ScalarMul( &result->col0, &mat->col0, vmathSoaV3GetX( scaleVec ) );
    vmathSoaV4ScalarMul( &result->col1, &mat->col1, vmathSoaV3GetY( scaleVec ) );
    vmathSoaV4ScalarMul( &result->col2, &mat->col2, vmathSoaV3GetZ( scaleVec ) );
    vmathSoaV4Copy( &result->col3, &mat->col3 );
}

static inline void vmathSoaM4PrependScale( VmathSoaMatrix4 *result, const VmathSoaVector3 *scaleVec, const VmathSoaMatrix4 *mat )
{
    VmathSoaVector4 scale4;
    vmathSoaV4MakeFromV3Scalar( &scale4, scaleVec, spu_splats(1.0f) );
    vmathSoaV4MulPerElem( &result->col0, &mat->col0, &scale4 );
    vmathSoaV4MulPerElem( &result->col1, &mat->col1, &scale4 );
    vmathSoaV4MulPerElem( &result->col2, &mat->col2, &scale4 );
    vmathSoaV4MulPerElem( &result->col3, &mat->col3, &scale4 );
}

static inline void vmathSoaM4MakeTranslation( VmathSoaMatrix4 *result, const VmathSoaVector3 *translateVec )
{
    vmathSoaV4MakeXAxis( &result->col0 );
    vmathSoaV4MakeYAxis( &result->col1 );
    vmathSoaV4MakeZAxis( &result->col2 );
    vmathSoaV4MakeFromV3Scalar( &result->col3, translateVec, spu_splats(1.0f) );
}

static inline void vmathSoaM4MakeLookAt( VmathSoaMatrix4 *result, const VmathSoaPoint3 *eyePos, const VmathSoaPoint3 *lookAtPos, const VmathSoaVector3 *upVec )
{
    VmathSoaMatrix4 m4EyeFrame;
    VmathSoaVector3 v3X, v3Y, v3Z, tmpV3_0, tmpV3_1;
    VmathSoaVector4 tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3;
    vmathSoaV3Normalize( &v3Y, upVec );
    vmathSoaP3Sub( &tmpV3_0, eyePos, lookAtPos );
    vmathSoaV3Normalize( &v3Z, &tmpV3_0 );
    vmathSoaV3Cross( &tmpV3_1, &v3Y, &v3Z );
    vmathSoaV3Normalize( &v3X, &tmpV3_1 );
    vmathSoaV3Cross( &v3Y, &v3Z, &v3X );
    vmathSoaV4MakeFromV3( &tmpV4_0, &v3X );
    vmathSoaV4MakeFromV3( &tmpV4_1, &v3Y );
    vmathSoaV4MakeFromV3( &tmpV4_2, &v3Z );
    vmathSoaV4MakeFromP3( &tmpV4_3, eyePos );
    vmathSoaM4MakeFromCols( &m4EyeFrame, &tmpV4_0, &tmpV4_1, &tmpV4_2, &tmpV4_3 );
    vmathSoaM4OrthoInverse( result, &m4EyeFrame );
}

static inline void vmathSoaM4MakePerspective( VmathSoaMatrix4 *result, vec_float4 fovyRadians, vec_float4 aspect, vec_float4 zNear, vec_float4 zFar )
{
    vec_float4 f, rangeInv;
    f = tanf4( spu_sub( spu_splats( _VECTORMATH_PI_OVER_2 ), spu_mul( spu_splats(0.5f), fovyRadians ) ) );
    rangeInv = recipf4( spu_sub( zNear, zFar ) );
    vmathSoaV4MakeFromElems( &result->col0, divf4( f, aspect ), spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col1, spu_splats(0.0f), f, spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col2, spu_splats(0.0f), spu_splats(0.0f), spu_mul( spu_add( zNear, zFar ), rangeInv ), spu_splats(-1.0f) );
    vmathSoaV4MakeFromElems( &result->col3, spu_splats(0.0f), spu_splats(0.0f), spu_mul( spu_mul( spu_mul( zNear, zFar ), rangeInv ), spu_splats(2.0f) ), spu_splats(0.0f) );
}

static inline void vmathSoaM4MakeFrustum( VmathSoaMatrix4 *result, vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar )
{
    vec_float4 sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf, n2;
    sum_rl = spu_add( right, left );
    sum_tb = spu_add( top, bottom );
    sum_nf = spu_add( zNear, zFar );
    inv_rl = recipf4( spu_sub( right, left ) );
    inv_tb = recipf4( spu_sub( top, bottom ) );
    inv_nf = recipf4( spu_sub( zNear, zFar ) );
    n2 = spu_add( zNear, zNear );
    vmathSoaV4MakeFromElems( &result->col0, spu_mul( n2, inv_rl ), spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col1, spu_splats(0.0f), spu_mul( n2, inv_tb ), spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col2, spu_mul( sum_rl, inv_rl ), spu_mul( sum_tb, inv_tb ), spu_mul( sum_nf, inv_nf ), spu_splats(-1.0f) );
    vmathSoaV4MakeFromElems( &result->col3, spu_splats(0.0f), spu_splats(0.0f), spu_mul( spu_mul( n2, inv_nf ), zFar ), spu_splats(0.0f) );
}

static inline void vmathSoaM4MakeOrthographic( VmathSoaMatrix4 *result, vec_float4 left, vec_float4 right, vec_float4 bottom, vec_float4 top, vec_float4 zNear, vec_float4 zFar )
{
    vec_float4 sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf;
    sum_rl = spu_add( right, left );
    sum_tb = spu_add( top, bottom );
    sum_nf = spu_add( zNear, zFar );
    inv_rl = recipf4( spu_sub( right, left ) );
    inv_tb = recipf4( spu_sub( top, bottom ) );
    inv_nf = recipf4( spu_sub( zNear, zFar ) );
    vmathSoaV4MakeFromElems( &result->col0, spu_add( inv_rl, inv_rl ), spu_splats(0.0f), spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col1, spu_splats(0.0f), spu_add( inv_tb, inv_tb ), spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col2, spu_splats(0.0f), spu_splats(0.0f), spu_add( inv_nf, inv_nf ), spu_splats(0.0f) );
    vmathSoaV4MakeFromElems( &result->col3, spu_mul( negatef4( sum_rl ), inv_rl ), spu_mul( negatef4( sum_tb ), inv_tb ), spu_mul( sum_nf, inv_nf ), spu_splats(1.0f) );
}

static inline void vmathSoaM4Select( VmathSoaMatrix4 *result, const VmathSoaMatrix4 *mat0, const VmathSoaMatrix4 *mat1, vec_uint4 select1 )
{
    vmathSoaV4Select( &result->col0, &mat0->col0, &mat1->col0, select1 );
    vmathSoaV4Select( &result->col1, &mat0->col1, &mat1->col1, select1 );
    vmathSoaV4Select( &result->col2, &mat0->col2, &mat1->col2, select1 );
    vmathSoaV4Select( &result->col3, &mat0->col3, &mat1->col3, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaM4Print( const VmathSoaMatrix4 *mat )
{
    VmathMatrix4 mat0, mat1, mat2, mat3;
    vmathSoaM4Get4Aos( mat, &mat0, &mat1, &mat2, &mat3 );
    printf("slot 0:\n");
    vmathM4Print( &mat0 );
    printf("slot 1:\n");
    vmathM4Print( &mat1 );
    printf("slot 2:\n");
    vmathM4Print( &mat2 );
    printf("slot 3:\n");
    vmathM4Print( &mat3 );
}

static inline void vmathSoaM4Prints( const VmathSoaMatrix4 *mat, const char *name )
{
    printf("%s:\n", name);
    vmathSoaM4Print( mat );
}

#endif

static inline void vmathSoaT3Copy( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm )
{
    vmathSoaV3Copy( &result->col0, &tfrm->col0 );
    vmathSoaV3Copy( &result->col1, &tfrm->col1 );
    vmathSoaV3Copy( &result->col2, &tfrm->col2 );
    vmathSoaV3Copy( &result->col3, &tfrm->col3 );
}

static inline void vmathSoaT3MakeFromScalar( VmathSoaTransform3 *result, vec_float4 scalar )
{
    vmathSoaV3MakeFromScalar( &result->col0, scalar );
    vmathSoaV3MakeFromScalar( &result->col1, scalar );
    vmathSoaV3MakeFromScalar( &result->col2, scalar );
    vmathSoaV3MakeFromScalar( &result->col3, scalar );
}

static inline void vmathSoaT3MakeFromCols( VmathSoaTransform3 *result, const VmathSoaVector3 *_col0, const VmathSoaVector3 *_col1, const VmathSoaVector3 *_col2, const VmathSoaVector3 *_col3 )
{
    vmathSoaV3Copy( &result->col0, _col0 );
    vmathSoaV3Copy( &result->col1, _col1 );
    vmathSoaV3Copy( &result->col2, _col2 );
    vmathSoaV3Copy( &result->col3, _col3 );
}

static inline void vmathSoaT3MakeFromM3V3( VmathSoaTransform3 *result, const VmathSoaMatrix3 *tfrm, const VmathSoaVector3 *translateVec )
{
    vmathSoaT3SetUpper3x3( result, tfrm );
    vmathSoaT3SetTranslation( result, translateVec );
}

static inline void vmathSoaT3MakeFromQV3( VmathSoaTransform3 *result, const VmathSoaQuat *unitQuat, const VmathSoaVector3 *translateVec )
{
    VmathSoaMatrix3 tmpM3_0;
    vmathSoaM3MakeFromQ( &tmpM3_0, unitQuat );
    vmathSoaT3SetUpper3x3( result, &tmpM3_0 );
    vmathSoaT3SetTranslation( result, translateVec );
}

static inline void vmathSoaT3MakeFromAos( VmathSoaTransform3 *result, const VmathTransform3 *tfrm )
{
    vmathSoaV3MakeFromAos( &result->col0, &tfrm->col0 );
    vmathSoaV3MakeFromAos( &result->col1, &tfrm->col1 );
    vmathSoaV3MakeFromAos( &result->col2, &tfrm->col2 );
    vmathSoaV3MakeFromAos( &result->col3, &tfrm->col3 );
}

static inline void vmathSoaT3MakeFrom4Aos( VmathSoaTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1, const VmathTransform3 *tfrm2, const VmathTransform3 *tfrm3 )
{
    vmathSoaV3MakeFrom4Aos( &result->col0, &tfrm0->col0, &tfrm1->col0, &tfrm2->col0, &tfrm3->col0 );
    vmathSoaV3MakeFrom4Aos( &result->col1, &tfrm0->col1, &tfrm1->col1, &tfrm2->col1, &tfrm3->col1 );
    vmathSoaV3MakeFrom4Aos( &result->col2, &tfrm0->col2, &tfrm1->col2, &tfrm2->col2, &tfrm3->col2 );
    vmathSoaV3MakeFrom4Aos( &result->col3, &tfrm0->col3, &tfrm1->col3, &tfrm2->col3, &tfrm3->col3 );
}

static inline void vmathSoaT3Get4Aos( const VmathSoaTransform3 *tfrm, VmathTransform3 *result0, VmathTransform3 *result1, VmathTransform3 *result2, VmathTransform3 *result3 )
{
    vmathSoaV3Get4Aos( &tfrm->col0, &result0->col0, &result1->col0, &result2->col0, &result3->col0 );
    vmathSoaV3Get4Aos( &tfrm->col1, &result0->col1, &result1->col1, &result2->col1, &result3->col1 );
    vmathSoaV3Get4Aos( &tfrm->col2, &result0->col2, &result1->col2, &result2->col2, &result3->col2 );
    vmathSoaV3Get4Aos( &tfrm->col3, &result0->col3, &result1->col3, &result2->col3, &result3->col3 );
}

static inline void vmathSoaT3SetCol0( VmathSoaTransform3 *result, const VmathSoaVector3 *_col0 )
{
    vmathSoaV3Copy( &result->col0, _col0 );
}

static inline void vmathSoaT3SetCol1( VmathSoaTransform3 *result, const VmathSoaVector3 *_col1 )
{
    vmathSoaV3Copy( &result->col1, _col1 );
}

static inline void vmathSoaT3SetCol2( VmathSoaTransform3 *result, const VmathSoaVector3 *_col2 )
{
    vmathSoaV3Copy( &result->col2, _col2 );
}

static inline void vmathSoaT3SetCol3( VmathSoaTransform3 *result, const VmathSoaVector3 *_col3 )
{
    vmathSoaV3Copy( &result->col3, _col3 );
}

static inline void vmathSoaT3SetCol( VmathSoaTransform3 *result, int col, const VmathSoaVector3 *vec )
{
    vmathSoaV3Copy( (&result->col0 + col), vec );
}

static inline void vmathSoaT3SetRow( VmathSoaTransform3 *result, int row, const VmathSoaVector4 *vec )
{
    vmathSoaV3SetElem( &result->col0, row, vmathSoaV4GetElem( vec, 0 ) );
    vmathSoaV3SetElem( &result->col1, row, vmathSoaV4GetElem( vec, 1 ) );
    vmathSoaV3SetElem( &result->col2, row, vmathSoaV4GetElem( vec, 2 ) );
    vmathSoaV3SetElem( &result->col3, row, vmathSoaV4GetElem( vec, 3 ) );
}

static inline void vmathSoaT3SetElem( VmathSoaTransform3 *result, int col, int row, vec_float4 val )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaT3GetCol( &tmpV3_0, result, col );
    vmathSoaV3SetElem( &tmpV3_0, row, val );
    vmathSoaT3SetCol( result, col, &tmpV3_0 );
}

static inline vec_float4 vmathSoaT3GetElem( const VmathSoaTransform3 *tfrm, int col, int row )
{
    VmathSoaVector3 tmpV3_0;
    vmathSoaT3GetCol( &tmpV3_0, tfrm, col );
    return vmathSoaV3GetElem( &tmpV3_0, row );
}

static inline void vmathSoaT3GetCol0( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm )
{
    vmathSoaV3Copy( result, &tfrm->col0 );
}

static inline void vmathSoaT3GetCol1( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm )
{
    vmathSoaV3Copy( result, &tfrm->col1 );
}

static inline void vmathSoaT3GetCol2( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm )
{
    vmathSoaV3Copy( result, &tfrm->col2 );
}

static inline void vmathSoaT3GetCol3( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm )
{
    vmathSoaV3Copy( result, &tfrm->col3 );
}

static inline void vmathSoaT3GetCol( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm, int col )
{
    vmathSoaV3Copy( result, (&tfrm->col0 + col) );
}

static inline void vmathSoaT3GetRow( VmathSoaVector4 *result, const VmathSoaTransform3 *tfrm, int row )
{
    vmathSoaV4MakeFromElems( result, vmathSoaV3GetElem( &tfrm->col0, row ), vmathSoaV3GetElem( &tfrm->col1, row ), vmathSoaV3GetElem( &tfrm->col2, row ), vmathSoaV3GetElem( &tfrm->col3, row ) );
}

static inline void vmathSoaT3Inverse( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm )
{
    VmathSoaVector3 tmp0, tmp1, tmp2, inv0, inv1, inv2, tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3, tmpV3_4, tmpV3_5;
    vec_float4 detinv;
    vmathSoaV3Cross( &tmp0, &tfrm->col1, &tfrm->col2 );
    vmathSoaV3Cross( &tmp1, &tfrm->col2, &tfrm->col0 );
    vmathSoaV3Cross( &tmp2, &tfrm->col0, &tfrm->col1 );
    detinv = recipf4( vmathSoaV3Dot( &tfrm->col2, &tmp2 ) );
    vmathSoaV3MakeFromElems( &inv0, spu_mul( tmp0.x, detinv ), spu_mul( tmp1.x, detinv ), spu_mul( tmp2.x, detinv ) );
    vmathSoaV3MakeFromElems( &inv1, spu_mul( tmp0.y, detinv ), spu_mul( tmp1.y, detinv ), spu_mul( tmp2.y, detinv ) );
    vmathSoaV3MakeFromElems( &inv2, spu_mul( tmp0.z, detinv ), spu_mul( tmp1.z, detinv ), spu_mul( tmp2.z, detinv ) );
    vmathSoaV3Copy( &result->col0, &inv0 );
    vmathSoaV3Copy( &result->col1, &inv1 );
    vmathSoaV3Copy( &result->col2, &inv2 );
    vmathSoaV3ScalarMul( &tmpV3_0, &inv0, tfrm->col3.x );
    vmathSoaV3ScalarMul( &tmpV3_1, &inv1, tfrm->col3.y );
    vmathSoaV3ScalarMul( &tmpV3_2, &inv2, tfrm->col3.z );
    vmathSoaV3Add( &tmpV3_3, &tmpV3_1, &tmpV3_2 );
    vmathSoaV3Add( &tmpV3_4, &tmpV3_0, &tmpV3_3 );
    vmathSoaV3Neg( &tmpV3_5, &tmpV3_4 );
    vmathSoaV3Copy( &result->col3, &tmpV3_5 );
}

static inline void vmathSoaT3OrthoInverse( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm )
{
    VmathSoaVector3 inv0, inv1, inv2, tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3, tmpV3_4, tmpV3_5;
    vmathSoaV3MakeFromElems( &inv0, tfrm->col0.x, tfrm->col1.x, tfrm->col2.x );
    vmathSoaV3MakeFromElems( &inv1, tfrm->col0.y, tfrm->col1.y, tfrm->col2.y );
    vmathSoaV3MakeFromElems( &inv2, tfrm->col0.z, tfrm->col1.z, tfrm->col2.z );
    vmathSoaV3Copy( &result->col0, &inv0 );
    vmathSoaV3Copy( &result->col1, &inv1 );
    vmathSoaV3Copy( &result->col2, &inv2 );
    vmathSoaV3ScalarMul( &tmpV3_0, &inv0, tfrm->col3.x );
    vmathSoaV3ScalarMul( &tmpV3_1, &inv1, tfrm->col3.y );
    vmathSoaV3ScalarMul( &tmpV3_2, &inv2, tfrm->col3.z );
    vmathSoaV3Add( &tmpV3_3, &tmpV3_1, &tmpV3_2 );
    vmathSoaV3Add( &tmpV3_4, &tmpV3_0, &tmpV3_3 );
    vmathSoaV3Neg( &tmpV3_5, &tmpV3_4 );
    vmathSoaV3Copy( &result->col3, &tmpV3_5 );
}

static inline void vmathSoaT3AbsPerElem( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm )
{
    vmathSoaV3AbsPerElem( &result->col0, &tfrm->col0 );
    vmathSoaV3AbsPerElem( &result->col1, &tfrm->col1 );
    vmathSoaV3AbsPerElem( &result->col2, &tfrm->col2 );
    vmathSoaV3AbsPerElem( &result->col3, &tfrm->col3 );
}

static inline void vmathSoaT3MulV3( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm, const VmathSoaVector3 *vec )
{
    vec_float4 tmpX, tmpY, tmpZ;
    tmpX = spu_add( spu_add( spu_mul( tfrm->col0.x, vec->x ), spu_mul( tfrm->col1.x, vec->y ) ), spu_mul( tfrm->col2.x, vec->z ) );
    tmpY = spu_add( spu_add( spu_mul( tfrm->col0.y, vec->x ), spu_mul( tfrm->col1.y, vec->y ) ), spu_mul( tfrm->col2.y, vec->z ) );
    tmpZ = spu_add( spu_add( spu_mul( tfrm->col0.z, vec->x ), spu_mul( tfrm->col1.z, vec->y ) ), spu_mul( tfrm->col2.z, vec->z ) );
    vmathSoaV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathSoaT3MulP3( VmathSoaPoint3 *result, const VmathSoaTransform3 *tfrm, const VmathSoaPoint3 *pnt )
{
    vec_float4 tmpX, tmpY, tmpZ;
    tmpX = spu_add( spu_add( spu_add( spu_mul( tfrm->col0.x, pnt->x ), spu_mul( tfrm->col1.x, pnt->y ) ), spu_mul( tfrm->col2.x, pnt->z ) ), tfrm->col3.x );
    tmpY = spu_add( spu_add( spu_add( spu_mul( tfrm->col0.y, pnt->x ), spu_mul( tfrm->col1.y, pnt->y ) ), spu_mul( tfrm->col2.y, pnt->z ) ), tfrm->col3.y );
    tmpZ = spu_add( spu_add( spu_add( spu_mul( tfrm->col0.z, pnt->x ), spu_mul( tfrm->col1.z, pnt->y ) ), spu_mul( tfrm->col2.z, pnt->z ) ), tfrm->col3.z );
    vmathSoaP3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathSoaT3Mul( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm0, const VmathSoaTransform3 *tfrm1 )
{
    VmathSoaTransform3 tmpResult;
    VmathSoaPoint3 tmpP3_0, tmpP3_1;
    vmathSoaT3MulV3( &tmpResult.col0, tfrm0, &tfrm1->col0 );
    vmathSoaT3MulV3( &tmpResult.col1, tfrm0, &tfrm1->col1 );
    vmathSoaT3MulV3( &tmpResult.col2, tfrm0, &tfrm1->col2 );
    vmathSoaP3MakeFromV3( &tmpP3_0, &tfrm1->col3 );
    vmathSoaT3MulP3( &tmpP3_1, tfrm0, &tmpP3_0 );
    vmathSoaV3MakeFromP3( &tmpResult.col3, &tmpP3_1 );
    vmathSoaT3Copy( result, &tmpResult );
}

static inline void vmathSoaT3MulPerElem( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm0, const VmathSoaTransform3 *tfrm1 )
{
    vmathSoaV3MulPerElem( &result->col0, &tfrm0->col0, &tfrm1->col0 );
    vmathSoaV3MulPerElem( &result->col1, &tfrm0->col1, &tfrm1->col1 );
    vmathSoaV3MulPerElem( &result->col2, &tfrm0->col2, &tfrm1->col2 );
    vmathSoaV3MulPerElem( &result->col3, &tfrm0->col3, &tfrm1->col3 );
}

static inline void vmathSoaT3MakeIdentity( VmathSoaTransform3 *result )
{
    vmathSoaV3MakeXAxis( &result->col0 );
    vmathSoaV3MakeYAxis( &result->col1 );
    vmathSoaV3MakeZAxis( &result->col2 );
    vmathSoaV3MakeFromScalar( &result->col3, spu_splats(0.0f) );
}

static inline void vmathSoaT3SetUpper3x3( VmathSoaTransform3 *result, const VmathSoaMatrix3 *tfrm )
{
    vmathSoaV3Copy( &result->col0, &tfrm->col0 );
    vmathSoaV3Copy( &result->col1, &tfrm->col1 );
    vmathSoaV3Copy( &result->col2, &tfrm->col2 );
}

static inline void vmathSoaT3GetUpper3x3( VmathSoaMatrix3 *result, const VmathSoaTransform3 *tfrm )
{
    vmathSoaM3MakeFromCols( result, &tfrm->col0, &tfrm->col1, &tfrm->col2 );
}

static inline void vmathSoaT3SetTranslation( VmathSoaTransform3 *result, const VmathSoaVector3 *translateVec )
{
    vmathSoaV3Copy( &result->col3, translateVec );
}

static inline void vmathSoaT3GetTranslation( VmathSoaVector3 *result, const VmathSoaTransform3 *tfrm )
{
    vmathSoaV3Copy( result, &tfrm->col3 );
}

static inline void vmathSoaT3MakeRotationX( VmathSoaTransform3 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV3MakeXAxis( &result->col0 );
    vmathSoaV3MakeFromElems( &result->col1, spu_splats(0.0f), c, s );
    vmathSoaV3MakeFromElems( &result->col2, spu_splats(0.0f), negatef4( s ), c );
    vmathSoaV3MakeFromScalar( &result->col3, spu_splats(0.0f) );
}

static inline void vmathSoaT3MakeRotationY( VmathSoaTransform3 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV3MakeFromElems( &result->col0, c, spu_splats(0.0f), negatef4( s ) );
    vmathSoaV3MakeYAxis( &result->col1 );
    vmathSoaV3MakeFromElems( &result->col2, s, spu_splats(0.0f), c );
    vmathSoaV3MakeFromScalar( &result->col3, spu_splats(0.0f) );
}

static inline void vmathSoaT3MakeRotationZ( VmathSoaTransform3 *result, vec_float4 radians )
{
    vec_float4 s, c;
    sincosf4( radians, &s, &c );
    vmathSoaV3MakeFromElems( &result->col0, c, s, spu_splats(0.0f) );
    vmathSoaV3MakeFromElems( &result->col1, negatef4( s ), c, spu_splats(0.0f) );
    vmathSoaV3MakeZAxis( &result->col2 );
    vmathSoaV3MakeFromScalar( &result->col3, spu_splats(0.0f) );
}

static inline void vmathSoaT3MakeRotationZYX( VmathSoaTransform3 *result, const VmathSoaVector3 *radiansXYZ )
{
    vec_float4 sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sincosf4( radiansXYZ->x, &sX, &cX );
    sincosf4( radiansXYZ->y, &sY, &cY );
    sincosf4( radiansXYZ->z, &sZ, &cZ );
    tmp0 = spu_mul( cZ, sY );
    tmp1 = spu_mul( sZ, sY );
    vmathSoaV3MakeFromElems( &result->col0, spu_mul( cZ, cY ), spu_mul( sZ, cY ), negatef4( sY ) );
    vmathSoaV3MakeFromElems( &result->col1, spu_sub( spu_mul( tmp0, sX ), spu_mul( sZ, cX ) ), spu_add( spu_mul( tmp1, sX ), spu_mul( cZ, cX ) ), spu_mul( cY, sX ) );
    vmathSoaV3MakeFromElems( &result->col2, spu_add( spu_mul( tmp0, cX ), spu_mul( sZ, sX ) ), spu_sub( spu_mul( tmp1, cX ), spu_mul( cZ, sX ) ), spu_mul( cY, cX ) );
    vmathSoaV3MakeFromScalar( &result->col3, spu_splats(0.0f) );
}

static inline void vmathSoaT3MakeRotationAxis( VmathSoaTransform3 *result, vec_float4 radians, const VmathSoaVector3 *unitVec )
{
    VmathSoaMatrix3 tmpM3_0;
    VmathSoaVector3 tmpV3_0;
    vmathSoaM3MakeRotationAxis( &tmpM3_0, radians, unitVec );
    vmathSoaV3MakeFromScalar( &tmpV3_0, spu_splats(0.0f) );
    vmathSoaT3MakeFromM3V3( result, &tmpM3_0, &tmpV3_0 );
}

static inline void vmathSoaT3MakeRotationQ( VmathSoaTransform3 *result, const VmathSoaQuat *unitQuat )
{
    VmathSoaMatrix3 tmpM3_0;
    VmathSoaVector3 tmpV3_0;
    vmathSoaM3MakeFromQ( &tmpM3_0, unitQuat );
    vmathSoaV3MakeFromScalar( &tmpV3_0, spu_splats(0.0f) );
    vmathSoaT3MakeFromM3V3( result, &tmpM3_0, &tmpV3_0 );
}

static inline void vmathSoaT3MakeScale( VmathSoaTransform3 *result, const VmathSoaVector3 *scaleVec )
{
    vmathSoaV3MakeFromElems( &result->col0, scaleVec->x, spu_splats(0.0f), spu_splats(0.0f) );
    vmathSoaV3MakeFromElems( &result->col1, spu_splats(0.0f), scaleVec->y, spu_splats(0.0f) );
    vmathSoaV3MakeFromElems( &result->col2, spu_splats(0.0f), spu_splats(0.0f), scaleVec->z );
    vmathSoaV3MakeFromScalar( &result->col3, spu_splats(0.0f) );
}

static inline void vmathSoaT3AppendScale( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm, const VmathSoaVector3 *scaleVec )
{
    vmathSoaV3ScalarMul( &result->col0, &tfrm->col0, vmathSoaV3GetX( scaleVec ) );
    vmathSoaV3ScalarMul( &result->col1, &tfrm->col1, vmathSoaV3GetY( scaleVec ) );
    vmathSoaV3ScalarMul( &result->col2, &tfrm->col2, vmathSoaV3GetZ( scaleVec ) );
    vmathSoaV3Copy( &result->col3, &tfrm->col3 );
}

static inline void vmathSoaT3PrependScale( VmathSoaTransform3 *result, const VmathSoaVector3 *scaleVec, const VmathSoaTransform3 *tfrm )
{
    vmathSoaV3MulPerElem( &result->col0, &tfrm->col0, scaleVec );
    vmathSoaV3MulPerElem( &result->col1, &tfrm->col1, scaleVec );
    vmathSoaV3MulPerElem( &result->col2, &tfrm->col2, scaleVec );
    vmathSoaV3MulPerElem( &result->col3, &tfrm->col3, scaleVec );
}

static inline void vmathSoaT3MakeTranslation( VmathSoaTransform3 *result, const VmathSoaVector3 *translateVec )
{
    vmathSoaV3MakeXAxis( &result->col0 );
    vmathSoaV3MakeYAxis( &result->col1 );
    vmathSoaV3MakeZAxis( &result->col2 );
    vmathSoaV3Copy( &result->col3, translateVec );
}

static inline void vmathSoaT3Select( VmathSoaTransform3 *result, const VmathSoaTransform3 *tfrm0, const VmathSoaTransform3 *tfrm1, vec_uint4 select1 )
{
    vmathSoaV3Select( &result->col0, &tfrm0->col0, &tfrm1->col0, select1 );
    vmathSoaV3Select( &result->col1, &tfrm0->col1, &tfrm1->col1, select1 );
    vmathSoaV3Select( &result->col2, &tfrm0->col2, &tfrm1->col2, select1 );
    vmathSoaV3Select( &result->col3, &tfrm0->col3, &tfrm1->col3, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaT3Print( const VmathSoaTransform3 *tfrm )
{
    VmathTransform3 mat0, mat1, mat2, mat3;
    vmathSoaT3Get4Aos( tfrm, &mat0, &mat1, &mat2, &mat3 );
    printf("slot 0:\n");
    vmathT3Print( &mat0 );
    printf("slot 1:\n");
    vmathT3Print( &mat1 );
    printf("slot 2:\n");
    vmathT3Print( &mat2 );
    printf("slot 3:\n");
    vmathT3Print( &mat3 );
}

static inline void vmathSoaT3Prints( const VmathSoaTransform3 *tfrm, const char *name )
{
    printf("%s:\n", name);
    vmathSoaT3Print( tfrm );
}

#endif

static inline void vmathSoaQMakeFromM3( VmathSoaQuat *result, const VmathSoaMatrix3 *tfrm )
{
    vec_float4 trace, radicand, scale, xx, yx, zx, xy, yy, zy, xz, yz, zz, tmpx, tmpy, tmpz, tmpw, qx, qy, qz, qw;
    vec_uint4 negTrace, ZgtX, ZgtY, YgtX;
    vec_uint4 largestXorY, largestYorZ, largestZorX;

    xx = tfrm->col0.x;
    yx = tfrm->col0.y;
    zx = tfrm->col0.z;
    xy = tfrm->col1.x;
    yy = tfrm->col1.y;
    zy = tfrm->col1.z;
    xz = tfrm->col2.x;
    yz = tfrm->col2.y;
    zz = tfrm->col2.z;

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

    result->x = qx;
    result->y = qy;
    result->z = qz;
    result->w = qw;
}

static inline void vmathSoaV3Outer( VmathSoaMatrix3 *result, const VmathSoaVector3 *tfrm0, const VmathSoaVector3 *tfrm1 )
{
    vmathSoaV3ScalarMul( &result->col0, tfrm0, vmathSoaV3GetX( tfrm1 ) );
    vmathSoaV3ScalarMul( &result->col1, tfrm0, vmathSoaV3GetY( tfrm1 ) );
    vmathSoaV3ScalarMul( &result->col2, tfrm0, vmathSoaV3GetZ( tfrm1 ) );
}

static inline void vmathSoaV4Outer( VmathSoaMatrix4 *result, const VmathSoaVector4 *tfrm0, const VmathSoaVector4 *tfrm1 )
{
    vmathSoaV4ScalarMul( &result->col0, tfrm0, vmathSoaV4GetX( tfrm1 ) );
    vmathSoaV4ScalarMul( &result->col1, tfrm0, vmathSoaV4GetY( tfrm1 ) );
    vmathSoaV4ScalarMul( &result->col2, tfrm0, vmathSoaV4GetZ( tfrm1 ) );
    vmathSoaV4ScalarMul( &result->col3, tfrm0, vmathSoaV4GetW( tfrm1 ) );
}

static inline void vmathSoaV3RowMul( VmathSoaVector3 *result, const VmathSoaVector3 *vec, const VmathSoaMatrix3 *mat )
{
    vec_float4 tmpX, tmpY, tmpZ;
    tmpX = spu_add( spu_add( spu_mul( vec->x, mat->col0.x ), spu_mul( vec->y, mat->col0.y ) ), spu_mul( vec->z, mat->col0.z ) );
    tmpY = spu_add( spu_add( spu_mul( vec->x, mat->col1.x ), spu_mul( vec->y, mat->col1.y ) ), spu_mul( vec->z, mat->col1.z ) );
    tmpZ = spu_add( spu_add( spu_mul( vec->x, mat->col2.x ), spu_mul( vec->y, mat->col2.y ) ), spu_mul( vec->z, mat->col2.z ) );
    vmathSoaV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathSoaV3CrossMatrix( VmathSoaMatrix3 *result, const VmathSoaVector3 *vec )
{
    vmathSoaV3MakeFromElems( &result->col0, spu_splats(0.0f), vec->z, negatef4( vec->y ) );
    vmathSoaV3MakeFromElems( &result->col1, negatef4( vec->z ), spu_splats(0.0f), vec->x );
    vmathSoaV3MakeFromElems( &result->col2, vec->y, negatef4( vec->x ), spu_splats(0.0f) );
}

static inline void vmathSoaV3CrossMatrixMul( VmathSoaMatrix3 *result, const VmathSoaVector3 *vec, const VmathSoaMatrix3 *mat )
{
    VmathSoaVector3 tmpV3_0, tmpV3_1, tmpV3_2;
    vmathSoaV3Cross( &tmpV3_0, vec, &mat->col0 );
    vmathSoaV3Cross( &tmpV3_1, vec, &mat->col1 );
    vmathSoaV3Cross( &tmpV3_2, vec, &mat->col2 );
    vmathSoaM3MakeFromCols( result, &tmpV3_0, &tmpV3_1, &tmpV3_2 );
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
