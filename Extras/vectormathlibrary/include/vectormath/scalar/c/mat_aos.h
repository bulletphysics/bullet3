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

#ifndef _VECTORMATH_MAT_AOS_C_H
#define _VECTORMATH_MAT_AOS_C_H

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
static inline void vmathM3Copy( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    vmathV3Copy( &result->col0, &mat->col0 );
    vmathV3Copy( &result->col1, &mat->col1 );
    vmathV3Copy( &result->col2, &mat->col2 );
}

static inline void vmathM3MakeFromScalar( VmathMatrix3 *result, float scalar )
{
    vmathV3MakeFromScalar( &result->col0, scalar );
    vmathV3MakeFromScalar( &result->col1, scalar );
    vmathV3MakeFromScalar( &result->col2, scalar );
}

static inline void vmathM3MakeFromQ( VmathMatrix3 *result, const VmathQuat *unitQuat )
{
    float qx, qy, qz, qw, qx2, qy2, qz2, qxqx2, qyqy2, qzqz2, qxqy2, qyqz2, qzqw2, qxqz2, qyqw2, qxqw2;
    qx = unitQuat->x;
    qy = unitQuat->y;
    qz = unitQuat->z;
    qw = unitQuat->w;
    qx2 = ( qx + qx );
    qy2 = ( qy + qy );
    qz2 = ( qz + qz );
    qxqx2 = ( qx * qx2 );
    qxqy2 = ( qx * qy2 );
    qxqz2 = ( qx * qz2 );
    qxqw2 = ( qw * qx2 );
    qyqy2 = ( qy * qy2 );
    qyqz2 = ( qy * qz2 );
    qyqw2 = ( qw * qy2 );
    qzqz2 = ( qz * qz2 );
    qzqw2 = ( qw * qz2 );
    vmathV3MakeFromElems( &result->col0, ( ( 1.0f - qyqy2 ) - qzqz2 ), ( qxqy2 + qzqw2 ), ( qxqz2 - qyqw2 ) );
    vmathV3MakeFromElems( &result->col1, ( qxqy2 - qzqw2 ), ( ( 1.0f - qxqx2 ) - qzqz2 ), ( qyqz2 + qxqw2 ) );
    vmathV3MakeFromElems( &result->col2, ( qxqz2 + qyqw2 ), ( qyqz2 - qxqw2 ), ( ( 1.0f - qxqx2 ) - qyqy2 ) );
}

static inline void vmathM3MakeFromCols( VmathMatrix3 *result, const VmathVector3 *_col0, const VmathVector3 *_col1, const VmathVector3 *_col2 )
{
    vmathV3Copy( &result->col0, _col0 );
    vmathV3Copy( &result->col1, _col1 );
    vmathV3Copy( &result->col2, _col2 );
}

static inline void vmathM3SetCol0( VmathMatrix3 *result, const VmathVector3 *_col0 )
{
    vmathV3Copy( &result->col0, _col0 );
}

static inline void vmathM3SetCol1( VmathMatrix3 *result, const VmathVector3 *_col1 )
{
    vmathV3Copy( &result->col1, _col1 );
}

static inline void vmathM3SetCol2( VmathMatrix3 *result, const VmathVector3 *_col2 )
{
    vmathV3Copy( &result->col2, _col2 );
}

static inline void vmathM3SetCol( VmathMatrix3 *result, int col, const VmathVector3 *vec )
{
    vmathV3Copy( (&result->col0 + col), vec );
}

static inline void vmathM3SetRow( VmathMatrix3 *result, int row, const VmathVector3 *vec )
{
    vmathV3SetElem( &result->col0, row, vmathV3GetElem( vec, 0 ) );
    vmathV3SetElem( &result->col1, row, vmathV3GetElem( vec, 1 ) );
    vmathV3SetElem( &result->col2, row, vmathV3GetElem( vec, 2 ) );
}

static inline void vmathM3SetElem( VmathMatrix3 *result, int col, int row, float val )
{
    VmathVector3 tmpV3_0;
    vmathM3GetCol( &tmpV3_0, result, col );
    vmathV3SetElem( &tmpV3_0, row, val );
    vmathM3SetCol( result, col, &tmpV3_0 );
}

static inline float vmathM3GetElem( const VmathMatrix3 *mat, int col, int row )
{
    VmathVector3 tmpV3_0;
    vmathM3GetCol( &tmpV3_0, mat, col );
    return vmathV3GetElem( &tmpV3_0, row );
}

static inline void vmathM3GetCol0( VmathVector3 *result, const VmathMatrix3 *mat )
{
    vmathV3Copy( result, &mat->col0 );
}

static inline void vmathM3GetCol1( VmathVector3 *result, const VmathMatrix3 *mat )
{
    vmathV3Copy( result, &mat->col1 );
}

static inline void vmathM3GetCol2( VmathVector3 *result, const VmathMatrix3 *mat )
{
    vmathV3Copy( result, &mat->col2 );
}

static inline void vmathM3GetCol( VmathVector3 *result, const VmathMatrix3 *mat, int col )
{
    vmathV3Copy( result, (&mat->col0 + col) );
}

static inline void vmathM3GetRow( VmathVector3 *result, const VmathMatrix3 *mat, int row )
{
    vmathV3MakeFromElems( result, vmathV3GetElem( &mat->col0, row ), vmathV3GetElem( &mat->col1, row ), vmathV3GetElem( &mat->col2, row ) );
}

static inline void vmathM3Transpose( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    VmathMatrix3 tmpResult;
    vmathV3MakeFromElems( &tmpResult.col0, mat->col0.x, mat->col1.x, mat->col2.x );
    vmathV3MakeFromElems( &tmpResult.col1, mat->col0.y, mat->col1.y, mat->col2.y );
    vmathV3MakeFromElems( &tmpResult.col2, mat->col0.z, mat->col1.z, mat->col2.z );
    vmathM3Copy( result, &tmpResult );
}

static inline void vmathM3Inverse( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    VmathVector3 tmp0, tmp1, tmp2;
    float detinv;
    vmathV3Cross( &tmp0, &mat->col1, &mat->col2 );
    vmathV3Cross( &tmp1, &mat->col2, &mat->col0 );
    vmathV3Cross( &tmp2, &mat->col0, &mat->col1 );
    detinv = ( 1.0f / vmathV3Dot( &mat->col2, &tmp2 ) );
    vmathV3MakeFromElems( &result->col0, ( tmp0.x * detinv ), ( tmp1.x * detinv ), ( tmp2.x * detinv ) );
    vmathV3MakeFromElems( &result->col1, ( tmp0.y * detinv ), ( tmp1.y * detinv ), ( tmp2.y * detinv ) );
    vmathV3MakeFromElems( &result->col2, ( tmp0.z * detinv ), ( tmp1.z * detinv ), ( tmp2.z * detinv ) );
}

static inline float vmathM3Determinant( const VmathMatrix3 *mat )
{
    VmathVector3 tmpV3_0;
    vmathV3Cross( &tmpV3_0, &mat->col0, &mat->col1 );
    return vmathV3Dot( &mat->col2, &tmpV3_0 );
}

static inline void vmathM3Add( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 )
{
    vmathV3Add( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV3Add( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV3Add( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathM3Sub( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 )
{
    vmathV3Sub( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV3Sub( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV3Sub( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathM3Neg( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    vmathV3Neg( &result->col0, &mat->col0 );
    vmathV3Neg( &result->col1, &mat->col1 );
    vmathV3Neg( &result->col2, &mat->col2 );
}

static inline void vmathM3AbsPerElem( VmathMatrix3 *result, const VmathMatrix3 *mat )
{
    vmathV3AbsPerElem( &result->col0, &mat->col0 );
    vmathV3AbsPerElem( &result->col1, &mat->col1 );
    vmathV3AbsPerElem( &result->col2, &mat->col2 );
}

static inline void vmathM3ScalarMul( VmathMatrix3 *result, const VmathMatrix3 *mat, float scalar )
{
    vmathV3ScalarMul( &result->col0, &mat->col0, scalar );
    vmathV3ScalarMul( &result->col1, &mat->col1, scalar );
    vmathV3ScalarMul( &result->col2, &mat->col2, scalar );
}

static inline void vmathM3MulV3( VmathVector3 *result, const VmathMatrix3 *mat, const VmathVector3 *vec )
{
    float tmpX, tmpY, tmpZ;
    tmpX = ( ( ( mat->col0.x * vec->x ) + ( mat->col1.x * vec->y ) ) + ( mat->col2.x * vec->z ) );
    tmpY = ( ( ( mat->col0.y * vec->x ) + ( mat->col1.y * vec->y ) ) + ( mat->col2.y * vec->z ) );
    tmpZ = ( ( ( mat->col0.z * vec->x ) + ( mat->col1.z * vec->y ) ) + ( mat->col2.z * vec->z ) );
    vmathV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathM3Mul( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 )
{
    VmathMatrix3 tmpResult;
    vmathM3MulV3( &tmpResult.col0, mat0, &mat1->col0 );
    vmathM3MulV3( &tmpResult.col1, mat0, &mat1->col1 );
    vmathM3MulV3( &tmpResult.col2, mat0, &mat1->col2 );
    vmathM3Copy( result, &tmpResult );
}

static inline void vmathM3MulPerElem( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1 )
{
    vmathV3MulPerElem( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV3MulPerElem( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV3MulPerElem( &result->col2, &mat0->col2, &mat1->col2 );
}

static inline void vmathM3MakeIdentity( VmathMatrix3 *result )
{
    vmathV3MakeXAxis( &result->col0 );
    vmathV3MakeYAxis( &result->col1 );
    vmathV3MakeZAxis( &result->col2 );
}

static inline void vmathM3MakeRotationX( VmathMatrix3 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV3MakeXAxis( &result->col0 );
    vmathV3MakeFromElems( &result->col1, 0.0f, c, s );
    vmathV3MakeFromElems( &result->col2, 0.0f, -s, c );
}

static inline void vmathM3MakeRotationY( VmathMatrix3 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV3MakeFromElems( &result->col0, c, 0.0f, -s );
    vmathV3MakeYAxis( &result->col1 );
    vmathV3MakeFromElems( &result->col2, s, 0.0f, c );
}

static inline void vmathM3MakeRotationZ( VmathMatrix3 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV3MakeFromElems( &result->col0, c, s, 0.0f );
    vmathV3MakeFromElems( &result->col1, -s, c, 0.0f );
    vmathV3MakeZAxis( &result->col2 );
}

static inline void vmathM3MakeRotationZYX( VmathMatrix3 *result, const VmathVector3 *radiansXYZ )
{
    float sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sX = sinf( radiansXYZ->x );
    cX = cosf( radiansXYZ->x );
    sY = sinf( radiansXYZ->y );
    cY = cosf( radiansXYZ->y );
    sZ = sinf( radiansXYZ->z );
    cZ = cosf( radiansXYZ->z );
    tmp0 = ( cZ * sY );
    tmp1 = ( sZ * sY );
    vmathV3MakeFromElems( &result->col0, ( cZ * cY ), ( sZ * cY ), -sY );
    vmathV3MakeFromElems( &result->col1, ( ( tmp0 * sX ) - ( sZ * cX ) ), ( ( tmp1 * sX ) + ( cZ * cX ) ), ( cY * sX ) );
    vmathV3MakeFromElems( &result->col2, ( ( tmp0 * cX ) + ( sZ * sX ) ), ( ( tmp1 * cX ) - ( cZ * sX ) ), ( cY * cX ) );
}

static inline void vmathM3MakeRotationAxis( VmathMatrix3 *result, float radians, const VmathVector3 *unitVec )
{
    float x, y, z, s, c, oneMinusC, xy, yz, zx;
    s = sinf( radians );
    c = cosf( radians );
    x = unitVec->x;
    y = unitVec->y;
    z = unitVec->z;
    xy = ( x * y );
    yz = ( y * z );
    zx = ( z * x );
    oneMinusC = ( 1.0f - c );
    vmathV3MakeFromElems( &result->col0, ( ( ( x * x ) * oneMinusC ) + c ), ( ( xy * oneMinusC ) + ( z * s ) ), ( ( zx * oneMinusC ) - ( y * s ) ) );
    vmathV3MakeFromElems( &result->col1, ( ( xy * oneMinusC ) - ( z * s ) ), ( ( ( y * y ) * oneMinusC ) + c ), ( ( yz * oneMinusC ) + ( x * s ) ) );
    vmathV3MakeFromElems( &result->col2, ( ( zx * oneMinusC ) + ( y * s ) ), ( ( yz * oneMinusC ) - ( x * s ) ), ( ( ( z * z ) * oneMinusC ) + c ) );
}

static inline void vmathM3MakeRotationQ( VmathMatrix3 *result, const VmathQuat *unitQuat )
{
    vmathM3MakeFromQ( result, unitQuat );
}

static inline void vmathM3MakeScale( VmathMatrix3 *result, const VmathVector3 *scaleVec )
{
    vmathV3MakeFromElems( &result->col0, scaleVec->x, 0.0f, 0.0f );
    vmathV3MakeFromElems( &result->col1, 0.0f, scaleVec->y, 0.0f );
    vmathV3MakeFromElems( &result->col2, 0.0f, 0.0f, scaleVec->z );
}

static inline void vmathM3AppendScale( VmathMatrix3 *result, const VmathMatrix3 *mat, const VmathVector3 *scaleVec )
{
    vmathV3ScalarMul( &result->col0, &mat->col0, vmathV3GetX( scaleVec ) );
    vmathV3ScalarMul( &result->col1, &mat->col1, vmathV3GetY( scaleVec ) );
    vmathV3ScalarMul( &result->col2, &mat->col2, vmathV3GetZ( scaleVec ) );
}

static inline void vmathM3PrependScale( VmathMatrix3 *result, const VmathVector3 *scaleVec, const VmathMatrix3 *mat )
{
    vmathV3MulPerElem( &result->col0, &mat->col0, scaleVec );
    vmathV3MulPerElem( &result->col1, &mat->col1, scaleVec );
    vmathV3MulPerElem( &result->col2, &mat->col2, scaleVec );
}

static inline void vmathM3Select( VmathMatrix3 *result, const VmathMatrix3 *mat0, const VmathMatrix3 *mat1, unsigned int select1 )
{
    vmathV3Select( &result->col0, &mat0->col0, &mat1->col0, select1 );
    vmathV3Select( &result->col1, &mat0->col1, &mat1->col1, select1 );
    vmathV3Select( &result->col2, &mat0->col2, &mat1->col2, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathM3Print( const VmathMatrix3 *mat )
{
    VmathVector3 tmpV3_0, tmpV3_1, tmpV3_2;
    vmathM3GetRow( &tmpV3_0, mat, 0 );
    vmathV3Print( &tmpV3_0 );
    vmathM3GetRow( &tmpV3_1, mat, 1 );
    vmathV3Print( &tmpV3_1 );
    vmathM3GetRow( &tmpV3_2, mat, 2 );
    vmathV3Print( &tmpV3_2 );
}

static inline void vmathM3Prints( const VmathMatrix3 *mat, const char *name )
{
    printf("%s:\n", name);
    vmathM3Print( mat );
}

#endif

static inline void vmathM4Copy( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( &result->col0, &mat->col0 );
    vmathV4Copy( &result->col1, &mat->col1 );
    vmathV4Copy( &result->col2, &mat->col2 );
    vmathV4Copy( &result->col3, &mat->col3 );
}

static inline void vmathM4MakeFromScalar( VmathMatrix4 *result, float scalar )
{
    vmathV4MakeFromScalar( &result->col0, scalar );
    vmathV4MakeFromScalar( &result->col1, scalar );
    vmathV4MakeFromScalar( &result->col2, scalar );
    vmathV4MakeFromScalar( &result->col3, scalar );
}

static inline void vmathM4MakeFromT3( VmathMatrix4 *result, const VmathTransform3 *mat )
{
    vmathV4MakeFromV3Scalar( &result->col0, &mat->col0, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col1, &mat->col1, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col2, &mat->col2, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col3, &mat->col3, 1.0f );
}

static inline void vmathM4MakeFromCols( VmathMatrix4 *result, const VmathVector4 *_col0, const VmathVector4 *_col1, const VmathVector4 *_col2, const VmathVector4 *_col3 )
{
    vmathV4Copy( &result->col0, _col0 );
    vmathV4Copy( &result->col1, _col1 );
    vmathV4Copy( &result->col2, _col2 );
    vmathV4Copy( &result->col3, _col3 );
}

static inline void vmathM4MakeFromM3V3( VmathMatrix4 *result, const VmathMatrix3 *mat, const VmathVector3 *translateVec )
{
    vmathV4MakeFromV3Scalar( &result->col0, &mat->col0, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col1, &mat->col1, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col2, &mat->col2, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col3, translateVec, 1.0f );
}

static inline void vmathM4MakeFromQV3( VmathMatrix4 *result, const VmathQuat *unitQuat, const VmathVector3 *translateVec )
{
    VmathMatrix3 mat;
    vmathM3MakeFromQ( &mat, unitQuat );
    vmathV4MakeFromV3Scalar( &result->col0, &mat.col0, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col1, &mat.col1, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col2, &mat.col2, 0.0f );
    vmathV4MakeFromV3Scalar( &result->col3, translateVec, 1.0f );
}

static inline void vmathM4SetCol0( VmathMatrix4 *result, const VmathVector4 *_col0 )
{
    vmathV4Copy( &result->col0, _col0 );
}

static inline void vmathM4SetCol1( VmathMatrix4 *result, const VmathVector4 *_col1 )
{
    vmathV4Copy( &result->col1, _col1 );
}

static inline void vmathM4SetCol2( VmathMatrix4 *result, const VmathVector4 *_col2 )
{
    vmathV4Copy( &result->col2, _col2 );
}

static inline void vmathM4SetCol3( VmathMatrix4 *result, const VmathVector4 *_col3 )
{
    vmathV4Copy( &result->col3, _col3 );
}

static inline void vmathM4SetCol( VmathMatrix4 *result, int col, const VmathVector4 *vec )
{
    vmathV4Copy( (&result->col0 + col), vec );
}

static inline void vmathM4SetRow( VmathMatrix4 *result, int row, const VmathVector4 *vec )
{
    vmathV4SetElem( &result->col0, row, vmathV4GetElem( vec, 0 ) );
    vmathV4SetElem( &result->col1, row, vmathV4GetElem( vec, 1 ) );
    vmathV4SetElem( &result->col2, row, vmathV4GetElem( vec, 2 ) );
    vmathV4SetElem( &result->col3, row, vmathV4GetElem( vec, 3 ) );
}

static inline void vmathM4SetElem( VmathMatrix4 *result, int col, int row, float val )
{
    VmathVector4 tmpV3_0;
    vmathM4GetCol( &tmpV3_0, result, col );
    vmathV4SetElem( &tmpV3_0, row, val );
    vmathM4SetCol( result, col, &tmpV3_0 );
}

static inline float vmathM4GetElem( const VmathMatrix4 *mat, int col, int row )
{
    VmathVector4 tmpV4_0;
    vmathM4GetCol( &tmpV4_0, mat, col );
    return vmathV4GetElem( &tmpV4_0, row );
}

static inline void vmathM4GetCol0( VmathVector4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( result, &mat->col0 );
}

static inline void vmathM4GetCol1( VmathVector4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( result, &mat->col1 );
}

static inline void vmathM4GetCol2( VmathVector4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( result, &mat->col2 );
}

static inline void vmathM4GetCol3( VmathVector4 *result, const VmathMatrix4 *mat )
{
    vmathV4Copy( result, &mat->col3 );
}

static inline void vmathM4GetCol( VmathVector4 *result, const VmathMatrix4 *mat, int col )
{
    vmathV4Copy( result, (&mat->col0 + col) );
}

static inline void vmathM4GetRow( VmathVector4 *result, const VmathMatrix4 *mat, int row )
{
    vmathV4MakeFromElems( result, vmathV4GetElem( &mat->col0, row ), vmathV4GetElem( &mat->col1, row ), vmathV4GetElem( &mat->col2, row ), vmathV4GetElem( &mat->col3, row ) );
}

static inline void vmathM4Transpose( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    VmathMatrix4 tmpResult;
    vmathV4MakeFromElems( &tmpResult.col0, mat->col0.x, mat->col1.x, mat->col2.x, mat->col3.x );
    vmathV4MakeFromElems( &tmpResult.col1, mat->col0.y, mat->col1.y, mat->col2.y, mat->col3.y );
    vmathV4MakeFromElems( &tmpResult.col2, mat->col0.z, mat->col1.z, mat->col2.z, mat->col3.z );
    vmathV4MakeFromElems( &tmpResult.col3, mat->col0.w, mat->col1.w, mat->col2.w, mat->col3.w );
    vmathM4Copy( result, &tmpResult );
}

static inline void vmathM4Inverse( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    VmathVector4 res0, res1, res2, res3;
    float mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, detInv;
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
    tmp0 = ( ( mK * mD ) - ( mC * mL ) );
    tmp1 = ( ( mO * mH ) - ( mG * mP ) );
    tmp2 = ( ( mB * mK ) - ( mJ * mC ) );
    tmp3 = ( ( mF * mO ) - ( mN * mG ) );
    tmp4 = ( ( mJ * mD ) - ( mB * mL ) );
    tmp5 = ( ( mN * mH ) - ( mF * mP ) );
    vmathV4SetX( &res0, ( ( ( mJ * tmp1 ) - ( mL * tmp3 ) ) - ( mK * tmp5 ) ) );
    vmathV4SetY( &res0, ( ( ( mN * tmp0 ) - ( mP * tmp2 ) ) - ( mO * tmp4 ) ) );
    vmathV4SetZ( &res0, ( ( ( mD * tmp3 ) + ( mC * tmp5 ) ) - ( mB * tmp1 ) ) );
    vmathV4SetW( &res0, ( ( ( mH * tmp2 ) + ( mG * tmp4 ) ) - ( mF * tmp0 ) ) );
    detInv = ( 1.0f / ( ( ( ( mA * res0.x ) + ( mE * res0.y ) ) + ( mI * res0.z ) ) + ( mM * res0.w ) ) );
    vmathV4SetX( &res1, ( mI * tmp1 ) );
    vmathV4SetY( &res1, ( mM * tmp0 ) );
    vmathV4SetZ( &res1, ( mA * tmp1 ) );
    vmathV4SetW( &res1, ( mE * tmp0 ) );
    vmathV4SetX( &res3, ( mI * tmp3 ) );
    vmathV4SetY( &res3, ( mM * tmp2 ) );
    vmathV4SetZ( &res3, ( mA * tmp3 ) );
    vmathV4SetW( &res3, ( mE * tmp2 ) );
    vmathV4SetX( &res2, ( mI * tmp5 ) );
    vmathV4SetY( &res2, ( mM * tmp4 ) );
    vmathV4SetZ( &res2, ( mA * tmp5 ) );
    vmathV4SetW( &res2, ( mE * tmp4 ) );
    tmp0 = ( ( mI * mB ) - ( mA * mJ ) );
    tmp1 = ( ( mM * mF ) - ( mE * mN ) );
    tmp2 = ( ( mI * mD ) - ( mA * mL ) );
    tmp3 = ( ( mM * mH ) - ( mE * mP ) );
    tmp4 = ( ( mI * mC ) - ( mA * mK ) );
    tmp5 = ( ( mM * mG ) - ( mE * mO ) );
    vmathV4SetX( &res2, ( ( ( mL * tmp1 ) - ( mJ * tmp3 ) ) + res2.x ) );
    vmathV4SetY( &res2, ( ( ( mP * tmp0 ) - ( mN * tmp2 ) ) + res2.y ) );
    vmathV4SetZ( &res2, ( ( ( mB * tmp3 ) - ( mD * tmp1 ) ) - res2.z ) );
    vmathV4SetW( &res2, ( ( ( mF * tmp2 ) - ( mH * tmp0 ) ) - res2.w ) );
    vmathV4SetX( &res3, ( ( ( mJ * tmp5 ) - ( mK * tmp1 ) ) + res3.x ) );
    vmathV4SetY( &res3, ( ( ( mN * tmp4 ) - ( mO * tmp0 ) ) + res3.y ) );
    vmathV4SetZ( &res3, ( ( ( mC * tmp1 ) - ( mB * tmp5 ) ) - res3.z ) );
    vmathV4SetW( &res3, ( ( ( mG * tmp0 ) - ( mF * tmp4 ) ) - res3.w ) );
    vmathV4SetX( &res1, ( ( ( mK * tmp3 ) - ( mL * tmp5 ) ) - res1.x ) );
    vmathV4SetY( &res1, ( ( ( mO * tmp2 ) - ( mP * tmp4 ) ) - res1.y ) );
    vmathV4SetZ( &res1, ( ( ( mD * tmp5 ) - ( mC * tmp3 ) ) + res1.z ) );
    vmathV4SetW( &res1, ( ( ( mH * tmp4 ) - ( mG * tmp2 ) ) + res1.w ) );
    vmathV4ScalarMul( &result->col0, &res0, detInv );
    vmathV4ScalarMul( &result->col1, &res1, detInv );
    vmathV4ScalarMul( &result->col2, &res2, detInv );
    vmathV4ScalarMul( &result->col3, &res3, detInv );
}

static inline void vmathM4AffineInverse( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    VmathTransform3 affineMat, tmpT3_0;
    VmathVector3 tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3;
    vmathV4GetXYZ( &tmpV3_0, &mat->col0 );
    vmathT3SetCol0( &affineMat, &tmpV3_0 );
    vmathV4GetXYZ( &tmpV3_1, &mat->col1 );
    vmathT3SetCol1( &affineMat, &tmpV3_1 );
    vmathV4GetXYZ( &tmpV3_2, &mat->col2 );
    vmathT3SetCol2( &affineMat, &tmpV3_2 );
    vmathV4GetXYZ( &tmpV3_3, &mat->col3 );
    vmathT3SetCol3( &affineMat, &tmpV3_3 );
    vmathT3Inverse( &tmpT3_0, &affineMat );
    vmathM4MakeFromT3( result, &tmpT3_0 );
}

static inline void vmathM4OrthoInverse( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    VmathTransform3 affineMat, tmpT3_0;
    VmathVector3 tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3;
    vmathV4GetXYZ( &tmpV3_0, &mat->col0 );
    vmathT3SetCol0( &affineMat, &tmpV3_0 );
    vmathV4GetXYZ( &tmpV3_1, &mat->col1 );
    vmathT3SetCol1( &affineMat, &tmpV3_1 );
    vmathV4GetXYZ( &tmpV3_2, &mat->col2 );
    vmathT3SetCol2( &affineMat, &tmpV3_2 );
    vmathV4GetXYZ( &tmpV3_3, &mat->col3 );
    vmathT3SetCol3( &affineMat, &tmpV3_3 );
    vmathT3OrthoInverse( &tmpT3_0, &affineMat );
    vmathM4MakeFromT3( result, &tmpT3_0 );
}

static inline float vmathM4Determinant( const VmathMatrix4 *mat )
{
    float dx, dy, dz, dw, mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5;
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
    tmp0 = ( ( mK * mD ) - ( mC * mL ) );
    tmp1 = ( ( mO * mH ) - ( mG * mP ) );
    tmp2 = ( ( mB * mK ) - ( mJ * mC ) );
    tmp3 = ( ( mF * mO ) - ( mN * mG ) );
    tmp4 = ( ( mJ * mD ) - ( mB * mL ) );
    tmp5 = ( ( mN * mH ) - ( mF * mP ) );
    dx = ( ( ( mJ * tmp1 ) - ( mL * tmp3 ) ) - ( mK * tmp5 ) );
    dy = ( ( ( mN * tmp0 ) - ( mP * tmp2 ) ) - ( mO * tmp4 ) );
    dz = ( ( ( mD * tmp3 ) + ( mC * tmp5 ) ) - ( mB * tmp1 ) );
    dw = ( ( ( mH * tmp2 ) + ( mG * tmp4 ) ) - ( mF * tmp0 ) );
    return ( ( ( ( mA * dx ) + ( mE * dy ) ) + ( mI * dz ) ) + ( mM * dw ) );
}

static inline void vmathM4Add( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 )
{
    vmathV4Add( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV4Add( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV4Add( &result->col2, &mat0->col2, &mat1->col2 );
    vmathV4Add( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathM4Sub( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 )
{
    vmathV4Sub( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV4Sub( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV4Sub( &result->col2, &mat0->col2, &mat1->col2 );
    vmathV4Sub( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathM4Neg( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    vmathV4Neg( &result->col0, &mat->col0 );
    vmathV4Neg( &result->col1, &mat->col1 );
    vmathV4Neg( &result->col2, &mat->col2 );
    vmathV4Neg( &result->col3, &mat->col3 );
}

static inline void vmathM4AbsPerElem( VmathMatrix4 *result, const VmathMatrix4 *mat )
{
    vmathV4AbsPerElem( &result->col0, &mat->col0 );
    vmathV4AbsPerElem( &result->col1, &mat->col1 );
    vmathV4AbsPerElem( &result->col2, &mat->col2 );
    vmathV4AbsPerElem( &result->col3, &mat->col3 );
}

static inline void vmathM4ScalarMul( VmathMatrix4 *result, const VmathMatrix4 *mat, float scalar )
{
    vmathV4ScalarMul( &result->col0, &mat->col0, scalar );
    vmathV4ScalarMul( &result->col1, &mat->col1, scalar );
    vmathV4ScalarMul( &result->col2, &mat->col2, scalar );
    vmathV4ScalarMul( &result->col3, &mat->col3, scalar );
}

static inline void vmathM4MulV4( VmathVector4 *result, const VmathMatrix4 *mat, const VmathVector4 *vec )
{
    float tmpX, tmpY, tmpZ, tmpW;
    tmpX = ( ( ( ( mat->col0.x * vec->x ) + ( mat->col1.x * vec->y ) ) + ( mat->col2.x * vec->z ) ) + ( mat->col3.x * vec->w ) );
    tmpY = ( ( ( ( mat->col0.y * vec->x ) + ( mat->col1.y * vec->y ) ) + ( mat->col2.y * vec->z ) ) + ( mat->col3.y * vec->w ) );
    tmpZ = ( ( ( ( mat->col0.z * vec->x ) + ( mat->col1.z * vec->y ) ) + ( mat->col2.z * vec->z ) ) + ( mat->col3.z * vec->w ) );
    tmpW = ( ( ( ( mat->col0.w * vec->x ) + ( mat->col1.w * vec->y ) ) + ( mat->col2.w * vec->z ) ) + ( mat->col3.w * vec->w ) );
    vmathV4MakeFromElems( result, tmpX, tmpY, tmpZ, tmpW );
}

static inline void vmathM4MulV3( VmathVector4 *result, const VmathMatrix4 *mat, const VmathVector3 *vec )
{
    result->x = ( ( ( mat->col0.x * vec->x ) + ( mat->col1.x * vec->y ) ) + ( mat->col2.x * vec->z ) );
    result->y = ( ( ( mat->col0.y * vec->x ) + ( mat->col1.y * vec->y ) ) + ( mat->col2.y * vec->z ) );
    result->z = ( ( ( mat->col0.z * vec->x ) + ( mat->col1.z * vec->y ) ) + ( mat->col2.z * vec->z ) );
    result->w = ( ( ( mat->col0.w * vec->x ) + ( mat->col1.w * vec->y ) ) + ( mat->col2.w * vec->z ) );
}

static inline void vmathM4MulP3( VmathVector4 *result, const VmathMatrix4 *mat, const VmathPoint3 *pnt )
{
    result->x = ( ( ( ( mat->col0.x * pnt->x ) + ( mat->col1.x * pnt->y ) ) + ( mat->col2.x * pnt->z ) ) + mat->col3.x );
    result->y = ( ( ( ( mat->col0.y * pnt->x ) + ( mat->col1.y * pnt->y ) ) + ( mat->col2.y * pnt->z ) ) + mat->col3.y );
    result->z = ( ( ( ( mat->col0.z * pnt->x ) + ( mat->col1.z * pnt->y ) ) + ( mat->col2.z * pnt->z ) ) + mat->col3.z );
    result->w = ( ( ( ( mat->col0.w * pnt->x ) + ( mat->col1.w * pnt->y ) ) + ( mat->col2.w * pnt->z ) ) + mat->col3.w );
}

static inline void vmathM4Mul( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 )
{
    VmathMatrix4 tmpResult;
    vmathM4MulV4( &tmpResult.col0, mat0, &mat1->col0 );
    vmathM4MulV4( &tmpResult.col1, mat0, &mat1->col1 );
    vmathM4MulV4( &tmpResult.col2, mat0, &mat1->col2 );
    vmathM4MulV4( &tmpResult.col3, mat0, &mat1->col3 );
    vmathM4Copy( result, &tmpResult );
}

static inline void vmathM4MulT3( VmathMatrix4 *result, const VmathMatrix4 *mat, const VmathTransform3 *tfrm1 )
{
    VmathMatrix4 tmpResult;
    VmathPoint3 tmpP3_0;
    vmathM4MulV3( &tmpResult.col0, mat, &tfrm1->col0 );
    vmathM4MulV3( &tmpResult.col1, mat, &tfrm1->col1 );
    vmathM4MulV3( &tmpResult.col2, mat, &tfrm1->col2 );
    vmathP3MakeFromV3( &tmpP3_0, &tfrm1->col3 );
    vmathM4MulP3( &tmpResult.col3, mat, &tmpP3_0 );
    vmathM4Copy( result, &tmpResult );
}

static inline void vmathM4MulPerElem( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1 )
{
    vmathV4MulPerElem( &result->col0, &mat0->col0, &mat1->col0 );
    vmathV4MulPerElem( &result->col1, &mat0->col1, &mat1->col1 );
    vmathV4MulPerElem( &result->col2, &mat0->col2, &mat1->col2 );
    vmathV4MulPerElem( &result->col3, &mat0->col3, &mat1->col3 );
}

static inline void vmathM4MakeIdentity( VmathMatrix4 *result )
{
    vmathV4MakeXAxis( &result->col0 );
    vmathV4MakeYAxis( &result->col1 );
    vmathV4MakeZAxis( &result->col2 );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4SetUpper3x3( VmathMatrix4 *result, const VmathMatrix3 *mat3 )
{
    vmathV4SetXYZ( &result->col0, &mat3->col0 );
    vmathV4SetXYZ( &result->col1, &mat3->col1 );
    vmathV4SetXYZ( &result->col2, &mat3->col2 );
}

static inline void vmathM4GetUpper3x3( VmathMatrix3 *result, const VmathMatrix4 *mat )
{
    vmathV4GetXYZ( &result->col0, &mat->col0 );
    vmathV4GetXYZ( &result->col1, &mat->col1 );
    vmathV4GetXYZ( &result->col2, &mat->col2 );
}

static inline void vmathM4SetTranslation( VmathMatrix4 *result, const VmathVector3 *translateVec )
{
    vmathV4SetXYZ( &result->col3, translateVec );
}

static inline void vmathM4GetTranslation( VmathVector3 *result, const VmathMatrix4 *mat )
{
    vmathV4GetXYZ( result, &mat->col3 );
}

static inline void vmathM4MakeRotationX( VmathMatrix4 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV4MakeXAxis( &result->col0 );
    vmathV4MakeFromElems( &result->col1, 0.0f, c, s, 0.0f );
    vmathV4MakeFromElems( &result->col2, 0.0f, -s, c, 0.0f );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationY( VmathMatrix4 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV4MakeFromElems( &result->col0, c, 0.0f, -s, 0.0f );
    vmathV4MakeYAxis( &result->col1 );
    vmathV4MakeFromElems( &result->col2, s, 0.0f, c, 0.0f );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationZ( VmathMatrix4 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV4MakeFromElems( &result->col0, c, s, 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col1, -s, c, 0.0f, 0.0f );
    vmathV4MakeZAxis( &result->col2 );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationZYX( VmathMatrix4 *result, const VmathVector3 *radiansXYZ )
{
    float sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sX = sinf( radiansXYZ->x );
    cX = cosf( radiansXYZ->x );
    sY = sinf( radiansXYZ->y );
    cY = cosf( radiansXYZ->y );
    sZ = sinf( radiansXYZ->z );
    cZ = cosf( radiansXYZ->z );
    tmp0 = ( cZ * sY );
    tmp1 = ( sZ * sY );
    vmathV4MakeFromElems( &result->col0, ( cZ * cY ), ( sZ * cY ), -sY, 0.0f );
    vmathV4MakeFromElems( &result->col1, ( ( tmp0 * sX ) - ( sZ * cX ) ), ( ( tmp1 * sX ) + ( cZ * cX ) ), ( cY * sX ), 0.0f );
    vmathV4MakeFromElems( &result->col2, ( ( tmp0 * cX ) + ( sZ * sX ) ), ( ( tmp1 * cX ) - ( cZ * sX ) ), ( cY * cX ), 0.0f );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationAxis( VmathMatrix4 *result, float radians, const VmathVector3 *unitVec )
{
    float x, y, z, s, c, oneMinusC, xy, yz, zx;
    s = sinf( radians );
    c = cosf( radians );
    x = unitVec->x;
    y = unitVec->y;
    z = unitVec->z;
    xy = ( x * y );
    yz = ( y * z );
    zx = ( z * x );
    oneMinusC = ( 1.0f - c );
    vmathV4MakeFromElems( &result->col0, ( ( ( x * x ) * oneMinusC ) + c ), ( ( xy * oneMinusC ) + ( z * s ) ), ( ( zx * oneMinusC ) - ( y * s ) ), 0.0f );
    vmathV4MakeFromElems( &result->col1, ( ( xy * oneMinusC ) - ( z * s ) ), ( ( ( y * y ) * oneMinusC ) + c ), ( ( yz * oneMinusC ) + ( x * s ) ), 0.0f );
    vmathV4MakeFromElems( &result->col2, ( ( zx * oneMinusC ) + ( y * s ) ), ( ( yz * oneMinusC ) - ( x * s ) ), ( ( ( z * z ) * oneMinusC ) + c ), 0.0f );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4MakeRotationQ( VmathMatrix4 *result, const VmathQuat *unitQuat )
{
    VmathTransform3 tmpT3_0;
    vmathT3MakeRotationQ( &tmpT3_0, unitQuat );
    vmathM4MakeFromT3( result, &tmpT3_0 );
}

static inline void vmathM4MakeScale( VmathMatrix4 *result, const VmathVector3 *scaleVec )
{
    vmathV4MakeFromElems( &result->col0, scaleVec->x, 0.0f, 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col1, 0.0f, scaleVec->y, 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col2, 0.0f, 0.0f, scaleVec->z, 0.0f );
    vmathV4MakeWAxis( &result->col3 );
}

static inline void vmathM4AppendScale( VmathMatrix4 *result, const VmathMatrix4 *mat, const VmathVector3 *scaleVec )
{
    vmathV4ScalarMul( &result->col0, &mat->col0, vmathV3GetX( scaleVec ) );
    vmathV4ScalarMul( &result->col1, &mat->col1, vmathV3GetY( scaleVec ) );
    vmathV4ScalarMul( &result->col2, &mat->col2, vmathV3GetZ( scaleVec ) );
    vmathV4Copy( &result->col3, &mat->col3 );
}

static inline void vmathM4PrependScale( VmathMatrix4 *result, const VmathVector3 *scaleVec, const VmathMatrix4 *mat )
{
    VmathVector4 scale4;
    vmathV4MakeFromV3Scalar( &scale4, scaleVec, 1.0f );
    vmathV4MulPerElem( &result->col0, &mat->col0, &scale4 );
    vmathV4MulPerElem( &result->col1, &mat->col1, &scale4 );
    vmathV4MulPerElem( &result->col2, &mat->col2, &scale4 );
    vmathV4MulPerElem( &result->col3, &mat->col3, &scale4 );
}

static inline void vmathM4MakeTranslation( VmathMatrix4 *result, const VmathVector3 *translateVec )
{
    vmathV4MakeXAxis( &result->col0 );
    vmathV4MakeYAxis( &result->col1 );
    vmathV4MakeZAxis( &result->col2 );
    vmathV4MakeFromV3Scalar( &result->col3, translateVec, 1.0f );
}

static inline void vmathM4MakeLookAt( VmathMatrix4 *result, const VmathPoint3 *eyePos, const VmathPoint3 *lookAtPos, const VmathVector3 *upVec )
{
    VmathMatrix4 m4EyeFrame;
    VmathVector3 v3X, v3Y, v3Z, tmpV3_0, tmpV3_1;
    VmathVector4 tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3;
    vmathV3Normalize( &v3Y, upVec );
    vmathP3Sub( &tmpV3_0, eyePos, lookAtPos );
    vmathV3Normalize( &v3Z, &tmpV3_0 );
    vmathV3Cross( &tmpV3_1, &v3Y, &v3Z );
    vmathV3Normalize( &v3X, &tmpV3_1 );
    vmathV3Cross( &v3Y, &v3Z, &v3X );
    vmathV4MakeFromV3( &tmpV4_0, &v3X );
    vmathV4MakeFromV3( &tmpV4_1, &v3Y );
    vmathV4MakeFromV3( &tmpV4_2, &v3Z );
    vmathV4MakeFromP3( &tmpV4_3, eyePos );
    vmathM4MakeFromCols( &m4EyeFrame, &tmpV4_0, &tmpV4_1, &tmpV4_2, &tmpV4_3 );
    vmathM4OrthoInverse( result, &m4EyeFrame );
}

static inline void vmathM4MakePerspective( VmathMatrix4 *result, float fovyRadians, float aspect, float zNear, float zFar )
{
    float f, rangeInv;
    f = tanf( ( (float)( _VECTORMATH_PI_OVER_2 ) - ( 0.5f * fovyRadians ) ) );
    rangeInv = ( 1.0f / ( zNear - zFar ) );
    vmathV4MakeFromElems( &result->col0, ( f / aspect ), 0.0f, 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col1, 0.0f, f, 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col2, 0.0f, 0.0f, ( ( zNear + zFar ) * rangeInv ), -1.0f );
    vmathV4MakeFromElems( &result->col3, 0.0f, 0.0f, ( ( ( zNear * zFar ) * rangeInv ) * 2.0f ), 0.0f );
}

static inline void vmathM4MakeFrustum( VmathMatrix4 *result, float left, float right, float bottom, float top, float zNear, float zFar )
{
    float sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf, n2;
    sum_rl = ( right + left );
    sum_tb = ( top + bottom );
    sum_nf = ( zNear + zFar );
    inv_rl = ( 1.0f / ( right - left ) );
    inv_tb = ( 1.0f / ( top - bottom ) );
    inv_nf = ( 1.0f / ( zNear - zFar ) );
    n2 = ( zNear + zNear );
    vmathV4MakeFromElems( &result->col0, ( n2 * inv_rl ), 0.0f, 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col1, 0.0f, ( n2 * inv_tb ), 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col2, ( sum_rl * inv_rl ), ( sum_tb * inv_tb ), ( sum_nf * inv_nf ), -1.0f );
    vmathV4MakeFromElems( &result->col3, 0.0f, 0.0f, ( ( n2 * inv_nf ) * zFar ), 0.0f );
}

static inline void vmathM4MakeOrthographic( VmathMatrix4 *result, float left, float right, float bottom, float top, float zNear, float zFar )
{
    float sum_rl, sum_tb, sum_nf, inv_rl, inv_tb, inv_nf;
    sum_rl = ( right + left );
    sum_tb = ( top + bottom );
    sum_nf = ( zNear + zFar );
    inv_rl = ( 1.0f / ( right - left ) );
    inv_tb = ( 1.0f / ( top - bottom ) );
    inv_nf = ( 1.0f / ( zNear - zFar ) );
    vmathV4MakeFromElems( &result->col0, ( inv_rl + inv_rl ), 0.0f, 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col1, 0.0f, ( inv_tb + inv_tb ), 0.0f, 0.0f );
    vmathV4MakeFromElems( &result->col2, 0.0f, 0.0f, ( inv_nf + inv_nf ), 0.0f );
    vmathV4MakeFromElems( &result->col3, ( -sum_rl * inv_rl ), ( -sum_tb * inv_tb ), ( sum_nf * inv_nf ), 1.0f );
}

static inline void vmathM4Select( VmathMatrix4 *result, const VmathMatrix4 *mat0, const VmathMatrix4 *mat1, unsigned int select1 )
{
    vmathV4Select( &result->col0, &mat0->col0, &mat1->col0, select1 );
    vmathV4Select( &result->col1, &mat0->col1, &mat1->col1, select1 );
    vmathV4Select( &result->col2, &mat0->col2, &mat1->col2, select1 );
    vmathV4Select( &result->col3, &mat0->col3, &mat1->col3, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathM4Print( const VmathMatrix4 *mat )
{
    VmathVector4 tmpV4_0, tmpV4_1, tmpV4_2, tmpV4_3;
    vmathM4GetRow( &tmpV4_0, mat, 0 );
    vmathV4Print( &tmpV4_0 );
    vmathM4GetRow( &tmpV4_1, mat, 1 );
    vmathV4Print( &tmpV4_1 );
    vmathM4GetRow( &tmpV4_2, mat, 2 );
    vmathV4Print( &tmpV4_2 );
    vmathM4GetRow( &tmpV4_3, mat, 3 );
    vmathV4Print( &tmpV4_3 );
}

static inline void vmathM4Prints( const VmathMatrix4 *mat, const char *name )
{
    printf("%s:\n", name);
    vmathM4Print( mat );
}

#endif

static inline void vmathT3Copy( VmathTransform3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( &result->col0, &tfrm->col0 );
    vmathV3Copy( &result->col1, &tfrm->col1 );
    vmathV3Copy( &result->col2, &tfrm->col2 );
    vmathV3Copy( &result->col3, &tfrm->col3 );
}

static inline void vmathT3MakeFromScalar( VmathTransform3 *result, float scalar )
{
    vmathV3MakeFromScalar( &result->col0, scalar );
    vmathV3MakeFromScalar( &result->col1, scalar );
    vmathV3MakeFromScalar( &result->col2, scalar );
    vmathV3MakeFromScalar( &result->col3, scalar );
}

static inline void vmathT3MakeFromCols( VmathTransform3 *result, const VmathVector3 *_col0, const VmathVector3 *_col1, const VmathVector3 *_col2, const VmathVector3 *_col3 )
{
    vmathV3Copy( &result->col0, _col0 );
    vmathV3Copy( &result->col1, _col1 );
    vmathV3Copy( &result->col2, _col2 );
    vmathV3Copy( &result->col3, _col3 );
}

static inline void vmathT3MakeFromM3V3( VmathTransform3 *result, const VmathMatrix3 *tfrm, const VmathVector3 *translateVec )
{
    vmathT3SetUpper3x3( result, tfrm );
    vmathT3SetTranslation( result, translateVec );
}

static inline void vmathT3MakeFromQV3( VmathTransform3 *result, const VmathQuat *unitQuat, const VmathVector3 *translateVec )
{
    VmathMatrix3 tmpM3_0;
    vmathM3MakeFromQ( &tmpM3_0, unitQuat );
    vmathT3SetUpper3x3( result, &tmpM3_0 );
    vmathT3SetTranslation( result, translateVec );
}

static inline void vmathT3SetCol0( VmathTransform3 *result, const VmathVector3 *_col0 )
{
    vmathV3Copy( &result->col0, _col0 );
}

static inline void vmathT3SetCol1( VmathTransform3 *result, const VmathVector3 *_col1 )
{
    vmathV3Copy( &result->col1, _col1 );
}

static inline void vmathT3SetCol2( VmathTransform3 *result, const VmathVector3 *_col2 )
{
    vmathV3Copy( &result->col2, _col2 );
}

static inline void vmathT3SetCol3( VmathTransform3 *result, const VmathVector3 *_col3 )
{
    vmathV3Copy( &result->col3, _col3 );
}

static inline void vmathT3SetCol( VmathTransform3 *result, int col, const VmathVector3 *vec )
{
    vmathV3Copy( (&result->col0 + col), vec );
}

static inline void vmathT3SetRow( VmathTransform3 *result, int row, const VmathVector4 *vec )
{
    vmathV3SetElem( &result->col0, row, vmathV4GetElem( vec, 0 ) );
    vmathV3SetElem( &result->col1, row, vmathV4GetElem( vec, 1 ) );
    vmathV3SetElem( &result->col2, row, vmathV4GetElem( vec, 2 ) );
    vmathV3SetElem( &result->col3, row, vmathV4GetElem( vec, 3 ) );
}

static inline void vmathT3SetElem( VmathTransform3 *result, int col, int row, float val )
{
    VmathVector3 tmpV3_0;
    vmathT3GetCol( &tmpV3_0, result, col );
    vmathV3SetElem( &tmpV3_0, row, val );
    vmathT3SetCol( result, col, &tmpV3_0 );
}

static inline float vmathT3GetElem( const VmathTransform3 *tfrm, int col, int row )
{
    VmathVector3 tmpV3_0;
    vmathT3GetCol( &tmpV3_0, tfrm, col );
    return vmathV3GetElem( &tmpV3_0, row );
}

static inline void vmathT3GetCol0( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col0 );
}

static inline void vmathT3GetCol1( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col1 );
}

static inline void vmathT3GetCol2( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col2 );
}

static inline void vmathT3GetCol3( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col3 );
}

static inline void vmathT3GetCol( VmathVector3 *result, const VmathTransform3 *tfrm, int col )
{
    vmathV3Copy( result, (&tfrm->col0 + col) );
}

static inline void vmathT3GetRow( VmathVector4 *result, const VmathTransform3 *tfrm, int row )
{
    vmathV4MakeFromElems( result, vmathV3GetElem( &tfrm->col0, row ), vmathV3GetElem( &tfrm->col1, row ), vmathV3GetElem( &tfrm->col2, row ), vmathV3GetElem( &tfrm->col3, row ) );
}

static inline void vmathT3Inverse( VmathTransform3 *result, const VmathTransform3 *tfrm )
{
    VmathVector3 tmp0, tmp1, tmp2, inv0, inv1, inv2, tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3, tmpV3_4, tmpV3_5;
    float detinv;
    vmathV3Cross( &tmp0, &tfrm->col1, &tfrm->col2 );
    vmathV3Cross( &tmp1, &tfrm->col2, &tfrm->col0 );
    vmathV3Cross( &tmp2, &tfrm->col0, &tfrm->col1 );
    detinv = ( 1.0f / vmathV3Dot( &tfrm->col2, &tmp2 ) );
    vmathV3MakeFromElems( &inv0, ( tmp0.x * detinv ), ( tmp1.x * detinv ), ( tmp2.x * detinv ) );
    vmathV3MakeFromElems( &inv1, ( tmp0.y * detinv ), ( tmp1.y * detinv ), ( tmp2.y * detinv ) );
    vmathV3MakeFromElems( &inv2, ( tmp0.z * detinv ), ( tmp1.z * detinv ), ( tmp2.z * detinv ) );
    vmathV3Copy( &result->col0, &inv0 );
    vmathV3Copy( &result->col1, &inv1 );
    vmathV3Copy( &result->col2, &inv2 );
    vmathV3ScalarMul( &tmpV3_0, &inv0, tfrm->col3.x );
    vmathV3ScalarMul( &tmpV3_1, &inv1, tfrm->col3.y );
    vmathV3ScalarMul( &tmpV3_2, &inv2, tfrm->col3.z );
    vmathV3Add( &tmpV3_3, &tmpV3_1, &tmpV3_2 );
    vmathV3Add( &tmpV3_4, &tmpV3_0, &tmpV3_3 );
    vmathV3Neg( &tmpV3_5, &tmpV3_4 );
    vmathV3Copy( &result->col3, &tmpV3_5 );
}

static inline void vmathT3OrthoInverse( VmathTransform3 *result, const VmathTransform3 *tfrm )
{
    VmathVector3 inv0, inv1, inv2, tmpV3_0, tmpV3_1, tmpV3_2, tmpV3_3, tmpV3_4, tmpV3_5;
    vmathV3MakeFromElems( &inv0, tfrm->col0.x, tfrm->col1.x, tfrm->col2.x );
    vmathV3MakeFromElems( &inv1, tfrm->col0.y, tfrm->col1.y, tfrm->col2.y );
    vmathV3MakeFromElems( &inv2, tfrm->col0.z, tfrm->col1.z, tfrm->col2.z );
    vmathV3Copy( &result->col0, &inv0 );
    vmathV3Copy( &result->col1, &inv1 );
    vmathV3Copy( &result->col2, &inv2 );
    vmathV3ScalarMul( &tmpV3_0, &inv0, tfrm->col3.x );
    vmathV3ScalarMul( &tmpV3_1, &inv1, tfrm->col3.y );
    vmathV3ScalarMul( &tmpV3_2, &inv2, tfrm->col3.z );
    vmathV3Add( &tmpV3_3, &tmpV3_1, &tmpV3_2 );
    vmathV3Add( &tmpV3_4, &tmpV3_0, &tmpV3_3 );
    vmathV3Neg( &tmpV3_5, &tmpV3_4 );
    vmathV3Copy( &result->col3, &tmpV3_5 );
}

static inline void vmathT3AbsPerElem( VmathTransform3 *result, const VmathTransform3 *tfrm )
{
    vmathV3AbsPerElem( &result->col0, &tfrm->col0 );
    vmathV3AbsPerElem( &result->col1, &tfrm->col1 );
    vmathV3AbsPerElem( &result->col2, &tfrm->col2 );
    vmathV3AbsPerElem( &result->col3, &tfrm->col3 );
}

static inline void vmathT3MulV3( VmathVector3 *result, const VmathTransform3 *tfrm, const VmathVector3 *vec )
{
    float tmpX, tmpY, tmpZ;
    tmpX = ( ( ( tfrm->col0.x * vec->x ) + ( tfrm->col1.x * vec->y ) ) + ( tfrm->col2.x * vec->z ) );
    tmpY = ( ( ( tfrm->col0.y * vec->x ) + ( tfrm->col1.y * vec->y ) ) + ( tfrm->col2.y * vec->z ) );
    tmpZ = ( ( ( tfrm->col0.z * vec->x ) + ( tfrm->col1.z * vec->y ) ) + ( tfrm->col2.z * vec->z ) );
    vmathV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathT3MulP3( VmathPoint3 *result, const VmathTransform3 *tfrm, const VmathPoint3 *pnt )
{
    float tmpX, tmpY, tmpZ;
    tmpX = ( ( ( ( tfrm->col0.x * pnt->x ) + ( tfrm->col1.x * pnt->y ) ) + ( tfrm->col2.x * pnt->z ) ) + tfrm->col3.x );
    tmpY = ( ( ( ( tfrm->col0.y * pnt->x ) + ( tfrm->col1.y * pnt->y ) ) + ( tfrm->col2.y * pnt->z ) ) + tfrm->col3.y );
    tmpZ = ( ( ( ( tfrm->col0.z * pnt->x ) + ( tfrm->col1.z * pnt->y ) ) + ( tfrm->col2.z * pnt->z ) ) + tfrm->col3.z );
    vmathP3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathT3Mul( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1 )
{
    VmathTransform3 tmpResult;
    VmathPoint3 tmpP3_0, tmpP3_1;
    vmathT3MulV3( &tmpResult.col0, tfrm0, &tfrm1->col0 );
    vmathT3MulV3( &tmpResult.col1, tfrm0, &tfrm1->col1 );
    vmathT3MulV3( &tmpResult.col2, tfrm0, &tfrm1->col2 );
    vmathP3MakeFromV3( &tmpP3_0, &tfrm1->col3 );
    vmathT3MulP3( &tmpP3_1, tfrm0, &tmpP3_0 );
    vmathV3MakeFromP3( &tmpResult.col3, &tmpP3_1 );
    vmathT3Copy( result, &tmpResult );
}

static inline void vmathT3MulPerElem( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1 )
{
    vmathV3MulPerElem( &result->col0, &tfrm0->col0, &tfrm1->col0 );
    vmathV3MulPerElem( &result->col1, &tfrm0->col1, &tfrm1->col1 );
    vmathV3MulPerElem( &result->col2, &tfrm0->col2, &tfrm1->col2 );
    vmathV3MulPerElem( &result->col3, &tfrm0->col3, &tfrm1->col3 );
}

static inline void vmathT3MakeIdentity( VmathTransform3 *result )
{
    vmathV3MakeXAxis( &result->col0 );
    vmathV3MakeYAxis( &result->col1 );
    vmathV3MakeZAxis( &result->col2 );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3SetUpper3x3( VmathTransform3 *result, const VmathMatrix3 *tfrm )
{
    vmathV3Copy( &result->col0, &tfrm->col0 );
    vmathV3Copy( &result->col1, &tfrm->col1 );
    vmathV3Copy( &result->col2, &tfrm->col2 );
}

static inline void vmathT3GetUpper3x3( VmathMatrix3 *result, const VmathTransform3 *tfrm )
{
    vmathM3MakeFromCols( result, &tfrm->col0, &tfrm->col1, &tfrm->col2 );
}

static inline void vmathT3SetTranslation( VmathTransform3 *result, const VmathVector3 *translateVec )
{
    vmathV3Copy( &result->col3, translateVec );
}

static inline void vmathT3GetTranslation( VmathVector3 *result, const VmathTransform3 *tfrm )
{
    vmathV3Copy( result, &tfrm->col3 );
}

static inline void vmathT3MakeRotationX( VmathTransform3 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV3MakeXAxis( &result->col0 );
    vmathV3MakeFromElems( &result->col1, 0.0f, c, s );
    vmathV3MakeFromElems( &result->col2, 0.0f, -s, c );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3MakeRotationY( VmathTransform3 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV3MakeFromElems( &result->col0, c, 0.0f, -s );
    vmathV3MakeYAxis( &result->col1 );
    vmathV3MakeFromElems( &result->col2, s, 0.0f, c );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3MakeRotationZ( VmathTransform3 *result, float radians )
{
    float s, c;
    s = sinf( radians );
    c = cosf( radians );
    vmathV3MakeFromElems( &result->col0, c, s, 0.0f );
    vmathV3MakeFromElems( &result->col1, -s, c, 0.0f );
    vmathV3MakeZAxis( &result->col2 );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3MakeRotationZYX( VmathTransform3 *result, const VmathVector3 *radiansXYZ )
{
    float sX, cX, sY, cY, sZ, cZ, tmp0, tmp1;
    sX = sinf( radiansXYZ->x );
    cX = cosf( radiansXYZ->x );
    sY = sinf( radiansXYZ->y );
    cY = cosf( radiansXYZ->y );
    sZ = sinf( radiansXYZ->z );
    cZ = cosf( radiansXYZ->z );
    tmp0 = ( cZ * sY );
    tmp1 = ( sZ * sY );
    vmathV3MakeFromElems( &result->col0, ( cZ * cY ), ( sZ * cY ), -sY );
    vmathV3MakeFromElems( &result->col1, ( ( tmp0 * sX ) - ( sZ * cX ) ), ( ( tmp1 * sX ) + ( cZ * cX ) ), ( cY * sX ) );
    vmathV3MakeFromElems( &result->col2, ( ( tmp0 * cX ) + ( sZ * sX ) ), ( ( tmp1 * cX ) - ( cZ * sX ) ), ( cY * cX ) );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3MakeRotationAxis( VmathTransform3 *result, float radians, const VmathVector3 *unitVec )
{
    VmathMatrix3 tmpM3_0;
    VmathVector3 tmpV3_0;
    vmathM3MakeRotationAxis( &tmpM3_0, radians, unitVec );
    vmathV3MakeFromScalar( &tmpV3_0, 0.0f );
    vmathT3MakeFromM3V3( result, &tmpM3_0, &tmpV3_0 );
}

static inline void vmathT3MakeRotationQ( VmathTransform3 *result, const VmathQuat *unitQuat )
{
    VmathMatrix3 tmpM3_0;
    VmathVector3 tmpV3_0;
    vmathM3MakeFromQ( &tmpM3_0, unitQuat );
    vmathV3MakeFromScalar( &tmpV3_0, 0.0f );
    vmathT3MakeFromM3V3( result, &tmpM3_0, &tmpV3_0 );
}

static inline void vmathT3MakeScale( VmathTransform3 *result, const VmathVector3 *scaleVec )
{
    vmathV3MakeFromElems( &result->col0, scaleVec->x, 0.0f, 0.0f );
    vmathV3MakeFromElems( &result->col1, 0.0f, scaleVec->y, 0.0f );
    vmathV3MakeFromElems( &result->col2, 0.0f, 0.0f, scaleVec->z );
    vmathV3MakeFromScalar( &result->col3, 0.0f );
}

static inline void vmathT3AppendScale( VmathTransform3 *result, const VmathTransform3 *tfrm, const VmathVector3 *scaleVec )
{
    vmathV3ScalarMul( &result->col0, &tfrm->col0, vmathV3GetX( scaleVec ) );
    vmathV3ScalarMul( &result->col1, &tfrm->col1, vmathV3GetY( scaleVec ) );
    vmathV3ScalarMul( &result->col2, &tfrm->col2, vmathV3GetZ( scaleVec ) );
    vmathV3Copy( &result->col3, &tfrm->col3 );
}

static inline void vmathT3PrependScale( VmathTransform3 *result, const VmathVector3 *scaleVec, const VmathTransform3 *tfrm )
{
    vmathV3MulPerElem( &result->col0, &tfrm->col0, scaleVec );
    vmathV3MulPerElem( &result->col1, &tfrm->col1, scaleVec );
    vmathV3MulPerElem( &result->col2, &tfrm->col2, scaleVec );
    vmathV3MulPerElem( &result->col3, &tfrm->col3, scaleVec );
}

static inline void vmathT3MakeTranslation( VmathTransform3 *result, const VmathVector3 *translateVec )
{
    vmathV3MakeXAxis( &result->col0 );
    vmathV3MakeYAxis( &result->col1 );
    vmathV3MakeZAxis( &result->col2 );
    vmathV3Copy( &result->col3, translateVec );
}

static inline void vmathT3Select( VmathTransform3 *result, const VmathTransform3 *tfrm0, const VmathTransform3 *tfrm1, unsigned int select1 )
{
    vmathV3Select( &result->col0, &tfrm0->col0, &tfrm1->col0, select1 );
    vmathV3Select( &result->col1, &tfrm0->col1, &tfrm1->col1, select1 );
    vmathV3Select( &result->col2, &tfrm0->col2, &tfrm1->col2, select1 );
    vmathV3Select( &result->col3, &tfrm0->col3, &tfrm1->col3, select1 );
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathT3Print( const VmathTransform3 *tfrm )
{
    VmathVector4 tmpV4_0, tmpV4_1, tmpV4_2;
    vmathT3GetRow( &tmpV4_0, tfrm, 0 );
    vmathV4Print( &tmpV4_0 );
    vmathT3GetRow( &tmpV4_1, tfrm, 1 );
    vmathV4Print( &tmpV4_1 );
    vmathT3GetRow( &tmpV4_2, tfrm, 2 );
    vmathV4Print( &tmpV4_2 );
}

static inline void vmathT3Prints( const VmathTransform3 *tfrm, const char *name )
{
    printf("%s:\n", name);
    vmathT3Print( tfrm );
}

#endif

static inline void vmathQMakeFromM3( VmathQuat *result, const VmathMatrix3 *tfrm )
{
    float trace, radicand, scale, xx, yx, zx, xy, yy, zy, xz, yz, zz, tmpx, tmpy, tmpz, tmpw, qx, qy, qz, qw;
    int negTrace, ZgtX, ZgtY, YgtX;
    int largestXorY, largestYorZ, largestZorX;

    xx = tfrm->col0.x;
    yx = tfrm->col0.y;
    zx = tfrm->col0.z;
    xy = tfrm->col1.x;
    yy = tfrm->col1.y;
    zy = tfrm->col1.z;
    xz = tfrm->col2.x;
    yz = tfrm->col2.y;
    zz = tfrm->col2.z;

    trace = ( ( xx + yy ) + zz );

    negTrace = ( trace < 0.0f );
    ZgtX = zz > xx;
    ZgtY = zz > yy;
    YgtX = yy > xx;
    largestXorY = ( !ZgtX || !ZgtY ) && negTrace;
    largestYorZ = ( YgtX || ZgtX ) && negTrace;
    largestZorX = ( ZgtY || !YgtX ) && negTrace;
    
    if ( largestXorY )
    {
        zz = -zz;
        xy = -xy;
    }
    if ( largestYorZ )
    {
        xx = -xx;
        yz = -yz;
    }
    if ( largestZorX )
    {
        yy = -yy;
        zx = -zx;
    }

    radicand = ( ( ( xx + yy ) + zz ) + 1.0f );
    scale = ( 0.5f * ( 1.0f / sqrtf( radicand ) ) );

    tmpx = ( ( zy - yz ) * scale );
    tmpy = ( ( xz - zx ) * scale );
    tmpz = ( ( yx - xy ) * scale );
    tmpw = ( radicand * scale );
    qx = tmpx;
    qy = tmpy;
    qz = tmpz;
    qw = tmpw;

    if ( largestXorY )
    {
        qx = tmpw;
        qy = tmpz;
        qz = tmpy;
        qw = tmpx;
    }
    if ( largestYorZ )
    {
        tmpx = qx;
        tmpz = qz;
        qx = qy;
        qy = tmpx;
        qz = qw;
        qw = tmpz;
    }

    result->x = qx;
    result->y = qy;
    result->z = qz;
    result->w = qw;
}

static inline void vmathV3Outer( VmathMatrix3 *result, const VmathVector3 *tfrm0, const VmathVector3 *tfrm1 )
{
    vmathV3ScalarMul( &result->col0, tfrm0, vmathV3GetX( tfrm1 ) );
    vmathV3ScalarMul( &result->col1, tfrm0, vmathV3GetY( tfrm1 ) );
    vmathV3ScalarMul( &result->col2, tfrm0, vmathV3GetZ( tfrm1 ) );
}

static inline void vmathV4Outer( VmathMatrix4 *result, const VmathVector4 *tfrm0, const VmathVector4 *tfrm1 )
{
    vmathV4ScalarMul( &result->col0, tfrm0, vmathV4GetX( tfrm1 ) );
    vmathV4ScalarMul( &result->col1, tfrm0, vmathV4GetY( tfrm1 ) );
    vmathV4ScalarMul( &result->col2, tfrm0, vmathV4GetZ( tfrm1 ) );
    vmathV4ScalarMul( &result->col3, tfrm0, vmathV4GetW( tfrm1 ) );
}

static inline void vmathV3RowMul( VmathVector3 *result, const VmathVector3 *vec, const VmathMatrix3 *mat )
{
    float tmpX, tmpY, tmpZ;
    tmpX = ( ( ( vec->x * mat->col0.x ) + ( vec->y * mat->col0.y ) ) + ( vec->z * mat->col0.z ) );
    tmpY = ( ( ( vec->x * mat->col1.x ) + ( vec->y * mat->col1.y ) ) + ( vec->z * mat->col1.z ) );
    tmpZ = ( ( ( vec->x * mat->col2.x ) + ( vec->y * mat->col2.y ) ) + ( vec->z * mat->col2.z ) );
    vmathV3MakeFromElems( result, tmpX, tmpY, tmpZ );
}

static inline void vmathV3CrossMatrix( VmathMatrix3 *result, const VmathVector3 *vec )
{
    vmathV3MakeFromElems( &result->col0, 0.0f, vec->z, -vec->y );
    vmathV3MakeFromElems( &result->col1, -vec->z, 0.0f, vec->x );
    vmathV3MakeFromElems( &result->col2, vec->y, -vec->x, 0.0f );
}

static inline void vmathV3CrossMatrixMul( VmathMatrix3 *result, const VmathVector3 *vec, const VmathMatrix3 *mat )
{
    VmathVector3 tmpV3_0, tmpV3_1, tmpV3_2;
    vmathV3Cross( &tmpV3_0, vec, &mat->col0 );
    vmathV3Cross( &tmpV3_1, vec, &mat->col1 );
    vmathV3Cross( &tmpV3_2, vec, &mat->col2 );
    vmathM3MakeFromCols( result, &tmpV3_0, &tmpV3_1, &tmpV3_2 );
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
