// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Matrix.cpp
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.

#ifdef WIN32
#if _MSC_VER >= 1310

#include "matrix.h"
#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
// Matrix33
Matrix33::Matrix33(const Quat& q)
{
	float wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;
	float *pIn = (float*)&q;
	float *pOut = (float*)this;

	float x = pIn[0], y = pIn[1], z = pIn[2], w = pIn[3];

	x2 = x + x; y2 = y + y;	z2 = z + z;
	xx = x * x2; yy = y * y2; zz = z * z2;
	wx = w * x2; wy = w * y2; wz = w * z2;
	xy = x * y2; xz = x * z2;
	yz = y * z2;

	pOut[0] = 1.0f - (yy + zz);
	pOut[1] = xy + wz;
	pOut[2] = xz - wy;
	pOut[3] = 0.0f;

	pOut[4] = xy - wz;
	pOut[5] = 1.0f - (xx + zz);
	pOut[6] = yz + wx;
	pOut[7] = 0.0f;

	pOut[8] = xz + wy;
	pOut[9] = yz - wx;
	pOut[10] = 1.0f - (xx + yy);
	pOut[11] = 0.0f;
}

const Matrix33 Inv(const Matrix33& m)
{
	// TODO: do this efficiently - for now we just use the Matrix44 version
	Matrix44 m44 = Inv(Matrix44(Vector4(m[0]), Vector4(m[1]), Vector4(m[2]), Vector4(Maths::UnitW)));
	return Matrix33(Vector3(m44[0]), Vector3(m44[1]), Vector3(m44[2]));
}


////////////////////////////////////////////////////////////////////////////////
// Matrix44

// calculate the inverse of a general 4x4 matrix
//
//     -1
//     A  = ___1__ adjoint A
//         det A
//
const Matrix44 Inv(const Matrix44& src)
{
	__m128	minor0, minor1, minor2, minor3;
	__m128	row0,   row1 = _mm_set_ps1(0.0f),   row2,   row3 = row1;
	__m128	det,    tmp1 = row1,  tmp2;

	Matrix44 tmp(src);
	float	*m = (float*)&tmp;

	tmp1	= _mm_loadh_pi( _mm_loadl_pi( tmp1, (__m64*)(m  ) ), (__m64*)(m+ 4) );
	row1	= _mm_loadh_pi( _mm_loadl_pi( row1, (__m64*)(m+8) ), (__m64*)(m+12) );

	row0	= _mm_shuffle_ps( tmp1, row1, 0x88 );
	row1	= _mm_shuffle_ps( row1, tmp1, 0xDD );

	tmp1	= _mm_loadh_pi( _mm_loadl_pi( tmp1, (__m64*)(m+ 2) ), (__m64*)(m+ 6) );
	row3	= _mm_loadh_pi( _mm_loadl_pi( row3, (__m64*)(m+10) ), (__m64*)(m+14) );

	row2	= _mm_shuffle_ps( tmp1, row3, 0x88 );
	row3	= _mm_shuffle_ps( row3, tmp1, 0xDD );
//	-----------------------------------------------
	tmp2	= _mm_mul_ps( row2, row3 );
	tmp1	= _mm_shuffle_ps( tmp2, tmp2, 0xB1);

	minor0	= _mm_mul_ps( row1, tmp1 );
	minor1	= _mm_mul_ps( row0, tmp1 );

	tmp1	= _mm_shuffle_ps( tmp1, tmp1, 0x4E );

	minor0	= _mm_sub_ps( _mm_mul_ps( row1, tmp1 ), minor0 );
	minor1	= _mm_sub_ps( _mm_mul_ps( row0, tmp1 ), minor1 );
	minor1	= _mm_shuffle_ps( minor1, minor1, 0x4E );
//	-----------------------------------------------
	tmp2	= _mm_mul_ps( row1, row2);
	tmp1	= _mm_shuffle_ps( tmp2, tmp2, 0xB1 );

	minor0	= _mm_add_ps( _mm_mul_ps( row3, tmp1 ), minor0  );
	minor3	= _mm_mul_ps( row0, tmp1 );

	tmp1	= _mm_shuffle_ps( tmp1, tmp1, 0x4E );

	minor0	= _mm_sub_ps( minor0, _mm_mul_ps( row3, tmp1 ) );
	minor3	= _mm_sub_ps( _mm_mul_ps( row0, tmp1 ), minor3 );
	minor3	= _mm_shuffle_ps( minor3, minor3, 0x4E );
//	-----------------------------------------------
	tmp2	= _mm_mul_ps( _mm_shuffle_ps( row1, row1, 0x4E ), row3 );
	tmp1	= _mm_shuffle_ps( tmp2, tmp2, 0xB1 );
	row2	= _mm_shuffle_ps( row2, row2, 0x4E );

	minor0	= _mm_add_ps( _mm_mul_ps( row2, tmp1 ), minor0 );
	minor2	= _mm_mul_ps( row0, tmp1 );

	tmp1	= _mm_shuffle_ps( tmp1, tmp1, 0x4E );

	minor0	= _mm_sub_ps( minor0, _mm_mul_ps( row2, tmp1 ) );
	minor2	= _mm_sub_ps( _mm_mul_ps( row0, tmp1 ), minor2 );
	minor2	= _mm_shuffle_ps( minor2, minor2, 0x4E );
//	-----------------------------------------------
	tmp2	= _mm_mul_ps( row0, row1);
	tmp1	= _mm_shuffle_ps( tmp2, tmp2, 0xB1 );	

	minor2	= _mm_add_ps( _mm_mul_ps( row3, tmp1 ), minor2 );
	minor3	= _mm_sub_ps( _mm_mul_ps( row2, tmp1 ), minor3 );
	
	tmp1	= _mm_shuffle_ps( tmp1, tmp1, 0x4E );
	
	minor2	= _mm_sub_ps( _mm_mul_ps( row3, tmp1 ), minor2 );
	minor3	= _mm_sub_ps( minor3, _mm_mul_ps( row2, tmp1 ) );
//	-----------------------------------------------
	tmp2	= _mm_mul_ps( row0, row3);
	tmp1	= _mm_shuffle_ps( tmp2, tmp2, 0xB1 );
	
	minor1	= _mm_sub_ps( minor1, _mm_mul_ps( row2, tmp1 ) );
	minor2	= _mm_add_ps( _mm_mul_ps( row1, tmp1 ), minor2 );	
	
	tmp1	= _mm_shuffle_ps( tmp1, tmp1, 0x4E );
	
	minor1	= _mm_add_ps( _mm_mul_ps( row2, tmp1 ), minor1 );
	minor2	= _mm_sub_ps( minor2, _mm_mul_ps( row1, tmp1 ) );	
//	-----------------------------------------------
	tmp2	= _mm_mul_ps( row0, row2);
	tmp1	= _mm_shuffle_ps( tmp2, tmp2, 0xB1 );
	
	minor1	= _mm_add_ps( _mm_mul_ps( row3, tmp1 ), minor1 );
	minor3	= _mm_sub_ps( minor3, _mm_mul_ps( row1, tmp1 ) );	
	
	tmp1	= _mm_shuffle_ps( tmp1, tmp1, 0x4E );
	
	minor1	= _mm_sub_ps( minor1, _mm_mul_ps( row3, tmp1 ) );
	minor3	= _mm_add_ps( _mm_mul_ps( row1, tmp1 ), minor3 );	
//	-----------------------------------------------
	det		= _mm_mul_ps( row0, minor0 );
	det		= _mm_add_ps( _mm_shuffle_ps( det, det, 0x4E ), det );
	det		= _mm_add_ss( _mm_shuffle_ps( det, det, 0xB1 ), det );
	tmp1	= _mm_rcp_ss( det );

	det		= _mm_sub_ss( _mm_add_ss( tmp1, tmp1 ), _mm_mul_ss( det, _mm_mul_ss( tmp1, tmp1 ) ) );
	det		= _mm_shuffle_ps( det, det, 0x00 );

	minor0	= _mm_mul_ps( det, minor0 );
	_mm_storel_pi( (__m64*)( m ), minor0 );
	_mm_storeh_pi( (__m64*)(m+2), minor0 );

	minor1	= _mm_mul_ps( det, minor1 );
	_mm_storel_pi( (__m64*)(m+4), minor1 );
	_mm_storeh_pi( (__m64*)(m+6), minor1 );

	minor2	= _mm_mul_ps( det, minor2 );
	_mm_storel_pi( (__m64*)(m+ 8), minor2 );
	_mm_storeh_pi( (__m64*)(m+10), minor2 );

	minor3	= _mm_mul_ps( det, minor3 );
	_mm_storel_pi( (__m64*)(m+12), minor3 );
	_mm_storeh_pi( (__m64*)(m+14), minor3 );

	return tmp;
}



////////////////////////////////////////////////////////////////////////////////
// Transform



////////////////////////////////////////////////////////////////////////////////
// Matrix66

#endif
#endif //#ifdef WIN32