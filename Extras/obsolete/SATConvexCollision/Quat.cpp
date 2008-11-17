// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Quat.cpp
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

#include "Quat.h"
#include "Maths.h"

bool Quat::IsFinite() const
{
	if (_finite(GetX()) && _finite(GetY()) && _finite(GetZ()) && _finite(GetW()))
		return true;
	return false;
}

Quat::Quat(const Matrix33& m)
{
	float tr, s, q[4];
	int i, j, k;
	int nxt[3] = {1, 2, 0};
	float x, y, z, w;

	// Check the diagonal
	tr = m[0][0] + m[1][1] + m[2][2];
	if (tr >= 0.0f)
	{
		// Diagonal is positive
		s = ::Sqrt(tr + 1.0f);
		w = s * 0.5f;
		s = 0.5f / s;
		x = (m[1][2] - m[2][1]) * s;
		y = (m[2][0] - m[0][2]) * s;
		z = (m[0][1] - m[1][0]) * s;
	}
	else
	{
		// Diagonal is negative
		i = 0;
		if (m[1][1] > m[0][0]) i = 1;
		if (m[2][2] > m[i][i]) i = 2;
		j = nxt[i];
		k = nxt[j];

		s = ::Sqrt((m[i][i] - (m[j][j] + m[k][k])) + 1.0f);
		q[i] = s * 0.5f;
		if (s != 0.0f) s = 0.5f / s;

		q[3] = (m[j][k] - m[k][j]) * s;
		q[j] = (m[i][j] + m[j][i]) * s;
		q[k] = (m[i][k] + m[k][i]) * s;

		x = q[0];
		y = q[1];
		z = q[2];
		w = q[3];
	}

	*this = Quat(x, y, z, w);
}

const Quat Slerp(const Quat& a, const Quat& b, const Scalar& t)
{
	Quat e;
	Scalar cosom, t0, t1;
	
	cosom = Dot(a, b);

	if (cosom < Scalar::Consts::Zero)
	{
		cosom = -cosom;
		e = -b;
	}
	else
		e = b;

	if (cosom < 0.9999f)
	{
		float omega = ::Acos(cosom);
		Scalar rcpSinom = Rcp(Scalar(::Sin(omega)));
		t0 = Scalar(::Sin((1.0f - (float)t) * omega)) * rcpSinom;
		t1 = Scalar(::Sin((float)t * omega)) * rcpSinom;
	}
	else
	{
		t0 = Scalar(1.0f) - t;
		t1 = t;
	}

	return a * t0 + e * t;
}

#endif 
#endif //#ifdef WIN32
