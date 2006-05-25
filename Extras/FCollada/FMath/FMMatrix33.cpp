/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"

static float __identity[] = { 1, 0, 0, 0, 1, 0 ,0, 0, 1 };
FMMatrix33 FMMatrix33::identity(__identity);

FMMatrix33::FMMatrix33(float* _m)
{
	m[0][0] = _m[0]; m[1][0] = _m[1]; m[2][0] = _m[2];
	m[0][1] = _m[3]; m[1][1] = _m[4]; m[2][1] = _m[5];
	m[0][2] = _m[6]; m[1][2] = _m[7]; m[2][2] = _m[8];
}

FMMatrix33& FMMatrix33::operator=(const FMMatrix33& copy)
{
	m[0][0] = copy.m[0][0]; m[0][1] = copy.m[0][1]; m[0][2] = copy.m[0][2];
	m[1][0] = copy.m[1][0]; m[1][1] = copy.m[1][1]; m[1][2] = copy.m[1][2];
	m[2][0] = copy.m[2][0]; m[2][1] = copy.m[2][1]; m[2][2] = copy.m[2][2];
	return *this;
}

// Returns the transpose of this matrix
FMMatrix33 FMMatrix33::Transposed() const
{
	FMMatrix33 mx;
	mx.m[0][0] = m[0][0]; mx.m[1][0] = m[0][1]; mx.m[2][0] = m[0][2];
	mx.m[0][1] = m[1][0]; mx.m[1][1] = m[1][1]; mx.m[2][1] = m[1][2];
	mx.m[0][2] = m[2][0]; mx.m[1][2] = m[2][1]; mx.m[2][2] = m[2][2];
	return mx;
}

FMMatrix33 FMMatrix33::RotationMatrix(float angle)
{
	FMMatrix33 m(identity);
	float c = cosf(angle), s = sinf(angle);
	m[0][0] = c; m[1][1] = c;
	m[0][1] = -s; m[1][0] = s;
	return m;
}

FMMatrix33 FMMatrix33::TranslationMatrix(float tx, float ty)
{
	FMMatrix33 m(identity);
	m[2][0] = tx; m[2][1] = ty;
	return m;
}

FMMatrix33 FMMatrix33::ScaleMatrix(float sx, float sy)
{
	FMMatrix33 m(identity);
	m[0][0] = sx; m[1][1] = sy;
	return m;
}

// Code taken and adapted from nVidia's nv_algebra: det2x2, invert, multiply
// -----
// Calculate the determinent of a 2x2 matrix
static inline float det2x2(float a1, float a2, float b1, float b2)
{
    return a1 * b2 - b1 * a2;
}

// Returns the inverse of this matrix
FMMatrix33 FMMatrix33::Inverted() const
{
	FMMatrix33 b;

	b.m[0][0] =  det2x2(m[1][1], m[1][2], m[2][1], m[2][2]);
	b.m[0][1] = -det2x2(m[0][1], m[0][2], m[2][1], m[2][2]);
	b.m[0][2] =  det2x2(m[0][1], m[0][2], m[1][1], m[1][2]);

	b.m[1][0] = -det2x2(m[1][0], m[1][2], m[2][0], m[2][2]);
	b.m[1][1] =  det2x2(m[0][0], m[0][2], m[2][0], m[2][2]);
	b.m[1][2] = -det2x2(m[0][0], m[0][2], m[1][0], m[1][2]);

	b.m[2][0] =  det2x2(m[1][0], m[1][1], m[2][0], m[2][1]);
	b.m[2][1] = -det2x2(m[0][0], m[0][1], m[2][0], m[2][1]);
	b.m[2][2] =  det2x2(m[0][0], m[0][1], m[1][0], m[1][1]);

	float det = (m[0][0] * b.m[0][0]) + (m[1][0] * b.m[0][1]) + (m[2][0] * b.m[0][2]);

	// We should consider throwing an exception if det < eps.
	if (IsEquivalent(det, 0.0f)) det = 0.01f;
	float oodet = 1.0f / det;

	b.m[0][0] *= oodet;
	b.m[0][1] *= oodet;
	b.m[0][2] *= oodet;

	b.m[1][0] *= oodet;
	b.m[1][1] *= oodet;
	b.m[1][2] *= oodet;

	b.m[2][0] *= oodet;
	b.m[2][1] *= oodet;
	b.m[2][2] *= oodet;

	return b;
}

FMMatrix33 operator*(const FMMatrix33& m1, const FMMatrix33& m2)
{
    FMMatrix33 mx;
    mx.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[1][0] * m2.m[0][1] + m1.m[2][0] * m2.m[0][2];
    mx.m[0][1] = m1.m[0][1] * m2.m[0][0] + m1.m[1][1] * m2.m[0][1] + m1.m[2][1] * m2.m[0][2];
    mx.m[0][2] = m1.m[0][2] * m2.m[0][0] + m1.m[1][2] * m2.m[0][1] + m1.m[2][2] * m2.m[0][2];
    mx.m[1][0] = m1.m[0][0] * m2.m[1][0] + m1.m[1][0] * m2.m[1][1] + m1.m[2][0] * m2.m[1][2];
    mx.m[1][1] = m1.m[0][1] * m2.m[1][0] + m1.m[1][1] * m2.m[1][1] + m1.m[2][1] * m2.m[1][2];
    mx.m[1][2] = m1.m[0][2] * m2.m[1][0] + m1.m[1][2] * m2.m[1][1] + m1.m[2][2] * m2.m[1][2];
    mx.m[2][0] = m1.m[0][0] * m2.m[2][0] + m1.m[1][0] * m2.m[2][1] + m1.m[2][0] * m2.m[2][2];
    mx.m[2][1] = m1.m[0][1] * m2.m[2][0] + m1.m[1][1] * m2.m[2][1] + m1.m[2][1] * m2.m[2][2];
    mx.m[2][2] = m1.m[0][2] * m2.m[2][0] + m1.m[1][2] * m2.m[2][1] + m1.m[2][2] * m2.m[2][2];
    return mx;
}
