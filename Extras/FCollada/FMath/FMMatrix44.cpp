/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FMath/FMMatrix44.h"

static float __identity[] = { 1, 0, 0, 0, 0, 1, 0 ,0 ,0, 0, 1, 0, 0, 0, 0, 1 };
FMMatrix44 FMMatrix44::Identity(__identity);

FMMatrix44::FMMatrix44(const float* _m)
{
	m[0][0] = _m[0]; m[1][0] = _m[1]; m[2][0] = _m[2]; m[3][0] = _m[3];
	m[0][1] = _m[4]; m[1][1] = _m[5]; m[2][1] = _m[6]; m[3][1] = _m[7];
	m[0][2] = _m[8]; m[1][2] = _m[9]; m[2][2] = _m[10]; m[3][2] = _m[11];
	m[0][3] = _m[12]; m[1][3] = _m[13]; m[2][3] = _m[14]; m[3][3] = _m[15];
}

FMMatrix44& FMMatrix44::operator=(const FMMatrix44& copy)
{
	m[0][0] = copy.m[0][0]; m[0][1] = copy.m[0][1]; m[0][2] = copy.m[0][2]; m[0][3] = copy.m[0][3];
	m[1][0] = copy.m[1][0]; m[1][1] = copy.m[1][1]; m[1][2] = copy.m[1][2]; m[1][3] = copy.m[1][3];
	m[2][0] = copy.m[2][0]; m[2][1] = copy.m[2][1]; m[2][2] = copy.m[2][2]; m[2][3] = copy.m[2][3];
	m[3][0] = copy.m[3][0]; m[3][1] = copy.m[3][1]; m[3][2] = copy.m[3][2]; m[3][3] = copy.m[3][3];
	return *this;
}

// Returns the transpose of this matrix
FMMatrix44 FMMatrix44::Transposed() const
{
	FMMatrix44 mx;
	mx.m[0][0] = m[0][0]; mx.m[1][0] = m[0][1]; mx.m[2][0] = m[0][2]; mx.m[3][0] = m[0][3];
	mx.m[0][1] = m[1][0]; mx.m[1][1] = m[1][1]; mx.m[2][1] = m[1][2]; mx.m[3][1] = m[1][3];
	mx.m[0][2] = m[2][0]; mx.m[1][2] = m[2][1]; mx.m[2][2] = m[2][2]; mx.m[3][2] = m[2][3];
	mx.m[0][3] = m[3][0]; mx.m[1][3] = m[3][1]; mx.m[2][3] = m[3][2]; mx.m[3][3] = m[3][3];
	return mx;
}

FMVector3 FMMatrix44::TransformCoordinate(const FMVector3& coordinate) const
{
	FMVector3 out;
	out.x = m[0][0] * coordinate.x + m[1][0] * coordinate.y + m[2][0] * coordinate.z + m[3][0];
	out.y = m[0][1] * coordinate.x + m[1][1] * coordinate.y + m[2][1] * coordinate.z + m[3][1];
	out.z = m[0][2] * coordinate.x + m[1][2] * coordinate.y + m[2][2] * coordinate.z + m[3][2];
	return out;
}

FMVector3 FMMatrix44::TransformVector(const FMVector3& v) const
{
	FMVector3 out;
	out.x = m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z;
	out.y = m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z;
	out.z = m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z;
	return out;
}

FMVector3 FMMatrix44::GetTranslation() const
{
	return FMVector3(m[3][0], m[3][1], m[3][2]);
}

static float det3x3(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3);

void FMMatrix44::Decompose(FMVector3& scale, FMVector3& rotation, FMVector3& translation, float& inverted) const
{
	// Assume Scale/Rotation/Translation, in this order.
    scale.x = sqrtf(m[0][0] * m[0][0] + m[1][0] * m[1][0] + m[2][0] * m[2][0]);
    scale.y = sqrtf(m[0][1] * m[0][1] + m[1][1] * m[1][1] + m[2][1] * m[2][1]);
    scale.z = sqrtf(m[0][2] * m[0][2] + m[1][2] * m[1][2] + m[2][2] * m[2][2]);

	inverted = FMath::Sign(det3x3(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]));

	// Calculate the rotation in Y first, using m[0][2], checking for out-of-bounds values
	float c;
	if (m[0][2] / scale.x >= 1.0f - FLT_TOLERANCE)
	{
		rotation.y = ((float) FMath::Pi) / -2.0f;
		c = 0.0f;
	}
	else if (m[0][2] / scale.x <= -1.0f + FLT_TOLERANCE)
	{
		rotation.y = ((float) FMath::Pi) / 2.0f;
		c = 0.0f;
	}
	else
	{
		rotation.y = asinf(-m[0][2] / scale.x);
		c = cosf(rotation.y);
	}

	// Using the cosine of the Y rotation will give us the rotation in X and Z.
	// Check for the infamous Gimbal Lock.
	if (fabsf(c) > 0.01f) 
	{
		float rx = m[2][2] / scale.z / c;
		float ry = m[1][2] / scale.z / c;
		rotation.x = atan2f(ry, rx);
		rx = m[0][0] / scale.x / c;
		ry = m[0][1] / scale.y / c;
		rotation.z = atan2f(ry, rx);
	}
	else
	{
		rotation.z = 0;
		float rx = m[1][1] / scale.y;
		float ry = -m[1][0] / scale.y;
		rotation.x = atan2f(ry, rx);
	}

	translation = GetTranslation();
}

// Code taken and adapted from nVidia's nv_algebra: det2x2, det3x3, invert, multiply
// -----
// Calculate the determinant of a 2x2 matrix
static float det2x2(float a1, float a2, float b1, float b2)
{
    return a1 * b2 - b1 * a2;
}

// Calculate the determinent of a 3x3 matrix
static float det3x3(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3)
{
    return a1 * det2x2(b2, b3, c2, c3) - b1 * det2x2(a2, a3, c2, c3) + c1 * det2x2(a2, a3, b2, b3);
}

// Returns the inverse of this matrix
FMMatrix44 FMMatrix44::Inverted() const
{
	FMMatrix44 b;

	b.m[0][0] =  det3x3(m[1][1], m[1][2], m[1][3], m[2][1], m[2][2], m[2][3], m[3][1], m[3][2], m[3][3]);
	b.m[0][1] = -det3x3(m[0][1], m[0][2], m[0][3], m[2][1], m[2][2], m[2][3], m[3][1], m[3][2], m[3][3]);
	b.m[0][2] =  det3x3(m[0][1], m[0][2], m[0][3], m[1][1], m[1][2], m[1][3], m[3][1], m[3][2], m[3][3]);
	b.m[0][3] = -det3x3(m[0][1], m[0][2], m[0][3], m[1][1], m[1][2], m[1][3], m[2][1], m[2][2], m[2][3]);

	b.m[1][0] = -det3x3(m[1][0], m[1][2], m[1][3], m[2][0], m[2][2], m[2][3], m[3][0], m[3][2], m[3][3]);
	b.m[1][1] =  det3x3(m[0][0], m[0][2], m[0][3], m[2][0], m[2][2], m[2][3], m[3][0], m[3][2], m[3][3]);
	b.m[1][2] = -det3x3(m[0][0], m[0][2], m[0][3], m[1][0], m[1][2], m[1][3], m[3][0], m[3][2], m[3][3]);
	b.m[1][3] =  det3x3(m[0][0], m[0][2], m[0][3], m[1][0], m[1][2], m[1][3], m[2][0], m[2][2], m[2][3]);

	b.m[2][0] =  det3x3(m[1][0], m[1][1], m[1][3], m[2][0], m[2][1], m[2][3], m[3][0], m[3][1], m[3][3]);
	b.m[2][1] = -det3x3(m[0][0], m[0][1], m[0][3], m[2][0], m[2][1], m[2][3], m[3][0], m[3][1], m[3][3]);
	b.m[2][2] =  det3x3(m[0][0], m[0][1], m[0][3], m[1][0], m[1][1], m[1][3], m[3][0], m[3][1], m[3][3]);
	b.m[2][3] = -det3x3(m[0][0], m[0][1], m[0][3], m[1][0], m[1][1], m[1][3], m[2][0], m[2][1], m[2][3]);

	b.m[3][0] = -det3x3(m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2], m[3][0], m[3][1], m[3][2]);
	b.m[3][1] =  det3x3(m[0][0], m[0][1], m[0][2], m[2][0], m[2][1], m[2][2], m[3][0], m[3][1], m[3][2]);
	b.m[3][2] = -det3x3(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[3][0], m[3][1], m[3][2]);
	b.m[3][3] =  det3x3(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]);

	float det = (m[0][0] * b.m[0][0]) + (m[1][0] * b.m[0][1]) + (m[2][0] * b.m[0][2]) + (m[3][0] * b.m[0][3]);

	if (IsEquivalent(det, 0.0f)) det = 0.01f;
	float oodet = 1.0f / det;

	b.m[0][0] *= oodet;
	b.m[0][1] *= oodet;
	b.m[0][2] *= oodet;
	b.m[0][3] *= oodet;

	b.m[1][0] *= oodet;
	b.m[1][1] *= oodet;
	b.m[1][2] *= oodet;
	b.m[1][3] *= oodet;

	b.m[2][0] *= oodet;
	b.m[2][1] *= oodet;
	b.m[2][2] *= oodet;
	b.m[2][3] *= oodet;

	b.m[3][0] *= oodet;
	b.m[3][1] *= oodet;
	b.m[3][2] *= oodet;
	b.m[3][3] *= oodet;

	return b;
}

FMMatrix44 operator*(const FMMatrix44& m1, const FMMatrix44& m2)
{
    FMMatrix44 mx;
    mx.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[1][0] * m2.m[0][1] + m1.m[2][0] * m2.m[0][2] + m1.m[3][0] * m2.m[0][3];
    mx.m[0][1] = m1.m[0][1] * m2.m[0][0] + m1.m[1][1] * m2.m[0][1] + m1.m[2][1] * m2.m[0][2] + m1.m[3][1] * m2.m[0][3];
    mx.m[0][2] = m1.m[0][2] * m2.m[0][0] + m1.m[1][2] * m2.m[0][1] + m1.m[2][2] * m2.m[0][2] + m1.m[3][2] * m2.m[0][3];
    mx.m[0][3] = m1.m[0][3] * m2.m[0][0] + m1.m[1][3] * m2.m[0][1] + m1.m[2][3] * m2.m[0][2] + m1.m[3][3] * m2.m[0][3];
    mx.m[1][0] = m1.m[0][0] * m2.m[1][0] + m1.m[1][0] * m2.m[1][1] + m1.m[2][0] * m2.m[1][2] + m1.m[3][0] * m2.m[1][3];
    mx.m[1][1] = m1.m[0][1] * m2.m[1][0] + m1.m[1][1] * m2.m[1][1] + m1.m[2][1] * m2.m[1][2] + m1.m[3][1] * m2.m[1][3];
    mx.m[1][2] = m1.m[0][2] * m2.m[1][0] + m1.m[1][2] * m2.m[1][1] + m1.m[2][2] * m2.m[1][2] + m1.m[3][2] * m2.m[1][3];
    mx.m[1][3] = m1.m[0][3] * m2.m[1][0] + m1.m[1][3] * m2.m[1][1] + m1.m[2][3] * m2.m[1][2] + m1.m[3][3] * m2.m[1][3];
    mx.m[2][0] = m1.m[0][0] * m2.m[2][0] + m1.m[1][0] * m2.m[2][1] + m1.m[2][0] * m2.m[2][2] + m1.m[3][0] * m2.m[2][3];
    mx.m[2][1] = m1.m[0][1] * m2.m[2][0] + m1.m[1][1] * m2.m[2][1] + m1.m[2][1] * m2.m[2][2] + m1.m[3][1] * m2.m[2][3];
    mx.m[2][2] = m1.m[0][2] * m2.m[2][0] + m1.m[1][2] * m2.m[2][1] + m1.m[2][2] * m2.m[2][2] + m1.m[3][2] * m2.m[2][3];
    mx.m[2][3] = m1.m[0][3] * m2.m[2][0] + m1.m[1][3] * m2.m[2][1] + m1.m[2][3] * m2.m[2][2] + m1.m[3][3] * m2.m[2][3];
    mx.m[3][0] = m1.m[0][0] * m2.m[3][0] + m1.m[1][0] * m2.m[3][1] + m1.m[2][0] * m2.m[3][2] + m1.m[3][0] * m2.m[3][3];
    mx.m[3][1] = m1.m[0][1] * m2.m[3][0] + m1.m[1][1] * m2.m[3][1] + m1.m[2][1] * m2.m[3][2] + m1.m[3][1] * m2.m[3][3];
    mx.m[3][2] = m1.m[0][2] * m2.m[3][0] + m1.m[1][2] * m2.m[3][1] + m1.m[2][2] * m2.m[3][2] + m1.m[3][2] * m2.m[3][3];
    mx.m[3][3] = m1.m[0][3] * m2.m[3][0] + m1.m[1][3] * m2.m[3][1] + m1.m[2][3] * m2.m[3][2] + m1.m[3][3] * m2.m[3][3];
    return mx;
}

FMMatrix44 FMMatrix44::TranslationMatrix(const FMVector3& translation)
{
	FMMatrix44 matrix;
	matrix[0][0] = 1.0f; matrix[0][1] = 0.0f; matrix[0][2] = 0.0f; matrix[0][3] = 0.0f;
	matrix[1][0] = 0.0f; matrix[1][1] = 1.0f; matrix[1][2] = 0.0f; matrix[1][3] = 0.0f;
	matrix[2][0] = 0.0f; matrix[2][1] = 0.0f; matrix[2][2] = 1.0f; matrix[2][3] = 0.0f;
	matrix[3][0] = translation.x; matrix[3][1] = translation.y; matrix[3][2] = translation.z; matrix[3][3] = 1.0f;
	return matrix;
}

FMMatrix44 FMMatrix44::AxisRotationMatrix(const FMVector3& axis, float angle)
{
	// Formulae comes from http://www.mines.edu/~gmurray/ArbitraryAxisRotation/ArbitraryAxisRotation.html
	FMMatrix44 matrix;
	FMVector3 a = (IsEquivalent(axis.LengthSquared(), 1.0f)) ? axis : axis.Normalize();
	float xSq = a.x * a.x;
	float ySq = a.y * a.y;
	float zSq = a.z * a.z;
	float cT = cosf(angle);
	float sT = sinf(angle);

	matrix[0][0] = xSq + (ySq + zSq) * cT;
	matrix[0][1] = a.x * a.y * (1.0f - cT) + a.z * sT;
	matrix[0][2] = a.x * a.z * (1.0f - cT) - a.y * sT;
	matrix[0][3] = 0;
	matrix[1][0] = a.x * a.y * (1.0f - cT) - a.z * sT;
	matrix[1][1] = ySq + (xSq + zSq) * cT;
	matrix[1][2] = a.y * a.z * (1.0f - cT) + a.x * sT;
	matrix[1][3] = 0;
	matrix[2][0] = a.x * a.z * (1.0f - cT) + a.y * sT;
	matrix[2][1] = a.y * a.z * (1.0f - cT) - a.x * sT;
	matrix[2][2] = zSq + (xSq + ySq) * cT;
	matrix[2][3] = 0;
	matrix[3][2] = matrix[3][1] = matrix[3][0] = 0;
	matrix[3][3] = 1;
	return matrix;
}

bool IsEquivalent(const FMMatrix44& m1, const FMMatrix44& m2)
{
	return IsEquivalent(m1.m[0][0], m2.m[0][0]) && IsEquivalent(m1.m[1][0], m2.m[1][0])
		&& IsEquivalent(m1.m[2][0], m2.m[2][0]) && IsEquivalent(m1.m[3][0], m2.m[3][0])
		&& IsEquivalent(m1.m[0][1], m2.m[0][1]) && IsEquivalent(m1.m[1][1], m2.m[1][1])
		&& IsEquivalent(m1.m[2][1], m2.m[2][1]) && IsEquivalent(m1.m[3][1], m2.m[3][1])
		&& IsEquivalent(m1.m[0][2], m2.m[0][2]) && IsEquivalent(m1.m[1][2], m2.m[1][2])
		&& IsEquivalent(m1.m[2][2], m2.m[2][2]) && IsEquivalent(m1.m[3][2], m2.m[3][2])
		&& IsEquivalent(m1.m[0][3], m2.m[0][3]) && IsEquivalent(m1.m[1][3], m2.m[1][3])
		&& IsEquivalent(m1.m[2][3], m2.m[2][3]) && IsEquivalent(m1.m[3][3], m2.m[3][3]);
}
