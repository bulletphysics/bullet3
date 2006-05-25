/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FMath/FMath.h"
#include "FMath/FMVector3.h"
#include "FMath/FMQuaternion.h"

// [Glaforte 03-08-2006] VERY EXPERIMENTAL CODE: DON'T USE.

FMQuaternion::FMQuaternion(const FMVector3& axis, float angle)
{
	float s = sinf(angle / 2.0f);
	x = axis.x * s;
	y = axis.y * s;
	z = axis.z * s;
	w = cosf(angle / 2.0f);
}

FMQuaternion FMQuaternion::operator*(const FMQuaternion& q) const
{
	FMQuaternion r;
	r.w = w * q.w - x * q.x - y * q.y - z * q.z;
	r.x = w * q.x + x * q.w + y * q.z - z * q.y;
	r.y = w * q.y + y * q.w + z * q.x - x * q.z;
	r.z = w * q.z + z * q.w + x * q.y - y * q.x;
	return r;
}

FMQuaternion FMQuaternion::EulerRotationQuaternion(float x, float y, float z)
{
	FMQuaternion qx(FMVector3::XAxis, x);
	FMQuaternion qy(FMVector3::YAxis, y);
	FMQuaternion qz(FMVector3::ZAxis, z);
	return qx * qy * qz;
}
