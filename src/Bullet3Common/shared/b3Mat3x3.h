
#ifndef B3_MAT3x3_H
#define B3_MAT3x3_H

#include "Bullet3Common/shared/b3Quat.h"


#ifdef __cplusplus

#include "Bullet3Common/b3Matrix3x3.h"

#define b3Mat3x3 b3Matrix3x3
#define b3Mat3x3ConstArg const b3Matrix3x3&

inline b3Mat3x3 b3QuatGetRotationMatrix(b3QuatConstArg quat)
{
	return b3Mat3x3(quat);
}

inline b3Mat3x3 b3AbsoluteMat3x3(b3Mat3x3ConstArg mat)
{
	return mat.absolute();
}

#define b3GetRow(m,row) m.getRow(row)

#else

typedef struct
{
	float4 m_row[3];
}b3Mat3x3;

#define b3Mat3x3ConstArg const b3Mat3x3
#define b3GetRow(m,row) (m.m_row[row])

inline b3Mat3x3 b3QuatGetRotationMatrix(b3Quat quat)
{
	float4 quat2 = (float4)(quat.x*quat.x, quat.y*quat.y, quat.z*quat.z, 0.f);
	b3Mat3x3 out;

	out.m_row[0].x=1-2*quat2.y-2*quat2.z;
	out.m_row[0].y=2*quat.x*quat.y-2*quat.w*quat.z;
	out.m_row[0].z=2*quat.x*quat.z+2*quat.w*quat.y;
	out.m_row[0].w = 0.f;

	out.m_row[1].x=2*quat.x*quat.y+2*quat.w*quat.z;
	out.m_row[1].y=1-2*quat2.x-2*quat2.z;
	out.m_row[1].z=2*quat.y*quat.z-2*quat.w*quat.x;
	out.m_row[1].w = 0.f;

	out.m_row[2].x=2*quat.x*quat.z-2*quat.w*quat.y;
	out.m_row[2].y=2*quat.y*quat.z+2*quat.w*quat.x;
	out.m_row[2].z=1-2*quat2.x-2*quat2.y;
	out.m_row[2].w = 0.f;

	return out;
}

inline b3Mat3x3 b3AbsoluteMat3x3(b3Mat3x3ConstArg matIn)
{
	b3Mat3x3 out;
	out.m_row[0] = fabs(matIn.m_row[0]);
	out.m_row[1] = fabs(matIn.m_row[1]);
	out.m_row[2] = fabs(matIn.m_row[2]);
	return out;
}

#endif





#endif //B3_MAT3x3_H
