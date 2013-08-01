#ifndef B3_VECTOR_FLOAT4_H
#define B3_VECTOR_FLOAT4_H

#include "Bullet3Common/b3Transform.h"

#define cross3(a,b) (a.cross(b))
#define float4 b3Vector3
#define make_float4(x,y,z,w) b3Vector4(x,y,z,w)

inline b3Vector3 transform(const b3Vector3* v, const b3Vector3* pos, const b3Quaternion* orn)
{
	b3Transform tr;
	tr.setIdentity();
	tr.setOrigin(*pos);
	tr.setRotation(*orn);
	b3Vector3 res = tr(*v);
	return res;
}

#endif //B3_VECTOR_FLOAT4_H
