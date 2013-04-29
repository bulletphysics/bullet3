
#ifndef B3_BROADPHASE_CALLBACK_H
#define B3_BROADPHASE_CALLBACK_H

#include "Bullet3Common/b3Vector3.h"
struct b3BroadphaseProxy;


struct	b3BroadphaseAabbCallback
{
	virtual ~b3BroadphaseAabbCallback() {}
	virtual bool	process(const b3BroadphaseProxy* proxy) = 0;
};


struct	b3BroadphaseRayCallback : public b3BroadphaseAabbCallback
{
	///added some cached data to accelerate ray-AABB tests
	b3Vector3		m_rayDirectionInverse;
	unsigned int	m_signs[3];
	b3Scalar		m_lambda_max;

	virtual ~b3BroadphaseRayCallback() {}
};

#endif //B3_BROADPHASE_CALLBACK_H
