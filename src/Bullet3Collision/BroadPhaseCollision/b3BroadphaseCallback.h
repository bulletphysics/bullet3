
#ifndef B3_BROADPHASE_CALLBACK_H
#define B3_BROADPHASE_CALLBACK_H

#include "Bullet3Common/b3Vector3.h"
struct btBroadphaseProxy;


struct	btBroadphaseAabbCallback
{
	virtual ~btBroadphaseAabbCallback() {}
	virtual bool	process(const btBroadphaseProxy* proxy) = 0;
};


struct	btBroadphaseRayCallback : public btBroadphaseAabbCallback
{
	///added some cached data to accelerate ray-AABB tests
	b3Vector3		m_rayDirectionInverse;
	unsigned int	m_signs[3];
	b3Scalar		m_lambda_max;

	virtual ~btBroadphaseRayCallback() {}
};

#endif //B3_BROADPHASE_CALLBACK_H
