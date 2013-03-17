
#ifndef BT_CONSTRAINT4_h
#define BT_CONSTRAINT4_h
#include "BulletCommon/btVector3.h"

ATTRIBUTE_ALIGNED16(struct) btGpuConstraint4
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btVector3 m_linear;//normal?
	btVector3 m_worldPos[4];
	btVector3 m_center;	//	friction
	float m_jacCoeffInv[4];
	float m_b[4];
	float m_appliedRambdaDt[4];

	float m_fJacCoeffInv[2];	//	friction
	float m_fAppliedRambdaDt[2];	//	friction

	unsigned int m_bodyA;
	unsigned int m_bodyB;

	unsigned int m_batchIdx;
	unsigned int m_paddings[1];

	inline	void setFrictionCoeff(float value) { m_linear[3] = value; }
	inline	float getFrictionCoeff() const { return m_linear[3]; }
};

#endif //BT_CONSTRAINT4_h

