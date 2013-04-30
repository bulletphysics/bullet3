
#ifndef B3_CONSTRAINT4_h
#define B3_CONSTRAINT4_h
#include "Bullet3Common/b3Vector3.h"

B3_ATTRIBUTE_ALIGNED16(struct) b3GpuConstraint4
{
	B3_DECLARE_ALIGNED_ALLOCATOR();

	b3Vector3 m_linear;//normal?
	b3Vector3 m_worldPos[4];
	b3Vector3 m_center;	//	friction
	float m_jacCoeffInv[4];
	float m_b[4];
	float m_appliedRambdaDt[4];
	float m_fJacCoeffInv[2];	//	friction
	float m_fAppliedRambdaDt[2];	//	friction

	unsigned int m_bodyA;
	unsigned int m_bodyB;
	int m_batchIdx;
	unsigned int m_paddings;

	inline	void setFrictionCoeff(float value) { m_linear[3] = value; }
	inline	float getFrictionCoeff() const { return m_linear[3]; }
};

#endif //B3_CONSTRAINT4_h

