#ifndef B3_SAP_AABB_H
#define B3_SAP_AABB_H

#include "Bullet3Common/b3Scalar.h"

B3_ATTRIBUTE_ALIGNED16(struct) b3SapAabb
{
	union
	{
		float m_min[4];
		int m_minIndices[4];
	};
	union
	{
		float m_max[4];
		int m_signedMaxIndices[4];
	};
};

#endif //B3_SAP_AABB_H
