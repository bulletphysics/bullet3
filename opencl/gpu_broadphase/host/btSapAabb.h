#ifndef BT_SAP_AABB_H
#define BT_SAP_AABB_H

struct btSapAabb
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

#endif //BT_SAP_AABB_H
