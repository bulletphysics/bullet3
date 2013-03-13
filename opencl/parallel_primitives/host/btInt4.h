#ifndef BT_INT4_H
#define BT_INT4_H

#include "btScalar.h"

ATTRIBUTE_ALIGNED16(struct) btUnsignedInt4
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	union
	{
		struct
		{
			unsigned int x,y,z,w;
		};
		struct
		{
			unsigned int s[4];
		};
	};
};

ATTRIBUTE_ALIGNED16(struct) btInt4
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	union
	{
		struct
		{
			int x,y,z,w;
		};
		struct
		{
			int s[4];
		};
	};
};

SIMD_FORCE_INLINE btInt4 btMakeInt4(int x, int y, int z, int w = 0)
{
	btInt4 v;
	v.s[0] = x; v.s[1] = y; v.s[2] = z; v.s[3] = w;
	return v;
}

SIMD_FORCE_INLINE btUnsignedInt4 btMakeUnsignedInt4(unsigned int x, unsigned int y, unsigned int z, unsigned int w = 0)
{
	btUnsignedInt4 v;
	v.s[0] = x; v.s[1] = y; v.s[2] = z; v.s[3] = w;
	return v;
}


#endif //BT_INT4_H
