#ifndef B3_OVERLAPPING_PAIR_H
#define B3_OVERLAPPING_PAIR_H

#include "Bullet3Common/b3Int2.h"

//typedef b3Int2 b3BroadphasePair;
struct b3BroadphasePair : public b3Int2
{
	explicit b3BroadphasePair(){}
	b3BroadphasePair(int xx,int yy)
	{
		if (xx < yy)
        { 
            x = xx; 
            y = yy;
        }
        else 
        { 
			x = yy;
            y = xx;
        }
	}
};

class b3BroadphasePairSortPredicate
{
	public:

		bool operator() ( const b3BroadphasePair& a, const b3BroadphasePair& b ) const
		{
			const int uidA0 = a.x;
			const int uidB0 = b.x;
			const int uidA1 = a.y;
			const int uidB1 = b.y;
			return uidA0 > uidB0 || (uidA0 == uidB0 && uidA1 > uidB1); 
		}
};

SIMD_FORCE_INLINE bool operator==(const b3BroadphasePair& a, const b3BroadphasePair& b) 
{
	 return (a.x == b.x ) && (a.y == b.y );
}

#endif //B3_OVERLAPPING_PAIR_H

