#ifndef B3_OVERLAPPING_PAIR_H
#define B3_OVERLAPPING_PAIR_H

#include "Bullet3Common/btInt2.h"

//typedef btInt2 btBroadphasePair;
struct btBroadphasePair : public btInt2
{
	explicit btBroadphasePair(){}
	btBroadphasePair(int xx,int yy)
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

class btBroadphasePairSortPredicate
{
	public:

		bool operator() ( const btBroadphasePair& a, const btBroadphasePair& b ) const
		{
			const int uidA0 = a.x;
			const int uidB0 = b.x;
			const int uidA1 = a.y;
			const int uidB1 = b.y;
			return uidA0 > uidB0 || (uidA0 == uidB0 && uidA1 > uidB1); 
		}
};

SIMD_FORCE_INLINE bool operator==(const btBroadphasePair& a, const btBroadphasePair& b) 
{
	 return (a.x == b.x ) && (a.y == b.y );
}

#endif //B3_OVERLAPPING_PAIR_H

