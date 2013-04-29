/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

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

B3_FORCE_INLINE bool operator==(const b3BroadphasePair& a, const b3BroadphasePair& b) 
{
	 return (a.x == b.x ) && (a.y == b.y );
}

#endif //B3_OVERLAPPING_PAIR_H

