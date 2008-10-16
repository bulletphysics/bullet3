/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef AABB_UTIL2
#define AABB_UTIL2

#include "btTransform.h"
#include "btVector3.h"
#include "btMinMax.h"


#define TEST_RAY_SLOPES 1


SIMD_FORCE_INLINE void AabbExpand (btVector3& aabbMin,
								   btVector3& aabbMax,
								   const btVector3& expansionMin,
								   const btVector3& expansionMax)
{
	aabbMin = aabbMin + expansionMin;
	aabbMax = aabbMax + expansionMax;
}

/// conservative test for overlap between two aabbs
SIMD_FORCE_INLINE bool TestPointAgainstAabb2(const btVector3 &aabbMin1, const btVector3 &aabbMax1,
								const btVector3 &point)
{
	bool overlap = true;
	overlap = (aabbMin1.getX() > point.getX() || aabbMax1.getX() < point.getX()) ? false : overlap;
	overlap = (aabbMin1.getZ() > point.getZ() || aabbMax1.getZ() < point.getZ()) ? false : overlap;
	overlap = (aabbMin1.getY() > point.getY() || aabbMax1.getY() < point.getY()) ? false : overlap;
	return overlap;
}


/// conservative test for overlap between two aabbs
SIMD_FORCE_INLINE bool TestAabbAgainstAabb2(const btVector3 &aabbMin1, const btVector3 &aabbMax1,
								const btVector3 &aabbMin2, const btVector3 &aabbMax2)
{
	bool overlap = true;
	overlap = (aabbMin1.getX() > aabbMax2.getX() || aabbMax1.getX() < aabbMin2.getX()) ? false : overlap;
	overlap = (aabbMin1.getZ() > aabbMax2.getZ() || aabbMax1.getZ() < aabbMin2.getZ()) ? false : overlap;
	overlap = (aabbMin1.getY() > aabbMax2.getY() || aabbMax1.getY() < aabbMin2.getY()) ? false : overlap;
	return overlap;
}

/// conservative test for overlap between triangle and aabb
SIMD_FORCE_INLINE bool TestTriangleAgainstAabb2(const btVector3 *vertices,
									const btVector3 &aabbMin, const btVector3 &aabbMax)
{
	const btVector3 &p1 = vertices[0];
	const btVector3 &p2 = vertices[1];
	const btVector3 &p3 = vertices[2];

	if (btMin(btMin(p1[0], p2[0]), p3[0]) > aabbMax[0]) return false;
	if (btMax(btMax(p1[0], p2[0]), p3[0]) < aabbMin[0]) return false;

	if (btMin(btMin(p1[2], p2[2]), p3[2]) > aabbMax[2]) return false;
	if (btMax(btMax(p1[2], p2[2]), p3[2]) < aabbMin[2]) return false;
  
	if (btMin(btMin(p1[1], p2[1]), p3[1]) > aabbMax[1]) return false;
	if (btMax(btMax(p1[1], p2[1]), p3[1]) < aabbMin[1]) return false;
	return true;
}


SIMD_FORCE_INLINE int	btOutcode(const btVector3& p,const btVector3& halfExtent) 
{
	return (p.getX()  < -halfExtent.getX() ? 0x01 : 0x0) |    
		   (p.getX() >  halfExtent.getX() ? 0x08 : 0x0) |
		   (p.getY() < -halfExtent.getY() ? 0x02 : 0x0) |    
		   (p.getY() >  halfExtent.getY() ? 0x10 : 0x0) |
		   (p.getZ() < -halfExtent.getZ() ? 0x4 : 0x0) |    
		   (p.getZ() >  halfExtent.getZ() ? 0x20 : 0x0);
}

/// http://jgt.akpeters.com/papers/EisemannEtAl07/
/// See test case in btDbvt::rayTestInternal on dynamic AABB tree, Bullet/src/BulletCollision/BroadphaseCollision/btDbvt.h


enum CLASSIFICATION
{ MMM, MMP, MPM, MPP, PMM, PMP, PPM, PPP, POO, MOO, OPO, OMO, OOP, OOM,
	OMM,OMP,OPM,OPP,MOM,MOP,POM,POP,MMO,MPO,PMO,PPO};

struct btRaySlope
{	
	//common variables
	float x, y, z;		// ray origin	
	float i, j, k;		// ray direction	
	float ii, ij, ik;	// inverses of direction components
	
	// ray slope
	int classification;
	float ibyj, jbyi, kbyj, jbyk, ibyk, kbyi; //slope
	float c_xy, c_xz, c_yx, c_yz, c_zx, c_zy;	
};

struct btAaboxSlope
{
	float x0, y0, z0, x1, y1, z1;
};

SIMD_FORCE_INLINE void btMakeRaySlope(float x, float y, float z, float i, float j, float k, btRaySlope *r)
{
	//common variables
	r->x = x;
	r->y = y;
	r->z = z;
	r->i = i;
	r->j = j;
	r->k = k;

	r->ii = 1.0f/i;
	r->ij = 1.0f/j;
	r->ik = 1.0f/k;

	//ray slope
	r->ibyj = r->i * r->ij;
	r->jbyi = r->j * r->ii;
	r->jbyk = r->j * r->ik;
	r->kbyj = r->k * r->ij;
	r->ibyk = r->i * r->ik;
	r->kbyi = r->k * r->ii;
	r->c_xy = r->y - r->jbyi * r->x;
	r->c_xz = r->z - r->kbyi * r->x;
	r->c_yx = r->x - r->ibyj * r->y;
	r->c_yz = r->z - r->kbyj * r->y;
	r->c_zx = r->x - r->ibyk * r->z;
	r->c_zy = r->y - r->jbyk * r->z;	

	//ray slope classification
	if(i < 0)
	{
		if(j < 0)
		{
			if(k < 0)
			{
				r->classification = MMM;
			}
			else if(k > 0){
				r->classification = MMP;
			}
			else//(k >= 0)
			{
				r->classification = MMO;
			}
		}
		else//(j >= 0)
		{
			if(k < 0)
			{
				r->classification = MPM;
				if(j==0)
					r->classification = MOM;
			}
			else//(k >= 0)
			{
				if((j==0) && (k==0))
					r->classification = MOO;	
				else if(k==0)
					r->classification = MPO;
				else if(j==0)
					r->classification = MOP;
				else
					r->classification = MPP;
			}
		}
	}
	else//(i >= 0)
	{
		if(j < 0)
		{
			if(k < 0)
			{
				r->classification = PMM;
				if(i==0)
					r->classification = OMM;
			}
			else//(k >= 0)
			{				
				if((i==0) && (k==0))
					r->classification = OMO;
				else if(k==0)
					r->classification = PMO;
				else if(i==0)
					r->classification = OMP;
				else
					r->classification = PMP;
			}
		}
		else//(j >= 0)
		{
			if(k < 0)
			{
				if((i==0) && (j==0))
					r->classification = OOM;
				else if(i==0)
					r->classification = OPM;
				else if(j==0)
					r->classification = POM;
				else
					r->classification = PPM;
			}
			else//(k > 0)
			{
				if(i==0)
				{
					if(j==0)
						r->classification = OOP;
					else if(k==0)
						r->classification = OPO;
					else
						r->classification = OPP;
				}
				else
				{
					if((j==0) && (k==0))
						r->classification = POO;
					else if(j==0)
						r->classification = POP;
					else if(k==0)
						r->classification = PPO;
					else
						r->classification = PPP;
				}
			}			
		}
	}
}


//SIMD_FORCE_INLINE bool slopeint_div(const btRaySlope* r, const btAaboxSlope* b, float *t)
SIMD_FORCE_INLINE bool btRaySlopeAabb(const btRaySlope* r, const btAaboxSlope* b, float *t)
{
	switch (r->classification)
	{
	case MMM:
		{
		if ((r->x < b->x0) || (r->y < b->y0) || (r->z < b->z0)
			|| (r->jbyi * b->x0 - b->y1 + r->c_xy > 0)
			|| (r->ibyj * b->y0 - b->x1 + r->c_yx > 0)
			|| (r->jbyk * b->z0 - b->y1 + r->c_zy > 0)
			|| (r->kbyj * b->y0 - b->z1 + r->c_yz > 0)
			|| (r->kbyi * b->x0 - b->z1 + r->c_xz > 0)
			|| (r->ibyk * b->z0 - b->x1 + r->c_zx > 0)
			)
			return false;
		
		*t = (b->x1 - r->x) / r->i;
		float t1 = (b->y1 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;
		float t2 = (b->z1 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}


	case MMP:
		{		
		if ((r->x < b->x0) || (r->y < b->y0) || (r->z > b->z1)
			|| (r->jbyi * b->x0 - b->y1 + r->c_xy > 0)
			|| (r->ibyj * b->y0 - b->x1 + r->c_yx > 0)
			|| (r->jbyk * b->z1 - b->y1 + r->c_zy > 0)
			|| (r->kbyj * b->y0 - b->z0 + r->c_yz < 0)
			|| (r->kbyi * b->x0 - b->z0 + r->c_xz < 0)
			|| (r->ibyk * b->z1 - b->x1 + r->c_zx > 0)
			)
			return false;
		*t = (b->x1 - r->x) / r->i;
		float t1 = (b->y1 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;
		float t2 = (b->z0 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}


	case MPM:
		{		
		if ((r->x < b->x0) || (r->y > b->y1) || (r->z < b->z0)
			|| (r->jbyi * b->x0 - b->y0 + r->c_xy < 0) 
			|| (r->ibyj * b->y1 - b->x1 + r->c_yx > 0)
			|| (r->jbyk * b->z0 - b->y0 + r->c_zy < 0) 
			|| (r->kbyj * b->y1 - b->z1 + r->c_yz > 0)
			|| (r->kbyi * b->x0 - b->z1 + r->c_xz > 0)
			|| (r->ibyk * b->z0 - b->x1 + r->c_zx > 0)
			)
			return false;
		
		*t = (b->x1 - r->x) / r->i;
		float t1 = (b->y0 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;
		float t2 = (b->z1 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case MPP:
		{
		if ((r->x < b->x0) || (r->y > b->y1) || (r->z > b->z1)
			|| (r->jbyi * b->x0 - b->y0 + r->c_xy < 0) 
			|| (r->ibyj * b->y1 - b->x1 + r->c_yx > 0)
			|| (r->jbyk * b->z1 - b->y0 + r->c_zy < 0)
			|| (r->kbyj * b->y1 - b->z0 + r->c_yz < 0)
			|| (r->kbyi * b->x0 - b->z0 + r->c_xz < 0)
			|| (r->ibyk * b->z1 - b->x1 + r->c_zx > 0)
			)
			return false;
		
		*t = (b->x1 - r->x) / r->i;
		float t1 = (b->y0 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;
		float t2 = (b->z0 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}	

	case PMM:
		{
		if ((r->x > b->x1) || (r->y < b->y0) || (r->z < b->z0)
			|| (r->jbyi * b->x1 - b->y1 + r->c_xy > 0)
			|| (r->ibyj * b->y0 - b->x0 + r->c_yx < 0)
			|| (r->jbyk * b->z0 - b->y1 + r->c_zy > 0)
			|| (r->kbyj * b->y0 - b->z1 + r->c_yz > 0)
			|| (r->kbyi * b->x1 - b->z1 + r->c_xz > 0)
			|| (r->ibyk * b->z0 - b->x0 + r->c_zx < 0)
			)
			return false;

		*t = (b->x0 - r->x) / r->i;
		float t1 = (b->y1 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;
		float t2 = (b->z1 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case PMP:
		{
		if ((r->x > b->x1) || (r->y < b->y0) || (r->z > b->z1)
			|| (r->jbyi * b->x1 - b->y1 + r->c_xy > 0)
			|| (r->ibyj * b->y0 - b->x0 + r->c_yx < 0)
			|| (r->jbyk * b->z1 - b->y1 + r->c_zy > 0)
			|| (r->kbyj * b->y0 - b->z0 + r->c_yz < 0)
			|| (r->kbyi * b->x1 - b->z0 + r->c_xz < 0)
			|| (r->ibyk * b->z1 - b->x0 + r->c_zx < 0)
			)
			return false;

		*t = (b->x0 - r->x) / r->i;
		float t1 = (b->y1 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;
		float t2 = (b->z0 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case PPM:
		{
		if ((r->x > b->x1) || (r->y > b->y1) || (r->z < b->z0)
			|| (r->jbyi * b->x1 - b->y0 + r->c_xy < 0)
			|| (r->ibyj * b->y1 - b->x0 + r->c_yx < 0)
			|| (r->jbyk * b->z0 - b->y0 + r->c_zy < 0) 
			|| (r->kbyj * b->y1 - b->z1 + r->c_yz > 0)
			|| (r->kbyi * b->x1 - b->z1 + r->c_xz > 0)
			|| (r->ibyk * b->z0 - b->x0 + r->c_zx < 0)
			)
			return false;
		
		*t = (b->x0 - r->x) / r->i;
		float t1 = (b->y0 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;
		float t2 = (b->z1 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case PPP:
		{
		if ((r->x > b->x1) || (r->y > b->y1) || (r->z > b->z1)
			|| (r->jbyi * b->x1 - b->y0 + r->c_xy < 0)
			|| (r->ibyj * b->y1 - b->x0 + r->c_yx < 0)
			|| (r->jbyk * b->z1 - b->y0 + r->c_zy < 0)
			|| (r->kbyj * b->y1 - b->z0 + r->c_yz < 0)
			|| (r->kbyi * b->x1 - b->z0 + r->c_xz < 0)
			|| (r->ibyk * b->z1 - b->x0 + r->c_zx < 0)
			)
			return false;
		
		*t = (b->x0 - r->x) / r->i;
		float t1 = (b->y0 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;
		float t2 = (b->z0 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case OMM:
		{
		if((r->x < b->x0) || (r->x > b->x1)
			|| (r->y < b->y0) || (r->z < b->z0)
			|| (r->jbyk * b->z0 - b->y1 + r->c_zy > 0)
			|| (r->kbyj * b->y0 - b->z1 + r->c_yz > 0)
			)
			return false;

		*t = (b->y1 - r->y) / r->j;
		float t2 = (b->z1 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case OMP:
		{
		if((r->x < b->x0) || (r->x > b->x1)
			|| (r->y < b->y0) || (r->z > b->z1)
			|| (r->jbyk * b->z1 - b->y1 + r->c_zy > 0)
			|| (r->kbyj * b->y0 - b->z0 + r->c_yz < 0)
			)
			return false;

		*t = (b->y1 - r->y) / r->j;
		float t2 = (b->z0 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case OPM:
		{
		if((r->x < b->x0) || (r->x > b->x1)
			|| (r->y > b->y1) || (r->z < b->z0)
			|| (r->jbyk * b->z0 - b->y0 + r->c_zy < 0) 
			|| (r->kbyj * b->y1 - b->z1 + r->c_yz > 0)
			)
			return false;

		*t = (b->y0 - r->y) / r->j;
		float t2 = (b->z1 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case OPP:
		{
		if((r->x < b->x0) || (r->x > b->x1)
			|| (r->y > b->y1) || (r->z > b->z1)
			|| (r->jbyk * b->z1 - b->y0 + r->c_zy < 0)
			|| (r->kbyj * b->y1 - b->z0 + r->c_yz < 0)
			)
			return false;

		*t = (b->y0 - r->y) / r->j;
		float t2 = (b->z0 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case MOM:
		{
		if((r->y < b->y0) || (r->y > b->y1)
			|| (r->x < b->x0) || (r->z < b->z0) 
			|| (r->kbyi * b->x0 - b->z1 + r->c_xz > 0)
			|| (r->ibyk * b->z0 - b->x1 + r->c_zx > 0)
			)
			return false;

		*t = (b->x1 - r->x) / r->i;
		float t2 = (b->z1 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case MOP:
		{
		if((r->y < b->y0) || (r->y > b->y1)
			|| (r->x < b->x0) || (r->z > b->z1)
			|| (r->kbyi * b->x0 - b->z0 + r->c_xz < 0)
			|| (r->ibyk * b->z1 - b->x1 + r->c_zx > 0)
			)
			return false;

		*t = (b->x1 - r->x) / r->i;
		float t2 = (b->z0 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case POM:
		{
		if((r->y < b->y0) || (r->y > b->y1)
			|| (r->x > b->x1) || (r->z < b->z0)
			|| (r->kbyi * b->x1 - b->z1 + r->c_xz > 0)
			|| (r->ibyk * b->z0 - b->x0 + r->c_zx < 0)
			)
			return false;

		*t = (b->x0 - r->x) / r->i;
		float t2 = (b->z1 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}	

	case POP:
		{
		if((r->y < b->y0) || (r->y > b->y1)
			|| (r->x > b->x1) || (r->z > b->z1)
			|| (r->kbyi * b->x1 - b->z0 + r->c_xz < 0)
			|| (r->ibyk * b->z1 - b->x0 + r->c_zx < 0)
			)
			return false;

		*t = (b->x0 - r->x) / r->i;
		float t2 = (b->z0 - r->z) / r->k;
		if(t2 > *t)
			*t = t2;

		return true;
		}

	case MMO:
		{
		if((r->z < b->z0) || (r->z > b->z1)
			|| (r->x < b->x0) || (r->y < b->y0)
			|| (r->jbyi * b->x0 - b->y1 + r->c_xy > 0)
			|| (r->ibyj * b->y0 - b->x1 + r->c_yx > 0)
			)
			return false;

		*t = (b->x1 - r->x) / r->i;
		float t1 = (b->y1 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;

		return true;
		}

	case MPO:
		{
		if((r->z < b->z0) || (r->z > b->z1)
			|| (r->x < b->x0) || (r->y > b->y1) 
			|| (r->jbyi * b->x0 - b->y0 + r->c_xy < 0) 
			|| (r->ibyj * b->y1 - b->x1 + r->c_yx > 0)
			)
			return false;

		*t = (b->x1 - r->x) / r->i;
		float t1 = (b->y0 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;

		return true;
		}

	case PMO:
		{
		if((r->z < b->z0) || (r->z > b->z1)
			|| (r->x > b->x1) || (r->y < b->y0) 
			|| (r->jbyi * b->x1 - b->y1 + r->c_xy > 0)
			|| (r->ibyj * b->y0 - b->x0 + r->c_yx < 0) 
			)
			return false;

		*t = (b->x0 - r->x) / r->i;
		float t1 = (b->y1 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;

		return true;
		}

	case PPO:
		{
		if((r->z < b->z0) || (r->z > b->z1)
			|| (r->x > b->x1) || (r->y > b->y1)  
			|| (r->jbyi * b->x1 - b->y0 + r->c_xy < 0)
			|| (r->ibyj * b->y1 - b->x0 + r->c_yx < 0)
			)
			return false;

		*t = (b->x0 - r->x) / r->i;
		float t1 = (b->y0 - r->y) / r->j;
		if(t1 > *t)
			*t = t1;

		return true;
		}
	case MOO:
		{
		if((r->x < b->x0)
			|| (r->y < b->y0) || (r->y > b->y1)
			|| (r->z < b->z0) || (r->z > b->z1)
			)
			return false;

		*t = (b->x1 - r->x) / r->i;
		return true;
		}

	case POO:
		{
		if((r->x > b->x1)
			|| (r->y < b->y0) || (r->y > b->y1)
			|| (r->z < b->z0) || (r->z > b->z1)
			)
			return false;

		*t = (b->x0 - r->x) / r->i;
		return true;
		}

	case OMO:
		{
		if((r->y < b->y0)
			|| (r->x < b->x0) || (r->x > b->x1)
			|| (r->z < b->z0) || (r->z > b->z1)
			)
			return false;
		
		*t = (b->y1 - r->y) / r->j;
		return true;
		}

	case OPO:
		{
		if((r->y > b->y1)
			|| (r->x < b->x0) || (r->x > b->x1)
			|| (r->z < b->z0) || (r->z > b->z1)
			)
			return false;

		*t = (b->y0 - r->y) / r->j;
		return true;
	}


	case OOM:
		{
		if((r->z < b->z0)
			|| (r->x < b->x0) || (r->x > b->x1)
			|| (r->y < b->y0) || (r->y > b->y1)
			)
			return false;

		*t = (b->z1 - r->z) / r->k;
		return true;
		}

	case OOP:
		{
		if((r->z > b->z1)
			|| (r->x < b->x0) || (r->x > b->x1)
			|| (r->y < b->y0) || (r->y > b->y1)
			)
			return false;

		*t = (b->z0 - r->z) / r->k;
		return true;
		}
	
	}

	return false;
}


SIMD_FORCE_INLINE bool btRayAabb2(const btVector3& rayFrom,
								  const btVector3& rayInvDirection,
								  const unsigned int raySign[3],
								  const btVector3 bounds[2],
								  btScalar& tmin,
								  btScalar lambda_min,
								  btScalar lambda_max)
{
	btScalar tmax, tymin, tymax, tzmin, tzmax;
	tmin = (bounds[raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
	tmax = (bounds[1-raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
	tymin = (bounds[raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();
	tymax = (bounds[1-raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();

	if ( (tmin > tymax) || (tymin > tmax) )
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	tzmin = (bounds[raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();
	tzmax = (bounds[1-raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();

	if ( (tmin > tzmax) || (tzmin > tmax) )
		return false;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;
	return ( (tmin < lambda_max) && (tmax > lambda_min) );
}

SIMD_FORCE_INLINE bool btRayAabb(const btVector3& rayFrom, 
								 const btVector3& rayTo, 
								 const btVector3& aabbMin, 
								 const btVector3& aabbMax,
					  btScalar& param, btVector3& normal) 
{
	btVector3 aabbHalfExtent = (aabbMax-aabbMin)* btScalar(0.5);
	btVector3 aabbCenter = (aabbMax+aabbMin)* btScalar(0.5);
	btVector3	source = rayFrom - aabbCenter;
	btVector3	target = rayTo - aabbCenter;
	int	sourceOutcode = btOutcode(source,aabbHalfExtent);
	int targetOutcode = btOutcode(target,aabbHalfExtent);
	if ((sourceOutcode & targetOutcode) == 0x0)
	{
		btScalar lambda_enter = btScalar(0.0);
		btScalar lambda_exit  = param;
		btVector3 r = target - source;
		int i;
		btScalar	normSign = 1;
		btVector3	hitNormal(0,0,0);
		int bit=1;

		for (int j=0;j<2;j++)
		{
			for (i = 0; i != 3; ++i)
			{
				if (sourceOutcode & bit)
				{
					btScalar lambda = (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
					if (lambda_enter <= lambda)
					{
						lambda_enter = lambda;
						hitNormal.setValue(0,0,0);
						hitNormal[i] = normSign;
					}
				}
				else if (targetOutcode & bit) 
				{
					btScalar lambda = (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
					btSetMin(lambda_exit, lambda);
				}
				bit<<=1;
			}
			normSign = btScalar(-1.);
		}
		if (lambda_enter <= lambda_exit)
		{
			param = lambda_enter;
			normal = hitNormal;
			return true;
		}
	}
	return false;
}



SIMD_FORCE_INLINE	void btTransformAabb(const btVector3& halfExtents, btScalar margin,const btTransform& t,btVector3& aabbMinOut,btVector3& aabbMaxOut)
{
	btVector3 halfExtentsWithMargin = halfExtents+btVector3(margin,margin,margin);
	btMatrix3x3 abs_b = t.getBasis().absolute();  
	btVector3 center = t.getOrigin();
	btVector3 extent = btVector3(abs_b[0].dot(halfExtentsWithMargin),
		   abs_b[1].dot(halfExtentsWithMargin),
		  abs_b[2].dot(halfExtentsWithMargin));
	aabbMinOut = center - extent;
	aabbMaxOut = center + extent;
}


SIMD_FORCE_INLINE	void btTransformAabb(const btVector3& localAabbMin,const btVector3& localAabbMax, btScalar margin,const btTransform& trans,btVector3& aabbMinOut,btVector3& aabbMaxOut)
{
		btAssert(localAabbMin.getX() <= localAabbMax.getX());
		btAssert(localAabbMin.getY() <= localAabbMax.getY());
		btAssert(localAabbMin.getZ() <= localAabbMax.getZ());
		btVector3 localHalfExtents = btScalar(0.5)*(localAabbMax-localAabbMin);
		localHalfExtents+=btVector3(margin,margin,margin);

		btVector3 localCenter = btScalar(0.5)*(localAabbMax+localAabbMin);
		btMatrix3x3 abs_b = trans.getBasis().absolute();  
		btVector3 center = trans(localCenter);
		btVector3 extent = btVector3(abs_b[0].dot(localHalfExtents),
			   abs_b[1].dot(localHalfExtents),
			  abs_b[2].dot(localHalfExtents));
		aabbMinOut = center-extent;
		aabbMaxOut = center+extent;
}


#endif


