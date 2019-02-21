//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_INTERSECTION_TRIANGLE_BOX_REF_H
#define GU_INTERSECTION_TRIANGLE_BOX_REF_H

#include "CmPhysXCommon.h"
#include "foundation/PxVec3.h"


/********************************************************/
/* AABB-triangle overlap test code                      */
/* by Tomas Akenine-M?r									*/
/* Function: int triBoxOverlap(float boxcenter[3],      */
/*          float boxhalfsize[3],float triverts[3][3]); */
/* History:                                             */
/*   2001-03-05: released the code in its first version */
/*   2001-06-18: changed the order of the tests, faster */
/*                                                      */
/* Acknowledgement: Many thanks to Pierre Terdiman for  */
/* suggestions and discussions on how to optimize code. */
/* Thanks to David Hunt for finding a ">="-bug!         */
/********************************************************/


namespace physx
{

#define CROSS(dest,v1,v2)		\
	dest.x=v1.y*v2.z-v1.z*v2.y;	\
	dest.y=v1.z*v2.x-v1.x*v2.z;	\
	dest.z=v1.x*v2.y-v1.y*v2.x; 

#define DOT(v1,v2) (v1.x*v2.x+v1.y*v2.y+v1.z*v2.z)

#define FINDMINMAX(x0, x1, x2, minimum, maximum)			\
	minimum = physx::intrinsics::selectMin(x0, x1);			\
	maximum = physx::intrinsics::selectMax(x0, x1);			\
	minimum = physx::intrinsics::selectMin(minimum, x2);	\
	maximum = physx::intrinsics::selectMax(maximum, x2);

	static PX_CUDA_CALLABLE PX_FORCE_INLINE Ps::IntBool planeBoxOverlap(const PxVec3& normal, PxReal d, const PxVec3& maxbox)
	{
		PxVec3 vmin, vmax;

		if (normal.x>0.0f)
		{
			vmin.x = -maxbox.x;
			vmax.x = maxbox.x;
		}
		else
		{
			vmin.x = maxbox.x;
			vmax.x = -maxbox.x;
		}

		if (normal.y>0.0f)
		{
			vmin.y = -maxbox.y;
			vmax.y = maxbox.y;
		}
		else
		{
			vmin.y = maxbox.y;
			vmax.y = -maxbox.y;
		}

		if (normal.z>0.0f)
		{
			vmin.z = -maxbox.z;
			vmax.z = maxbox.z;
		}
		else
		{
			vmin.z = maxbox.z;
			vmax.z = -maxbox.z;
		}

		if (normal.dot(vmin) + d >  0.0f) return Ps::IntFalse;
		if (normal.dot(vmax) + d >= 0.0f) return Ps::IntTrue;
		return Ps::IntFalse;
	}

	/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb)							\
	p0 = a*v0.y - b*v0.z;									\
	p2 = a*v2.y - b*v2.z;									\
	minimum = physx::intrinsics::selectMin(p0, p2);			\
	maximum = physx::intrinsics::selectMax(p0, p2);			\
	rad = fa * extents.y + fb * extents.z;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

#define AXISTEST_X2(a, b, fa, fb)							\
	p0 = a*v0.y - b*v0.z;									\
	p1 = a*v1.y - b*v1.z;									\
	minimum = physx::intrinsics::selectMin(p0, p1);			\
	maximum = physx::intrinsics::selectMax(p0, p1);			\
	rad = fa * extents.y + fb * extents.z;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

	/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb)							\
	p0 = -a*v0.x + b*v0.z;									\
	p2 = -a*v2.x + b*v2.z;									\
	minimum = physx::intrinsics::selectMin(p0, p2);			\
	maximum = physx::intrinsics::selectMax(p0, p2);			\
	rad = fa * extents.x + fb * extents.z;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

#define AXISTEST_Y1(a, b, fa, fb)							\
	p0 = -a*v0.x + b*v0.z;									\
	p1 = -a*v1.x + b*v1.z;									\
	minimum = physx::intrinsics::selectMin(p0, p1);			\
	maximum = physx::intrinsics::selectMax(p0, p1);			\
	rad = fa * extents.x + fb * extents.z;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

	/*======================== Z-tests ========================*/
#define AXISTEST_Z12(a, b, fa, fb)							\
	p1 = a*v1.x - b*v1.y;									\
	p2 = a*v2.x - b*v2.y;									\
	minimum = physx::intrinsics::selectMin(p1, p2);			\
	maximum = physx::intrinsics::selectMax(p1, p2);			\
	rad = fa * extents.x + fb * extents.y;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

#define AXISTEST_Z0(a, b, fa, fb)							\
	p0 = a*v0.x - b*v0.y;									\
	p1 = a*v1.x - b*v1.y;									\
	minimum = physx::intrinsics::selectMin(p0, p1);			\
	maximum = physx::intrinsics::selectMax(p0, p1);			\
	rad = fa * extents.x + fb * extents.y;					\
	if(minimum>rad || maximum<-rad) return Ps::IntFalse;

	namespace Gu
	{

		static PX_CUDA_CALLABLE PX_FORCE_INLINE Ps::IntBool intersectTriangleBox_RefImpl(const PxVec3& boxcenter, const PxVec3& extents, const PxVec3& tp0, const PxVec3& tp1, const PxVec3& tp2)
		{
			/*    use separating axis theorem to test overlap between triangle and box */
			/*    need to test for overlap in these directions: */
			/*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
			/*       we do not even need to test these) */
			/*    2) normal of the triangle */
			/*    3) crossproduct(edge from tri, {x,y,z}-directin) */
			/*       this gives 3x3=9 more tests */

			// This is the fastest branch on Sun - move everything so that the boxcenter is in (0,0,0)
			const PxVec3 v0 = tp0 - boxcenter;
			const PxVec3 v1 = tp1 - boxcenter;
			const PxVec3 v2 = tp2 - boxcenter;

			// compute triangle edges
			const PxVec3 e0 = v1 - v0;	// tri edge 0
			const PxVec3 e1 = v2 - v1;	// tri edge 1
			const PxVec3 e2 = v0 - v2;	// tri edge 2

			float minimum, maximum, rad, p0, p1, p2;

			// Bullet 3: test the 9 tests first (this was faster)
			float fex = PxAbs(e0.x);
			float fey = PxAbs(e0.y);
			float fez = PxAbs(e0.z);
			AXISTEST_X01(e0.z, e0.y, fez, fey);
			AXISTEST_Y02(e0.z, e0.x, fez, fex);
			AXISTEST_Z12(e0.y, e0.x, fey, fex);

			fex = PxAbs(e1.x);
			fey = PxAbs(e1.y);
			fez = PxAbs(e1.z);
			AXISTEST_X01(e1.z, e1.y, fez, fey);
			AXISTEST_Y02(e1.z, e1.x, fez, fex);
			AXISTEST_Z0(e1.y, e1.x, fey, fex);

			fex = PxAbs(e2.x);
			fey = PxAbs(e2.y);
			fez = PxAbs(e2.z);
			AXISTEST_X2(e2.z, e2.y, fez, fey);
			AXISTEST_Y1(e2.z, e2.x, fez, fex);
			AXISTEST_Z12(e2.y, e2.x, fey, fex);

			// Bullet 1:
			//  first test overlap in the {x,y,z}-directions
			//  find minimum, maximum of the triangle each direction, and test for overlap in
			//  that direction -- this is equivalent to testing a minimal AABB around
			//  the triangle against the AABB

			// test in X-direction
			FINDMINMAX(v0.x, v1.x, v2.x, minimum, maximum);
			if (minimum>extents.x || maximum<-extents.x) return Ps::IntFalse;

			// test in Y-direction
			FINDMINMAX(v0.y, v1.y, v2.y, minimum, maximum);
			if (minimum>extents.y || maximum<-extents.y) return Ps::IntFalse;

			// test in Z-direction
			FINDMINMAX(v0.z, v1.z, v2.z, minimum, maximum);
			if (minimum>extents.z || maximum<-extents.z) return Ps::IntFalse;

			// Bullet 2:
			//  test if the box intersects the plane of the triangle
			//  compute plane equation of triangle: normal*x+d=0
			PxVec3 normal;
			CROSS(normal, e0, e1);
			const float d = -DOT(normal, v0);	// plane eq: normal.x+d=0
			if (!planeBoxOverlap(normal, d, extents)) return Ps::IntFalse;

			return Ps::IntTrue;	// box and triangle overlaps
		}
	}

#undef CROSS
#undef DOT
#undef FINDMINMAX
#undef AXISTEST_X01
#undef AXISTEST_X2
#undef AXISTEST_Y02
#undef AXISTEST_Y1
#undef AXISTEST_Z12
#undef AXISTEST_Z0

}

#endif

