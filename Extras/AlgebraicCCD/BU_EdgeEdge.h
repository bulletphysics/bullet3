/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BU_EDGEEDGE
#define BU_EDGEEDGE

class BU_Screwing;
#include <LinearMath/btTransform.h>
#include <LinearMath/btPoint3.h>
#include <LinearMath/btVector3.h>

//class BUM_Point2;

#include <LinearMath/btScalar.h>

///BU_EdgeEdge implements algebraic time of impact calculation between two (angular + linear) moving edges.
class BU_EdgeEdge
{
public:
	

	BU_EdgeEdge();
	bool GetTimeOfImpact(
		const BU_Screwing& screwAB,
		const btPoint3& a,//edge in object A
		const btVector3& u,
		const btPoint3& c,//edge in object B
		const btVector3& v,
		btScalar &minTime,
		btScalar &lamda,
		btScalar& mu
		);
private:

	bool Calc2DRotationPointPoint(const btPoint3& rotPt, btScalar rotRadius, btScalar rotW,const btPoint3& intersectPt,btScalar& minTime);
	bool GetTimeOfImpactbteralCase(
		const BU_Screwing& screwAB,
		const btPoint3& a,//edge in object A
		const btVector3& u,
		const btPoint3& c,//edge in object B
		const btVector3& v,
		btScalar &minTime,
		btScalar &lamda,
		btScalar& mu

		);

	
	bool GetTimeOfImpactVertexEdge(
		const BU_Screwing& screwAB,
		const btPoint3& a,//edge in object A
		const btVector3& u,
		const btPoint3& c,//edge in object B
		const btVector3& v,
		btScalar &minTime,
		btScalar &lamda,
		btScalar& mu

		);

};

#endif //BU_EDGEEDGE
