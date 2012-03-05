/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "pfx_simplex_solver.h"
#include "pfx_gjk_solver.h"

namespace sce {
namespace PhysicsEffects {
inline static
bool operator ==(const PfxVector3 &a,const PfxVector3 &b)
{
	return lengthSqr(a-b) < (SCE_PFX_GJK_EPSILON * SCE_PFX_GJK_EPSILON);
}

bool PfxSimplexSolver::closest(PfxVector3& v)
{
	bool ret = false;

	bc.reset();

	switch(numVertices) {
		case 0:
		ret = false;
		break;

		case 1:
		{
			PfxVector3 tmpP = P[0];
			PfxVector3 tmpQ = Q[0];
			v = tmpP-tmpQ;
			bc.reset();
			bc.setBarycentricCoordinates(1.0f,0.0f,0.0f,0.0f);
			ret = bc.isValid();
		}
		break;

		case 2:
		{
			PfxVector3 dir = W[1] - W[0];
			PfxFloat t = dot(-W[0],dir) / dot(dir,dir);

			if(t < 0.0f) t = 0.0f;
			if(t > 1.0f) t = 1.0f;

			bc.setBarycentricCoordinates(1-t,t,0.0f,0.0f);

			PfxVector3 tmpP = P[0] + t * (P[1] - P[0]);
			PfxVector3 tmpQ = Q[0] + t * (Q[1] - Q[0]);
			v = tmpP - tmpQ;

			reduceVertices();

			ret = bc.isValid();
			break;
		}

		case 3: 
		{ 
			const PfxVector3& a = W[0]; 
			const PfxVector3& b = W[1]; 
			const PfxVector3& c = W[2]; 

			closestPointTriangleFromOrigin(a,b,c,bc);

		PfxVector3 tmpP = P[0] * bc.barycentricCoords[0] + 
					   P[1] * bc.barycentricCoords[1] + 
					   P[2] * bc.barycentricCoords[2]; 

		PfxVector3 tmpQ = Q[0] * bc.barycentricCoords[0] + 
					   Q[1] * bc.barycentricCoords[1] + 
					   Q[2] * bc.barycentricCoords[2]; 

			v = tmpP-tmpQ; 

			reduceVertices(); 
			ret = bc.isValid(); 
			break; 
		}

		case 4:
		{
			const PfxVector3& a = W[0];
			const PfxVector3& b = W[1];
			const PfxVector3& c = W[2];
			const PfxVector3& d = W[3];

			if(closestPointTetrahedronFromOrigin(a,b,c,d,bc)) {
			PfxVector3 tmpP = P[0] * bc.barycentricCoords[0] +
						   P[1] * bc.barycentricCoords[1] +
						   P[2] * bc.barycentricCoords[2] +
						   P[3] * bc.barycentricCoords[3];

			PfxVector3 tmpQ = Q[0] * bc.barycentricCoords[0] +
						   Q[1] * bc.barycentricCoords[1] +
						   Q[2] * bc.barycentricCoords[2] +
						   Q[3] * bc.barycentricCoords[3];
				v = tmpP-tmpQ;

				reduceVertices();
				ret = bc.isValid();
			} else {
				// 原点が内部に存在→交差している
				ret = true;
				v = PfxVector3(0.0f);
			}
			break;
		}
	};

	return ret;
}

bool PfxSimplexSolver::inSimplex(const PfxVector3& w)
{
	for(int i=0;i<numVertices;i++) {
		if(W[i] == w)
			return true;
	}
	return false;
}

bool PfxSimplexSolver::closestPointTriangleFromOrigin(const PfxVector3 &a,const  PfxVector3 &b,const  PfxVector3 &c,PfxBarycentricCoords& result)
{
	result.usedVertices = 0;
	PfxVector3 p(0.0f);

    PfxVector3 ab = b - a;
    PfxVector3 ac = c - a;
    PfxVector3 ap = p - a;
    PfxFloat d1 = dot(ab,ap);
    PfxFloat d2 = dot(ac,ap);
    if(d1 <= 0.0f && d2 <= 0.0f) {
		result.closest = a;
		result.setBarycentricCoordinates(1.0f,0.0f,0.0f,0.0f);
		return true;
	}

    PfxVector3 bp = p - b;
    PfxFloat d3 = dot(ab,bp);
    PfxFloat d4 = dot(ac,bp);
    if(d3 >= 0.0f && d4 <= d3) {
		result.closest = b;
		result.setBarycentricCoordinates(0.0f,1.0f,0.0f,0.0f);
		return true;
	}

    PfxFloat vc = d1*d4 - d3*d2;
    if(vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        PfxFloat v = d1 / (d1 - d3);
		result.closest = a + v * ab;
		result.setBarycentricCoordinates(1.0f-v,v,0.0f,0.0f);
		return true;
    }

    PfxVector3 cp = p - c;
    PfxFloat d5 = dot(ab,cp);
    PfxFloat d6 = dot(ac,cp);
    if(d6 >= 0.0f && d5 <= d6) {
		result.closest = c;
		result.setBarycentricCoordinates(0.0f,0.0f,1.0f,0.0f);
		return true;
	}

    PfxFloat vb = d5*d2 - d1*d6;
    if(vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        PfxFloat w = d2 / (d2 - d6);
		result.closest = a + w * ac;
		result.setBarycentricCoordinates(1.0f-w,0.0f,w,0.0f);
		return true;
    }

    PfxFloat va = d3*d6 - d5*d4;
    if(va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        PfxFloat w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		result.closest = b + w * (c - b);
		result.setBarycentricCoordinates(0.0f,1.0f-w,w,0.0f);
		return true;		
    }

    PfxFloat denom = PfxFloat(1.0) / (va + vb + vc);
    PfxFloat v = vb * denom;
    PfxFloat w = vc * denom;
    
	result.closest = a + ab * v + ac * w;
	result.setBarycentricCoordinates(1.0f-v-w,v,w,0.0f);
	
	return true;
}

bool PfxSimplexSolver::closestPointTetrahedronFromOrigin(const PfxVector3 &a,const  PfxVector3 &b,const  PfxVector3 &c,const  PfxVector3 &d,PfxBarycentricCoords& finalResult)
{
	PfxBarycentricCoords tempResult;
	PfxVector3 p(0.0f);

	finalResult.closest = p;
	finalResult.usedVertices = 0;

	bool pointOutsideABC = originOutsideOfPlane(a, b, c, d);
	bool pointOutsideACD = originOutsideOfPlane(a, c, d, b);
	bool pointOutsideADB = originOutsideOfPlane(a, d, b, c);
	bool pointOutsideBDC = originOutsideOfPlane(b, d, c, a);

	if(!pointOutsideABC && !pointOutsideACD && !pointOutsideADB && !pointOutsideBDC)
		return false;

	PfxFloat bestSqDist = SCE_PFX_FLT_MAX;

	if(pointOutsideABC) {
		closestPointTriangleFromOrigin(a, b, c,tempResult);
		PfxVector3 q = tempResult.closest;
		PfxFloat sqDist = dot((q - p),(q - p));
		if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
			finalResult.setBarycentricCoordinates(
					tempResult.barycentricCoords[0],
					tempResult.barycentricCoords[1],
					tempResult.barycentricCoords[2],
					0);
		}
    }
  
	if(pointOutsideACD) {
		closestPointTriangleFromOrigin(a, c, d,tempResult);
		PfxVector3 q = tempResult.closest;
        PfxFloat sqDist = dot((q - p),(q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
			finalResult.setBarycentricCoordinates(
					tempResult.barycentricCoords[0],
					0,
					tempResult.barycentricCoords[1],
					tempResult.barycentricCoords[2]);
		}
    }
	
	if(pointOutsideADB) {
		closestPointTriangleFromOrigin(a, d, b,tempResult);
		PfxVector3 q = tempResult.closest;
        PfxFloat sqDist = dot((q - p),(q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
			finalResult.setBarycentricCoordinates(
					tempResult.barycentricCoords[0],
					tempResult.barycentricCoords[2],
					0,
					tempResult.barycentricCoords[1]);
		}
    }

	if(pointOutsideBDC) {
		closestPointTriangleFromOrigin(b, d, c,tempResult);
		PfxVector3 q = tempResult.closest;
        PfxFloat sqDist = dot((q - p),(q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
			finalResult.setBarycentricCoordinates(
					0,
					tempResult.barycentricCoords[0],
					tempResult.barycentricCoords[2],
					tempResult.barycentricCoords[1]);
		}
    }

    return true;
}
} //namespace PhysicsEffects
} //namespace sce
