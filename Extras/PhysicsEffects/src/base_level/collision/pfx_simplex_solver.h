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

#ifndef _SCE_PFX_SIMPLEX_SOLVER_H
#define _SCE_PFX_SIMPLEX_SOLVER_H

#include "../../../include/physics_effects/base_level/base/pfx_common.h"

namespace sce {
namespace PhysicsEffects {


///////////////////////////////////////////////////////////////////////////////
// Voronoi Simplex Solver

struct SCE_PFX_ALIGNED(16) PfxBarycentricCoords {
	PfxVector3 closest;
PfxFloat barycentricCoords[4];
	unsigned int usedVertices;
	SCE_PFX_PADDING(1,12)

	void reset()
	{
	barycentricCoords[0] = 0.0f;
	barycentricCoords[1] = 0.0f;
	barycentricCoords[2] = 0.0f;
	barycentricCoords[3] = 0.0f;
		usedVertices = 0;
	}

	bool isValid()
	{
	return   (barycentricCoords[0] >= 0.0f) &&
			 (barycentricCoords[1] >= 0.0f) &&
			 (barycentricCoords[2] >= 0.0f) &&
			 (barycentricCoords[3] >= 0.0f);
	}

void setBarycentricCoordinates(PfxFloat a,PfxFloat b,PfxFloat c,PfxFloat d)
{
	barycentricCoords[0] = a;
	barycentricCoords[1] = b;
	barycentricCoords[2] = c;
	barycentricCoords[3] = d;
	if(a != 0.0f) usedVertices |= 1<<3;
	if(b != 0.0f) usedVertices |= 1<<2;
	if(c != 0.0f) usedVertices |= 1<<1;
	if(d != 0.0f) usedVertices |= 1;
}
};

class PfxSimplexSolver {
private:
	 static const int MAX_VERTS = 4;

public:
	int	numVertices;
	SCE_PFX_PADDING(1,12)
	PfxVector3	W[MAX_VERTS];
	PfxVector3	P[MAX_VERTS];
	PfxVector3	Q[MAX_VERTS];

	PfxBarycentricCoords bc;

	inline void	removeVertex(int index);
	inline void	reduceVertices ();

	inline bool	originOutsideOfPlane(const PfxVector3& a, const PfxVector3& b, const PfxVector3& c, const PfxVector3& d);
	bool	closestPointTetrahedronFromOrigin(const PfxVector3 &a,const  PfxVector3 &b,const  PfxVector3 &c,const  PfxVector3 &d, PfxBarycentricCoords& result);
	bool	closestPointTriangleFromOrigin(const PfxVector3 &a,const  PfxVector3 &b,const  PfxVector3 &c,PfxBarycentricCoords& result);

public:
	void reset()
	{
		numVertices = 0;
		bc.reset();
	}

	inline void addVertex(const PfxVector3& w_, const PfxVector3& p_, const PfxVector3& q_);

	bool closest(PfxVector3& v);

	bool fullSimplex() const
	{
		return (numVertices == 4);
	}

	bool inSimplex(const PfxVector3& w);
};

inline
void	PfxSimplexSolver::removeVertex(int index)
{
	SCE_PFX_ASSERT(numVertices>0);
	numVertices--;
	W[index] = W[numVertices];
	P[index] = P[numVertices];
	Q[index] = Q[numVertices];
}

inline
void	PfxSimplexSolver::reduceVertices ()
{
	if ((numVertices >= 4) && (!(bc.usedVertices&0x01)))
		removeVertex(3);

	if ((numVertices >= 3) && (!(bc.usedVertices&0x02)))
		removeVertex(2);

	if ((numVertices >= 2) && (!(bc.usedVertices&0x04)))
		removeVertex(1);
	
	if ((numVertices >= 1) && (!(bc.usedVertices&0x08)))
		removeVertex(0);
}

inline
void PfxSimplexSolver::addVertex(const PfxVector3& w, const PfxVector3& p, const PfxVector3& q)
{
	W[numVertices] = w;
	P[numVertices] = p;
	Q[numVertices] = q;
	numVertices++;
}

inline
bool PfxSimplexSolver::originOutsideOfPlane(const PfxVector3& a, const PfxVector3& b, const PfxVector3& c, const PfxVector3& d)
{
	PfxVector3 normal = cross((b-a),(c-a));

    PfxFloat signp = dot(-a,normal);
    PfxFloat signd = dot((d - a),normal);

	return signp * signd < 0.0f;
}

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_SIMPLEX_SOLVER_H
