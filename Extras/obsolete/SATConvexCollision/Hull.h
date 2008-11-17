// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// Hull.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#ifndef SAT_HULL_H
#define SAT_HULL_H

#include "Maths.h"
#include "Shape.h"


class DynWorld;
class HullContactCollector;

/// Hull implements a convex collision detection algorithm based on Separating Axis Theorem (SAT). It is an alternative to GJK.
/// It calculates the separating axis, and based on that it calculates the contact manifold (points) in one go.
/// The separating axis calculation is approximated, not all edge-edge calculations are performed (performance reasons).
/// Future idea is to combine this with GJK for polyhedra: GJK to calculate the separating axis, and Hull clipping code to calculate the full set of contacts.
class Hull : public Shape
{
	friend class ShapeCollider;

public:
	struct Edge
	{
		short m_verts[2];
		short m_faces[2];
		short m_nextEdge[2];	// for each m_face
	};

	struct Face
	{
		short m_numEdges;
		short m_firstEdge;
	};

private:
	static const int kMaxVerts = 256;
	static const int kMaxFaces = 256;
	static const int kMaxEdges = 256;

	short m_numVerts;
	short m_numFaces;
	short m_numEdges;

	Point3* m_pVerts;
	Face* m_pFaces;
	Edge* m_pEdges;
	Plane* m_pPlanes;

	// hull construction stuff
	static const int kTmpFaceMaxVerts = 64;
	struct TmpFace
	{
		short m_index;
		short m_next;
		short m_numVerts;
		short m_verts[kTmpFaceMaxVerts];
		short m_edges[kTmpFaceMaxVerts];
		Plane m_plane;
	};

	struct TmpEdge
	{
		short m_index;
		short m_next;
		short m_verts[2];
		short m_faces[2];
	};

	static short s_firstFreeTmpFace;
	static short s_firstUsedTmpFace;
	static TmpFace* s_pTmpFaces;

	static short s_firstFreeTmpEdge;
	static short s_firstUsedTmpEdge;
	static TmpEdge* s_pTmpEdges;

	static const Point3* s_pPoints;

	static short AllocTmpFace();
	static void FreeTmpFace(short face);
	static TmpFace* GetTmpFace(short index) {if (index < 0) return 0; return s_pTmpFaces + index;}

	static short AllocTmpEdge();
	static void FreeTmpEdge(short edge);
	static TmpEdge* GetTmpEdge(short index) {if (index < 0) return 0; return s_pTmpEdges + index;}

	static short MatchOrAddEdge(short vert0, short vert1, short face);
	static void UnmatchOrRemoveEdge(short edge, short face);

	static short AddTmpFace(short vert0, short vert1, short vert2);
	static short AddTmpFace(short numVerts, short* pVerts);
	static short AddTmpFace(short vert0, short numOtherVerts, short* pVerts);
	static void RemoveTmpFace(short face);

	static bool TmpFaceAddPoint(short point, short face);

	static int RemoveVisibleFaces(const Point3& point);
	static void FillHole(short newVertex);
	static Hull* MakeHullFromTemp();

public:
	Hull();
	~Hull();

//	ObjectType GetObjectType() const {return kTypeHull;}

	short getNumVertices() const;
	short GetNumFaces() const;
	short getNumEdges() const;

	const Point3& getVertex(short index) const;
	const Face& GetFace(short index) const;
	const Edge& getEdge(short index) const;
	const Plane& getPlane(short index) const;

	short GetFaceFirstEdge(short face) const;
	short GetFaceNextEdge(short face, short prevEdge) const;

	short GetEdgeVertex0(short face, short edge) const;
	short GetEdgeVertex1(short face, short edge) const;

	short GetEdgeOtherFace(short edge, short face) const;

	Point3 GetFaceCentroid(short face) const;

	//static void ProcessHullHull(btSeparation& sep);
	static void ProcessHullHull(btSeparation& sep,const Hull& shapeA,const Hull& shapeB,const Transform& trA,const Transform& trB, HullContactCollector* collector);

	virtual void ComputeInertia(const Transform& transform, Point3& centerOfMass, Matrix33& inertia, float totalMass) const;
	virtual Bounds3 ComputeBounds(const Transform& transform) const;

	static Hull* MakeHull(int numPoints, const Point3* pPoints);

	//for contact generation


	
	/// Clips a face to the back of a plane
	static int ClipFace(int numVerts, Point3** ppVtxIn, Point3** ppVtxOut, const Plane& plane);

	static bool GetSeparationHullHull(btSeparation& sep, const Point3* pVertsA, const Point3* pVertsB,
										  const Transform& trA, const Transform& trB,
										  const Hull& hullA,
										  const Hull& hullB
										  );

	static int AddContactsHullHull(btSeparation& sep, const Point3* pVertsA, const Point3* pVertsB,
									   const Transform& trA, const Transform& trB,const Hull& hullA,const Hull& hullB,
									   HullContactCollector* hullContactCollector);



};



#include "hull.inl"

#endif //SAT_HULL_H