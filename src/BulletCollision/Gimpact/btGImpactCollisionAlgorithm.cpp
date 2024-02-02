/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
This is a modified version of the Bullet Continuous Collision Detection and Physics Library
*/

/*
Author: Francisco Leon Najera
Concave-Concave Collision

*/

#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "btGImpactCollisionAlgorithm.h"
#include "btContactProcessing.h"
#include "LinearMath/btQuickprof.h"

#include <chrono>
#include <algorithm>
#include <execution>
#include <list>
#include <map>
#include <array>
#include <span>

//! Class for accessing the plane equation
class btPlaneShape : public btStaticPlaneShape
{
public:
	btPlaneShape(const btVector3& v, float f)
		: btStaticPlaneShape(v, f)
	{
	}

	void get_plane_equation(btVector4& equation)
	{
		equation[0] = m_planeNormal[0];
		equation[1] = m_planeNormal[1];
		equation[2] = m_planeNormal[2];
		equation[3] = m_planeConstant;
	}

	void get_plane_equation_transformed(const btTransform& trans, btVector4& equation) const
	{
		const btVector3 normal = trans.getBasis() * m_planeNormal;
		equation[0] = normal[0];
		equation[1] = normal[1];
		equation[2] = normal[2];
		equation[3] = normal.dot(trans * (m_planeConstant * m_planeNormal));
	}
};

//////////////////////////////////////////////////////////////////////////////////////////////
#ifdef TRI_COLLISION_PROFILING

btClock g_triangle_clock;

float g_accum_triangle_collision_time = 0;
int g_count_triangle_collision = 0;

void bt_begin_gim02_tri_time()
{
	g_triangle_clock.reset();
}

void bt_end_gim02_tri_time()
{
	g_accum_triangle_collision_time += g_triangle_clock.getTimeMicroseconds();
	g_count_triangle_collision++;
}
#endif  //TRI_COLLISION_PROFILING
//! Retrieving shapes shapes
/*!
Declared here due of insuficent space on Pool allocators
*/
//!@{
class GIM_ShapeRetriever
{
public:
	const btGImpactShapeInterface* m_gim_shape;
	btTriangleShapeEx m_trishape;
	btTetrahedronShapeEx m_tetrashape;

public:
	class ChildShapeRetriever
	{
	public:
		GIM_ShapeRetriever* m_parent;
		virtual const btCollisionShape* getChildShape(int index)
		{
			return m_parent->m_gim_shape->getChildShape(index);
		}
		virtual ~ChildShapeRetriever() {}
	};

	class TriangleShapeRetriever : public ChildShapeRetriever
	{
	public:
		virtual btCollisionShape* getChildShape(int index)
		{
			m_parent->m_gim_shape->getBulletTriangle(index, m_parent->m_trishape);
			return &m_parent->m_trishape;
		}
		virtual ~TriangleShapeRetriever() {}
	};

	class TetraShapeRetriever : public ChildShapeRetriever
	{
	public:
		virtual btCollisionShape* getChildShape(int index)
		{
			m_parent->m_gim_shape->getBulletTetrahedron(index, m_parent->m_tetrashape);
			return &m_parent->m_tetrashape;
		}
	};

public:
	ChildShapeRetriever m_child_retriever;
	TriangleShapeRetriever m_tri_retriever;
	TetraShapeRetriever m_tetra_retriever;
	ChildShapeRetriever* m_current_retriever;

	GIM_ShapeRetriever(const btGImpactShapeInterface* gim_shape)
	{
		m_gim_shape = gim_shape;
		//select retriever
		if (m_gim_shape->needsRetrieveTriangles())
		{
			m_current_retriever = &m_tri_retriever;
		}
		else if (m_gim_shape->needsRetrieveTetrahedrons())
		{
			m_current_retriever = &m_tetra_retriever;
		}
		else
		{
			m_current_retriever = &m_child_retriever;
		}

		m_current_retriever->m_parent = this;
	}

	const btCollisionShape* getChildShape(int index)
	{
		return m_current_retriever->getChildShape(index);
	}
};

//!@}

#ifdef TRI_COLLISION_PROFILING

//! Gets the average time in miliseconds of tree collisions
float btGImpactCollisionAlgorithm::getAverageTreeCollisionTime()
{
	return btGImpactBoxSet::getAverageTreeCollisionTime();
}

//! Gets the average time in miliseconds of triangle collisions
float btGImpactCollisionAlgorithm::getAverageTriangleCollisionTime()
{
	if (g_count_triangle_collision == 0) return 0;

	float avgtime = g_accum_triangle_collision_time;
	avgtime /= (float)g_count_triangle_collision;

	g_accum_triangle_collision_time = 0;
	g_count_triangle_collision = 0;

	return avgtime;
}

#endif  //TRI_COLLISION_PROFILING

btGImpactCollisionAlgorithm::btGImpactCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
	: btActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap)
{
	m_manifoldPtr = NULL;
	m_convex_algorithm = NULL;
}

btGImpactCollisionAlgorithm::~btGImpactCollisionAlgorithm()
{
	clearCache();
}

void btGImpactCollisionAlgorithm::addContactPoint(const btCollisionObjectWrapper* body0Wrap,
												  const btCollisionObjectWrapper* body1Wrap,
												  const btVector3& point,
												  const btVector3& normal,
												  btScalar distance)
{
	m_resultOut->setShapeIdentifiersA(m_part0, m_triface0);
	m_resultOut->setShapeIdentifiersB(m_part1, m_triface1);
	checkManifold(body0Wrap, body1Wrap);
	m_resultOut->addContactPoint(normal, point, distance);
}

void btGImpactCollisionAlgorithm::shape_vs_shape_collision(
	const btCollisionObjectWrapper* body0Wrap,
	const btCollisionObjectWrapper* body1Wrap,
	const btCollisionShape* shape0,
	const btCollisionShape* shape1)
{
	{
		btCollisionAlgorithm* algor = newAlgorithm(body0Wrap, body1Wrap);
		// post :	checkManifold is called

		m_resultOut->setShapeIdentifiersA(m_part0, m_triface0);
		m_resultOut->setShapeIdentifiersB(m_part1, m_triface1);

		algor->processCollision(body0Wrap, body1Wrap, *m_dispatchInfo, m_resultOut);

		algor->~btCollisionAlgorithm();
		m_dispatcher->freeCollisionAlgorithm(algor);
	}
}

void btGImpactCollisionAlgorithm::convex_vs_convex_collision(
	const btCollisionObjectWrapper* body0Wrap,
	const btCollisionObjectWrapper* body1Wrap,
	const btCollisionShape* shape0,
	const btCollisionShape* shape1)
{
	m_resultOut->setShapeIdentifiersA(m_part0, m_triface0);
	m_resultOut->setShapeIdentifiersB(m_part1, m_triface1);

	btCollisionObjectWrapper ob0(body0Wrap, shape0, body0Wrap->getCollisionObject(), body0Wrap->getWorldTransform(), m_part0, m_triface0);
	btCollisionObjectWrapper ob1(body1Wrap, shape1, body1Wrap->getCollisionObject(), body1Wrap->getWorldTransform(), m_part1, m_triface1);
	checkConvexAlgorithm(&ob0, &ob1);
	m_convex_algorithm->processCollision(&ob0, &ob1, *m_dispatchInfo, m_resultOut);
}

void btGImpactCollisionAlgorithm::gimpact_vs_gimpact_find_pairs(
	const btGimpactVsGimpactGroupedParams& grpParams,
	const btTransform& trans0,
	const btTransform& trans1,
	ThreadLocalGImpactResult& perThreadIntermediateResults, btPairSet& auxPairSet, bool findOnlyFirstPair)
{
	if (grpParams.shape0->hasBoxSet() && grpParams.shape1->hasBoxSet())
	{
		btGImpactBoxSet::find_collision(grpParams.shape0->getBoxSet(), trans0, grpParams.shape1->getBoxSet(), trans1, perThreadIntermediateResults, auxPairSet, findOnlyFirstPair, grpParams);
	}
	else
	{
		btAABB boxshape0;
		btAABB boxshape1;
		int i = grpParams.shape0->getNumChildShapes();

		while (i--)
		{
			grpParams.shape0->getChildAabb(i, trans0, boxshape0.m_min, boxshape0.m_max);

			int j = grpParams.shape1->getNumChildShapes();
			while (j--)
			{
				grpParams.shape1->getChildAabb(i, trans1, boxshape1.m_min, boxshape1.m_max);

				if (boxshape1.has_collision(boxshape0))
				{
					auxPairSet.push_back({i, j});
				}
			}
		}
	}
}

void btGImpactCollisionAlgorithm::gimpact_vs_shape_find_pairs(
	const btTransform& trans0,
	const btTransform& trans1,
	const btGImpactShapeInterface* shape0,
	const btCollisionShape* shape1,
	btAlignedObjectArray<int>& collided_primitives)
{
	btAABB boxshape;

	if (shape0->hasBoxSet())
	{
		btTransform trans1to0 = trans0.inverse();
		trans1to0 *= trans1;

		shape1->getAabb(trans1to0, boxshape.m_min, boxshape.m_max);

		shape0->getBoxSet()->boxQuery(boxshape, collided_primitives);
	}
	else
	{
		shape1->getAabb(trans1, boxshape.m_min, boxshape.m_max);

		btAABB boxshape0;
		int i = shape0->getNumChildShapes();

		while (i--)
		{
			shape0->getChildAabb(i, trans0, boxshape0.m_min, boxshape0.m_max);

			if (boxshape.has_collision(boxshape0))
			{
				collided_primitives.push_back(i);
			}
		}
	}
}

void btGImpactCollisionAlgorithm::collide_gjk_triangles(const btCollisionObjectWrapper* body0Wrap,
														const btCollisionObjectWrapper* body1Wrap,
														const btGImpactMeshShapePart* shape0,
														const btGImpactMeshShapePart* shape1,
														const int* pairs, int pair_count)
{
	btTriangleShapeEx tri0;
	btTriangleShapeEx tri1;

	shape0->lockChildShapes();
	shape1->lockChildShapes();

	const int* pair_pointer = pairs;

	while (pair_count--)
	{
		m_triface0 = *(pair_pointer);
		m_triface1 = *(pair_pointer + 1);
		pair_pointer += 2;

		shape0->getBulletTriangle(m_triface0, tri0);
		shape1->getBulletTriangle(m_triface1, tri1);

		//collide two convex shapes
		if (tri0.overlap_test_conservative(tri1))
		{
			convex_vs_convex_collision(body0Wrap, body1Wrap, &tri0, &tri1);
		}
	}

	shape0->unlockChildShapes();
	shape1->unlockChildShapes();
}

int frameCnt = 0;
// Designed to be used with MeshLab. The generated dbg file is opened there and then the dbg_*_bbox bounding box file is imported into the same scene.
// I could not get MeshLab to import both triangles and lines in one file - therefore two files are generated.
void debug_pairs(const std::span<const std::pair<int, int>>& pairSpan, const btTransform& orgtrans0, const btTransform& orgtrans1,
											  const btGImpactMeshShapePart* shape0, const btGImpactMeshShapePart* shape1)
{
	BT_BOX_BOX_TRANSFORM_CACHE trans_cache_1to0;
	trans_cache_1to0.calc_from_homogenic(orgtrans0, orgtrans1);

	std::map<int, int> occurrences, occurrences2;
	std::multimap<int, int> occurrences_rev, occurrences_rev2;
	for (int i = 0; i < pairSpan.size(); ++i)
	{
		occurrences[pairSpan[i].first]++;
		occurrences2[pairSpan[i].second]++;
	}
	for (auto elem : occurrences)
	{
		occurrences_rev.insert({elem.second, elem.first});
	}
	for (auto elem : occurrences2)
	{
		occurrences_rev2.insert({elem.second, elem.first});
	}
	int writeSize = 10000;
	bool write = occurrences_rev.size() > writeSize;
	FILE* fh = nullptr;
	std::string fname;
	std::string fname_base = std::to_string(frameCnt++) + "_" + std::to_string(occurrences_rev.size()) + "_" + std::to_string(occurrences_rev2.size()) + "_" + std::to_string(pairSpan.size());
	fname = "dbg" + fname_base + ".obj";
	if (write)
		fh = fopen(fname.c_str(), "w");
	int maxOccur = 1;
	int bboxCnt = 0;
	if (!occurrences_rev.empty())
	{
		maxOccur = occurrences_rev.rbegin()->first;
	}
	for (auto elemIter = occurrences_rev.rbegin(); elemIter != occurrences_rev.rend(); ++elemIter)
	{
		btPrimitiveTriangle ptri0;
		btPrimitiveTriangle ptri1;

		auto faceIndex = elemIter->second;
		shape0->getPrimitiveTriangle(faceIndex, ptri0);
		if (fh)
		{
			btScalar occur = elemIter->first / (btScalar)maxOccur;
			fprintf(fh, "v %f %f %f %f %f %f\n", ptri0.m_vertices[0].x(), ptri0.m_vertices[0].y(), ptri0.m_vertices[0].z(), occur, occur, occur);
			fprintf(fh, "v %f %f %f %f %f %f\n", ptri0.m_vertices[1].x(), ptri0.m_vertices[1].y(), ptri0.m_vertices[1].z(), occur, occur, occur);
			fprintf(fh, "v %f %f %f %f %f %f\n", ptri0.m_vertices[2].x(), ptri0.m_vertices[2].y(), ptri0.m_vertices[2].z(), occur, occur, occur);
		}
	}
	maxOccur = 1;
	if (!occurrences_rev2.empty())
	{
		maxOccur = occurrences_rev2.rbegin()->first;
	}
	for (auto elemIter = occurrences_rev2.rbegin(); elemIter != occurrences_rev2.rend(); ++elemIter)
	{
		btPrimitiveTriangle ptri0;
		btPrimitiveTriangle ptri1;

		shape1->getPrimitiveTriangle(elemIter->second, ptri0);
		btTransform tran;
		tran.setOrigin(trans_cache_1to0.m_T1to0);
		tran.setBasis(trans_cache_1to0.m_R1to0);
		ptri0.applyTransform(tran);
		if (fh)
		{
			btScalar occur = elemIter->first / (btScalar)maxOccur;
			fprintf(fh, "v %f %f %f %f %f %f\n", ptri0.m_vertices[0].x(), ptri0.m_vertices[0].y(), ptri0.m_vertices[0].z(), occur, occur, occur);
			fprintf(fh, "v %f %f %f %f %f %f\n", ptri0.m_vertices[1].x(), ptri0.m_vertices[1].y(), ptri0.m_vertices[1].z(), occur, occur, occur);
			fprintf(fh, "v %f %f %f %f %f %f\n", ptri0.m_vertices[2].x(), ptri0.m_vertices[2].y(), ptri0.m_vertices[2].z(), occur, occur, occur);
		}
	}

	int fcnt = 1;
	for (auto elemIter = occurrences_rev.rbegin(); elemIter != occurrences_rev.rend(); ++elemIter)
	{
		if (fh)
			fprintf(fh, "f %d %d %d\n", fcnt, fcnt + 1, fcnt + 2);
		fcnt += 3;
	}
	for (auto elemIter = occurrences_rev2.rbegin(); elemIter != occurrences_rev2.rend(); ++elemIter)
	{
		if (fh)
			fprintf(fh, "f %d %d %d\n", fcnt, fcnt + 1, fcnt + 2);
		fcnt += 3;
	}
	if (fh)
		fclose(fh);

	if (write)
	{
		fname = "dbg" + fname_base + "_bbox.obj";
		fh = fopen(fname.c_str(), "w");
	}
	fcnt = 1;
	if (!occurrences_rev.empty())
	{
		auto elemIter = occurrences_rev.rbegin();
		int faceIndex = elemIter->second;
		btTransform tr;
		tr.setIdentity();
		btVector3 col(1.0f, 0.0f, 0.0f);

		auto addBBoxVerts = [&](const btGImpactMeshShapePart* shp, int face, btTransform& t, const btVector3& color)
		{
			btTransform trUnused;
			trUnused.setIdentity();
			btAABB box0;
			shp->getChildAabb(face, trUnused, box0.m_min, box0.m_max);
			std::array<btVector3, 8> verts;
			verts[0] = btVector3(box0.m_min.x(), box0.m_min.y(), box0.m_min.z());
			verts[1] = btVector3(box0.m_max.x(), box0.m_min.y(), box0.m_min.z());
			verts[2] = btVector3(box0.m_max.x(), box0.m_max.y(), box0.m_min.z());
			verts[3] = btVector3(box0.m_min.x(), box0.m_max.y(), box0.m_min.z());
			verts[4] = btVector3(box0.m_min.x(), box0.m_min.y(), box0.m_max.z());
			verts[5] = btVector3(box0.m_max.x(), box0.m_min.y(), box0.m_max.z());
			verts[6] = btVector3(box0.m_max.x(), box0.m_max.y(), box0.m_max.z());
			verts[7] = btVector3(box0.m_min.x(), box0.m_max.y(), box0.m_max.z());
			for (auto& v : verts)
			{
				v = t(v);
			}
			if (fh)
			{
				for (auto& v : verts)
					fprintf(fh, "v %f %f %f %f %f %f\n", v.x(), v.y(), v.z(), color.x(), color.y(), color.z());
			}
			++bboxCnt;
		};

		addBBoxVerts(shape0, faceIndex, tr, col);

		std::list<int> opposingFaces;
		for (int i = 0; i < pairSpan.size(); ++i)
		{
			if (pairSpan[i].first == faceIndex)
			{
				opposingFaces.push_back(pairSpan[i].second);
			}
		}
		tr.setOrigin(trans_cache_1to0.m_T1to0);
		tr.setBasis(trans_cache_1to0.m_R1to0);
		col = btVector3(0.0f, 0.0f, 1.0f);
		for (auto oppFace : opposingFaces)
		{
			addBBoxVerts(shape1, oppFace, tr, col);
		}
	}
	for (int b = 0; b < bboxCnt; ++b)
	{
		if (fh)
		{
			fprintf(fh, "l %d %d\n", fcnt, fcnt + 1);
			fprintf(fh, "l %d %d\n", fcnt + 1, fcnt + 2);
			fprintf(fh, "l %d %d\n", fcnt + 2, fcnt + 3);
			fprintf(fh, "l %d %d\n", fcnt + 3, fcnt);

			fprintf(fh, "l %d %d\n", fcnt + 4, fcnt + 5);
			fprintf(fh, "l %d %d\n", fcnt + 5, fcnt + 6);
			fprintf(fh, "l %d %d\n", fcnt + 6, fcnt + 7);
			fprintf(fh, "l %d %d\n", fcnt + 7, fcnt + 4);

			fprintf(fh, "l %d %d\n", fcnt, fcnt + 4);
			fprintf(fh, "l %d %d\n", fcnt + 1, fcnt + 5);
			fprintf(fh, "l %d %d\n", fcnt + 2, fcnt + 6);
			fprintf(fh, "l %d %d\n", fcnt + 3, fcnt + 7);
			fcnt += 8;
		}
	}
	if (fh)
		fclose(fh);
}

void btGImpactCollisionAlgorithm::collide_sat_triangles_pre(const btCollisionObjectWrapper* body0Wrap,
														const btCollisionObjectWrapper* body1Wrap,
														const btGImpactMeshShapePart* shape0,
														const btGImpactMeshShapePart* shape1,
														btGimpactVsGimpactGroupedParams& grpParams
														)
{
	grpParams.orgtrans0 = body0Wrap->getWorldTransform();
	grpParams.orgtrans1 = body1Wrap->getWorldTransform();

	//printf("pair_count %d\n", pair_count);

	bool isStatic0 = body0Wrap->getCollisionObject()->isStaticObject();
	bool isStatic1 = body1Wrap->getCollisionObject()->isStaticObject();

	grpParams.shape0 = shape0;
	grpParams.shape1 = shape1;
	grpParams.doUnstuck = (isStatic0 ? body1Wrap : body0Wrap)->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_DO_UNSTUCK;
	grpParams.lastSafeTrans0 = isStatic0 ? grpParams.orgtrans0 : body0Wrap->getCollisionObject()->getLastSafeWorldTransform();
	grpParams.lastSafeTrans1 = isStatic1 ? grpParams.orgtrans1 : body1Wrap->getCollisionObject()->getLastSafeWorldTransform();
	
	const auto& prevTimeMap = m_dispatcher->getPreviouslyConsumedTime();
	auto timeIter = prevTimeMap.find({body0Wrap->getCollisionObject()->getUserIndex(), body1Wrap->getCollisionObject()->getUserIndex()});
	if (timeIter != prevTimeMap.end())
	{
		grpParams.previouslyConsumedTime = timeIter->second;
	}
	else
	{
		grpParams.previouslyConsumedTime = 0;
	}

	shape0->lockChildShapes();
	shape1->lockChildShapes();
}

void btGImpactCollisionAlgorithm::collide_sat_triangles_post(const ThreadLocalGImpactResult* perThreadIntermediateResults,
															 const std::list<btGImpactIntermediateResult>* intermediateResults,
															const btCollisionObjectWrapper* body0Wrap,
															const btCollisionObjectWrapper* body1Wrap,
															const btGImpactMeshShapePart* shape0,
															const btGImpactMeshShapePart* shape1)
{
	if (perThreadIntermediateResults)
	{
		for (const auto& perThreadIntermediateResult : *perThreadIntermediateResults)
		{
			for (const auto& ir : perThreadIntermediateResult)
			{
				addContactPoint(body0Wrap, body1Wrap,
								ir.point,
								ir.normal,
								ir.depth);
			}
		}
	}
	if (intermediateResults)
	{
		for (const auto& ir : *intermediateResults)
		{
			addContactPoint(body0Wrap, body1Wrap,
							ir.point,
							ir.normal,
							ir.depth);
		}
	}

	shape0->unlockChildShapes();
	shape1->unlockChildShapes();
}

void btGImpactCollisionAlgorithm::collide_sat_triangles_aux(const btCollisionObjectWrapper* body0Wrap,
														const btCollisionObjectWrapper* body1Wrap,
														const btGImpactMeshShapePart* shape0,
														const btGImpactMeshShapePart* shape1,
														const btPairSet& auxPairSet)
{
	btGimpactVsGimpactGroupedParams grpParams(m_triface0, m_triface1);
	collide_sat_triangles_pre(body0Wrap, body1Wrap, shape0, shape1, grpParams);

	std::list<btGImpactIntermediateResult> intermediateResults;
	for (auto pairIter = auxPairSet.begin(); pairIter != auxPairSet.end(); ++pairIter)
	{
		btGImpactPairEval::EvalPair(*pairIter, grpParams, nullptr, &intermediateResults);
	}

	collide_sat_triangles_post(nullptr, &intermediateResults, body0Wrap, body1Wrap, shape0, shape1);
}

void btGImpactCollisionAlgorithm::gimpact_vs_gimpact(
	const btCollisionObjectWrapper* body0Wrap,
	const btCollisionObjectWrapper* body1Wrap,
	const btGImpactShapeInterface* shape0,
	const btGImpactShapeInterface* shape1)
{
	if (shape0->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE)
	{
		const btGImpactMeshShape* meshshape0 = static_cast<const btGImpactMeshShape*>(shape0);
		m_part0 = meshshape0->getMeshPartCount();

		while (m_part0--)
		{
			gimpact_vs_gimpact(body0Wrap, body1Wrap, meshshape0->getMeshPart(m_part0), shape1);
		}

		return;
	}

	if (shape1->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE)
	{
		const btGImpactMeshShape* meshshape1 = static_cast<const btGImpactMeshShape*>(shape1);
		m_part1 = meshshape1->getMeshPartCount();

		while (m_part1--)
		{
			gimpact_vs_gimpact(body0Wrap, body1Wrap, shape0, meshshape1->getMeshPart(m_part1));
		}

		return;
	}

	bool lowDetail0, lowDetail1;
	bool isTol0 = body0Wrap->getCollisionObject()->isToleratingInitialCollisionsAll(lowDetail0);
	bool isTol1 = body1Wrap->getCollisionObject()->isToleratingInitialCollisionsAll(lowDetail1);
	bool isGhost0 = body0Wrap->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE;
	bool isGhost1 = body1Wrap->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE;
	bool findOnlyFirstPair = isTol0 || isTol1;
	if (body0Wrap->getCollisionObject()->isToleratingCertainInitialCollisions() || body1Wrap->getCollisionObject()->isToleratingCertainInitialCollisions())
	{
		bool check0 = body0Wrap->getCollisionObject()->checkIsTolerated(body1Wrap->getCollisionObject());
		bool check1 = body1Wrap->getCollisionObject()->checkIsTolerated(body0Wrap->getCollisionObject());
		findOnlyFirstPair = (check0 || check1);
	}
	bool generateManifoldForGhost = isGhost0 || isGhost1;
	findOnlyFirstPair |= generateManifoldForGhost;

	if (findOnlyFirstPair && (lowDetail0 || lowDetail1))
	{
		const btCollisionObject* checked = nullptr;
		if (isTol0)
			checked = body0Wrap->getCollisionObject();
		else if (isTol1)
			checked = body1Wrap->getCollisionObject();
		auto& participants = isTol0 ? m_dispatcher->getInitialCollisionParticipants0() : m_dispatcher->getInitialCollisionParticipants1();
		if (checked && participants.find(checked) != participants.end())
		{
			// There already was some collision with some other body and the details are low. No need to waste time checking with this and the remaining bodies
			return;
		}
	}

	auxPairSet.clear();
	perThreadIntermediateResults.clear();

	btGimpactVsGimpactGroupedParams grpParams(m_triface0, m_triface1);

	if (shape0->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART &&
		shape1->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART)
	{
		const btGImpactMeshShapePart* shapepart0 = static_cast<const btGImpactMeshShapePart*>(shape0);
		const btGImpactMeshShapePart* shapepart1 = static_cast<const btGImpactMeshShapePart*>(shape1);
		collide_sat_triangles_pre(body0Wrap, body1Wrap, shapepart0, shapepart1, grpParams);
	}

	auto start = std::chrono::steady_clock::now();
	gimpact_vs_gimpact_find_pairs(grpParams, grpParams.orgtrans0, grpParams.orgtrans1, perThreadIntermediateResults, auxPairSet, findOnlyFirstPair);
	auto end = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	m_dispatcher->addPreviouslyConsumedTime({body0Wrap->getCollisionObject()->getUserIndex(), body1Wrap->getCollisionObject()->getUserIndex()}, duration.count());

	bool pairsExist = false;
	for (auto perThreadIter = perThreadIntermediateResults.begin(); perThreadIter != perThreadIntermediateResults.end(); ++perThreadIter)
	{
		pairsExist |= !perThreadIter->empty();
		if (pairsExist)
			break;
	}
	if (!pairsExist)
		return;

	if (findOnlyFirstPair && !generateManifoldForGhost)
	{
		m_dispatcher->addInitialCollisionParticipant({body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject()});
		return;
	}

	if (shape0->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART &&
		shape1->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART)
	{
		const btGImpactMeshShapePart* shapepart0 = static_cast<const btGImpactMeshShapePart*>(shape0);
		const btGImpactMeshShapePart* shapepart1 = static_cast<const btGImpactMeshShapePart*>(shape1);
		collide_sat_triangles_post(&perThreadIntermediateResults, nullptr, body0Wrap, body1Wrap, shapepart0, shapepart1);
	}

	//printf("pairset.size() %d\n", pairset.size());

	if (shape0->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART &&
		shape1->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART)
	{
		const btGImpactMeshShapePart* shapepart0 = static_cast<const btGImpactMeshShapePart*>(shape0);
		const btGImpactMeshShapePart* shapepart1 = static_cast<const btGImpactMeshShapePart*>(shape1);
//specialized function
#ifdef BULLET_TRIANGLE_COLLISION
		collide_gjk_triangles(body0Wrap, body1Wrap, shapepart0, shapepart1, &pairset[0].m_index1, pairset.size());
#else
		collide_sat_triangles_aux(body0Wrap, body1Wrap, shapepart0, shapepart1, auxPairSet);
#endif

		return;
	}

	btAssert(false); // Removed some code here not relevant to my use case.
}

void btGImpactCollisionAlgorithm::gimpact_vs_shape(const btCollisionObjectWrapper* body0Wrap,
												   const btCollisionObjectWrapper* body1Wrap,
												   const btGImpactShapeInterface* shape0,
												   const btCollisionShape* shape1, bool swapped)
{
	if (shape0->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE)
	{
		const btGImpactMeshShape* meshshape0 = static_cast<const btGImpactMeshShape*>(shape0);
		int& part = swapped ? m_part1 : m_part0;
		part = meshshape0->getMeshPartCount();

		while (part--)
		{
			gimpact_vs_shape(body0Wrap,
							 body1Wrap,
							 meshshape0->getMeshPart(part),
							 shape1, swapped);
		}

		return;
	}

#ifdef GIMPACT_VS_PLANE_COLLISION
	if (shape0->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE_PART &&
		shape1->getShapeType() == STATIC_PLANE_PROXYTYPE)
	{
		const btGImpactMeshShapePart* shapepart = static_cast<const btGImpactMeshShapePart*>(shape0);
		const btStaticPlaneShape* planeshape = static_cast<const btStaticPlaneShape*>(shape1);
		gimpacttrimeshpart_vs_plane_collision(body0Wrap, body1Wrap, shapepart, planeshape, swapped);
		return;
	}

#endif

	if (shape1->isCompound())
	{
		const btCompoundShape* compoundshape = static_cast<const btCompoundShape*>(shape1);
		gimpact_vs_compoundshape(body0Wrap, body1Wrap, shape0, compoundshape, swapped);
		return;
	}
	else if (shape1->isConcave())
	{
		const btConcaveShape* concaveshape = static_cast<const btConcaveShape*>(shape1);
		gimpact_vs_concave(body0Wrap, body1Wrap, shape0, concaveshape, swapped);
		return;
	}

	btTransform orgtrans0 = body0Wrap->getWorldTransform();

	btTransform orgtrans1 = body1Wrap->getWorldTransform();

	btAlignedObjectArray<int> collided_results;

	gimpact_vs_shape_find_pairs(orgtrans0, orgtrans1, shape0, shape1, collided_results);

	if (collided_results.size() == 0) return;

	shape0->lockChildShapes();

	GIM_ShapeRetriever retriever0(shape0);

	bool child_has_transform0 = shape0->childrenHasTransform();

	int i = collided_results.size();

	while (i--)
	{
		int child_index = collided_results[i];
		if (swapped)
			m_triface1 = child_index;
		else
			m_triface0 = child_index;

		const btCollisionShape* colshape0 = retriever0.getChildShape(child_index);

		btTransform tr0 = body0Wrap->getWorldTransform();

		if (child_has_transform0)
		{
			tr0 = orgtrans0 * shape0->getChildTransform(child_index);
		}

		btCollisionObjectWrapper ob0(body0Wrap, colshape0, body0Wrap->getCollisionObject(), body0Wrap->getWorldTransform(), m_part0, m_triface0);
		const btCollisionObjectWrapper* prevObj;

		if (m_resultOut->getBody0Wrap()->getCollisionObject() == ob0.getCollisionObject())
		{
			prevObj = m_resultOut->getBody0Wrap();
			m_resultOut->setBody0Wrap(&ob0);
		}
		else
		{
			prevObj = m_resultOut->getBody1Wrap();
			m_resultOut->setBody1Wrap(&ob0);
		}

		//collide two shapes
		if (swapped)
		{
			shape_vs_shape_collision(body1Wrap, &ob0, shape1, colshape0);
		}
		else
		{
			shape_vs_shape_collision(&ob0, body1Wrap, colshape0, shape1);
		}

		if (m_resultOut->getBody0Wrap()->getCollisionObject() == ob0.getCollisionObject())
		{
			m_resultOut->setBody0Wrap(prevObj);
		}
		else
		{
			m_resultOut->setBody1Wrap(prevObj);
		}
	}

	shape0->unlockChildShapes();
}

void btGImpactCollisionAlgorithm::gimpact_vs_compoundshape(const btCollisionObjectWrapper* body0Wrap,
														   const btCollisionObjectWrapper* body1Wrap,
														   const btGImpactShapeInterface* shape0,
														   const btCompoundShape* shape1, bool swapped)
{
	btTransform orgtrans1 = body1Wrap->getWorldTransform();

	int i = shape1->getNumChildShapes();
	while (i--)
	{
		const btCollisionShape* colshape1 = shape1->getChildShape(i);
		btTransform childtrans1 = orgtrans1 * shape1->getChildTransform(i);

		btCollisionObjectWrapper ob1(body1Wrap, colshape1, body1Wrap->getCollisionObject(), childtrans1, -1, i);

		const btCollisionObjectWrapper* tmp = 0;
		if (m_resultOut->getBody0Wrap()->getCollisionObject() == ob1.getCollisionObject())
		{
			tmp = m_resultOut->getBody0Wrap();
			m_resultOut->setBody0Wrap(&ob1);
		}
		else
		{
			tmp = m_resultOut->getBody1Wrap();
			m_resultOut->setBody1Wrap(&ob1);
		}
		//collide child shape
		gimpact_vs_shape(body0Wrap, &ob1,
						 shape0, colshape1, swapped);

		if (m_resultOut->getBody0Wrap()->getCollisionObject() == ob1.getCollisionObject())
		{
			m_resultOut->setBody0Wrap(tmp);
		}
		else
		{
			m_resultOut->setBody1Wrap(tmp);
		}
	}
}

void btGImpactCollisionAlgorithm::gimpacttrimeshpart_vs_plane_collision(
	const btCollisionObjectWrapper* body0Wrap,
	const btCollisionObjectWrapper* body1Wrap,
	const btGImpactMeshShapePart* shape0,
	const btStaticPlaneShape* shape1, bool swapped)
{
	btTransform orgtrans0 = body0Wrap->getWorldTransform();
	btTransform orgtrans1 = body1Wrap->getWorldTransform();

	const btPlaneShape* planeshape = static_cast<const btPlaneShape*>(shape1);
	btVector4 plane;
	planeshape->get_plane_equation_transformed(orgtrans1, plane);

	//test box against plane

	btAABB tribox;
	shape0->getAabb(orgtrans0, tribox.m_min, tribox.m_max);
	tribox.increment_margin(planeshape->getMargin());

	if (tribox.plane_classify(plane) != BT_CONST_COLLIDE_PLANE) return;

	shape0->lockChildShapes();

	btScalar margin = shape0->getMargin() + planeshape->getMargin();

	btVector3 vertex;
	int vi = shape0->getVertexCount();
	while (vi--)
	{
		shape0->getVertex(vi, vertex);
		vertex = orgtrans0(vertex);

		btScalar distance = vertex.dot(plane) - plane[3] - margin;

		if (distance < 0.0)  //add contact
		{
			if (swapped)
			{
				addContactPoint(body1Wrap, body0Wrap,
								vertex,
								-plane,
								distance);
			}
			else
			{
				addContactPoint(body0Wrap, body1Wrap,
								vertex,
								plane,
								distance);
			}
		}
	}

	shape0->unlockChildShapes();
}

class btGImpactTriangleCallback : public btTriangleCallback
{
public:
	btGImpactCollisionAlgorithm* algorithm;
	const btCollisionObjectWrapper* body0Wrap;
	const btCollisionObjectWrapper* body1Wrap;
	const btGImpactShapeInterface* gimpactshape0;
	bool swapped;
	btScalar margin;

	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
	{
		btTriangleShapeEx tri1(triangle[0], triangle[1], triangle[2]);
		tri1.setMargin(margin);
		if (swapped)
		{
			algorithm->setPart0(partId);
			algorithm->setFace0(triangleIndex);
		}
		else
		{
			algorithm->setPart1(partId);
			algorithm->setFace1(triangleIndex);
		}

		btCollisionObjectWrapper ob1Wrap(body1Wrap, &tri1, body1Wrap->getCollisionObject(), body1Wrap->getWorldTransform(), partId, triangleIndex);
		const btCollisionObjectWrapper* tmp = 0;

		if (algorithm->internalGetResultOut()->getBody0Wrap()->getCollisionObject() == ob1Wrap.getCollisionObject())
		{
			tmp = algorithm->internalGetResultOut()->getBody0Wrap();
			algorithm->internalGetResultOut()->setBody0Wrap(&ob1Wrap);
		}
		else
		{
			tmp = algorithm->internalGetResultOut()->getBody1Wrap();
			algorithm->internalGetResultOut()->setBody1Wrap(&ob1Wrap);
		}

		algorithm->gimpact_vs_shape(
			body0Wrap, &ob1Wrap, gimpactshape0, &tri1, swapped);

		if (algorithm->internalGetResultOut()->getBody0Wrap()->getCollisionObject() == ob1Wrap.getCollisionObject())
		{
			algorithm->internalGetResultOut()->setBody0Wrap(tmp);
		}
		else
		{
			algorithm->internalGetResultOut()->setBody1Wrap(tmp);
		}
	}
};

void btGImpactCollisionAlgorithm::gimpact_vs_concave(
	const btCollisionObjectWrapper* body0Wrap,
	const btCollisionObjectWrapper* body1Wrap,
	const btGImpactShapeInterface* shape0,
	const btConcaveShape* shape1, bool swapped)
{
	//create the callback
	btGImpactTriangleCallback tricallback;
	tricallback.algorithm = this;
	tricallback.body0Wrap = body0Wrap;
	tricallback.body1Wrap = body1Wrap;
	tricallback.gimpactshape0 = shape0;
	tricallback.swapped = swapped;
	tricallback.margin = shape1->getMargin();

	//getting the trimesh AABB
	btTransform gimpactInConcaveSpace;

	gimpactInConcaveSpace = body1Wrap->getWorldTransform().inverse() * body0Wrap->getWorldTransform();

	btVector3 minAABB, maxAABB;
	shape0->getAabb(gimpactInConcaveSpace, minAABB, maxAABB);

	shape1->processAllTriangles(&tricallback, minAABB, maxAABB);
}

void btGImpactCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	clearCache();

	m_resultOut = resultOut;
	m_dispatchInfo = &dispatchInfo;
	const btGImpactShapeInterface* gimpactshape0;
	const btGImpactShapeInterface* gimpactshape1;

	if (body0Wrap->getCollisionShape()->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
	{
		gimpactshape0 = static_cast<const btGImpactShapeInterface*>(body0Wrap->getCollisionShape());

		if (body1Wrap->getCollisionShape()->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		{
			gimpactshape1 = static_cast<const btGImpactShapeInterface*>(body1Wrap->getCollisionShape());

			gimpact_vs_gimpact(body0Wrap, body1Wrap, gimpactshape0, gimpactshape1);
		}
		else
		{
			gimpact_vs_shape(body0Wrap, body1Wrap, gimpactshape0, body1Wrap->getCollisionShape(), false);
		}
	}
	else if (body1Wrap->getCollisionShape()->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
	{
		gimpactshape1 = static_cast<const btGImpactShapeInterface*>(body1Wrap->getCollisionShape());

		gimpact_vs_shape(body1Wrap, body0Wrap, gimpactshape1, body0Wrap->getCollisionShape(), true);
	}

	// Ensure that gContactProcessedCallback is called for concave shapes.
	if (getLastManifold())
	{
		m_resultOut->refreshContactPoints();
	}
}

btScalar btGImpactCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	return 1.f;
}

btThreadPoolForBvh* btGImpactCollisionAlgorithm::m_threadPool = nullptr;

///////////////////////////////////// REGISTERING ALGORITHM //////////////////////////////////////////////

//! Use this function for register the algorithm externally
void btGImpactCollisionAlgorithm::registerAlgorithm(btCollisionDispatcher* dispatcher)
{
	static btGImpactCollisionAlgorithm::CreateFunc s_gimpact_cf;

	int i;

	for (i = 0; i < MAX_BROADPHASE_COLLISION_TYPES; i++)
	{
		dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE, i, &s_gimpact_cf);
	}

	for (i = 0; i < MAX_BROADPHASE_COLLISION_TYPES; i++)
	{
		dispatcher->registerCollisionCreateFunc(i, GIMPACT_SHAPE_PROXYTYPE, &s_gimpact_cf);
	}
}
