/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans


#include "btConvexUtility.h"
#include "LinearMath/btConvexHullComputer.h"
#include "LinearMath/btGrahamScan2dConvexHull.h"
#include "LinearMath/btQuaternion.h"

bool	btConvexUtility::initializePolyhedralFeatures(const btAlignedObjectArray<btVector3>& orgVertices, bool mergeCoplanarTriangles)
{
	

	btConvexHullComputer conv;
	conv.compute(&orgVertices[0].getX(), sizeof(btVector3),orgVertices.size(),0.f,0.f);

	btAlignedObjectArray<btVector3> faceNormals;
	int numFaces = conv.faces.size();
	faceNormals.resize(numFaces);
	btConvexHullComputer* convexUtil = &conv;

	
	btAlignedObjectArray<btFace>	tmpFaces;
	tmpFaces.resize(numFaces);

	int numVertices = convexUtil->vertices.size();
	m_vertices.resize(numVertices);
	for (int p=0;p<numVertices;p++)
	{
		m_vertices[p] = convexUtil->vertices[p];
	}


	for (int i=0;i<numFaces;i++)
	{
		int face = convexUtil->faces[i];
		//printf("face=%d\n",face);
		const btConvexHullComputer::Edge*  firstEdge = &convexUtil->edges[face];
		const btConvexHullComputer::Edge*  edge = firstEdge;

		btVector3 edges[3];
		int numEdges = 0;
		//compute face normals

		btScalar maxCross2 = 0.f;
		int chosenEdge = -1;

		do
		{
			
			int src = edge->getSourceVertex();
			tmpFaces[i].m_indices.push_back(src);
			int targ = edge->getTargetVertex();
			btVector3 wa = convexUtil->vertices[src];

			btVector3 wb = convexUtil->vertices[targ];
			btVector3 newEdge = wb-wa;
			newEdge.normalize();
			if (numEdges<2)
				edges[numEdges++] = newEdge;

			edge = edge->getNextEdgeOfFace();
		} while (edge!=firstEdge);

		btScalar planeEq = 1e30f;

		
		if (numEdges==2)
		{
			faceNormals[i] = edges[0].cross(edges[1]);
			faceNormals[i].normalize();
			tmpFaces[i].m_plane[0] = faceNormals[i].getX();
			tmpFaces[i].m_plane[1] = faceNormals[i].getY();
			tmpFaces[i].m_plane[2] = faceNormals[i].getZ();
			tmpFaces[i].m_plane[3] = planeEq;

		}
		else
		{
			btAssert(0);//degenerate?
			faceNormals[i].setZero();
		}

		for (int v=0;v<tmpFaces[i].m_indices.size();v++)
		{
			btScalar eq = m_vertices[tmpFaces[i].m_indices[v]].dot(faceNormals[i]);
			if (planeEq>eq)
			{
				planeEq=eq;
			}
		}
		tmpFaces[i].m_plane[3] = -planeEq;
	}

	//merge coplanar faces

	btScalar faceWeldThreshold= 0.999f;
	btAlignedObjectArray<int> todoFaces;
	for (int i=0;i<tmpFaces.size();i++)
		todoFaces.push_back(i);

	while (todoFaces.size())
	{
		btAlignedObjectArray<int> coplanarFaceGroup;
		int refFace = todoFaces[todoFaces.size()-1];

		coplanarFaceGroup.push_back(refFace);
		btFace& faceA = tmpFaces[refFace];
		todoFaces.pop_back();

		btVector3 faceNormalA(faceA.m_plane[0],faceA.m_plane[1],faceA.m_plane[2]);
		for (int j=todoFaces.size()-1;j>=0;j--)
		{
			int i = todoFaces[j];
			btFace& faceB = tmpFaces[i];
			btVector3 faceNormalB(faceB.m_plane[0],faceB.m_plane[1],faceB.m_plane[2]);
			if (faceNormalA.dot(faceNormalB)>faceWeldThreshold)
			{
				coplanarFaceGroup.push_back(i);
				todoFaces.remove(i);
			}
		}


		bool did_merge = false;
		if (mergeCoplanarTriangles && coplanarFaceGroup.size()>1)
		{
			//do the merge: use Graham Scan 2d convex hull

			btAlignedObjectArray<GrahamVector2> orgpoints;

			for (int i=0;i<coplanarFaceGroup.size();i++)
			{

				btFace& face = tmpFaces[coplanarFaceGroup[i]];
				btVector3 faceNormal(face.m_plane[0],face.m_plane[1],face.m_plane[2]);
				btVector3 xyPlaneNormal(0,0,1);

				btQuaternion rotationArc = shortestArcQuat(faceNormal,xyPlaneNormal);
				
				for (int f=0;f<face.m_indices.size();f++)
				{
					int orgIndex = face.m_indices[f];
					btVector3 pt = m_vertices[orgIndex];
					btVector3 rotatedPt =  quatRotate(rotationArc,pt);
					rotatedPt.setZ(0);
					bool found = false;

					for (int i=0;i<orgpoints.size();i++)
					{
						//if ((orgpoints[i].m_orgIndex == orgIndex) || ((rotatedPt-orgpoints[i]).length2()<0.0001))
						if (orgpoints[i].m_orgIndex == orgIndex)
						{
							found=true;
							break;
						}
					}
					if (!found)
						orgpoints.push_back(GrahamVector2(rotatedPt,orgIndex));
				}
			}

			btFace combinedFace;
			for (int i=0;i<4;i++)
				combinedFace.m_plane[i] = tmpFaces[coplanarFaceGroup[0]].m_plane[i];

			btAlignedObjectArray<GrahamVector2> hull;
			GrahamScanConvexHull2D(orgpoints,hull);

			for (int i=0;i<hull.size();i++)
			{
				combinedFace.m_indices.push_back(hull[i].m_orgIndex);
				for(int k = 0; k < orgpoints.size(); k++) {
					if(orgpoints[k].m_orgIndex == hull[i].m_orgIndex) {
						orgpoints[k].m_orgIndex = -1; // invalidate...
						break;
			}
				}
			}
			// are there rejected vertices?
			bool reject_merge = false;
			for(int i = 0; i < orgpoints.size(); i++) {
				if(orgpoints[i].m_orgIndex == -1)
					continue; // this is in the hull...
				// this vertex is rejected -- is anybody else using this vertex?
				for(int j = 0; j < tmpFaces.size(); j++) {
					btFace& face = tmpFaces[j];
					// is this a face of the current coplanar group?
					bool is_in_current_group = false;
					for(int k = 0; k < coplanarFaceGroup.size(); k++) {
						if(coplanarFaceGroup[k] == j) {
							is_in_current_group = true;
							break;
						}
					}
					if(is_in_current_group) // ignore this face...
						continue;
					// does this face use this rejected vertex?
					for(int v = 0; v < face.m_indices.size(); v++) {
						if(face.m_indices[v] == orgpoints[i].m_orgIndex) {
							// this rejected vertex is used in another face -- reject merge
							reject_merge = true;
							break;
						}
					}
					if(reject_merge)
						break;
				}
				if(reject_merge)
					break;
			}
			if(!reject_merge) {
				// do this merge!
				did_merge = true;
			m_faces.push_back(combinedFace);
			}
		}
		if(!did_merge)
		{
			for (int i=0;i<coplanarFaceGroup.size();i++)
			{
				m_faces.push_back(tmpFaces[coplanarFaceGroup[i]]);
			}
		}

	}
	return true;
}
