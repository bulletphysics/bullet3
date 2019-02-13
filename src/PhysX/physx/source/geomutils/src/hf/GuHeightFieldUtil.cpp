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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PsIntrinsics.h"
#include "GuHeightFieldUtil.h"
#include "GuSweepSharedTests.h"

#include "PsFoundation.h"
#include "GuHeightField.h"
#include "GuEntityReport.h"
#include "PxMeshScale.h"

using namespace physx;

void Gu::HeightFieldUtil::computeLocalBounds(PxBounds3& bounds) const
{
	const PxMeshScale scale(PxVec3(mHfGeom->rowScale, mHfGeom->heightScale, mHfGeom->columnScale), PxQuat(PxIdentity));
	const PxMat33 mat33 = scale.toMat33();

	bounds.minimum = mat33.transform(mHeightField->getData().mAABB.getMin());
	bounds.maximum = mat33.transform(mHeightField->getData().mAABB.getMax());

	// PT: HFs will assert in Gu::intersectRayAABB2() if we don't deal with that
	const float deltaY = GU_MIN_AABB_EXTENT*0.5f - (bounds.maximum.y - bounds.minimum.y);
	if(deltaY>0.0f)
	{
		bounds.maximum.y += deltaY*0.6f;
		bounds.minimum.y -= deltaY*0.6f;
	}
}

PxU32 Gu::HeightFieldUtil::getFaceIndexAtShapePoint(PxReal x, PxReal z) const
{
	if (isShapePointOnHeightField(x, z)) 
	{
		const PxU32 triangleIndex = mHeightField->getTriangleIndex(x * mOneOverRowScale, z * mOneOverColumnScale);
		return (mHeightField->getTriangleMaterial(triangleIndex) != PxHeightFieldMaterial::eHOLE) ? triangleIndex : 0xffffffff;
	}
	return 0xffffffff;
}

PxVec3 Gu::HeightFieldUtil::getVertexNormal(PxU32 vertexIndex, PxU32 row, PxU32 column) const
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(mHeightField->isValidVertex(vertexIndex));
#endif
	//	PxU32 edges[8];
	//	const PxU32 edgeCount = mHeightField.getVertexEdgeIndices(vertexIndex, edges);

	//const PxU32 nbColumns = mHeightField.getData().columns;
	//const PxU32 row = vertexIndex / nbColumns;
	//const PxU32 column = vertexIndex % nbColumns;
	PX_ASSERT(row == vertexIndex / mHeightField->getData().columns);
	PX_ASSERT(column == vertexIndex % mHeightField->getData().columns);
	EdgeData edgeIndices[8];
	const PxU32 edgeCount = ::getVertexEdgeIndices(*mHeightField, vertexIndex, row, column, edgeIndices);

	PxVec3 n(0.0f);
	PxVec3 tn;
	for (PxU32 i=0; i<edgeCount; i++)
	{
		PxU32 faces[2];
		//		const PxU32 faceCount = mHeightField.getEdgeTriangleIndices(edges[i], faces);
		const PxU32 faceCount = ::getEdgeTriangleIndices(*mHeightField, edgeIndices[i], faces);

		for(PxU32 j=0; j<faceCount; j++)
		{
			if (mHeightField->getTriangleMaterial(faces[j]) != PxHeightFieldMaterial::eHOLE)
			{
				tn = hf2shapen(mHeightField->getTriangleNormalInternal(faces[j])).getNormalized();
				n+=tn;
			}
		}
	}

	return n.getNormalized();
}

PxU32 Gu::HeightFieldUtil::findClosestPointsOnCell(
	PxU32 row, PxU32 column, PxVec3 point,
	PxVec3* PX_RESTRICT closestPoints, PxU32* PX_RESTRICT featureCodes,
	bool testFaces, bool testEdges, bool skipEdgesIfFaceHits) const
{
	PxU32 count = 0;

	const PxU32 offset = row * mHeightField->getNbColumnsFast() + column;
	const PxU32 firstEdgeIndex = 3 * offset;

	// ptchernev TODO:
	// move the material assignments to an else in the ifs on triangle material
	// instead of doing it all the time

	PX_ASSERT(row < (mHeightField->getNbRowsFast() - 1));
	PX_ASSERT(column < (mHeightField->getNbColumnsFast() - 1));
	const bool lastRow		= (row == (mHeightField->getNbRowsFast() - 2));
	const bool lastColumn	= (column == (mHeightField->getNbColumnsFast() - 2));

	bool testVertex0		= testEdges;
	bool testColumnEdge0	= testEdges;
	bool testRowEdge0		= testEdges;
	bool testDiagonal		= testEdges;
	bool testVertex1		= lastColumn && testEdges;
	bool testVertex2		= lastRow && testEdges;

	bool testRowEdge1		= lastColumn && testEdges;
	bool testColumnEdge1	= lastRow && testEdges;
	bool testVertex3		= lastRow && lastColumn && testEdges;

	const PxU32 triangleIndex0 = offset << 1;
	const PxMaterialTableIndex materialIndex0 = mHeightField->getTriangleMaterial(triangleIndex0);
	const PxU32 triangleIndex1 = triangleIndex0 + 1;
	const PxMaterialTableIndex materialIndex1 = mHeightField->getTriangleMaterial(triangleIndex1);

	if (testFaces)
	{
		if (materialIndex0 != PxHeightFieldMaterial::eHOLE)
		{
			// face 0
			PxVec3 closestPoint;
			if (findProjectionOnTriangle(triangleIndex0, row, column, point, closestPoint))
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(triangleIndex0, eFACE);
				count++;
				testRowEdge0 = false;
				testVertex0 = false;
				testVertex2 = false;
				testDiagonal = false;
			}
		}

		if (materialIndex1 != PxHeightFieldMaterial::eHOLE)
		{
			// face 1			
			PxVec3 closestPoint;
			if (findProjectionOnTriangle(triangleIndex1, row, column, point, closestPoint))
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(triangleIndex1, eFACE);
				count++;
				testRowEdge1 = false;
				testVertex1 = false;
				testVertex3 = false;
				testDiagonal = false;
			}
		}
		if (!testEdges)
			return count;
	}

	// if there were any face contacts and we asked to skip edges if face contacts, return current count here
	if (count && skipEdgesIfFaceHits)
		return count;

	const PxU32 nbColumns = mHeightField->getNbColumnsFast();
	if (testVertex0 || testColumnEdge0 || testVertex1)
	{
		PxVec3 closestPoint;
		PxReal t = findClosestPointOnEdge(firstEdgeIndex, offset, row, column, point, closestPoint);
		if (t <= 0)
		{
			if (testVertex0 && 0xffffffff != (getVertexFaceIndex(offset, row, column)))
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(row*nbColumns+column, eVERTEX);
				count++;
			}
			testVertex0 = false;
		}
		else if (t < 1)
		{
			if (testColumnEdge0 && 0xffffffff != (getEdgeFaceIndex(firstEdgeIndex)))
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(firstEdgeIndex, eEDGE);
				count++;
			}
		}
		else
		{
			if (testVertex1 && 0xffffffff != (getVertexFaceIndex(offset + 1, row, column + 1)))
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(row*nbColumns+column+1, eVERTEX);
				count++;
			}
		}
	}

	if (testVertex0 || testRowEdge0 || testVertex2)
	{
		PxVec3 closestPoint;
		PxReal t = findClosestPointOnEdge(firstEdgeIndex + 2, offset, row, column, point, closestPoint);
		if (t <= 0) 
		{
			if (testVertex0 && 0xffffffff != (getVertexFaceIndex(offset, row, column))) 
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(row*nbColumns+column, eVERTEX);
				count++;
			}
		}
		else if(t < 1)
		{
			if (testRowEdge0 && 0xffffffff != (getEdgeFaceIndex(firstEdgeIndex + 2))) 
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(firstEdgeIndex+2, eEDGE);
				count++;
			}
		}
		else 
		{
			if (testVertex2 && 0xffffffff != (getVertexFaceIndex(offset + nbColumns, row + 1, column))) 
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode((row+1)*nbColumns+column, eVERTEX);
				count++;
			}
		}
	}

	if (testColumnEdge1)
	{
		PxVec3 closestPoint;
		PxReal t = findClosestPointOnEdge(firstEdgeIndex + 3 * nbColumns, offset + nbColumns, row + 1, column, point, closestPoint);
		if (t <= 0)
			; // do nothing
		else if (t < 1)
		{
			const PxU32 edgeIndex3 = firstEdgeIndex + 3 * nbColumns;
			if (0xffffffff != (getEdgeFaceIndex(edgeIndex3)))
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(edgeIndex3, eEDGE);
				count++;
			}
		}
	}

	if (testRowEdge1)
	{
		PxVec3 closestPoint;
		PxReal t = findClosestPointOnEdge(firstEdgeIndex + 5, offset + 1, row, column + 1, point, closestPoint);
		if (t <= 0)
			; // do nothing
		else if (t < 1)
		{
			if (0xffffffff != (getEdgeFaceIndex(firstEdgeIndex + 5)))
			{
				closestPoints[count] = closestPoint;
				if (featureCodes) featureCodes[count] = makeFeatureCode(firstEdgeIndex+5, eEDGE);
				count++;
			}
		}
	}

	if (testVertex3 && 0xffffffff != (getVertexFaceIndex(offset + nbColumns + 1, row + 1, column + 1)))
	{
		closestPoints[count] = PxVec3((row + 1) * mHfGeom->rowScale, mHfGeom->heightScale * mHeightField->getHeight(offset + mHeightField->getNbColumnsFast() + 1), (column + 1) * mHfGeom->columnScale);
		if (featureCodes) featureCodes[count] = makeFeatureCode((row+1)*nbColumns+column+1, eVERTEX);
		count++;
	}

	if (testDiagonal && 0xffffffff != (getEdgeFaceIndex(firstEdgeIndex + 1)))
	{
		PxVec3 closestPoint;
		PxReal t = findClosestPointOnEdge(firstEdgeIndex + 1, offset, row, column, point, closestPoint);
		if (t <= 0) 
			; // do nothing
		else if (t < 1) 
		{
			closestPoints[count] = closestPoint;
			if (featureCodes) featureCodes[count] = makeFeatureCode(firstEdgeIndex+1, eEDGE);
			count++;
		}
	}

	return count;
}

//PxReal Gu::HeightFieldUtil::findClosestPointOnEdge(PxU32 edgeIndex, const PxVec3& point, PxVec3& closestPoint) const
PxReal Gu::HeightFieldUtil::findClosestPointOnEdge(
	PxU32 edgeIndex, PxU32 cell, PxU32 row, PxU32 column, const PxVec3& point, PxVec3& closestPoint) const
{
//	const PxU32 cell = edgeIndex / 3;
	PX_ASSERT(cell == edgeIndex / 3);
//	const PxU32 row = cell / mHeightField->getNbColumnsFast();
	PX_ASSERT(row == cell / mHeightField->getNbColumnsFast());
//	const PxU32 column = cell % mHeightField->getNbColumnsFast();
	PX_ASSERT(column == cell % mHeightField->getNbColumnsFast());

	PxVec3 origin, direction;
	PxReal lengthSquared;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
	case 0:
		{
			const PxReal y0 = mHfGeom->heightScale * mHeightField->getHeight(cell);
			const PxReal y1 = mHfGeom->heightScale * mHeightField->getHeight(cell + 1);
			origin = PxVec3(row * mHfGeom->rowScale, y0, column * mHfGeom->columnScale);
			const PxReal dy = y1 - y0;
			direction = PxVec3(0, dy, mHfGeom->columnScale);
			lengthSquared = mHfGeom->columnScale * mHfGeom->columnScale + dy * dy;
		}
		break;
	case 1:
		if (mHeightField->isZerothVertexShared(cell))
		{
			const PxReal y0 = mHfGeom->heightScale * mHeightField->getHeight(cell);
			const PxReal y3 = mHfGeom->heightScale * mHeightField->getHeight(cell + mHeightField->getNbColumnsFast() + 1);
			origin = PxVec3(row * mHfGeom->rowScale, y0, column * mHfGeom->columnScale);
			const PxReal dy = y3 - y0;
			direction = PxVec3(mHfGeom->rowScale, dy, mHfGeom->columnScale);
			lengthSquared = mHfGeom->rowScale * mHfGeom->rowScale + mHfGeom->columnScale * mHfGeom->columnScale + dy * dy;
		}
		else
		{
			const PxReal y1 = mHfGeom->heightScale * mHeightField->getHeight(cell + 1);
			const PxReal y2 = mHfGeom->heightScale * mHeightField->getHeight(cell + mHeightField->getNbColumnsFast());
			origin = PxVec3(row * mHfGeom->rowScale, y1, (column + 1) * mHfGeom->columnScale);
			const PxReal dy = y2 - y1;
			direction = PxVec3(mHfGeom->rowScale, dy, -mHfGeom->columnScale);
			lengthSquared = mHfGeom->rowScale * mHfGeom->rowScale + mHfGeom->columnScale * mHfGeom->columnScale + dy * dy;
		}
		break;
	case 2:
		{
			const PxReal y0 = mHfGeom->heightScale * mHeightField->getHeight(cell);
			const PxReal y2 = mHfGeom->heightScale * mHeightField->getHeight(cell + mHeightField->getNbColumnsFast());
			origin = PxVec3(row * mHfGeom->rowScale, y0, column * mHfGeom->columnScale);
			const PxReal dy = y2 - y0;
			direction = PxVec3(mHfGeom->rowScale, dy, 0);
			lengthSquared = mHfGeom->rowScale * mHfGeom->rowScale + dy * dy;
		}
		break;
	default:
		origin = direction = PxVec3(PxReal(0));
		lengthSquared = 0.0f;
		PX_ALWAYS_ASSERT_MESSAGE("Invalid edge index in findClosestPointOnEdge");
	} //	switch (edgeIndex % 3)

	const PxVec3 relative = point - origin;
	const PxReal t = relative.dot(direction) / lengthSquared;
	if (t < 0)
		closestPoint = origin;
	else if (t > 1)
		closestPoint = origin + direction;
	else
		closestPoint = origin + direction * t;

	return t;
}

PxU32 Gu::HeightFieldUtil::getVertexFaceIndex(PxU32 vertexIndex, PxU32 row, PxU32 column) const
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(mHeightField->isValidVertex(vertexIndex));
#endif

//	PxU32 edgeIndices[8];
//	const PxU32 count = mHeightField->getVertexEdgeIndices(vertexIndex, edgeIndices);

//const PxU32 nbColumns = mHeightField->getData().columns;
//const PxU32 row = vertexIndex / nbColumns;
//const PxU32 column = vertexIndex % nbColumns;
PX_ASSERT(row == vertexIndex / mHeightField->getData().columns);
PX_ASSERT(column == vertexIndex % mHeightField->getData().columns);
EdgeData edgeIndices[8];
const PxU32 count = ::getVertexEdgeIndices(*mHeightField, vertexIndex, row, column, edgeIndices);

	for (PxU32 i = 0; i<count; i+= 2)
	{
		const PxU32 index = getEdgeFaceIndex(edgeIndices[i].edgeIndex, edgeIndices[i].cell, edgeIndices[i].row, edgeIndices[i].column);
		if (index != 0xffffffff) return index;
	}
	return 0xffffffff;
}

PxU32 Gu::HeightFieldUtil::getEdgeFaceIndex(PxU32 edgeIndex) const
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(mHeightField->isValidEdge(edgeIndex));
#endif
	PxU32 faceIndices[2];
	const PxU32 count = mHeightField->getEdgeTriangleIndices(edgeIndex, faceIndices);
	if (count > 1) 
	{
		// ptchernev TODO: this is a bit arbitrary
		if (mHeightField->getTriangleMaterial(faceIndices[0]) != PxHeightFieldMaterial::eHOLE) return faceIndices[0];
		if (mHeightField->getTriangleMaterial(faceIndices[1]) != PxHeightFieldMaterial::eHOLE) return faceIndices[1];
	} 
	else 
	{
		if (mHeightField->getTriangleMaterial(faceIndices[0]) != PxHeightFieldMaterial::eHOLE) return faceIndices[0];
	}
	return 0xffffffff;
}

PxU32 Gu::HeightFieldUtil::getEdgeFaceIndex(PxU32 edgeIndex, PxU32 cell, PxU32 row, PxU32 column) const
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(mHeightField->isValidEdge(edgeIndex));
#endif
	PxU32 faceIndices[2];
	const PxU32 count = mHeightField->getEdgeTriangleIndices(edgeIndex, faceIndices, cell, row, column);
	if (count > 1) 
	{
		// ptchernev TODO: this is a bit arbitrary
		if (mHeightField->getTriangleMaterial(faceIndices[0]) != PxHeightFieldMaterial::eHOLE) return faceIndices[0];
		if (mHeightField->getTriangleMaterial(faceIndices[1]) != PxHeightFieldMaterial::eHOLE) return faceIndices[1];
	} 
	else 
	{
		if (mHeightField->getTriangleMaterial(faceIndices[0]) != PxHeightFieldMaterial::eHOLE) return faceIndices[0];
	}
	return 0xffffffff;
}

PxU32 Gu::HeightFieldUtil::getEdgeFaceIndex(PxU32 edgeIndex, PxU32 count, const PxU32* PX_RESTRICT faceIndices) const
{
	PX_UNUSED(edgeIndex);

#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(mHeightField->isValidEdge(edgeIndex));
#endif
//	PxU32 faceIndices[2];
//	const PxU32 count = mHeightField->getEdgeTriangleIndices(edgeIndex, faceIndices);
	if (count > 1) 
	{
		// ptchernev TODO: this is a bit arbitrary
		if (mHeightField->getTriangleMaterial(faceIndices[0]) != PxHeightFieldMaterial::eHOLE) return faceIndices[0];
		if (mHeightField->getTriangleMaterial(faceIndices[1]) != PxHeightFieldMaterial::eHOLE) return faceIndices[1];
	} 
	else 
	{
		if (mHeightField->getTriangleMaterial(faceIndices[0]) != PxHeightFieldMaterial::eHOLE) return faceIndices[0];
	}
	return 0xffffffff;
}

bool Gu::HeightFieldUtil::findProjectionOnTriangle(PxU32 triangleIndex, PxU32 row, PxU32 column, const PxVec3& point, PxVec3& projection) const
{
	const PxU32 cell = (triangleIndex >> 1);
	PX_ASSERT(row == cell / mHeightField->getNbColumnsFast());
	PX_ASSERT(column == cell % mHeightField->getNbColumnsFast());
	const PxReal y0 = mHfGeom->heightScale * mHeightField->getHeight(cell);
	const PxReal y1 = mHfGeom->heightScale * mHeightField->getHeight(cell + 1);
	const PxReal y2 = mHfGeom->heightScale * mHeightField->getHeight(cell + mHeightField->getNbColumnsFast());
	const PxReal y3 = mHfGeom->heightScale * mHeightField->getHeight(cell + mHeightField->getNbColumnsFast() + 1);
	PxVec3 origin;
	PxReal h0, h1, h2, uInvScale, vInvScale;

	// specify a triangle according to current triangle index as origin, 3 heights and (uInvScale, vInvScale) vector for (z,x)
	// set uInvScale in h1-h0; vScale in h2-h0 direction
	if (mHeightField->isZerothVertexShared(cell)) 
	{
		//    COLUMN -->
		//
		// R  0---1
		// O  |\ 1|
		// W  | \ |
		// |  |0 \|
		// |  2---3
		// V 
		if ((triangleIndex & 1) == 0)
		{
			// case 0
			// face 0
			origin = PxVec3((row + 1) * mHfGeom->rowScale, y2, column * mHfGeom->columnScale);
			// origin -> 2
			// verts -> 2,3,0
			h0 = y2;
			h1 = y3;
			h2 = y0;
			uInvScale = mOneOverColumnScale;
			vInvScale = -mOneOverRowScale;
		}
		else // if (testFace1)
		{
			// case 1
			// face 1			
			origin = PxVec3(row * mHfGeom->rowScale, y1, (column + 1) * mHfGeom->columnScale);
			// origin -> 1
			// verts -> 1,0,3
			h0 = y1;
			h1 = y0;
			h2 = y3;
			uInvScale = -mOneOverColumnScale;
			vInvScale = mOneOverRowScale;
		}
	}
	else
	{
		//    COLUMN -->
		//
		// R  0---1
		// O  |0 /|
		// W  | / |
		// |  |/ 1|
		// |  2---3
		// V 
		if ((triangleIndex & 1) == 0)
		{
			// case 2
			// face 0			
			origin = PxVec3(row * mHfGeom->rowScale, y0, column * mHfGeom->columnScale);
			// origin -> 0
			// verts -> 0,1,2
			h0 = y0;
			h1 = y1;
			h2 = y2;
			uInvScale = mOneOverColumnScale;
			vInvScale = mOneOverRowScale;
		}
		else
		{
			// case 3
			// face 1			
			origin = PxVec3((row + 1) * mHfGeom->rowScale, y3, (column + 1) * mHfGeom->columnScale);
			// origin -> 3
			// verts -> 3,2,1
			h0 = y3;
			h1 = y2;
			h2 = y1;
			uInvScale = -mOneOverColumnScale;
			vInvScale = -mOneOverRowScale;
		}
	}

	// vector from triangle origin to point we want to project
	const PxVec3 relative = point - origin;

	// Looking at the triangle diagram for case 2
	// The normal computation should be
	// n = (p1-p0) x (p2-p0)
	// For a right handed cross product that's pointing into the screen (negative h), so -n is in the direction of increasing h
	// cs = column scale, rs = row scale, h10 = h1-h0; u=column, v=row
	//		   (i   j    k);
	// p1-p0 = (cs, h10, 0); this is column, u and z
	// p2-p0 = (0,  h20, rs); this is row, v and x
	// n = (h10*rs, -cs*rs, +cs*h20)
	// n/(cs*rs) = (h10/cs, -1, h20/rs)
	// -n = (-h10/cs, 1, -h20/rs)
	PxReal h10 = h1-h0, h20 = h2-h0;
	PxReal nu = -h10 * uInvScale;
	PxReal nv = -h20 * vInvScale;

	PxVec3 n(nv, 1.0f, nu); // for whatever reason.. x is v, z is u.
	//n *= 1.0f / PxSqrt(nu*nu + nv*nv + 1.0f); // technically we need to do this but since later

	// project relative onto the n plane, it gives us unclipped projection onto the triangle
	// the computation without sqrt shortcut is relPrj = relative - n.dot(relative)*n, but because we divide by sqrt^2 we skip sqrt
	PxVec3 relPrj = relative - n.dot(relative)* (1.0f/(nu*nu+nv*nv+1.0f)) * n;

	// project relPrj onto 2d UV plane with h = 0 oriented according to vInvScale, uInvScale (for x and z)
	// to convert to HF cell coords we'd multiply by inv scale, after that the coords should be >0 and the sum within 1 to be
	// inside of a 2d triangle
	PxReal scaledX = relPrj.x * vInvScale, scaledZ = relPrj.z * uInvScale;
	//PxVec3 testProjection = relPrj + origin;
	//PxVec3 testN = (point - testProjection).getNormalized();
	if (scaledX > 0.0f && scaledZ > 0.0f && scaledX + scaledZ < 1.0f)
	{
		projection = relPrj + origin;
		return true;
	}

	return false;
}

void Gu::HeightFieldUtil::getEdge(PxU32 edgeIndex, PxU32 cell, PxU32 row, PxU32 column, PxVec3& origin, PxVec3& extent) const
{
#ifdef PX_HEIGHTFIELD_DEBUG		
	PX_ASSERT(mHeightField->isValidEdge(edgeIndex));
#endif
//	const PxU32 cell = edgeIndex / 3;
	PX_ASSERT(cell == edgeIndex / 3);
//	const PxU32 row = cell / mHeightField->getNbColumnsFast();
	PX_ASSERT(row == cell / mHeightField->getNbColumnsFast());
//	const PxU32 column = cell % mHeightField->getNbColumnsFast();
	PX_ASSERT(column == cell % mHeightField->getNbColumnsFast());

//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
	case 0:
		{
			const PxReal y0 = mHfGeom->heightScale * mHeightField->getHeight(cell);
			const PxReal y1 = mHfGeom->heightScale * mHeightField->getHeight(cell + 1);
			origin = PxVec3(row * mHfGeom->rowScale, y0, column * mHfGeom->columnScale);
			extent = PxVec3(0, y1 - y0, mHfGeom->columnScale);
		}
		break;
	case 1:
		if (mHeightField->isZerothVertexShared(cell))
		{
			const PxReal y0 = mHfGeom->heightScale * mHeightField->getHeight(cell);
			const PxReal y3 = mHfGeom->heightScale * mHeightField->getHeight(cell + mHeightField->getNbColumnsFast() + 1);
			origin = PxVec3(row * mHfGeom->rowScale, y0, column * mHfGeom->columnScale);
			extent = PxVec3(mHfGeom->rowScale, y3 - y0, mHfGeom->columnScale);
		}
		else
		{
			const PxReal y1 = mHfGeom->heightScale * mHeightField->getHeight(cell + 1);
			const PxReal y2 = mHfGeom->heightScale * mHeightField->getHeight(cell + mHeightField->getNbColumnsFast());
			origin = PxVec3(row * mHfGeom->rowScale, y1, (column + 1) * mHfGeom->columnScale);
			extent = PxVec3(mHfGeom->rowScale, y2 - y1, -mHfGeom->columnScale);
		}
		break;
	case 2:
		{
			const PxReal y0 = mHfGeom->heightScale * mHeightField->getHeight(cell);
			const PxReal y2 = mHfGeom->heightScale * mHeightField->getHeight(cell + mHeightField->getNbColumnsFast());
			origin = PxVec3(row * mHfGeom->rowScale, y0, column * mHfGeom->columnScale);
			extent = PxVec3(mHfGeom->rowScale, y2 - y0, 0);
		}
		break;
	}
}

bool Gu::HeightFieldUtil::overlapAABBTriangles(const PxTransform& pose, const PxBounds3& bounds, PxU32 flags, EntityReport<PxU32>* callback) const
{
	PX_ASSERT(!bounds.isEmpty());

	PxBounds3 localBounds = (flags & GuHfQueryFlags::eWORLD_SPACE) ? PxBounds3::transformFast(pose.getInverse(), bounds) : bounds;

	localBounds.minimum.x *= mOneOverRowScale;
	localBounds.minimum.y *= mOneOverHeightScale;
	localBounds.minimum.z *= mOneOverColumnScale;

	localBounds.maximum.x *= mOneOverRowScale;
	localBounds.maximum.y *= mOneOverHeightScale;
	localBounds.maximum.z *= mOneOverColumnScale;

	if(mHfGeom->rowScale < 0.0f)
		Ps::swap(localBounds.minimum.x, localBounds.maximum.x);

	if(mHfGeom->columnScale < 0.0f)
		Ps::swap(localBounds.minimum.z, localBounds.maximum.z);

	// early exit for aabb does not overlap in XZ plane
	// DO NOT MOVE: since rowScale / columnScale may be negative this has to be done after scaling localBounds
	const PxU32	nbRows = mHeightField->getNbRowsFast();
	const PxU32	nbColumns = mHeightField->getNbColumnsFast();
	if(localBounds.minimum.x > float(nbRows - 1))
		return false;
	if(localBounds.minimum.z > float(nbColumns - 1))
		return false;
	if(localBounds.maximum.x < 0.0f)
		return false;
	if(localBounds.maximum.z < 0.0f)
		return false;

	const PxU32 minRow = mHeightField->getMinRow(localBounds.minimum.x);
	const PxU32 maxRow = mHeightField->getMaxRow(localBounds.maximum.x);
	const PxU32 minColumn = mHeightField->getMinColumn(localBounds.minimum.z);
	const PxU32 maxColumn = mHeightField->getMaxColumn(localBounds.maximum.z);

	PxU32 maxNbTriangles = 2 * (maxColumn - minColumn) * (maxRow - minRow);

	if(!maxNbTriangles)
		return false;

	if(flags & GuHfQueryFlags::eFIRST_CONTACT)
		maxNbTriangles = 1;

	const PxU32 bufferSize = HF_SWEEP_REPORT_BUFFER_SIZE;
	PxU32 indexBuffer[bufferSize];
	PxU32 indexBufferUsed = 0;
	PxU32 nb = 0;

	PxU32 offset = minRow * mHeightField->getNbColumnsFast() + minColumn;

	const PxReal miny = localBounds.minimum.y;
	const PxReal maxy = localBounds.maximum.y;

	for(PxU32 row=minRow; row<maxRow; row++)
	{
		for(PxU32 column=minColumn; column<maxColumn; column++)
		{
			const PxReal h0 = mHeightField->getHeight(offset);
			const PxReal h1 = mHeightField->getHeight(offset + 1);
			const PxReal h2 = mHeightField->getHeight(offset + mHeightField->getNbColumnsFast());
			const PxReal h3 = mHeightField->getHeight(offset + mHeightField->getNbColumnsFast() + 1);
			if(!((maxy < h0 && maxy < h1 && maxy < h2 && maxy < h3) || (miny > h0 && miny > h1 && miny > h2 && miny > h3)))
			{
				const PxU32 material0 = mHeightField->getMaterialIndex0(offset);
				if(material0 != PxHeightFieldMaterial::eHOLE) 
				{
					if(indexBufferUsed >= bufferSize)
					{
						callback->onEvent(indexBufferUsed, indexBuffer);
						indexBufferUsed = 0;
					}

					indexBuffer[indexBufferUsed++] = offset << 1;
					nb++;

					if(flags & GuHfQueryFlags::eFIRST_CONTACT)
						goto search_done;
				}

				const PxU32 material1 = mHeightField->getMaterialIndex1(offset);
				if(material1 != PxHeightFieldMaterial::eHOLE)
				{
					if(indexBufferUsed >= bufferSize)
					{
						callback->onEvent(indexBufferUsed, indexBuffer);
						indexBufferUsed = 0;
					}

					indexBuffer[indexBufferUsed++] = (offset << 1) + 1;
					nb++;

					if(flags & GuHfQueryFlags::eFIRST_CONTACT)
						goto search_done;
				}
			}
			offset++;
		}
		offset += (mHeightField->getNbColumnsFast() - (maxColumn - minColumn));
	}

search_done:

	if(indexBufferUsed > 0)
		callback->onEvent(indexBufferUsed, indexBuffer);

	return nb > 0;
}

PxU32 Gu::HeightFieldUtil::getTriangle(const PxTransform& pose, PxTriangle& worldTri,
									   PxU32* _vertexIndices, PxU32* adjacencyIndices, PxTriangleID triangleIndex, bool worldSpaceTranslation, bool worldSpaceRotation) const
{
#if PX_CHECKED
	if (!mHeightField->isValidTriangle(triangleIndex)) 
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "HeightFieldShape::getTriangle: Invalid triangle index!");
		return 0;
	}
#endif

	PxVec3 handedness(1.0f);	// Vector to invert normal coordinates according to the heightfield scales
	bool wrongHanded = false;
	if (mHfGeom->columnScale < 0)
	{
		wrongHanded = !wrongHanded;
		handedness.z = -1.0f;
	}
	if (mHfGeom->rowScale < 0)
	{
		wrongHanded = !wrongHanded;
		handedness.x = -1.0f;
	}

/*	if (0) // ptchernev: Iterating over triangles becomes a pain.
	{
		if (mHeightField.getTriangleMaterial(triangleIndex) == mHfGeom.holeMaterialIndex)
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "HeightFieldShape::getTriangle: Non-existing triangle (triangle has hole material)!");
			return 0;
		}
	}*/

	PxU32 vertexIndices[3];
	mHeightField->getTriangleVertexIndices(triangleIndex, vertexIndices[0], vertexIndices[1+wrongHanded], vertexIndices[2-wrongHanded]);

	if(adjacencyIndices)
	{
		mHeightField->getTriangleAdjacencyIndices(	triangleIndex, vertexIndices[0], vertexIndices[1+wrongHanded], vertexIndices[2-wrongHanded],
													adjacencyIndices[wrongHanded ? 2 : 0], adjacencyIndices[1], adjacencyIndices[wrongHanded ? 0 : 2]);
	}

	if(_vertexIndices)
	{
		_vertexIndices[0] = vertexIndices[0];
		_vertexIndices[1] = vertexIndices[1];
		_vertexIndices[2] = vertexIndices[2];
	}

	if (worldSpaceRotation)
	{
		if (worldSpaceTranslation)
		{
			for (PxU32 vi = 0; vi < 3; vi++)
				worldTri.verts[vi] = hf2worldp(pose, mHeightField->getVertex(vertexIndices[vi]));
		}
		else
		{
			for (PxU32 vi = 0; vi < 3; vi++)
			{
				// TTP 2390 
				// local space here is rotated (but not translated) world space
				worldTri.verts[vi] = pose.q.rotate(hf2shapep(mHeightField->getVertex(vertexIndices[vi])));
			}
		}
	}
	else
	{
		const PxVec3 offset = worldSpaceTranslation ? pose.p : PxVec3(0.0f);
		for (PxU32 vi = 0; vi < 3; vi++)
			worldTri.verts[vi] = hf2shapep(mHeightField->getVertex(vertexIndices[vi])) + offset;
	}
	return PxU32(mHeightField->getTriangleMaterial(triangleIndex) != PxHeightFieldMaterial::eHOLE);
}
