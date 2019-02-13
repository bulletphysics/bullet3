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

#ifndef GU_SWEEP_MTD_H
#define GU_SWEEP_MTD_H

namespace physx
{
	class PxConvexMeshGeometry;
	class PxTriangleMeshGeometry;
	class PxGeometry;
	class PxHeightFieldGeometry;

namespace Gu
{
	class Sphere;
	class Capsule;

	bool computeCapsule_TriangleMeshMTD(const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, Gu::CapsuleV& capsuleV, PxReal inflatedRadius, bool isDoubleSided, PxSweepHit& hit);

	bool computeCapsule_HeightFieldMTD(const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, Gu::CapsuleV& capsuleV, PxReal inflatedRadius, bool isDoubleSided, const PxU32 flags, PxSweepHit& hit);

	bool computeBox_TriangleMeshMTD(const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const Gu::Box& box, const PxTransform& boxTransform, PxReal inflation,
									bool isDoubleSided,  PxSweepHit& hit);    

	bool computeBox_HeightFieldMTD(	const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const Gu::Box& box, const PxTransform& boxTransform, PxReal inflation, 
									bool isDoubleSided, const PxU32 flags, PxSweepHit& hit);

	bool computeConvex_TriangleMeshMTD(	const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexTransform, PxReal inflation,
										bool isDoubleSided, PxSweepHit& hit);

	bool computeConvex_HeightFieldMTD(	const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexTransform, PxReal inflation,
										bool isDoubleSided, const PxU32 flags, PxSweepHit& hit);

	bool computeSphere_SphereMTD(const Sphere& sphere0, const Sphere& sphere1, PxSweepHit& hit);
	bool computeSphere_CapsuleMTD(const Sphere& sphere, const Capsule& capsule, PxSweepHit& hit);

	bool computeCapsule_CapsuleMTD(const Capsule& capsule0, const Capsule& capsule1, PxSweepHit& hit);

	bool computePlane_CapsuleMTD(const PxPlane& plane, const Capsule& capsule, PxSweepHit& hit);
	bool computePlane_BoxMTD(const PxPlane& plane, const Box& box, PxSweepHit& hit);
	bool computePlane_ConvexMTD(const PxPlane& plane, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, PxSweepHit& hit);

	// PT: wrapper just to avoid duplicating these lines.
	PX_FORCE_INLINE void setupSweepHitForMTD(PxSweepHit& sweepHit, bool hasContacts, const PxVec3& unitDir)
	{
		sweepHit.flags = PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
		if(!hasContacts)
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
		else
		{
			//ML: touching contact. We need to overwrite the normal to the negative of sweep direction
			if (sweepHit.distance == 0.0f)
			{
				sweepHit.normal = -unitDir;
			}
			sweepHit.flags |= PxHitFlag::ePOSITION;
			
		}
	}
}

}


#endif
