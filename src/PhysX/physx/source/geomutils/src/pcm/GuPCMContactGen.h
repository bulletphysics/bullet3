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

#ifndef GU_PCM_CONTACT_GEN_H
#define GU_PCM_CONTACT_GEN_H


#include "GuConvexSupportTable.h"
#include "GuPersistentContactManifold.h"
#include "GuShapeConvex.h"
#include "GuSeparatingAxes.h"
#include "GuGJKType.h"
#include "GuGJKUtil.h"

namespace physx
{

namespace Gu
{


	//full contact gen code for box/convexhull vs convexhull
	bool generateFullContactManifold(Gu::PolygonalData& polyData0, Gu::PolygonalData& polyData1, Gu::SupportLocal* map0, Gu::SupportLocal* map1, Gu::PersistentContact* manifoldContacts, 
		PxU32& numContacts, const Ps::aos::FloatVArg contactDist, const Ps::aos::Vec3VArg normal, const Ps::aos::Vec3VArg closestA, const Ps::aos::Vec3VArg closestB, 
		const PxReal toleranceA, const PxReal toleranceB, bool doOverlapTest, Cm::RenderOutput* renderOutput, const PxReal toleranceLength);

	//full contact gen code for capsule vs convexhulll
	bool generateFullContactManifold(const Gu::CapsuleV& capsule, Gu::PolygonalData& polyData, Gu::SupportLocal* map, const Ps::aos::PsMatTransformV& aToB,  Gu::PersistentContact* manifoldContacts, 
		PxU32& numContacts, const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& normal, const Ps::aos::Vec3VArg closest, const PxReal tolerance, bool doOverlapTest, const PxReal toleranceScale);

	//full contact gen code for capsule vs box
	bool generateCapsuleBoxFullContactManifold(const Gu::CapsuleV& capsule, Gu::PolygonalData& polyData, Gu::SupportLocal* map, const Ps::aos::PsMatTransformV& aToB, Gu::PersistentContact* manifoldContacts, PxU32& numContacts,
		const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& normal, const Ps::aos::Vec3VArg closest, const PxReal boxMargin, const bool doOverlapTest, const PxReal toeranceScale);

	//based on the gjk status to decide whether we should do full contact gen with GJK/EPA normal. Also, this method store
	//GJK/EPA point to the manifold in case full contact gen doesn't generate any contact
	bool addGJKEPAContacts(const Gu::GjkConvex* relativeConvex, const Gu::GjkConvex* localConvex, const Ps::aos::PsMatTransformV& aToB, Gu::GjkStatus status,
		Gu::PersistentContact* manifoldContacts, const Ps::aos::FloatV replaceBreakingThreshold, const Ps::aos::FloatV toleranceLength, GjkOutput& output,
		Gu::PersistentContactManifold& manifold);

	//MTD code for box/convexhull vs box/convexhull
	bool computeMTD(Gu::PolygonalData& polyData0, Gu::PolygonalData& polyData1,  SupportLocal* map0, SupportLocal* map1, Ps::aos::FloatV& penDepth, Ps::aos::Vec3V& normal);
	
	//MTD code for capsule vs box/convexhull
	bool computeMTD(const Gu::CapsuleV& capsule, Gu::PolygonalData& polyData, Gu::SupportLocal* map, Ps::aos::FloatV& penDepth, Ps::aos::Vec3V& normal);

	void buildPartialHull(const Gu::PolygonalData& polyData, SupportLocal* map, Gu::SeparatingAxes& validAxes, const Ps::aos::Vec3VArg v, const Ps::aos::Vec3VArg _dir);

	//full contact gen code for sphere vs convexhull
	bool generateSphereFullContactManifold(const Gu::CapsuleV& capsule, Gu::PolygonalData& polyData, Gu::SupportLocal* map, Gu::PersistentContact* manifoldContacts, PxU32& numContacts,
		const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& normal, const bool doOverlapTest);

}  
}

#endif
