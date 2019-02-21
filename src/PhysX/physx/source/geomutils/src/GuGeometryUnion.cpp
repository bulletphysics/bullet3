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

#include "GuGeometryUnion.h"

#include "GuConvexMesh.h"
#include "GuTriangleMesh.h"
#include "GuHeightField.h"
#include "PsFoundation.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

static PX_FORCE_INLINE Gu::ConvexMesh& getConvexMesh(PxConvexMesh* pxcm)
{ 
	return *static_cast<Gu::ConvexMesh*>(pxcm);
}

static PX_FORCE_INLINE Gu::TriangleMesh& getTriangleMesh(PxTriangleMesh* pxtm)
{
	return *static_cast<Gu::TriangleMesh*>(pxtm);
}

static PX_FORCE_INLINE Gu::HeightField& getHeightField(PxHeightField* pxhf)
{
	return *static_cast<Gu::HeightField*>(pxhf);
}

// PT: TODO: optimize all these data copies
void Gu::GeometryUnion::set(const PxGeometry& g)
{
	switch(g.getType())
	{
		case PxGeometryType::eBOX:
		{
			reinterpret_cast<PxBoxGeometry&>(mGeometry) = static_cast<const PxBoxGeometry&>(g);
		}
		break;

		case PxGeometryType::eCAPSULE:
		{
			reinterpret_cast<PxCapsuleGeometry&>(mGeometry) = static_cast<const PxCapsuleGeometry&>(g);
		}
		break;

		case PxGeometryType::eSPHERE:
		{
			reinterpret_cast<PxSphereGeometry&>(mGeometry) = static_cast<const PxSphereGeometry&>(g);
			reinterpret_cast<PxCapsuleGeometry&>(mGeometry).halfHeight = 0.0f;		//AM: make sphere geometry also castable as a zero height capsule.
		}
		break;

		case PxGeometryType::ePLANE:
		{
			reinterpret_cast<PxPlaneGeometry&>(mGeometry) = static_cast<const PxPlaneGeometry&>(g);
		}
		break;

		case PxGeometryType::eCONVEXMESH:
		{
			reinterpret_cast<PxConvexMeshGeometry&>(mGeometry) = static_cast<const PxConvexMeshGeometry&>(g);
			reinterpret_cast<PxConvexMeshGeometryLL&>(mGeometry).hullData = &(::getConvexMesh(get<PxConvexMeshGeometryLL>().convexMesh).getHull());
			reinterpret_cast<PxConvexMeshGeometryLL&>(mGeometry).gpuCompatible = ::getConvexMesh(get<PxConvexMeshGeometryLL>().convexMesh).isGpuCompatible();
		}
		break;

		case PxGeometryType::eTRIANGLEMESH:
		{
			reinterpret_cast<PxTriangleMeshGeometry&>(mGeometry) = static_cast<const PxTriangleMeshGeometry&>(g);
			reinterpret_cast<PxTriangleMeshGeometryLL&>(mGeometry).meshData = &(::getTriangleMesh(get<PxTriangleMeshGeometryLL>().triangleMesh));
			reinterpret_cast<PxTriangleMeshGeometryLL&>(mGeometry).materialIndices = (::getTriangleMesh(get<PxTriangleMeshGeometryLL>().triangleMesh).getMaterials());
			reinterpret_cast<PxTriangleMeshGeometryLL&>(mGeometry).materials = MaterialIndicesStruct();
		}
		break;

		case PxGeometryType::eHEIGHTFIELD:
		{
			reinterpret_cast<PxHeightFieldGeometry&>(mGeometry) = static_cast<const PxHeightFieldGeometry&>(g);
			reinterpret_cast<PxHeightFieldGeometryLL&>(mGeometry).heightFieldData = &::getHeightField(get<PxHeightFieldGeometryLL>().heightField).getData();
			reinterpret_cast<PxHeightFieldGeometryLL&>(mGeometry).materials = MaterialIndicesStruct();
		}
		break;

		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			PX_ALWAYS_ASSERT_MESSAGE("geometry type not handled");
		break;
	}
}




