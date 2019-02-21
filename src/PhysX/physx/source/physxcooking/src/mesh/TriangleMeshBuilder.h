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


#ifndef PX_COLLISION_TriangleMeshBUILDER
#define PX_COLLISION_TriangleMeshBUILDER

#include "GuMeshData.h"
#include "cooking/PxCooking.h"
#include "MeshBuilder.h"

namespace physx
{
	namespace Gu
	{
		class EdgeListBuilder;
	}

	class TriangleMeshBuilder : public MeshBulider
	{
		public:
											TriangleMeshBuilder(Gu::TriangleMeshData& mesh, const PxCookingParams& params);
		virtual								~TriangleMeshBuilder();

		virtual	PxMeshMidPhase::Enum		getMidphaseID()									const	= 0;
		// Called by base code when midphase structure should be built
		virtual	void						createMidPhaseStructure()								= 0;

		// Called by base code when midphase structure should be saved
		virtual	void						saveMidPhaseStructure(PxOutputStream& stream, bool mismatch)	const	= 0;
		// Called by base code when mesh index format has changed and the change should be reflected in midphase structure
		virtual	void						onMeshIndexFormatChange()								{}

				bool						cleanMesh(bool validate, PxTriangleMeshCookingResult::Enum* condition);
				void						remapTopology(const PxU32* order);
		
				void						createSharedEdgeData(bool buildAdjacencies, bool buildActiveEdges);

				void						recordTriangleIndices();
				void						createGRBMidPhaseAndData(const PxU32 originalTriangleCount);
				void						createGRBData();

				bool						loadFromDesc(const PxTriangleMeshDesc&, PxTriangleMeshCookingResult::Enum* condition, bool validate = false);
				bool						save(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params) const;
				void						checkMeshIndicesSize();
	PX_FORCE_INLINE	Gu::TriangleMeshData&	getMeshData()	{ return mMeshData;	}
	protected:
				//void						computeLocalBounds();
				bool						importMesh(const PxTriangleMeshDesc& desc, const PxCookingParams& params, PxTriangleMeshCookingResult::Enum* condition, bool validate = false);

				TriangleMeshBuilder& operator=(const TriangleMeshBuilder&);
				Gu::EdgeListBuilder*		edgeList;
				const PxCookingParams&		mParams;
				Gu::TriangleMeshData&		mMeshData;

				void						releaseEdgeList();
				void						createEdgeList();
	};

	class RTreeTriangleMeshBuilder : public TriangleMeshBuilder
	{
		public:
											RTreeTriangleMeshBuilder(const PxCookingParams& params);
		virtual								~RTreeTriangleMeshBuilder();

		virtual	PxMeshMidPhase::Enum		getMidphaseID()	const	{ return PxMeshMidPhase::eBVH33;	}
		virtual	void						createMidPhaseStructure();
		virtual	void						saveMidPhaseStructure(PxOutputStream& stream, bool mismatch)	const;

				Gu::RTreeTriangleData		mData;
	};

	class BV4TriangleMeshBuilder : public TriangleMeshBuilder
	{
		public:
											BV4TriangleMeshBuilder(const PxCookingParams& params);
		virtual								~BV4TriangleMeshBuilder();

		virtual	PxMeshMidPhase::Enum		getMidphaseID()	const	{ return PxMeshMidPhase::eBVH34;	}
		virtual	void						createMidPhaseStructure();
		virtual	void						saveMidPhaseStructure(PxOutputStream& stream, bool mismatch)	const;
		virtual	void						onMeshIndexFormatChange();

				Gu::BV4TriangleData			mData;
	};

	class BV32TriangleMeshBuilder
	{
	public:
		static	void						createMidPhaseStructure(const PxCookingParams& params, Gu::TriangleMeshData& meshData, Gu::BV32Tree& bv32Tree);
		static	void						saveMidPhaseStructure(Gu::BV32Tree* tree, PxOutputStream& stream, bool mismatch);
	};


}

#endif
