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


#ifndef PX_PHYSICS_NP_SHAPE_MANAGER
#define PX_PHYSICS_NP_SHAPE_MANAGER

#include "NpShape.h"
#include "CmPtrTable.h"
#include "SqSceneQueryManager.h"
#include "GuBVHStructure.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
#include "CmRenderOutput.h"
#endif

namespace physx
{

namespace Sq
{
	class SceneQueryManager;
	class PruningStructure;
}

class NpScene;

class NpShapeManager : public Ps::UserAllocated
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
	static			void					getBinaryMetaData(PxOutputStream& stream);
											NpShapeManager(const PxEMPTY);
					void					exportExtraData(PxSerializationContext& stream);
					void					importExtraData(PxDeserializationContext& context);
//~PX_SERIALIZATION
											NpShapeManager();
											~NpShapeManager();

	PX_FORCE_INLINE	PxU32					getNbShapes()		const	{ return mShapes.getCount();									}
	PX_FORCE_INLINE	NpShape* const*			getShapes()			const	{ return reinterpret_cast<NpShape*const*>(mShapes.getPtrs());	}
					PxU32					getShapes(PxShape** buffer, PxU32 bufferSize, PxU32 startIndex=0) const;

					void					attachShape(NpShape& shape, PxRigidActor& actor);
					bool					detachShape(NpShape& s, PxRigidActor &actor, bool wakeOnLostTouch);
					void					detachAll(NpScene *scene, const PxRigidActor& actor);

					void					teardownSceneQuery(Sq::SceneQueryManager& sqManager, const NpShape& shape);
					void					setupSceneQuery(Sq::SceneQueryManager& sqManager, const PxRigidActor& actor, const NpShape& shape);

					void					addPrunerShape(Sq::SceneQueryManager& sqManager, PxU32 index, const NpShape& shape, const PxRigidActor& actor, bool dynamic, const PxBounds3* bound, bool hasPrunerStructure);

	PX_FORCE_INLINE void					setPrunerData(PxU32 index, Sq::PrunerData data)
											{
												PX_ASSERT(index<getNbShapes());
												mSceneQueryData.getPtrs()[index] = reinterpret_cast<void*>(data);
											}

	PX_FORCE_INLINE Sq::PrunerData			getPrunerData(PxU32 index)	const
											{
												PX_ASSERT(index<getNbShapes());
												return Sq::PrunerData(mSceneQueryData.getPtrs()[index]);
											}

					void					setupAllSceneQuery(NpScene* scene, const PxRigidActor& actor, bool hasPrunerStructure, const PxBounds3* bounds=NULL, const Gu::BVHStructure* bvhStructure = NULL);
					void					teardownAllSceneQuery(Sq::SceneQueryManager& sqManager, const PxRigidActor& actor);
					void					markAllSceneQueryForUpdate(Sq::SceneQueryManager& shapeManager, const PxRigidActor& actor);
					
					Sq::PrunerData			findSceneQueryData(const NpShape& shape, Sq::PrunerCompoundId& compoundId) const;
					Sq::PrunerData			findSceneQueryData(const NpShape& shape) const;

					PxBounds3				getWorldBounds(const PxRigidActor&) const;

	PX_FORCE_INLINE	void					setPruningStructure(Sq::PruningStructure* ps) { mPruningStructure = ps;		}
	PX_FORCE_INLINE	Sq::PruningStructure*	getPruningStructure()					const { return mPruningStructure;	}

					void					addBVHStructureShapes(Sq::SceneQueryManager& sqManager, const PxRigidActor& actor, const Gu::BVHStructure* bvhStructure);
	PX_FORCE_INLINE	Sq::PrunerCompoundId	getSqCompoundId()					const { return mSqCompoundId; }
	PX_FORCE_INLINE	bool					isSqCompound()						const { return (mSqCompoundId == Sq::INVALID_PRUNERHANDLE) ? false : true; }

					void					clearShapesOnRelease(Scb::Scene& s, PxRigidActor&);
					void					releaseExclusiveUserReferences();

#if PX_ENABLE_DEBUG_VISUALIZATION
					void					visualize(Cm::RenderOutput& out, NpScene* scene, const PxRigidActor& actor);
#endif
					// for batching
	PX_FORCE_INLINE	const Cm::PtrTable&		getShapeTable() const 		{	return mShapes; }
protected:
					void					setupSceneQuery(Sq::SceneQueryManager& sqManager, const PxRigidActor& actor, PxU32 index);
					void					teardownSceneQuery(Sq::SceneQueryManager& sqManager, PxU32 index);

					// PT: TODO: revisit this. We don't need two arrays.
					Cm::PtrTable			mShapes;
					Cm::PtrTable			mSceneQueryData;	// 1-1 correspondence with shapes - TODO: allocate on scene insertion or combine with the shape array for better caching
					Sq::PrunerCompoundId	mSqCompoundId;
					Sq::PruningStructure*	mPruningStructure;  // Shape scene query data are pre-build in pruning structure
};

}

#endif
