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

#include "SqPruningStructure.h"
#include "SqAABBPruner.h"
#include "SqAABBTree.h"
#include "SqBounds.h"

#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpShape.h"

#include "GuBounds.h"

#include "CmTransformUtils.h"
#include "CmUtils.h"

#include "ScbShape.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

//////////////////////////////////////////////////////////////////////////

#define NB_OBJECTS_PER_NODE	4

//////////////////////////////////////////////////////////////////////////
PruningStructure::PruningStructure(PxBaseFlags baseFlags)
	: PxPruningStructure(baseFlags)
{
}

//////////////////////////////////////////////////////////////////////////
PruningStructure::PruningStructure()
	: PxPruningStructure(PxConcreteType::ePRUNING_STRUCTURE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mNbActors(0), mActors(0), mValid(true)
{
	for (PxU32 i = 0; i < 2; i++)
	{
		mNbNodes[i] = 0;
		mNbObjects[i] = 0;
		mAABBTreeIndices[i] = NULL;
		mAABBTreeNodes[i] = NULL;
	}
}

//////////////////////////////////////////////////////////////////////////
PruningStructure::~PruningStructure()
{	
	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		for (PxU32 i = 0; i < 2; i++)
		{
			if(mAABBTreeIndices[i])
			{			
				PX_FREE(mAABBTreeIndices[i]);
			}
			if (mAABBTreeNodes[i])
			{
				PX_FREE(mAABBTreeNodes[i]);
			}
		}

		if(mActors)
		{
			PX_FREE(mActors);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void PruningStructure::release()
{
	// if we release the pruning structure we set the pruner structure to NUUL
	for (PxU32 i = 0; i < mNbActors; i++)
	{		
		PX_ASSERT(mActors[i]);			

		PxType type = mActors[i]->getConcreteType();
		if (type == PxConcreteType::eRIGID_STATIC)
		{
			static_cast<NpRigidStatic*>(mActors[i])->getShapeManager().setPruningStructure(NULL);
		}
		else if (type == PxConcreteType::eRIGID_DYNAMIC)
		{
			static_cast<NpRigidDynamic*>(mActors[i])->getShapeManager().setPruningStructure(NULL);
		}
	}

	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		delete this;
	}
	else
	{
		this->~PruningStructure();
	}
}

template <typename ActorType>
static void getShapeBounds(PxRigidActor* actor, bool dynamic, PxBounds3* bounds, PxU32& numShapes)
{
	PruningIndex::Enum treeStructure = dynamic ? PruningIndex::eDYNAMIC : PruningIndex::eSTATIC;
	ActorType& a = *static_cast<ActorType*>(actor);
	const PxU32 nbShapes = a.getNbShapes();
	for (PxU32 iShape = 0; iShape < nbShapes; iShape++)
	{
		NpShape* shape = a.getShapeManager().getShapes()[iShape];
		if (shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
		{
			const Scb::Shape& scbShape = shape->getScbShape();
			const Scb::Actor& scbActor = a.getScbActorFast();

			(gComputeBoundsTable[treeStructure])(*bounds, scbShape, scbActor);
			bounds++;
			numShapes++;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
bool PruningStructure::build(PxRigidActor*const* actors, PxU32 nbActors)
{
	PX_ASSERT(actors);
	PX_ASSERT(nbActors > 0);
	
	PxU32 numShapes[2] = { 0, 0 };	
	// parse the actors first to get the shapes size
	for (PxU32 actorsDone = 0; actorsDone < nbActors; actorsDone++)
	{
		if (actorsDone + 1 < nbActors)
			Ps::prefetch(actors[actorsDone + 1], sizeof(NpRigidDynamic));	// worst case: PxRigidStatic is smaller

		PxType type = actors[actorsDone]->getConcreteType();
		const PxRigidActor& actor = *(actors[actorsDone]);

		Scb::ControlState::Enum cs = NpActor::getScbFromPxActor(actor).getControlState();
		if (!((cs == Scb::ControlState::eNOT_IN_SCENE) || ((cs == Scb::ControlState::eREMOVE_PENDING))))
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PrunerStructure::build: Actor already assigned to a scene!");
			return false;
		}

		const PxU32 nbShapes = actor.getNbShapes();
		bool hasQueryShape = false;
		for (PxU32 iShape = 0; iShape < nbShapes; iShape++)
		{
			PxShape* shape;
			actor.getShapes(&shape, 1, iShape);
			if(shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
			{
				hasQueryShape = true;
				if (type == PxConcreteType::eRIGID_STATIC)
					numShapes[PruningIndex::eSTATIC]++;
				else
					numShapes[PruningIndex::eDYNAMIC]++;
			}
		}

		// each provided actor must have a query shape
		if(!hasQueryShape)
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PrunerStructure::build: Provided actor has no scene query shape!");
			return false;
		}

		if (type == PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* rs = static_cast<NpRigidStatic*>(actors[actorsDone]);
			if(rs->getShapeManager().getPruningStructure())
			{
				Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PrunerStructure::build: Provided actor has already a pruning structure!");
				return false;
			}			
			rs->getShapeManager().setPruningStructure(this);
		}
		else if (type == PxConcreteType::eRIGID_DYNAMIC)
		{
			NpRigidDynamic* rd = static_cast<NpRigidDynamic*>(actors[actorsDone]);			
			if (rd->getShapeManager().getPruningStructure())
			{
				Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PrunerStructure::build: Provided actor has already a pruning structure!");
				return false;
			}
			rd->getShapeManager().setPruningStructure(this);
		}
		else 
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PrunerStructure::build: Provided actor is not a rigid actor!");
			return false;
		}
	}
	
	PxBounds3* bounds[2] = { NULL, NULL };

	for (PxU32 i = 0; i < 2; i++)
	{
		if(numShapes[i])
		{
			bounds[i] = reinterpret_cast<PxBounds3*>(PX_ALLOC(sizeof(PxBounds3)*(numShapes[i] + 1), "Pruner bounds"));			
		}
	}

	// now I go again and gather bounds and payload
	numShapes[PruningIndex::eSTATIC] = 0;
	numShapes[PruningIndex::eDYNAMIC] = 0;
	for (PxU32 actorsDone = 0; actorsDone < nbActors; actorsDone++)
	{
		PxType type = actors[actorsDone]->getConcreteType();
		if (type == PxConcreteType::eRIGID_STATIC)
		{
			getShapeBounds<NpRigidStatic>(actors[actorsDone], false, 
				&bounds[PruningIndex::eSTATIC][numShapes[PruningIndex::eSTATIC]], numShapes[PruningIndex::eSTATIC]);
		}
		else if (type == PxConcreteType::eRIGID_DYNAMIC)
		{
			getShapeBounds<NpRigidDynamic>(actors[actorsDone], true, 
				&bounds[PruningIndex::eDYNAMIC][numShapes[PruningIndex::eDYNAMIC]], numShapes[PruningIndex::eDYNAMIC]);
		}
	}
	
	AABBTree aabbTrees[2];
	for (PxU32 i = 0; i < 2; i++)
	{
		mNbObjects[i] = numShapes[i];
		if (numShapes[i])
		{
			// create the AABB tree
			AABBTreeBuildParams sTB;
			sTB.mNbPrimitives = numShapes[i];
			sTB.mAABBArray = bounds[i];
			sTB.mLimit = NB_OBJECTS_PER_NODE;
			bool status = aabbTrees[i].build(sTB);

			PX_UNUSED(status);
			PX_ASSERT(status);

			// store the tree nodes
			mNbNodes[i] = aabbTrees[i].getNbNodes();
			mAABBTreeNodes[i] = reinterpret_cast<AABBTreeRuntimeNode*>(PX_ALLOC(sizeof(AABBTreeRuntimeNode)*mNbNodes[i], "AABBTreeRuntimeNode"));
			PxMemCopy(mAABBTreeNodes[i], aabbTrees[i].getNodes(), sizeof(AABBTreeRuntimeNode)*mNbNodes[i]);
			mAABBTreeIndices[i] = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*mNbObjects[i], "PxU32"));
			PxMemCopy(mAABBTreeIndices[i], aabbTrees[i].getIndices(), sizeof(PxU32)*mNbObjects[i]);

			// discard the data
			PX_FREE(bounds[i]);
		}		
	}

	// store the actors for verification and serialization
	mNbActors = nbActors;
	mActors = reinterpret_cast<PxActor**>(PX_ALLOC(sizeof(PxActor*)*mNbActors, "PxActor*"));
	PxMemCopy(mActors, actors, sizeof(PxActor*)*mNbActors);

	return true;
}

//////////////////////////////////////////////////////////////////////////

PruningStructure* PruningStructure::createObject(PxU8*& address, PxDeserializationContext& context)
{
	PruningStructure* obj = new (address)PruningStructure(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(PruningStructure);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::resolveReferences(PxDeserializationContext& context)
{
	if (!isValid())
		return;

	for (PxU32 i = 0; i < mNbActors; i++)
	{
		context.translatePxBase(mActors[i]);
	}	
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::requiresObjects(PxProcessPxBaseCallback& c)
{
	if (!isValid())		
		return;
	
	for (PxU32 i = 0; i < mNbActors; i++)
	{
		c.process(*mActors[i]);
	}
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::exportExtraData(PxSerializationContext& stream)
{
	if (!isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PrunerStructure::exportExtraData: Pruning structure is invalid!");
		return;
	}

	for (PxU32 i = 0; i < 2; i++)
	{
		if (mAABBTreeNodes[i])
		{
			// store nodes
			stream.alignData(PX_SERIAL_ALIGN);
			stream.writeData(mAABBTreeNodes[i], mNbNodes[i] * sizeof(AABBTreeRuntimeNode));
		}

		if(mAABBTreeIndices[i])
		{
			// store indices
			stream.alignData(PX_SERIAL_ALIGN);
			stream.writeData(mAABBTreeIndices[i], mNbObjects[i] * sizeof(PxU32));
		}
	}

	if(mActors)
	{
		// store actor pointers
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mActors, mNbActors * sizeof(PxActor*));
	}
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::importExtraData(PxDeserializationContext& context)
{
	if (!isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PrunerStructure::importExtraData: Pruning structure is invalid!");
		return;
	}

	for (PxU32 i = 0; i < 2; i++)
	{
		if (mAABBTreeNodes[i])
		{
			mAABBTreeNodes[i] = context.readExtraData<Sq::AABBTreeRuntimeNode, PX_SERIAL_ALIGN>(mNbNodes[i]);
		}
		if(mAABBTreeIndices[i])
		{
			mAABBTreeIndices[i] = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(mNbObjects[i]);
		}
	}

	if (mActors)
	{
		// read actor pointers
		mActors = context.readExtraData<PxActor*, PX_SERIAL_ALIGN>(mNbActors);
	}
}

//////////////////////////////////////////////////////////////////////////

PxU32 PruningStructure::getRigidActors(PxRigidActor** userBuffer, PxU32 bufferSize, PxU32 startIndex/* =0 */) const
{	
	if(!isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PrunerStructure::getRigidActors: Pruning structure is invalid!");
		return 0;
	}

	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mActors, mNbActors);
}

//////////////////////////////////////////////////////////////////////////

void PruningStructure::invalidate(PxActor* actor)
{
	PX_ASSERT(actor);

	// remove actor from the actor list to avoid mem corruption
	// this slow, but should be called only with error msg send to user about invalid behavior
	for (PxU32 i = 0; i < mNbActors; i++)
	{
		if(mActors[i] == actor)
		{
			// set pruning structure to NULL and remove the actor from the list
			PxType type = mActors[i]->getConcreteType();
			if (type == PxConcreteType::eRIGID_STATIC)
			{
				static_cast<NpRigidStatic*>(mActors[i])->getShapeManager().setPruningStructure(NULL);
			}
			else if (type == PxConcreteType::eRIGID_DYNAMIC)
			{
				static_cast<NpRigidDynamic*>(mActors[i])->getShapeManager().setPruningStructure(NULL);
			}

			mActors[i] = mActors[mNbActors--];
			break;
		}		
	}

	mValid = false;
}

