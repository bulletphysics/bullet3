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


#ifndef PXS_ISLAND_MANAGER_TYPES_H
#define PXS_ISLAND_MANAGER_TYPES_H

#include "CmPhysXCommon.h"

namespace physx
{

class PxsContactManager;
class PxsRigidBody;
namespace Dy
{
	struct Constraint;
	class Articulation;
}

typedef PxU32 NodeType;
typedef PxU32 EdgeType;
typedef PxU32 IslandType;
#define INVALID_NODE 0xffffffff
#define INVALID_EDGE 0xffffffff
#define INVALID_ISLAND 0xffffffff

namespace Dy
{
	typedef size_t ArticulationLinkHandle;
}

//----------------------------------------------------------------------------//

template <class T, T INVLD> class PxsIslandManagerHook
{
	friend class PxsIslandManager;
	T index;

public:

	static const T INVALID = INVLD;

	PX_FORCE_INLINE PxsIslandManagerHook(): index(INVLD) {}
	PX_FORCE_INLINE PxsIslandManagerHook(const T id): index(id) {}
	PX_FORCE_INLINE PxsIslandManagerHook(const PxsIslandManagerHook<T,INVLD>& src) : index(src.index) {}
	PX_FORCE_INLINE ~PxsIslandManagerHook(){}

	PX_FORCE_INLINE bool isManaged() const { return index!=INVLD; }

private:
};

typedef PxsIslandManagerHook<NodeType,INVALID_NODE> PxsIslandManagerNodeHook;
typedef PxsIslandManagerHook<EdgeType,INVALID_EDGE> PxsIslandManagerEdgeHook;
typedef PxsIslandManagerHook<IslandType,INVALID_ISLAND> PxsIslandManagerIslandHook;

//----------------------------------------------------------------------------//

/** 
\brief PxsIslandObjects contains arrays of all rigid bodies, articulations, contact managers, and constraints that 
belong to all awake islands.  The per array indices denoting the ownership per island are stored in PxsIslandIndices.

@see PxsIslandManager::getIslandObjects
*/
struct PxsIslandObjects
{
	/**
	\brief Array of all rigid bodies in all awake islands. 

	\note Each rigid body corresponds to the void* passed to PxsIslandManager::addBody.  The PxsRigidBody ptr is computed
	by adding the rigid body offset value (passed to PxsIslandManager::create) to the void* pointer 
	ie [(PxsRigidBody*)((PxU8*)owner + rigidBodyOffset)]

	@see PxsIslandManager::addBody, PxsIslandManager::create
	*/
	PxsRigidBody*const*			bodies;	

	/**
	\brief Array of all articulation roots in all awake islands.

	\note Each Articulation* corresponds to Dy::getArticulation(articLinkHandle) where
	articLinkHandle is the handle passed to PxsIslandManager::setArticulationRootLinkHandle.

	@see PxsIslandManager::setArticulationRootLinkHandle, Dy::getArticulation
	*/
	Dy::Articulation*const*		articulations;

	/**
	\brief Array of all articulation roots in all awake islands.

	\note Each void* corresponds to the void* passed to PxsIslandManager::setArticulationRootLinkHandle

	@see PxsIslandManager::setArticulationRootLinkHandle
	*/
	void*const*					articulationOwners;

	/**
	\brief Array of all contact managers in all awake islands.

	@see PxsIslandManager::setEdgeRigidCM
	*/
	struct PxsIndexedContactManager*	contactManagers;

	/**
	\brief Array of all constraints in all awake islands.

	@see PxsIslandManager::setEdgeConstraint
	*/
	struct PxsIndexedConstraint*		constraints;


	PxsIslandObjects() : bodies(NULL), articulations(NULL), articulationOwners(NULL), contactManagers(NULL), constraints(NULL)
	{
	}
};


//----------------------------------------------------------------------------//

/** 
\brief An array of PxsIslandIndices describes the rigid bodies, articulations, contacts and constraints that 
belong to each island.

\note Given an array of PxsIslandIndices, the rigid bodies of the ith island span the inclusive range:
		(PxsIslandObjects::bodies[PxsIslandIndices[i]], PxsIslandObjects::bodies[PxsIslandIndices[i+1]-1]) 

\note Given an array of PxsIslandIndices, the constraints of the ith island span the inclusive range:
		(PxsIslandObjects::constraints[PxsIslandIndices[i]], PxsIslandObjects::constraints[PxsIslandIndices[i+1]-1]) 

@see PxsIslandObjects::getIslandIndices, PxsIslandObjects::getIslandCount
*/
class PxsIslandIndices
{
public:

	PxsIslandIndices(){}
	~PxsIslandIndices(){}

	/**
	\brief Return true if the corresponding island has a contact with a static rigid body.
	*/
	PX_FORCE_INLINE bool getHasStaticContact() const
	{
		return (1 & hasStaticContact) ? true : false;
	}

	/**
	\brief The starting index of island rigid bodies in the array PxsIslandObjects::bodies 
	*/
	NodeType	bodies;

	/**
	\brief The starting index of island articulations in the arrays PxsIslandObjects::articulations and PxsIslandObjects::articulationOwners

	\note The total number of articulations is clamped at 32767 on any platform that uses 16-bit handles.
	*/
	NodeType	articulations : 8*sizeof(NodeType)-1;

private:

	NodeType	hasStaticContact : 1;

public:

	/**
	\brief The starting index of island contact managers in the array PxsIslandObjects::contactManagers.
	*/
	EdgeType	contactManagers;

	/**
	\brief The starting index of island constraints in the array PxsIslandObjects::constraints.

	\note islandId is for internal use only and is used for tracking islands that need a second pass.
	*/
	union
	{
		EdgeType	constraints;	
		IslandType	islandId;		
	};

//private:

	/**
	\brief Internal use only.
	*/
	PX_FORCE_INLINE void setHasStaticContact(const bool b) 
	{
		hasStaticContact = NodeType(b ? 1 : 0);
	}
};
PX_COMPILE_TIME_ASSERT(0==(0x07 & sizeof(PxsIslandIndices)));


//----------------------------------------------------------------------------//

typedef Dy::ArticulationLinkHandle PxsNodeType;


/**
\brief Each contact manager or constraint references two separate bodies, where
a body can be a dynamic rigid body, a kinematic rigid body, an articulation or a static.
The struct PxsIndexedInteraction describes the bodies that make up the pair.
*/
struct PxsIndexedInteraction
{
	/**
	\brief An enumerated list of all possible body types.
	A body type is stored for each body in the pair.
	*/
	enum Enum
	{
		eBODY = 0,
		eKINEMATIC = 1,
		eARTICULATION = 2,
		eWORLD = 3
	};

	/**
	\brief An index describing how to access body0

	\note If body0 is a dynamic (eBODY) rigid body then solverBody0 is an index into PxsIslandObjects::bodies.
	\note If body0 is a kinematic (eKINEMATIC) rigid body then solverBody0 is an index into PxsIslandManager::getActiveKinematics.

	\note If body0 is a static (eWORLD) then solverBody0 is PX_MAX_U32 or PX_MAX_U64, depending on the platform being 32- or 64-bit.

	\note If body0 is an articulation then the articulation is found directly from Dy::getArticulation(articulation0)
	*/
	union
	{
		PxsNodeType					solverBody0;
		Dy::ArticulationLinkHandle	articulation0;
	};

	/**
	\brief An index describing how to access body1

	\note If body1 is a dynamic (eBODY) rigid body then solverBody1 is an index into PxsIslandObjects::bodies.
	\note If body1 is a kinematic (eKINEMATIC) rigid body then solverBody1 is an index into PxsIslandManager::getActiveKinematics.

	\note If body1 is a static (eWORLD) then solverBody1 is PX_MAX_U32 or PX_MAX_U64, depending on the platform being 32- or 64-bit.

	\note If body1 is an articulation then the articulation is found directly from Dy::getArticulation(articulation1)
	*/
	union
	{
		PxsNodeType					solverBody1;
		Dy::ArticulationLinkHandle	articulation1;
	};

	/**
	\brief The type (eBODY, eKINEMATIC etc) of body0
	*/
	PxU8 indexType0;

	/**
	\brief The type (eBODY, eKINEMATIC etc) of body1
	*/
	PxU8 indexType1;

	PxU8 pad[2];
};

/**
@see PxsIslandObjects, PxsIndexedInteraction
*/
struct PxsIndexedContactManager : public PxsIndexedInteraction
{
	/**
	\brief The contact manager corresponds to the value set in PxsIslandManager::setEdgeRigidCM
	*/
	PxsContactManager* contactManager;
	
	PxsIndexedContactManager(PxsContactManager* cm) : contactManager(cm) {}
};
#if !PX_X64
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxsIndexedContactManager) & 0x0f));
#endif

/**
@see PxsIslandObjects, PxsIndexedInteraction
*/
struct PxsIndexedConstraint : public PxsIndexedInteraction
{
	/**
	\brief The constraint corresponds to the value set in PxsIslandManager::setEdgeConstraint
	*/
	Dy::Constraint* constraint;

	PxsIndexedConstraint(Dy::Constraint* c) : constraint(c) {}
};
#if !PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxsIndexedConstraint) & 0x0f));
#endif

//----------------------------------------------------------------------------//

/**
\brief Any sleeping contact pair that finds itself in an awake island after 1st pass island gen
must participate in 2nd pass narrowphase so that contacts can be generated.

\note Contact managers in sleeping pairs are NULL until PxsIslandManager::setWokenPairContactManagers is complete.

@see PxsIslandManager::getNarrowPhaseSecondPassContactManagers, PxsIslandManager::getNumNarrowPhaseSecondPassContactManagers, 
PxsIslandManager::setWokenPairContactManagers
*/
struct PxsNarrowPhaseSecondPassContactManager
{
	/**
	\brief The contact manager that is to participate in 2nd pass narrowphase.

	\note This pointer is NULL after 1st pass island gen and remains NULL until PxsIslandManager::setWokenPairContactManagers 
	completes.
	*/
	PxsContactManager* mCM;

	/**
	\brief The corresponding entry in PxsIslandObjects::contactManagers.

	\note All sleeping pairs have a null contact manager during 1st pass island gen.  After 1st pass island gen completes,
	the bodies to be woken are externally processed.  Waking up bodies generates contact managers and passes the pointer to the
	corresponding edge.  So that the contact manager can be efficiently passed to PxsIslandObjects we store mEdgeId and mSolverCMId.
	The contact manager pointers are set in PxsIslandManager::setWokenPairContactManagers
	*/
	EdgeType mSolverCMId;	//Keeps a track of which entries in the solver islands temporarily have a null contact manager 

	/**
	\brief The internal id of the corresponding edge.
	*/
	EdgeType mEdgeId;
};


} //namespace physx


#endif //PXS_ISLAND_MANAGER_TYPES_H
