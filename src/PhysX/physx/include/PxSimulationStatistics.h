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


#ifndef PX_SIMULATION_STATISTICS
#define PX_SIMULATION_STATISTICS
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "foundation/PxAssert.h"
#include "geometry/PxGeometry.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Class used to retrieve statistics for a simulation step.

@see PxScene::getSimulationStatistics()
*/
class PxSimulationStatistics
{
public:

	/**
	\brief Different types of rigid body collision pair statistics.
	@see getRbPairStats
	*/
	enum RbPairStatsType
	{
		/**
		\brief Shape pairs processed as discrete contact pairs for the current simulation step.
		*/
		eDISCRETE_CONTACT_PAIRS,

		/**
		\brief Shape pairs processed as swept integration pairs for the current simulation step.

		\note Counts the pairs for which special CCD (continuous collision detection) work was actually done and NOT the number of pairs which were configured for CCD. 
		Furthermore, there can be multiple CCD passes and all processed pairs of all passes are summed up, hence the number can be larger than the amount of pairs which have been configured for CCD.

		@see PxPairFlag::eDETECT_CCD_CONTACT,
		*/
		eCCD_PAIRS,

		/**
		\brief Shape pairs processed with user contact modification enabled for the current simulation step.

		@see PxContactModifyCallback
		*/
		eMODIFIED_CONTACT_PAIRS,

		/**
		\brief Trigger shape pairs processed for the current simulation step.

		@see PxShapeFlag::eTRIGGER_SHAPE
		*/
		eTRIGGER_PAIRS
	};


//objects:
	/**
	\brief Number of active PxConstraint objects (joints etc.) for the current simulation step.
	*/
	PxU32   nbActiveConstraints;

	/**
	\brief Number of active dynamic bodies for the current simulation step.

	\note Does not include active kinematic bodies
	*/
	PxU32   nbActiveDynamicBodies;

	/**
	\brief Number of active kinematic bodies for the current simulation step.
	
	\note Kinematic deactivation occurs at the end of the frame after the last call to PxRigidDynamic::setKinematicTarget() was called so kinematics that are
	deactivated in a given frame will be included by this counter.
	*/
	PxU32   nbActiveKinematicBodies;

	/**
	\brief Number of static bodies for the current simulation step.
	*/
	PxU32	nbStaticBodies;

	/**
	\brief Number of dynamic bodies for the current simulation step.

	\note Includes inactive and kinematic bodies, and articulation links
	*/
	PxU32   nbDynamicBodies;

	/**
	\brief Number of shapes of each geometry type.
	*/

	PxU32	nbShapes[PxGeometryType::eGEOMETRY_COUNT];

	/**
	\brief Number of aggregates in the scene.
	*/
	PxU32	nbAggregates;
	
	/**
	\brief Number of articulations in the scene.
	*/
	PxU32	nbArticulations;

//solver:
	/**
	\brief The number of 1D axis constraints(joints+contact) present in the current simulation step.
	*/
	PxU32	nbAxisSolverConstraints;

	/**
	\brief The size (in bytes) of the compressed contact stream in the current simulation step
	*/
	PxU32   compressedContactSize;

	/**
	\brief The total required size (in bytes) of the contact constraints in the current simulation step
	*/
	PxU32   requiredContactConstraintMemory;

	/**
	\brief The peak amount of memory (in bytes) that was allocated for constraints (this includes joints) in the current simulation step
	*/
	PxU32   peakConstraintMemory;

//broadphase:
	/**
	\brief Get number of broadphase volumes added for the current simulation step.

	\return Number of broadphase volumes added.
	*/
	PX_FORCE_INLINE	PxU32 getNbBroadPhaseAdds() const
	{
		return nbBroadPhaseAdds;
	}

	/**
	\brief Get number of broadphase volumes removed for the current simulation step.

	\return Number of broadphase volumes removed.
	*/
	PX_FORCE_INLINE	PxU32 getNbBroadPhaseRemoves() const
	{
		return nbBroadPhaseRemoves;
	}

//collisions:
	/**
	\brief Get number of shape collision pairs of a certain type processed for the current simulation step.

	There is an entry for each geometry pair type.

	\note entry[i][j] = entry[j][i], hence, if you want the sum of all pair
	      types, you need to discard the symmetric entries

	\param[in] pairType The type of pair for which to get information
	\param[in] g0 The geometry type of one pair object
	\param[in] g1 The geometry type of the other pair object
	\return Number of processed pairs of the specified geometry types.
	*/
	PxU32 getRbPairStats(RbPairStatsType pairType, PxGeometryType::Enum g0, PxGeometryType::Enum g1) const
	{
		PX_ASSERT_WITH_MESSAGE(	(pairType >= eDISCRETE_CONTACT_PAIRS) &&
								(pairType <= eTRIGGER_PAIRS),
								"Invalid pairType in PxSimulationStatistics::getRbPairStats");

		if (g0 >= PxGeometryType::eGEOMETRY_COUNT || g1 >= PxGeometryType::eGEOMETRY_COUNT)
		{
			PX_ASSERT(false);
			return 0;
		}

		PxU32 nbPairs = 0;
		switch(pairType)
		{
			case eDISCRETE_CONTACT_PAIRS:
				nbPairs = nbDiscreteContactPairs[g0][g1];
				break;
			case eCCD_PAIRS:
				nbPairs = nbCCDPairs[g0][g1];
				break;
			case eMODIFIED_CONTACT_PAIRS:
				nbPairs = nbModifiedContactPairs[g0][g1];
				break;
			case eTRIGGER_PAIRS:
				nbPairs = nbTriggerPairs[g0][g1];
				break;
		}
		return nbPairs;
	}

	/**
	\brief Total number of (non CCD) pairs reaching narrow phase
	*/
	PxU32	nbDiscreteContactPairsTotal;

	/**
	\brief Total number of (non CCD) pairs for which contacts are successfully cached (<=nbDiscreteContactPairsTotal)
	\note This includes pairs for which no contacts are generated, it still counts as a cache hit.
	*/
	PxU32	nbDiscreteContactPairsWithCacheHits;

	/**
	\brief Total number of (non CCD) pairs for which at least 1 contact was generated (<=nbDiscreteContactPairsTotal)
	*/
	PxU32	nbDiscreteContactPairsWithContacts;

	/**
	\brief Number of new pairs found by BP this frame
	*/
	PxU32	nbNewPairs;

	/**
	\brief Number of lost pairs from BP this frame
	*/
	PxU32	nbLostPairs;

	/**
	\brief Number of new touches found by NP this frame
	*/
	PxU32	nbNewTouches;

	/**
	\brief Number of lost touches from NP this frame
	*/
	PxU32	nbLostTouches;

	/**
	\brief Number of partitions used by the solver this frame
	*/
	PxU32	nbPartitions;

	PxSimulationStatistics() :
		nbActiveConstraints					(0),
		nbActiveDynamicBodies				(0),
		nbActiveKinematicBodies				(0),
		nbStaticBodies						(0),
		nbDynamicBodies						(0),
		nbAggregates						(0),
		nbArticulations						(0),
		nbAxisSolverConstraints				(0),
		compressedContactSize				(0),
		requiredContactConstraintMemory		(0),
		peakConstraintMemory				(0),
		nbDiscreteContactPairsTotal			(0),
		nbDiscreteContactPairsWithCacheHits	(0),
		nbDiscreteContactPairsWithContacts	(0),
		nbNewPairs							(0),
		nbLostPairs							(0),
		nbNewTouches						(0),
		nbLostTouches						(0),
		nbPartitions						(0)
	{
		nbBroadPhaseAdds = 0;
		nbBroadPhaseRemoves = 0;

		for(PxU32 i=0; i < PxGeometryType::eGEOMETRY_COUNT; i++)
		{
			for(PxU32 j=0; j < PxGeometryType::eGEOMETRY_COUNT; j++)
			{
				nbDiscreteContactPairs[i][j] = 0;
				nbModifiedContactPairs[i][j] = 0;
				nbCCDPairs[i][j] = 0;
				nbTriggerPairs[i][j] = 0;
			}
		}

		for(PxU32 i=0; i < PxGeometryType::eGEOMETRY_COUNT; i++)
		{
			nbShapes[i] = 0;
		}
	}


	//
	// We advise to not access these members directly. Use the provided accessor methods instead.
	//
//broadphase:
	PxU32	nbBroadPhaseAdds;
	PxU32	nbBroadPhaseRemoves;

//collisions:
	PxU32   nbDiscreteContactPairs[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];
	PxU32   nbCCDPairs[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];
	PxU32   nbModifiedContactPairs[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];
	PxU32   nbTriggerPairs[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
