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


#ifndef PX_PHYSICS_NX_ARTICULATION_RC
#define PX_PHYSICS_NX_ARTICULATION_RC
/** \addtogroup physics
@{ */

#include "PxArticulationBase.h"
#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	namespace Cm
	{
		class SpatialVector;
	}

	class PxContactJoint;

	struct PxArticulationFlag
	{
		enum Enum
		{
			eFIX_BASE = (1 << 1)
		};
	};

	
	class PxJoint;

	typedef PxFlags<PxArticulationFlag::Enum, PxU8> PxArticulationFlags;
	PX_FLAGS_OPERATORS(PxArticulationFlag::Enum, PxU8)

	//PxKinematicJacobian is in world space 6x6 matrix
	class PxKinematicJacobian
	{
	public:
		//in each single column, top part is angular term and bottom is linear term
		PxReal j[6][6];//[column][row]

		PxU32	nbColumns;
	};

	struct PxArticulationRootLinkData
	{

		PxTransform				transform;
		PxVec3					linVel;
		PxVec3					angVel;
		PxVec3					linAcel;
		PxVec3					angAcel;
	};

	class PxArticulationCache
	{
	public:
		enum Enum
		{
			eVELOCITY		= (1 << 0),		//!< The joint velocities this frame. Note, this is the accumulated joint velocities, not change in joint velocity.
			eACCELERATION	= (1 << 1),		//!< The joint accelerations this frame. Delta velocity can be computed from acceleration * dt.
			ePOSITION		= (1 << 2),		//!< The joint positions this frame. Note, this is the accumulated joint positions over frames, not change in joint position.
			eFORCE			= (1 << 3),		//!< The joint forces this frame. Note, the application should provide these values for the forward dynamic. If the application is using inverse dynamic, this is the joint force returned.
			eROOT			= (1 << 4),		//!< Root link transform, velocity and acceleration. Note, when the application call applyCache with eROOT flag, it won't apply root link's acceleration to the simulation
			eALL			= (eVELOCITY | eACCELERATION | ePOSITION| eROOT)
		};
		PxArticulationCache() : coefficentMatrix(NULL), lambda(NULL)
		{}

		Cm::SpatialVector*		externalForces; // N total number of links
		PxKinematicJacobian*	jacobian; //this store jacobian matrix
		PxReal*					massMatrix; //N X N (dof X dof)
		PxReal*					jointVelocity; //N total Dofs
		PxReal*					jointAcceleration; //N total Dofs
		PxReal*					jointPosition; //N total Dofs
		PxReal*					jointForce; //N total Dofs

		//application need to allocate those memory and assign them to the cache
		PxReal*					coefficentMatrix; 
		PxReal*					lambda;

		//root link data
		PxArticulationRootLinkData	rootLinkData;

		//These three members won't be set to zero when zeroCache get called 
		void*					scratchMemory;		//this is used for internal calculation
		void*					scratchAllocator;
		PxU32					version; //cache version. If the articulation configulation change, the cache is invalid

		
	};

	typedef PxFlags<PxArticulationCache::Enum, PxU8> PxArticulationCacheFlags;
	PX_FLAGS_OPERATORS(PxArticulationCache::Enum, PxU8)


	/**
	\brief a tree structure of bodies connected by joints that is treated as a unit by the dynamics solver

	Articulations are more expensive to simulate than the equivalent collection of
	PxRigidDynamic and PxJoint structures, but because the dynamics solver treats
	each articulation as a single object, they are much less prone to separation and
	have better support for actuation. An articulation may have at most 64 links.

	@see PxArticulationJoint PxArticulationLink PxPhysics.createArticulation
	*/

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4435)
#endif

	class PxArticulationReducedCoordinate : public PxArticulationBase
	{
	public:

		virtual		void							release() = 0;


		/**
		\brief Sets flags on the articulation

		\param[in] flags Articulation flags

		*/
		virtual		void							setArticulationFlags(PxArticulationFlags flags) = 0;

		/**
		\brief Raises or clears a flag on the articulation

		\param[in] flag The articulation flag
		\param[in] value true/false indicating whether to raise or clear the flag

		*/
		virtual		void							setArticulationFlag(PxArticulationFlag::Enum flag, bool value) = 0;

		/**
		\brief return PxArticulationFlags
		*/
		virtual		PxArticulationFlags				getArticulationFlags() const = 0;

		/**
		\brief returns the total Dofs of the articulation
		*/
		virtual			PxU32						getDofs() const = 0;

		/**
		\brief create an articulation cache

		\note this call may only be made on articulations that are in a scene, and may not be made during simulation
		*/
		virtual		PxArticulationCache*			createCache() const = 0;

		/**
		\brief Get the size of the articulation cache

		\note this call may only be made on articulations that are in a scene, and may not be made during simulation
		*/
		virtual		PxU32							getCacheDataSize() const = 0;

		/**
		\brief zero all data in the articulation cache beside the cache version

		\note this call may only be made on articulations that are in a scene, and may not be made during simulation
		*/
		virtual		void							zeroCache(PxArticulationCache& cache) = 0;

	
		/**
		\brief apply the user defined data in the cache to the articulation system

		\param[in] cache articulation data.
		\param[in] flag The mode to use when determine which value in the cache will be applied to the articulation
		\param[in] autowake Specify if the call should wake up the articulation if it is currently asleep. If true and the current wake counter value is smaller than #PxSceneDesc::wakeCounterResetValue it will get increased to the reset value.

		@see createCache copyInternalStateToCache
		*/
		virtual		void							applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, bool autowake = true) = 0;

		/**
		\brief copy the internal data of the articulation to the cache

		\param[in] cache articulation data
		\param[in] flag this indicates what kind of data the articulation system need to copy to the cache

		@see createCache applyCache
		*/
		virtual		void						copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const = 0;
	

		/**
		\brief release an articulation cache

		\param[in] cache the cache to release

		@see createCache applyCache copyInternalStateToCache
		*/
		virtual		void						releaseCache(PxArticulationCache& cache) const = 0;
	

		/**
		\brief reduce the maximum data format to the reduced internal data
		\param[in] maximum joint data format
		\param[out] reduced joint data format
		*/
		virtual		void						packJointData(const PxReal* maximum, PxReal* reduced) const = 0;

		/**
		\brief turn the reduced internal data to maximum joint data format
		\param[in] reduced joint data format
		\param[out] maximum joint data format
		*/
		virtual		void						unpackJointData(const PxReal* reduced, PxReal* maximum) const = 0;

		/**
		\brief initialize all the common data for inverse dynamic
		*/
		virtual		void						commonInit() const = 0;


		/**
		\brief determine the statically balance of the joint force of gravity for entire articulation. External force, joint velocity and joint acceleration
		are set to zero, the joint force returned will be purely determined by gravity.

		\param[out] cache return joint forces which can counteract gravity force
		
		@see commonInit
		*/
		virtual		void						computeGeneralizedGravityForce(PxArticulationCache& cache) const = 0;

		/**
		\brief determine coriolise and centrifugal force. External force, gravity and joint acceleration
		are set to zero, the joint force return will be coriolise and centrifugal force for each joint.

		\param[in] cache data
		
		@see commonInit
		*/
		virtual		void						computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const = 0;

		/**
		\brief determine joint force change caused by external force. Gravity, joint acceleration and joint velocity
		are all set to zero.

		\param[in] cache data

		@see commonInit
		*/
		virtual		void						computeGeneralizedExternalForce(PxArticulationCache& cache) const = 0;
		/**
		\brief determine the joint acceleration for each joint
		This is purely calculates the change in joint acceleration due to change in the joint force

		\param[in] cache articulation data
		
		@see commonInit
		*/
		virtual		void						computeJointAcceleration(PxArticulationCache& cache) const = 0;


		/**
		\brief determine the joint force
		This is purely calculates the change in joint force due to change in the joint acceleration
		This means gravity and joint velocity will be zero

		\param[in] cache return joint force 
		
		@see commonInit
		*/
		virtual		void						computeJointForce(PxArticulationCache& cache) const = 0;

		/**
		\brief compute the kinematic jacobian for each joint from end effector to the root in world space
		\param[in] linkID is the end effector id
		\param[in] cache return jacobian matrix

		@see commonInit
		*/
		virtual		void						computeKinematicJacobian(const PxU32 linkID, PxArticulationCache& cache) const = 0;


		/**
		\brief compute the coefficent matrix for contact force. PxContactJoint is the contact point
		\param[out] cache returs the coefficent matrix. Each column is the joint force effected by a contact based on impulse strength 1
		@see commonInit
		*/
		virtual		void						computeCoefficentMatrix(PxArticulationCache& cache) const = 0;

		
		/**
		\brief compute the lambda value when the test impulse is 1
		\param[in] initialState the initial state of the articulation system
		\param[in] jointTorque M(q)*qddot + C(q,qdot) + g(q)
		\param[in] maxIter maximum number of solver iterations to run. If the system converges, fewer iterations may be used. 
		\param[out] cache returns the coefficent matrix. Each column is the joint force effected by a contact based on impulse strength 1
		@see commonInit
		*/
		virtual		bool						computeLambda(PxArticulationCache& cache, PxArticulationCache& initialState, const PxReal* const jointTorque, const PxU32 maxIter) const = 0;
		
		/**
		\brief compute the joint-space inertia matrix
		\param[in] cache articulation data

		@see commonInit
		*/
		virtual		void						computeGeneralizedMassMatrix(PxArticulationCache& cache) const = 0;
	
		/**
		\brief add loop joint to the articulation system for inverse dynamic
		\param[in] joint required to add loop joint

		@see commonInit
		*/
		virtual		void						addLoopJoint(PxJoint* joint) = 0;

		/**
		\brief remove loop joint from the articulation system
		\param[in] joint required to remove loop joint

		@see commonInit
		*/
		virtual		void						removeLoopJoint(PxJoint* joint) = 0;

		/**
		\brief returns the number of loop joints in the articulation
		\return number of loop joints
		*/

		virtual		PxU32						getNbLoopJoints() const = 0;

		/**
		\brief returns the set of loop constraints in the articulation

		\param[in] userBuffer buffer into which to write an array of constraints pointers
		\param[in] bufferSize the size of the buffer. If this is not large enough to contain all the pointers to links,
		only as many as will fit are written.
		\param[in] startIndex Index of first link pointer to be retrieved

		\return the number of links written into the buffer.

		@see ArticulationLink
		*/

		virtual		PxU32						getLoopJoints(PxJoint** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

		/**
		\brief returns the required size of coeffient matrix in the articulation. The coefficient matrix is number of constraint(loop joints) by total dofs. Constraint Torque = transpose(K) * lambda(). Lambda is a vector of number of constraints
		\return bite size of the coefficient matrix(nc * n)
		*/

		virtual		PxU32						getCoefficentMatrixSize() const = 0;

		/**
		\brief teleport root link to a new location
		\param[in] pose the new location of the root link
		\param[in] autowake wake up the articulation system
	
		@see commonInit
		*/
		virtual		void						teleportRootLink(const PxTransform& pose, bool autowake) = 0;

	protected:
		PX_INLINE					PxArticulationReducedCoordinate(PxType concreteType, PxBaseFlags baseFlags) : PxArticulationBase(concreteType, baseFlags) {}
		PX_INLINE					PxArticulationReducedCoordinate(PxBaseFlags baseFlags) : PxArticulationBase(baseFlags) {}
		virtual						~PxArticulationReducedCoordinate() {}
	};

#if PX_VC
#pragma warning(pop)
#endif

#if !PX_DOXYGEN
} // namespace physx
#endif


  /** @} */
#endif
