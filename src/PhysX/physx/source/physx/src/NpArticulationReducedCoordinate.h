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


#ifndef PX_PHYSICS_NP_ARTICULATION_RC
#define PX_PHYSICS_NP_ARTICULATION_RC

#include "PxArticulationReducedCoordinate.h"
#include "CmPhysXCommon.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
#include "CmRenderOutput.h"
#endif

#include "ScbArticulation.h"
#include "NpArticulationLink.h"
#include "NpArticulationTemplate.h"
#include "NpArticulationJointReducedCoordinate.h"

namespace physx
{

	class NpArticulationLink;
	class NpScene;
	class NpAggregate;
	class PxAggregate;
	class PxJoint;

	class NpArticulationReducedCoordinate : public NpArticulationTemplate<PxArticulationReducedCoordinate>
	{
		//= ATTENTION! =====================================================================================
		// Changing the data layout of this class breaks the binary serialization format.  See comments for 
		// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
		// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
		// accordingly.
		//==================================================================================================
	public:
		virtual										~NpArticulationReducedCoordinate();

		//KS - todo - re-enable serialization later...
		// PX_SERIALIZATION
		NpArticulationReducedCoordinate(PxBaseFlags baseFlags) : NpArticulationTemplate(baseFlags)
		{
			mType = PxArticulationBase::eReducedCoordinate;
		}

		/*virtual			void						exportExtraData(PxSerializationContext& stream);
		void						importExtraData(PxDeserializationContext& context);
		void						resolveReferences(PxDeserializationContext& context);
		virtual	        void						requires(PxProcessPxBaseCallback& c);
		static			NpArticulation*				createObject(PxU8*& address, PxDeserializationContext& context);
		static			void						getBinaryMetaData(PxOutputStream& stream);*/
		//~PX_SERIALIZATION

		//---------------------------------------------------------------------------------
		// PxArticulation implementation
		//---------------------------------------------------------------------------------

		//---------------------------------------------------------------------------------
		// PxArticulationReducedCoordinate implementation
		//---------------------------------------------------------------------------------
		virtual		void						setArticulationFlags(PxArticulationFlags flags);

		virtual		void						setArticulationFlag(PxArticulationFlag::Enum flag, bool value);

		virtual		PxArticulationFlags			getArticulationFlags() const;

		virtual		PxU32						getDofs() const;

		virtual		PxArticulationCache*		createCache() const;

		virtual		PxU32						getCacheDataSize() const;

		virtual		void						zeroCache(PxArticulationCache& cache);

		virtual		void						applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, bool autowake);

		virtual		void						copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const;

		virtual		void						releaseCache(PxArticulationCache& cache) const;

		virtual		void						packJointData(const PxReal* maximum, PxReal* reduced) const;

		virtual		void						unpackJointData(const PxReal* reduced, PxReal* maximum) const;

		virtual		void						commonInit() const;

		virtual		void						computeGeneralizedGravityForce(PxArticulationCache& cache) const;

		virtual		void						computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const;

		virtual		void						computeGeneralizedExternalForce(PxArticulationCache& cache) const;

		virtual		void						computeJointAcceleration(PxArticulationCache& cache) const;
		
		virtual		void						computeJointForce(PxArticulationCache& cache) const;

		virtual		void						computeKinematicJacobian(const PxU32 linkID, PxArticulationCache& cache) const;

		virtual		void						computeCoefficentMatrix(PxArticulationCache& cache) const;

		virtual		bool						computeLambda(PxArticulationCache& cache, PxArticulationCache& rollBackCache, const PxReal* const jointTorque, const PxU32 maxIter) const;

		virtual		void						computeGeneralizedMassMatrix(PxArticulationCache& cache) const;

		virtual		void						addLoopJoint(PxJoint* joint);

		virtual		void						removeLoopJoint(PxJoint* constraint);

		virtual		PxU32						getNbLoopJoints() const;

		virtual		PxU32						getLoopJoints(PxJoint** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const;

		virtual		PxU32						getCoefficentMatrixSize() const;

		virtual		void						teleportRootLink(const PxTransform& pose, bool autowake);

		virtual		const char*					getConcreteTypeName() const { return "PxArticulation"; }

		//---------------------------------------------------------------------------------
		// Miscellaneous
		//---------------------------------------------------------------------------------
		NpArticulationReducedCoordinate();

		virtual		bool			isKindOf(const char* name) const { return !::strcmp("PxArticulation", name) || PxBase::isKindOf(name); }

		virtual PxArticulationJointBase* createArticulationJoint(PxArticulationLink& parent,
			const PxTransform& parentFrame,
			PxArticulationLink& child,
			const PxTransform& childFrame);
		virtual void					 releaseArticulationJoint(PxArticulationJointBase* joint);

	private:

		Ps::Array<PxJoint*>			mLoopJoints;

		friend class NpScene;
	};


}

#endif
