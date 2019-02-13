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


#ifndef PX_PHYSICS_NP_ARTICULATION
#define PX_PHYSICS_NP_ARTICULATION

#include "PxArticulation.h"
#include "CmPhysXCommon.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
#include "CmRenderOutput.h"
#endif

#include "ScbArticulation.h"
#include "NpArticulationLink.h"
#include "NpArticulationTemplate.h"
#include "NpArticulationJoint.h"

namespace physx
{

class NpArticulationLink;
class NpScene;
class NpAggregate;
class PxAggregate;

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4250)
#endif


class NpArticulation : public NpArticulationTemplate<PxArticulation>
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

public:
	virtual										~NpArticulation();

// PX_SERIALIZATION
												NpArticulation(PxBaseFlags baseFlags) : NpArticulationTemplate(baseFlags)
												 {}
	
	virtual			void						exportExtraData(PxSerializationContext& stream);
					void						importExtraData(PxDeserializationContext& context);
					void						resolveReferences(PxDeserializationContext& context);
	virtual	        void						requiresObjects(PxProcessPxBaseCallback& c);
	static			NpArticulation*				createObject(PxU8*& address, PxDeserializationContext& context);
	static			void						getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

	//---------------------------------------------------------------------------------
	// PxArticulation implementation
	//---------------------------------------------------------------------------------

	virtual			PxU32						getInternalDriveIterations() const;
	virtual			void						setInternalDriveIterations(PxU32 iterations);

	virtual			PxU32						getExternalDriveIterations() const;
	virtual			void						setExternalDriveIterations(PxU32 iterations);

	virtual			PxU32						getMaxProjectionIterations() const;
	virtual			void						setMaxProjectionIterations(PxU32 iterations);

	virtual			PxReal						getSeparationTolerance() const;
	virtual			void						setSeparationTolerance(PxReal tolerance);


	virtual		PxArticulationDriveCache* 
								createDriveCache(PxReal compliance, PxU32 driveIterations) const;

	virtual		void			updateDriveCache(PxArticulationDriveCache& cache, PxReal compliance, PxU32 driveIterations) const;

	virtual		void			releaseDriveCache(PxArticulationDriveCache&) const;

	virtual		void			applyImpulse(PxArticulationLink*,
											 const PxArticulationDriveCache& driveCache,
											 const PxVec3& force,
											 const PxVec3& torque);

	virtual		void			computeImpulseResponse(PxArticulationLink*,
													   PxVec3& linearResponse, 
													   PxVec3& angularResponse,
													   const PxArticulationDriveCache& driveCache,
													   const PxVec3& force,
													   const PxVec3& torque) const;

	virtual		const char*		getConcreteTypeName() const { return "PxArticulation"; }

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
												NpArticulation();


	virtual		bool			isKindOf(const char* name) const { return !::strcmp("PxArticulation", name) || PxBase::isKindOf(name); }

	virtual PxArticulationJointBase* createArticulationJoint(PxArticulationLink& parent,
		const PxTransform& parentFrame,
		PxArticulationLink& child,
		const PxTransform& childFrame);
	virtual void					 releaseArticulationJoint(PxArticulationJointBase* joint);
};

#if PX_VC
#pragma warning(pop)
#endif

}

#endif
