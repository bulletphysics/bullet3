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


#include "NpCast.h"
#include "NpWriteCheck.h"
#include "NpReadCheck.h"
#include "NpArticulationJointReducedCoordinate.h"

namespace physx
{
	NpArticulationJointReducedCoordinate* NpArticulationJointReducedCoordinate::createObject(PxU8*& address, PxDeserializationContext& context)
	{
		NpArticulationJointReducedCoordinate* obj = new (address) NpArticulationJointReducedCoordinate(PxBaseFlags(0));
		address += sizeof(NpArticulationJointReducedCoordinate);
		obj->importExtraData(context);
		obj->resolveReferences(context);
		return obj;
	}

	void NpArticulationJointReducedCoordinate::getBinaryMetaData(PxOutputStream& stream)
	{
		// 184 => 200 => 192 => 224 => 208 bytes
		PX_DEF_BIN_METADATA_VCLASS(stream, NpArticulationJointReducedCoordinate)
		PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationJointReducedCoordinate, PxBase)

		PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationJointReducedCoordinate, PxArticulationJointImpl, mImpl, 0)

	}
	//~PX_SERIALIZATION
	NpArticulationJointReducedCoordinate::NpArticulationJointReducedCoordinate(NpArticulationLink& parent,
		const PxTransform& parentFrame,
		NpArticulationLink& child,
		const PxTransform& childFrame) :
		NpArticulationJointTemplate(parent, parentFrame, child, childFrame, PxArticulationBase::eReducedCoordinate)
	{

	}

	NpArticulationJointReducedCoordinate::~NpArticulationJointReducedCoordinate()
	{
	}


	void NpArticulationJointReducedCoordinate::setJointType(PxArticulationJointType::Enum jointType)
	{
		NP_WRITE_CHECK(getOwnerScene());
		PX_CHECK_AND_RETURN(jointType != PxArticulationJointType::eUNDEFINED, "PxArticulationJointReducedCoordinate::setJointType valid joint type(ePRISMATIC, eREVOLUTE, eSPHERICAL, eFIX) need to be set");
		mImpl.getScbArticulationJoint().setJointType(jointType);
	}
	PxArticulationJointType::Enum NpArticulationJointReducedCoordinate::getJointType() const
	{
		NP_READ_CHECK(getOwnerScene());
		return mImpl.getScbArticulationJoint().getJointType();
	}

#if PX_CHECKED
	bool NpArticulationJointReducedCoordinate::isValidMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
	{

		PxArticulationJointType::Enum type = getJointType();

		bool valid = true;
		switch (type)
		{
		case PxArticulationJointType::ePRISMATIC:
		{
			if (axis < PxArticulationAxis::eX && motion != PxArticulationMotion::eLOCKED)
				valid = false;
			else if(motion != PxArticulationMotion::eLOCKED)
			{
				//Check to ensure that we only have zero DOFs already active...
				for (PxU32 i = PxArticulationAxis::eX; i <= PxArticulationAxis::eZ; i++)
				{
					if(i != PxU32(axis) && mImpl.mJoint.getMotion(PxArticulationAxis::Enum(i)) != PxArticulationMotion::eLOCKED)
						valid = false;
				}
			}
			break;
		}
		case PxArticulationJointType::eREVOLUTE:
		{
			if (axis >= PxArticulationAxis::eX && motion != PxArticulationMotion::eLOCKED)
				valid = false;
			else if (motion != PxArticulationMotion::eLOCKED)
			{

				for (PxU32 i = PxArticulationAxis::eTWIST; i < PxArticulationAxis::eX; i++)
				{
					if (i != PxU32(axis) && this->mImpl.mJoint.getMotion(PxArticulationAxis::Enum(i)) != PxArticulationMotion::eLOCKED)
						valid = false;
				}
			}
			break;
		}
		case PxArticulationJointType::eSPHERICAL:
		{
			if (axis >= PxArticulationAxis::eX && motion != PxArticulationMotion::eLOCKED)
				valid = false;
			break;
		}
		case PxArticulationJointType::eFIX:
		{
			if (motion != PxArticulationMotion::eLOCKED)
				valid = false;
			break;
		}
		case PxArticulationJointType::eUNDEFINED:
		{
			valid = false;
			break;
		}
		default:
			break;
		}

		return valid;
	}

	/*bool NpArticulationJointReducedCoordinate::isValidType(PxArticulationJointType::Enum type)
	{
		bool hasPrismatic = (getMotion(PxArticulationAxis::eX) != PxArticulationMotion::eLOCKED) ||
			(getMotion(PxArticulationAxis::eY) != PxArticulationMotion::eLOCKED) ||
			(getMotion(PxArticulationAxis::eZ) != PxArticulationMotion::eLOCKED);

		bool hasRotation = (getMotion(PxArticulationAxis::eTWIST) != PxArticulationMotion::eLOCKED) ||
			(getMotion(PxArticulationAxis::eSWING1) != PxArticulationMotion::eLOCKED) ||
			(getMotion(PxArticulationAxis::eSWING2) != PxArticulationMotion::eLOCKED);

		if (type == PxArticulationJointType::eFIX)
			return !hasPrismatic && !hasRotation;
		if (type == PxArticulationJointType::eREVOLUTE || type == PxArticulationJointType::eSPHERICAL)
			return !hasPrismatic;
		if (type == PxArticulationJointType::ePRISMATIC)
			return !hasRotation;

		return true;
	}*/
#endif

	void NpArticulationJointReducedCoordinate::setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
	{
		NP_WRITE_CHECK(getOwnerScene());
		PX_CHECK_AND_RETURN(getJointType() != PxArticulationJointType::eUNDEFINED, "PxArticulationJointReducedCoordinate::setMotion valid joint type(ePRISMATIC, eREVOLUTE, eSPHERICAL or eFIX) has to be set before setMotion");
		PX_CHECK_AND_RETURN(isValidMotion(axis, motion), "PxArticulationJointReducedCoordinate::setMotion illegal motion state requested.");
		mImpl.getScbArticulationJoint().setMotion(axis, motion);
		reinterpret_cast<PxArticulationImpl*>(getChild().getArticulation().getImpl())->increaseCacheVersion();
	}

	PxArticulationMotion::Enum NpArticulationJointReducedCoordinate::getMotion(PxArticulationAxis::Enum axis) const
	{
		NP_READ_CHECK(getOwnerScene());
		return mImpl.getScbArticulationJoint().getMotion(axis);
	}

	void NpArticulationJointReducedCoordinate::setFrictionCoefficient(const PxReal coefficient)
	{
		NP_WRITE_CHECK(getOwnerScene());

		mImpl.getScbArticulationJoint().setFrictionCoefficient(coefficient);
	}

	PxReal NpArticulationJointReducedCoordinate::getFrictionCoefficient() const
	{
		NP_READ_CHECK(getOwnerScene());

		return mImpl.getScbArticulationJoint().getFrictionCoefficient();
	}

	void NpArticulationJointReducedCoordinate::setMaxJointVelocity(const PxReal maxJointV)
	{
		NP_WRITE_CHECK(getOwnerScene());

		mImpl.getScbArticulationJoint().setMaxJointVelocity(maxJointV);
	}

	PxReal NpArticulationJointReducedCoordinate::getMaxJointVelocity() const
	{
		NP_READ_CHECK(getOwnerScene());

		return mImpl.getScbArticulationJoint().getMaxJointVelocity();
	}

	void NpArticulationJointReducedCoordinate::setLimit(PxArticulationAxis::Enum axis, const PxReal lowLimit, const PxReal highLimit)
	{
		mImpl.getScbArticulationJoint().setLimit(axis, lowLimit, highLimit);
	}
	void NpArticulationJointReducedCoordinate::getLimit(PxArticulationAxis::Enum axis, PxReal& lowLimit, PxReal& highLimit)
	{
		mImpl.getScbArticulationJoint().getLimit(axis, lowLimit, highLimit);
	}
	void NpArticulationJointReducedCoordinate::setDrive(PxArticulationAxis::Enum axis, const PxReal stiffness, const PxReal damping, const PxReal maxForce, bool isAccelerationDrive)
	{
		mImpl.getScbArticulationJoint().setDrive(axis, stiffness, damping, maxForce, isAccelerationDrive);
	}
	void NpArticulationJointReducedCoordinate::getDrive(PxArticulationAxis::Enum axis, PxReal& stiffness, PxReal& damping, PxReal& maxForce, bool& isAcceleration)
	{
		mImpl.getScbArticulationJoint().getDrive(axis, stiffness, damping, maxForce, isAcceleration);
	}
	void NpArticulationJointReducedCoordinate::setDriveTarget(PxArticulationAxis::Enum axis, const PxReal target)
	{
		mImpl.getScbArticulationJoint().setDriveTarget(axis, target);
	}
	void NpArticulationJointReducedCoordinate::setDriveVelocity(PxArticulationAxis::Enum axis, const PxReal targetVel)
	{
		mImpl.getScbArticulationJoint().setDriveVelocity(axis, targetVel);
	}
	PxReal NpArticulationJointReducedCoordinate::getDriveTarget(PxArticulationAxis::Enum axis)
	{
		return mImpl.getScbArticulationJoint().getDriveTarget(axis);
	}
	PxReal NpArticulationJointReducedCoordinate::getDriveVelocity(PxArticulationAxis::Enum axis)
	{
		return mImpl.getScbArticulationJoint().getDriveVelocity(axis);
	}
}
