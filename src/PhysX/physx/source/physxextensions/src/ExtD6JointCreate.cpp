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

#include "PxD6JointCreate.h"
#include "PxD6Joint.h"
#include "PxFixedJoint.h"
#include "PxRevoluteJoint.h"
#include "PxSphericalJoint.h"
#include "PxPrismaticJoint.h"
#include "PxDistanceJoint.h"
#include "PxPhysics.h"
#include "foundation/PxMathUtils.h"

using namespace physx;

static const PxVec3 gX(1.0f, 0.0f, 0.0f);

static void setRotY(PxMat33& m, PxReal angle)
{
	m = PxMat33(PxIdentity);

	const PxReal cos = cosf(angle);
	const PxReal sin = sinf(angle);

	m[0][0] = m[2][2] = cos;
	m[0][2] = -sin;
	m[2][0] = sin;
}

static void setRotZ(PxMat33& m, PxReal angle)
{
	m = PxMat33(PxIdentity);

	const PxReal cos = cosf(angle);
	const PxReal sin = sinf(angle);

	m[0][0] = m[1][1] = cos;
	m[0][1] = sin;
	m[1][0] = -sin;
}

static PxQuat getRotYQuat(float angle)
{
	PxMat33 m;
	setRotY(m, angle);
	return PxQuat(m);
}

static PxQuat getRotZQuat(float angle)
{
	PxMat33 m;
	setRotZ(m, angle);
	return PxQuat(m);
}

PxJoint* physx::PxD6JointCreate_Fixed(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, bool useD6)
{
	const PxTransform jointFrame0(localPos0);
	const PxTransform jointFrame1(localPos1);
	if(useD6)
		// PT: by default all D6 axes are locked, i.e. it is a fixed joint.
		return PxD6JointCreate(physics, actor0, jointFrame0, actor1, jointFrame1);
	else
		return PxFixedJointCreate(physics, actor0, jointFrame0, actor1, jointFrame1);
}

PxJoint* physx::PxD6JointCreate_Distance(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, float maxDist, bool useD6)
{
	const PxTransform localFrame0(localPos0);
	const PxTransform localFrame1(localPos1);

	if(useD6)
	{
		PxD6Joint* j = PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		j->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
		j->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
		j->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
		j->setDistanceLimit(PxJointLinearLimit(PxTolerancesScale(), maxDist));
		return j;
	}
	else
	{
		PxDistanceJoint* j = PxDistanceJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		j->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
		j->setMaxDistance(maxDist);
		return j;
	}
}

PxJoint* physx::PxD6JointCreate_Prismatic(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, const PxVec3& axis, float minLimit, float maxLimit, bool useD6)
{
	const PxQuat q = PxShortestRotation(gX, axis);
	const PxTransform localFrame0(localPos0, q);
	const PxTransform localFrame1(localPos1, q);

	const PxJointLinearLimitPair limit(PxTolerancesScale(), minLimit, maxLimit);

	if(useD6)
	{
		PxD6Joint* j = PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		j->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
		if(minLimit==maxLimit)
			j->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
		else if(minLimit>maxLimit)
			j->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
		else// if(minLimit<maxLimit)
		{
			j->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
			j->setLinearLimit(PxD6Axis::eX, limit);
		}
		return j;
	}
	else
	{
		PxPrismaticJoint* j = PxPrismaticJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		if(minLimit<maxLimit)
		{
			j->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
			j->setLimit(limit);
		}
		return j;
	}
}

PxJoint* physx::PxD6JointCreate_Revolute(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, const PxVec3& axis, float minLimit, float maxLimit, bool useD6)
{
	const PxQuat q = PxShortestRotation(gX, axis);
	const PxTransform localFrame0(localPos0, q);
	const PxTransform localFrame1(localPos1, q);

	const PxJointAngularLimitPair limit(minLimit, maxLimit);

	if(useD6)
	{
		PxD6Joint* j = PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		if(minLimit==maxLimit)
			j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);
		else if(minLimit>maxLimit)
			j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		else// if(minLimit<maxLimit)
		{
			j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
			j->setTwistLimit(limit);
		}
		return j;
	}
	else
	{
		PxRevoluteJoint* j = PxRevoluteJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		if(minLimit<maxLimit)
		{
			j->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
			j->setLimit(limit);
		}
		return j;
	}
}

PxJoint* physx::PxD6JointCreate_Spherical(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, const PxVec3& axis, float limit1, float limit2, bool useD6)
{
	const PxQuat q = PxShortestRotation(gX, axis);
	const PxTransform localFrame0(localPos0, q);
	const PxTransform localFrame1(localPos1, q);

	const PxJointLimitCone limit(limit1, limit2);

	if(useD6)
	{
		PxD6Joint* j = PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		if(limit1>0.0f && limit2>0.0f)
		{
			j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
			j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
			j->setSwingLimit(limit);
		}
		else
		{
			j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
			j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
		}
		return j;
	}
	else
	{
		PxSphericalJoint* j = PxSphericalJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		if(limit1>0.0f && limit2>0.0f)
		{
			j->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
			j->setLimitCone(limit);
		}
		return j;
	}
}

PxJoint* physx::PxD6JointCreate_GenericCone(float& apiroty, float& apirotz, PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, float minLimit1, float maxLimit1, float minLimit2, float maxLimit2, bool useD6)
{
	const float DesiredMinSwingY = minLimit1;
	const float DesiredMaxSwingY = maxLimit1;
	const float DesiredMinSwingZ = minLimit2;
	const float DesiredMaxSwingZ = maxLimit2;
	const float APIMaxY = (DesiredMaxSwingY - DesiredMinSwingY)*0.5f;
	const float APIMaxZ = (DesiredMaxSwingZ - DesiredMinSwingZ)*0.5f;
	const float APIRotY = (DesiredMaxSwingY + DesiredMinSwingY)*0.5f;
	const float APIRotZ = (DesiredMaxSwingZ + DesiredMinSwingZ)*0.5f;
	apiroty = APIRotY;
	apirotz = APIRotZ;

	const PxQuat RotY = getRotYQuat(APIRotY);
	const PxQuat RotZ = getRotZQuat(APIRotZ);
	const PxQuat Rot = RotY * RotZ;

	const PxTransform localFrame0(localPos0, Rot);
	const PxTransform localFrame1(localPos1);

	const PxJointLimitCone limit(APIMaxY, APIMaxZ);

	if(useD6)
	{
		PxD6Joint* j = PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
		j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
		j->setSwingLimit(limit);
		return j;
	}
	else
	{
		PxSphericalJoint* j = PxSphericalJointCreate(physics, actor0, localFrame0, actor1, localFrame1);
		j->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
		j->setLimitCone(limit);
		return j;
	}
}

PxJoint* physx::PxD6JointCreate_Pyramid(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, const PxVec3& axis,
										float minLimit1, float maxLimit1, float minLimit2, float maxLimit2)
{
	const PxQuat q = PxShortestRotation(gX, axis);
	const PxTransform localFrame0(localPos0, q);
	const PxTransform localFrame1(localPos1, q);

	const PxJointLimitPyramid limit(minLimit1, maxLimit1, minLimit2, maxLimit2);

	PxD6Joint* j = PxD6JointCreate(physics, actor0, localFrame0, actor1, localFrame1);
	j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	if(limit.isValid())
	{
		j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
		j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
		j->setPyramidSwingLimit(limit);
	}
	else
	{
		j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	}
	return j;
}
