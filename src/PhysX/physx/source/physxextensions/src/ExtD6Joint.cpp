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

#include "ExtD6Joint.h"
#include "ExtConstraintHelper.h"
#include "CmRenderOutput.h"
#include "CmConeLimitHelper.h"
#include "PxTolerancesScale.h"
#include "CmUtils.h"
#include "PxConstraint.h"

#include "common/PxSerialFramework.h"

using namespace physx;
using namespace Ext;

PxD6Joint* physx::PxD6JointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxD6JointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxD6JointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxD6JointCreate: actors must be different");
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxD6JointCreate: at least one actor must be dynamic");

	D6Joint* j;
	PX_NEW_SERIALIZED(j, D6Joint)(physics.getTolerancesScale(), actor0, localFrame0, actor1, localFrame1);
	if(j->attach(physics, actor0, actor1))
		return j;

	PX_DELETE(j);
	return NULL;
}

D6Joint::D6Joint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
	D6JointT(PxJointConcreteType::eD6, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, actor0, localFrame0, actor1, localFrame1, sizeof(D6JointData), "D6JointData"),
	mRecomputeMotion	(true)
{
	D6JointData* data = static_cast<D6JointData*>(mData);

	for(PxU32 i=0;i<6;i++)
		data->motion[i] = PxD6Motion::eLOCKED;

	data->twistLimit		= PxJointAngularLimitPair(-PxPi/2, PxPi/2);
	data->swingLimit		= PxJointLimitCone(PxPi/2, PxPi/2);
	data->pyramidSwingLimit	= PxJointLimitPyramid(-PxPi/2, PxPi/2, -PxPi/2, PxPi/2);
	data->distanceLimit		= PxJointLinearLimit(scale, PX_MAX_F32);
	data->distanceMinDist	= 1e-6f*scale.length;

	data->linearLimitX		= PxJointLinearLimitPair(scale);
	data->linearLimitY		= PxJointLinearLimitPair(scale);
	data->linearLimitZ		= PxJointLinearLimitPair(scale);

	for(PxU32 i=0;i<PxD6Drive::eCOUNT;i++)
		data->drive[i] = PxD6JointDrive();

	data->drivePosition			= PxTransform(PxIdentity);
	data->driveLinearVelocity	= PxVec3(0.0f);
	data->driveAngularVelocity	= PxVec3(0.0f);

	data->projectionLinearTolerance = 1e10f;
	data->projectionAngularTolerance = PxPi;

	data->mUseDistanceLimit = false;
	data->mUseNewLinearLimits = false;
	data->mUseConeLimit = false;
	data->mUsePyramidLimits = false;
}

PxD6Motion::Enum D6Joint::getMotion(PxD6Axis::Enum index) const
{	
	return data().motion[index];	
}

void D6Joint::setMotion(PxD6Axis::Enum index, PxD6Motion::Enum t)
{	
	data().motion[index] = t; 
	mRecomputeMotion = true; 
	markDirty(); 
}

PxReal D6Joint::getTwistAngle() const
{
	return getTwistAngle_Internal();
}

PxReal D6Joint::getSwingYAngle()	const
{
	return getSwingYAngle_Internal();
}

PxReal D6Joint::getSwingZAngle()	const
{
	return getSwingZAngle_Internal();
}

PxD6JointDrive D6Joint::getDrive(PxD6Drive::Enum index) const
{	
	return data().drive[index];	
}

void D6Joint::setDrive(PxD6Drive::Enum index, const PxD6JointDrive& d)
{	
	PX_CHECK_AND_RETURN(d.isValid(), "PxD6Joint::setDrive: drive is invalid"); 

	data().drive[index] = d; 
	mRecomputeMotion = true; 
	markDirty(); 
}

void D6Joint::setDistanceLimit(const PxJointLinearLimit& l)
{	
	PX_CHECK_AND_RETURN(l.isValid(), "PxD6Joint::setDistanceLimit: limit invalid");
	data().distanceLimit = l;
	data().mUseDistanceLimit = true;
	markDirty(); 
}

PxJointLinearLimit D6Joint::getDistanceLimit() const
{	
	return data().distanceLimit;
}

void D6Joint::setLinearLimit(PxD6Axis::Enum axis, const PxJointLinearLimitPair& limit)
{
	PX_CHECK_AND_RETURN(axis>=PxD6Axis::eX && axis<=PxD6Axis::eZ, "PxD6Joint::setLinearLimit: invalid axis value");
	PX_CHECK_AND_RETURN(limit.isValid(), "PxD6Joint::setLinearLimit: limit invalid");
	D6JointData& d = data();
	if(axis==PxD6Axis::eX)
		d.linearLimitX = limit;
	else if(axis==PxD6Axis::eY)
		d.linearLimitY = limit;
	else if(axis==PxD6Axis::eZ)
		d.linearLimitZ = limit;
	else
		return;
	d.mUseNewLinearLimits = true;
	markDirty(); 
}

PxJointLinearLimitPair D6Joint::getLinearLimit(PxD6Axis::Enum axis) const
{
	PX_CHECK_AND_RETURN_VAL(axis>=PxD6Axis::eX && axis<=PxD6Axis::eZ, "PxD6Joint::getLinearLimit: invalid axis value", PxJointLinearLimitPair(PxTolerancesScale(), 0.0f, 0.0f));
	const D6JointData& d = data();
	if(axis==PxD6Axis::eX)
		return d.linearLimitX;
	else if(axis==PxD6Axis::eY)
		return d.linearLimitY;
	else if(axis==PxD6Axis::eZ)
		return d.linearLimitZ;
	return PxJointLinearLimitPair(PxTolerancesScale(), 0.0f, 0.0f);
}

PxJointAngularLimitPair D6Joint::getTwistLimit() const
{	
	return data().twistLimit;	
}

void D6Joint::setTwistLimit(const PxJointAngularLimitPair& l)
{	
	PX_CHECK_AND_RETURN(l.isValid(), "PxD6Joint::setTwistLimit: limit invalid");
	// PT: the tangent version is not compatible with the double-cover feature, since the potential limit extent in that case is 4*PI.
	// i.e. we'd potentially take the tangent of something equal to PI/2. So the tangent stuff makes the limits less accurate, and it
	// also reduces the available angular range for the joint. All that for questionable performance gains.
	PX_CHECK_AND_RETURN(l.lower>-PxTwoPi && l.upper<PxTwoPi , "PxD6Joint::twist limit must be strictly between -2*PI and 2*PI");

	data().twistLimit = l; 
	markDirty(); 
}

PxJointLimitPyramid D6Joint::getPyramidSwingLimit() const
{	
	return data().pyramidSwingLimit;	
}

void D6Joint::setPyramidSwingLimit(const PxJointLimitPyramid& l)
{	
	PX_CHECK_AND_RETURN(l.isValid(), "PxD6Joint::setPyramidSwingLimit: limit invalid");

	data().pyramidSwingLimit = l; 
	data().mUsePyramidLimits = true;
	markDirty(); 
}

PxJointLimitCone D6Joint::getSwingLimit() const
{	
	return data().swingLimit;	
}

void D6Joint::setSwingLimit(const PxJointLimitCone& l)
{	
	PX_CHECK_AND_RETURN(l.isValid(), "PxD6Joint::setSwingLimit: limit invalid");

	data().swingLimit = l; 
	data().mUseConeLimit = true;
	markDirty(); 
}

PxTransform D6Joint::getDrivePosition() const
{	
	return data().drivePosition;	
}

void D6Joint::setDrivePosition(const PxTransform& pose, bool autowake)
{	
	PX_CHECK_AND_RETURN(pose.isSane(), "PxD6Joint::setDrivePosition: pose invalid");
	data().drivePosition = pose.getNormalized(); 
	if(autowake)
		wakeUpActors();
	markDirty(); 
}

void D6Joint::getDriveVelocity(PxVec3& linear, PxVec3& angular)	const
{	
	linear = data().driveLinearVelocity;
	angular = data().driveAngularVelocity; 
}

void D6Joint::setDriveVelocity(const PxVec3& linear, const PxVec3& angular, bool autowake)
{	
	PX_CHECK_AND_RETURN(linear.isFinite() && angular.isFinite(), "PxD6Joint::setDriveVelocity: velocity invalid");
	data().driveLinearVelocity = linear; 
	data().driveAngularVelocity = angular; 
	if(autowake)
		wakeUpActors();
	markDirty();
}

void D6Joint::setProjectionAngularTolerance(PxReal tolerance)
{	
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance >=0 && tolerance <= PxPi, "PxD6Joint::setProjectionAngularTolerance: tolerance invalid");
	data().projectionAngularTolerance = tolerance;	
	markDirty();
}

PxReal D6Joint::getProjectionAngularTolerance()	const
{	
	return data().projectionAngularTolerance; 
}

void D6Joint::setProjectionLinearTolerance(PxReal tolerance)
{	
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance >=0, "PxD6Joint::setProjectionLinearTolerance: invalid parameter");
	data().projectionLinearTolerance = tolerance;	
	markDirty(); 
}

PxReal D6Joint::getProjectionLinearTolerance() const	
{	
	return data().projectionLinearTolerance;		
}

void* D6Joint::prepareData()
{
	D6JointData& d = data();

	if(mRecomputeMotion)
	{
		mRecomputeMotion = false;

		d.driving = 0;
		d.limited = 0;
		d.locked = 0;

		for(PxU32 i=0;i<PxD6Axis::eCOUNT;i++)
		{
			if(d.motion[i] == PxD6Motion::eLIMITED)
				d.limited |= 1<<i;
			else if(d.motion[i] == PxD6Motion::eLOCKED)
				d.locked |= 1<<i;
		}

		// a linear direction isn't driven if it's locked
		if(active(PxD6Drive::eX) && d.motion[PxD6Axis::eX]!=PxD6Motion::eLOCKED) d.driving |= 1<< PxD6Drive::eX;
		if(active(PxD6Drive::eY) && d.motion[PxD6Axis::eY]!=PxD6Motion::eLOCKED) d.driving |= 1<< PxD6Drive::eY;
		if(active(PxD6Drive::eZ) && d.motion[PxD6Axis::eZ]!=PxD6Motion::eLOCKED) d.driving |= 1<< PxD6Drive::eZ;

		// SLERP drive requires all angular dofs unlocked, and inhibits swing/twist

		const bool swing1Locked = d.motion[PxD6Axis::eSWING1] == PxD6Motion::eLOCKED;
		const bool swing2Locked = d.motion[PxD6Axis::eSWING2] == PxD6Motion::eLOCKED;
		const bool twistLocked  = d.motion[PxD6Axis::eTWIST]  == PxD6Motion::eLOCKED;

		if(active(PxD6Drive::eSLERP) && !swing1Locked && !swing2Locked && !twistLocked)
			d.driving |= 1<<PxD6Drive::eSLERP;
		else
		{
			if(active(PxD6Drive::eTWIST) && !twistLocked) 
				d.driving |= 1<<PxD6Drive::eTWIST;
			if(active(PxD6Drive::eSWING) && (!swing1Locked || !swing2Locked)) 
				d.driving |= 1<< PxD6Drive::eSWING;
		}
	}

	this->D6JointT::prepareData();

	return mData;
}

bool D6Joint::attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1)
{
	mPxConstraint = physics.createConstraint(actor0, actor1, *this, sShaders, sizeof(D6JointData));
	return mPxConstraint!=NULL;
}

void D6Joint::exportExtraData(PxSerializationContext& stream)
{
	if(mData)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mData, sizeof(D6JointData));
	}
	stream.writeName(mName);
}

void D6Joint::importExtraData(PxDeserializationContext& context)
{
	if(mData)
		mData = context.readExtraData<D6JointData, PX_SERIAL_ALIGN>();

	context.readName(mName);
}

void D6Joint::resolveReferences(PxDeserializationContext& context)
{
	setPxConstraint(resolveConstraintPtr(context, getPxConstraint(), getConnector(), sShaders));	
}

D6Joint* D6Joint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	D6Joint* obj = new (address) D6Joint(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(D6Joint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

// global function to share the joint shaders with API capture	
const PxConstraintShaderTable* Ext::GetD6JointShaderTable() 
{ 
	return &D6Joint::getConstraintShaderTable();
}

//~PX_SERIALIZATION

// Notes:
/*

This used to be in the linear drive model:

	if(motion[PxD6Axis::eX+i] == PxD6Motion::eLIMITED)
	{
		if(data.driveLinearVelocity[i] < 0.0f && cB2cA.p[i] < -mLimits[PxD6Limit::eLINEAR].mValue ||
			data.driveLinearVelocity[i] > 0.0f && cB2cA.p[i] > mLimits[PxD6Limit::eLINEAR].mValue)
			continue;
	}

it doesn't seem like a good idea though, because it turns off drive altogether, despite the fact that positional
drive might pull us back in towards the limit. Might be better to make the drive unilateral so it can only pull
us in from the limit

This used to be in angular locked:

	// Angular locked
	//TODO fix this properly. 	
	if(PxAbs(cB2cA.q.x) < 0.0001f) cB2cA.q.x = 0;
	if(PxAbs(cB2cA.q.y) < 0.0001f) cB2cA.q.y = 0;
	if(PxAbs(cB2cA.q.z) < 0.0001f) cB2cA.q.z = 0;
	if(PxAbs(cB2cA.q.w) < 0.0001f) cB2cA.q.w = 0;
*/

static PxQuat truncate(const PxQuat& qIn, PxReal minCosHalfTol, bool& truncated)
{
	const PxQuat q = qIn.w >= 0.0f ? qIn : -qIn;
	truncated = q.w < minCosHalfTol;
	if(!truncated)
		return q;
	const PxVec3 v = q.getImaginaryPart().getNormalized() * PxSqrt(1.0f - minCosHalfTol * minCosHalfTol);
	return PxQuat(v.x, v.y, v.z, minCosHalfTol);
}

// we decompose the quaternion as q1 * q2, where q1 is a rotation orthogonal to the unit axis, and q2 a rotation around it.
// (so for example if 'axis' is the twist axis, this is separateSwingTwist).
static PxQuat project(const PxQuat& q, const PxVec3& axis, PxReal cosHalfTol, bool& truncated)
{
	const PxReal a = q.getImaginaryPart().dot(axis);
	const PxQuat q2 = PxAbs(a) >= 1e-6f ? PxQuat(a*axis.x, a*axis.y, a*axis.z, q.w).getNormalized() : PxQuat(PxIdentity);
	const PxQuat q1 = q * q2.getConjugate();

	PX_ASSERT(PxAbs(q1.getImaginaryPart().dot(q2.getImaginaryPart())) < 1e-6f);

	return truncate(q1, cosHalfTol, truncated) * q2;
}

// Here's how the angular part works:
// * if no DOFs are locked, there's nothing to do.
// * if all DOFs are locked, we just truncate the rotation
// * if two DOFs are locked
//  * we decompose the rotation into swing * twist, where twist is a rotation around the free DOF and swing is a rotation around an axis orthogonal to the free DOF
//  * then we truncate swing
// The case of one locked DOF is currently unimplemented, but one option would be:
// * if one DOF is locked (the tricky case), we define the 'free' axis as follows (as the velocity solver prep function does)
// TWIST: cB[0]
// SWING1: cB[0].cross(cA[2])
// SWING2: cB[0].cross(cA[1])
// then, as above, we decompose into swing * free, and truncate the free rotation

//export this in the physx namespace so we can unit test it
namespace physx
{
PxQuat angularProject(PxU32 lockedDofs, const PxQuat& q, PxReal cosHalfTol, bool& truncated)
{
	PX_ASSERT(lockedDofs <= 7);
	truncated = false;

	switch(lockedDofs)
	{
	case 0: return q;
	case 1: return q;		// currently unimplemented
	case 2: return q;		// currently unimplemented
	case 3: return project(q, PxVec3(0.0f, 0.0f, 1.0f), cosHalfTol, truncated);
	case 4: return q;		// currently unimplemented
	case 5: return project(q, PxVec3(0.0f, 1.0f, 0.0f), cosHalfTol, truncated);
	case 6: return project(q, PxVec3(1.0f, 0.0f, 0.0f), cosHalfTol, truncated);
	case 7: return truncate(q, cosHalfTol, truncated);
	default: return PxQuat(PxIdentity);
	}
}
}

static void D6JointProject(const void* constantBlock, PxTransform& bodyAToWorld, PxTransform& bodyBToWorld, bool projectToA)
{
	const D6JointData& data = *reinterpret_cast<const D6JointData*>(constantBlock);

	PxTransform cA2w, cB2w, cB2cA, projected;
	joint::computeDerived(data, bodyAToWorld, bodyBToWorld, cA2w, cB2w, cB2cA, false);

	const PxVec3 v(data.locked & 1 ? cB2cA.p.x : 0.0f,
		data.locked & 2 ? cB2cA.p.y : 0.0f,
		data.locked & 4 ? cB2cA.p.z : 0.0f);

	bool linearTrunc, angularTrunc = false;
	projected.p = joint::truncateLinear(v, data.projectionLinearTolerance, linearTrunc) + (cB2cA.p - v);

	projected.q = angularProject(data.locked >> 3, cB2cA.q, PxCos(data.projectionAngularTolerance / 2), angularTrunc);

	if(linearTrunc || angularTrunc)
		joint::projectTransforms(bodyAToWorld, bodyBToWorld, cA2w, cB2w, projected, data, projectToA);
}

static PX_FORCE_INLINE PxReal computePhi(const PxQuat& q)
{
	PxQuat twist = q;
	twist.normalize();

	PxReal angle = twist.getAngle();
	if(twist.x<0.0f)
		angle = -angle;
	return angle;
}

static void visualizeAngularLimit(PxConstraintVisualizer& viz, const D6JointData& data, const PxTransform& t, float swingYZ, float swingW, float swingLimitYZ)
{
	bool active = PxAbs(computeSwingAngle(swingYZ, swingW)) > swingLimitYZ - data.swingLimit.contactDistance;					
	viz.visualizeAngularLimit(t, -swingLimitYZ, swingLimitYZ, active);
}

static void visualizeDoubleCone(PxConstraintVisualizer& viz, const D6JointData& data, const PxTransform& t, float sin, float swingLimitYZ)
{
	const PxReal angle = PxAsin(sin);
	const PxReal pad = data.swingLimit.contactDistance;
	const PxReal low = -swingLimitYZ;
	const PxReal high = swingLimitYZ;

	const bool active = isLimitActive(data.swingLimit, pad, angle, low, high);
	viz.visualizeDoubleCone(t, swingLimitYZ, active);
}

// PT: TODO: refactor with spherical joint code
static void visualizeCone(PxConstraintVisualizer& viz, const D6JointData& data, const PxQuat& swing, const PxTransform& cA2w)
{
	const PxVec3 swingAngle(0.0f, computeSwingAngle(swing.y, swing.w), computeSwingAngle(swing.z, swing.w));
	const PxReal pad = data.swingLimit.isSoft() ? 0.0f : data.swingLimit.contactDistance;
	Cm::ConeLimitHelperTanLess coneHelper(data.swingLimit.yAngle, data.swingLimit.zAngle, pad);
	viz.visualizeLimitCone(cA2w, PxTan(data.swingLimit.zAngle / 4), PxTan(data.swingLimit.yAngle / 4), !coneHelper.contains(swingAngle));
}

static PX_FORCE_INLINE bool isLinearLimitActive(const PxJointLinearLimitPair& limit, float ordinate)
{
	const PxReal pad = limit.isSoft() ? 0.0f : limit.contactDistance;
	return (ordinate < limit.lower + pad) || (ordinate > limit.upper - pad);
}

static void visualizeLine(PxConstraintVisualizer& viz, const PxVec3& origin, const PxVec3& axis, const PxJointLinearLimitPair& limit, float ordinate)
{
	const bool active = isLinearLimitActive(limit, ordinate);
	const PxVec3 p0 = origin + axis * limit.lower;
	const PxVec3 p1 = origin + axis * limit.upper;
	viz.visualizeLine(p0, p1, active ? 0xff0000u : 0xffffffu);
}

static void visualizeQuad(PxConstraintVisualizer& viz, const PxVec3& origin,	const PxVec3& axis0, const PxJointLinearLimitPair& limit0, float ordinate0,
																				const PxVec3& axis1, const PxJointLinearLimitPair& limit1, float ordinate1)
{
	const bool active0 = isLinearLimitActive(limit0, ordinate0);
	const bool active1 = isLinearLimitActive(limit1, ordinate1);
	const PxU32 color = (active0 || active1) ? 0xff0000u : 0xffffffu;

	const PxVec3 l0 = axis0 * limit0.lower;
	const PxVec3 u0 = axis0 * limit0.upper;
	const PxVec3 l1 = axis1 * limit1.lower;
	const PxVec3 u1 = axis1 * limit1.upper;

	const PxVec3 p0 = origin + l0 + l1;
	const PxVec3 p1 = origin + u0 + l1;
	const PxVec3 p2 = origin + u0 + u1;
	const PxVec3 p3 = origin + l0 + u1;

	viz.visualizeLine(p0, p1, color);
	viz.visualizeLine(p1, p2, color);
	viz.visualizeLine(p2, p3, color);
	viz.visualizeLine(p3, p0, color);
}

static void visualizeBox(PxConstraintVisualizer& viz, const PxVec3& origin,	const PxVec3& axis0, const PxJointLinearLimitPair& limit0, float ordinate0,
																			const PxVec3& axis1, const PxJointLinearLimitPair& limit1, float ordinate1,
																			const PxVec3& axis2, const PxJointLinearLimitPair& limit2, float ordinate2)
{
	const bool active0 = isLinearLimitActive(limit0, ordinate0);
	const bool active1 = isLinearLimitActive(limit1, ordinate1);
	const bool active2 = isLinearLimitActive(limit2, ordinate2);
	const PxU32 color = (active0 || active1 || active2) ? 0xff0000u : 0xffffffu;

	const PxVec3 l0 = axis0 * limit0.lower;
	const PxVec3 u0 = axis0 * limit0.upper;
	const PxVec3 l1 = axis1 * limit1.lower;
	const PxVec3 u1 = axis1 * limit1.upper;
	const PxVec3 l2 = axis2 * limit2.lower;
	const PxVec3 u2 = axis2 * limit2.upper;

	const PxVec3 p0 = origin + l0 + l1 + l2;
	const PxVec3 p1 = origin + u0 + l1 + l2;
	const PxVec3 p2 = origin + u0 + u1 + l2;
	const PxVec3 p3 = origin + l0 + u1 + l2;
	const PxVec3 p0b = origin + l0 + l1 + u2;
	const PxVec3 p1b = origin + u0 + l1 + u2;
	const PxVec3 p2b = origin + u0 + u1 + u2;
	const PxVec3 p3b = origin + l0 + u1 + u2;

	viz.visualizeLine(p0, p1, color);
	viz.visualizeLine(p1, p2, color);
	viz.visualizeLine(p2, p3, color);
	viz.visualizeLine(p3, p0, color);
	viz.visualizeLine(p0b, p1b, color);
	viz.visualizeLine(p1b, p2b, color);
	viz.visualizeLine(p2b, p3b, color);
	viz.visualizeLine(p3b, p0b, color);
	viz.visualizeLine(p0, p0b, color);
	viz.visualizeLine(p1, p1b, color);
	viz.visualizeLine(p2, p2b, color);
	viz.visualizeLine(p3, p3b, color);
}

static float computeLimitedDistance(const D6JointData& data, const PxTransform& cB2cA, const PxMat33& cA2w_m, PxVec3& _limitDir)
{
	PxVec3 limitDir(0.0f);

	for(PxU32 i = 0; i<3; i++)
	{
		if(data.limited & (1 << (PxD6Axis::eX + i)))
			limitDir += cA2w_m[i] * cB2cA.p[i];
	}

	_limitDir = limitDir;
	return limitDir.magnitude();
}

void _setRotY(PxMat33& m, PxReal angle)
{
	m = PxMat33(PxIdentity);

	const PxReal cos = cosf(angle);
	const PxReal sin = sinf(angle);

	m[0][0] = m[2][2] = cos;
	m[0][2] = -sin;
	m[2][0] = sin;
}

void _setRotZ(PxMat33& m, PxReal angle)
{
	m = PxMat33(PxIdentity);

	const PxReal cos = cosf(angle);
	const PxReal sin = sinf(angle);

	m[0][0] = m[1][1] = cos;
	m[0][1] = sin;
	m[1][0] = -sin;
}

PxQuat _getRotYQuat(float angle)
{
	PxMat33 m;
	_setRotY(m, angle);
	return PxQuat(m);
}

PxQuat _getRotZQuat(float angle)
{
	PxMat33 m;
	_setRotZ(m, angle);
	return PxQuat(m);
}

void _setRotX(PxMat33& m, PxReal angle)
{
	m = PxMat33(PxIdentity);

	const PxReal cos = cosf(angle);
	const PxReal sin = sinf(angle);

	m[1][1] = m[2][2] = cos;
	m[1][2] = sin;
	m[2][1] = -sin;
}

PxQuat _getRotXQuat(float angle)
{
	PxMat33 m;
	_setRotX(m, angle);
	return PxQuat(m);
}

static void drawPyramid(PxConstraintVisualizer& viz, const D6JointData& data, const PxTransform& cA2w, const PxQuat& swing, bool useY, bool useZ)
{
	struct Local
	{
		static void drawArc(PxConstraintVisualizer& _viz, const PxTransform& _cA2w, float ymin, float ymax, float zmin, float zmax, PxU32 color)
		{
			// PT: we use 32 segments for the cone case, so that's 32/4 segments per arc in the pyramid case
			const PxU32 nb = 32/4;
			PxVec3 prev(0.0f);
			for(PxU32 i=0;i<nb;i++)
			{
				const float coeff = float(i)/float(nb-1);
				const float y = coeff*ymax + (1.0f-coeff)*ymin;
				const float z = coeff*zmax + (1.0f-coeff)*zmin;

				const float r = 1.0f;
				PxMat33 my;	_setRotZ(my, z);
				PxMat33 mz;	_setRotY(mz, y);
				const PxVec3 p0 = (my*mz).transform(PxVec3(r, 0.0f, 0.0f));
				const PxVec3 p0w = _cA2w.transform(p0);
				_viz.visualizeLine(_cA2w.p, p0w, color);
				if(i)
					_viz.visualizeLine(prev, p0w, color);
				prev = p0w;
			}
		}
	};

	const PxJointLimitPyramid& l = data.pyramidSwingLimit;
	const bool activeY = useY ? isLimitActive(l, l.contactDistance, computeSwingAngle(swing.y, swing.w), l.yAngleMin, l.yAngleMax) : false;
	const bool activeZ = useZ ? isLimitActive(l, l.contactDistance, computeSwingAngle(swing.z, swing.w), l.zAngleMin, l.zAngleMax) : false;
	const PxU32 color = (activeY||activeZ) ? PxDebugColor::eARGB_RED : PxDebugColor::eARGB_GREY;

	Local::drawArc(viz, cA2w, l.yAngleMin, l.yAngleMin, l.zAngleMin, l.zAngleMax, color);
	Local::drawArc(viz, cA2w, l.yAngleMax, l.yAngleMax, l.zAngleMin, l.zAngleMax, color);
	Local::drawArc(viz, cA2w, l.yAngleMin, l.yAngleMax, l.zAngleMin, l.zAngleMin, color);
	Local::drawArc(viz, cA2w, l.yAngleMin, l.yAngleMax, l.zAngleMax, l.zAngleMax, color);
}

static void D6JointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	const PxU32 SWING1_FLAG = 1<<PxD6Axis::eSWING1, 
			    SWING2_FLAG = 1<<PxD6Axis::eSWING2, 
				TWIST_FLAG  = 1<<PxD6Axis::eTWIST;

	const PxU32 ANGULAR_MASK = SWING1_FLAG | SWING2_FLAG | TWIST_FLAG;
	const PxU32 LINEAR_MASK = 1<<PxD6Axis::eX | 1<<PxD6Axis::eY | 1<<PxD6Axis::eZ;

	PX_UNUSED(ANGULAR_MASK);
	PX_UNUSED(LINEAR_MASK);

	const D6JointData& data = *reinterpret_cast<const D6JointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
		viz.visualizeJointFrames(cA2w, cB2w);

	if(flags & PxConstraintVisualizationFlag::eLIMITS)
	{
	// PT: it is a mistake to use the neighborhood operator since it
	// prevents us from using the quat's double-cover feature.
//		if(cA2w.q.dot(cB2w.q)<0.0f)
//			cB2w.q = -cB2w.q;

		const PxTransform cB2cA = cA2w.transformInv(cB2w);	
		const PxMat33 cA2w_m(cA2w.q), cB2w_m(cB2w.q);

		if(data.mUseNewLinearLimits)
		{
			switch(data.limited)
			{
				case 1<<PxD6Axis::eX:
					visualizeLine(viz, cA2w.p, cA2w_m.column0, data.linearLimitX, cB2cA.p.x);
				break;
				case 1<<PxD6Axis::eY:
					visualizeLine(viz, cA2w.p, cA2w_m.column1, data.linearLimitY, cB2cA.p.y);
				break;
				case 1<<PxD6Axis::eZ:
					visualizeLine(viz, cA2w.p, cA2w_m.column2, data.linearLimitZ, cB2cA.p.z);
				break;
				case 1<<PxD6Axis::eX|1<<PxD6Axis::eY:
					visualizeQuad(viz, cA2w.p, cA2w_m.column0, data.linearLimitX, cB2cA.p.x, cA2w_m.column1, data.linearLimitY, cB2cA.p.y);
				break;
				case 1<<PxD6Axis::eX|1<<PxD6Axis::eZ:
					visualizeQuad(viz, cA2w.p, cA2w_m.column0, data.linearLimitX, cB2cA.p.x, cA2w_m.column2, data.linearLimitZ, cB2cA.p.z);
				break;
				case 1<<PxD6Axis::eY|1<<PxD6Axis::eZ:
					visualizeQuad(viz, cA2w.p, cA2w_m.column1, data.linearLimitY, cB2cA.p.y, cA2w_m.column2, data.linearLimitZ, cB2cA.p.z);
				break;
				case 1<<PxD6Axis::eX|1<<PxD6Axis::eY|1<<PxD6Axis::eZ:
					visualizeBox(viz, cA2w.p, cA2w_m.column0, data.linearLimitX, cB2cA.p.x, cA2w_m.column1, data.linearLimitY, cB2cA.p.y, cA2w_m.column2, data.linearLimitZ, cB2cA.p.z);
				break;
			}
		}

		if(data.mUseDistanceLimit)	// PT: old linear/distance limit
		{
			PxVec3 limitDir;

			const float distance = computeLimitedDistance(data, cB2cA, cA2w_m, limitDir);

			// visualise only if some of the axis is limited
			if(distance > data.distanceMinDist)
			{
				PxU32 color = 0x00ff00;
				if(distance>data.distanceLimit.value)
					color = 0xff0000;

				viz.visualizeLine(cA2w.p, cB2w.p, color);
			}
		}

		PxQuat swing, twist;
		Ps::separateSwingTwist(cB2cA.q, swing, twist);

		const PxVec3& bX = cB2w_m.column0;
		const PxVec3& aY = cA2w_m.column1;
		const PxVec3& aZ = cA2w_m.column2;

		if(data.limited&TWIST_FLAG)
		{
			const PxReal angle = computePhi(twist);
			const PxReal pad = data.twistLimit.contactDistance;
			const PxReal low = data.twistLimit.lower;
			const PxReal high = data.twistLimit.upper;

			const bool active = isLimitActive(data.twistLimit, pad, angle, low, high);
			viz.visualizeAngularLimit(cA2w, data.twistLimit.lower, data.twistLimit.upper, active);
		}

		const bool swing1Limited = (data.limited & SWING1_FLAG)!=0, swing2Limited = (data.limited & SWING2_FLAG)!=0;

		if(swing1Limited && swing2Limited)
		{
			if(data.mUseConeLimit)
				visualizeCone(viz, data, swing, cA2w);

			if(data.mUsePyramidLimits)
				drawPyramid(viz, data, cA2w, swing, true, true);
		}
		else if(swing1Limited ^ swing2Limited)
		{
			const PxTransform yToX(PxVec3(0.0f), PxQuat(-PxPi/2.0f, PxVec3(0.0f, 0.0f, 1.0f)));
			const PxTransform zToX(PxVec3(0.0f), PxQuat(PxPi/2.0f, PxVec3(0.0f, 1.0f, 0.0f)));

			if(swing1Limited)
			{
				if(data.locked & SWING2_FLAG)
				{
					if(data.mUsePyramidLimits)
						drawPyramid(viz, data, cA2w, swing, true, false);
					else
						visualizeAngularLimit(viz, data, cA2w * yToX, swing.y, swing.w, data.swingLimit.yAngle);	// PT: swing Y limited, swing Z locked
				}
				else
					if(!data.mUsePyramidLimits)
						visualizeDoubleCone(viz, data, cA2w * zToX, aZ.dot(bX), data.swingLimit.yAngle);			// PT: swing Y limited, swing Z free
			}
			else 
			{
				if(data.locked & SWING1_FLAG)
				{
					if(data.mUsePyramidLimits)
						drawPyramid(viz, data, cA2w, swing, false, true);
					else
						visualizeAngularLimit(viz, data, cA2w * zToX, swing.z, swing.w, data.swingLimit.zAngle);	// PT: swing Z limited, swing Y locked
				}
				else
					if(!data.mUsePyramidLimits)
						visualizeDoubleCone(viz, data, cA2w * yToX, aY.dot(bX), data.swingLimit.zAngle);			// PT: swing Z limited, swing Y free
			}
		}
	}
}

static PX_FORCE_INLINE void setupSingleSwingLimit(joint::ConstraintHelper& ch, const D6JointData& data, const PxVec3& axis, float swingYZ, float swingW, float swingLimitYZ)
{
	ch.anglePair(computeSwingAngle(swingYZ, swingW), -swingLimitYZ, swingLimitYZ, data.swingLimit.contactDistance, axis, data.swingLimit);
}

static PX_FORCE_INLINE void setupDualConeSwingLimits(joint::ConstraintHelper& ch, const D6JointData& data, const PxVec3& axis, float sin, float swingLimitYZ)
{
	ch.anglePair(PxAsin(sin), -swingLimitYZ, swingLimitYZ, data.swingLimit.contactDistance, axis.getNormalized(), data.swingLimit);
}

// PT: TODO: refactor with spherical joint code
static void setupConeSwingLimits(joint::ConstraintHelper& ch, const D6JointData& data, const PxQuat& swing, const PxTransform& cA2w)
{
	PxVec3 axis;
	PxReal error;
	const PxReal pad = data.swingLimit.isSoft() ? 0.0f : data.swingLimit.contactDistance;
	const Cm::ConeLimitHelperTanLess coneHelper(data.swingLimit.yAngle, data.swingLimit.zAngle, pad);
	bool active = coneHelper.getLimit(swing, axis, error);
	if(active)
		ch.angularLimit(cA2w.rotate(axis), error, data.swingLimit);
}

static void setupPyramidSwingLimits(joint::ConstraintHelper& ch, const D6JointData& data, const PxQuat& swing, const PxTransform& cA2w, bool useY, bool useZ)
{
	const PxQuat q = cA2w.q * swing;
	const PxJointLimitPyramid& l = data.pyramidSwingLimit;
	if(useY)
		ch.anglePair(computeSwingAngle(swing.y, swing.w), l.yAngleMin, l.yAngleMax, l.contactDistance, q.getBasisVector1(), l);
	if(useZ)
		ch.anglePair(computeSwingAngle(swing.z, swing.w), l.zAngleMin, l.zAngleMax, l.contactDistance, q.getBasisVector2(), l);
}

static void setupLinearLimit(joint::ConstraintHelper& ch, const PxJointLinearLimitPair& limit, const float origin, const PxVec3& axis)
{
	ch.linearLimit(axis, origin, limit.upper, limit);
	ch.linearLimit(-axis, -origin, -limit.lower, limit);
}

static PxU32 D6JointSolverPrep(Px1DConstraint* constraints,
	PxVec3& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool useExtendedLimits,
	PxVec3& cA2wOut, PxVec3& cB2wOut)
{
	const D6JointData& data = *reinterpret_cast<const D6JointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	const PxU32 SWING1_FLAG = 1<<PxD6Axis::eSWING1;
	const PxU32 SWING2_FLAG = 1<<PxD6Axis::eSWING2;
	const PxU32 TWIST_FLAG = 1<<PxD6Axis::eTWIST;

	const PxU32 ANGULAR_MASK = SWING1_FLAG | SWING2_FLAG | TWIST_FLAG;
	const PxU32 LINEAR_MASK = 1<<PxD6Axis::eX | 1<<PxD6Axis::eY | 1<<PxD6Axis::eZ;

	const PxD6JointDrive* drives = data.drive;
	PxU32 locked = data.locked;
	const PxU32 limited = data.limited;
	const PxU32 driving = data.driving;

	// PT: it is a mistake to use the neighborhood operator since it
	// prevents us from using the quat's double-cover feature.
	if(!useExtendedLimits && cA2w.q.dot(cB2w.q)<0.0f)	// minimum dist quat (equiv to flipping cB2bB.q, which we don't use anywhere)
		cB2w.q = -cB2w.q;

	const PxTransform cB2cA = cA2w.transformInv(cB2w);	

	PX_ASSERT(data.c2b[0].isValid());
	PX_ASSERT(data.c2b[1].isValid());
	PX_ASSERT(cA2w.isValid());
	PX_ASSERT(cB2w.isValid());
	PX_ASSERT(cB2cA.isValid());

	const PxMat33 cA2w_m(cA2w.q);
	const PxMat33 cB2w_m(cB2w.q);

	// handy for swing computation
	const PxVec3& bX = cB2w_m.column0;
	const PxVec3& aY = cA2w_m.column1;
	const PxVec3& aZ = cA2w_m.column2;

	if(driving & ((1<<PxD6Drive::eX)|(1<<PxD6Drive::eY)|(1<<PxD6Drive::eZ)))
	{
		// TODO: make drive unilateral if we are outside the limit
		const PxVec3 posErr = data.drivePosition.p - cB2cA.p;
		for(PxU32 i=0; i<3; i++)
		{
			// -driveVelocity because velTarget is child (body1) - parent (body0) and Jacobian is 1 for body0 and -1 for parent
			if(driving & (1<<(PxD6Drive::eX+i)))
				ch.linear(cA2w_m[i], -data.driveLinearVelocity[i], posErr[i], drives[PxD6Drive::eX+i]); 
		}
	}

	if(driving & ((1<<PxD6Drive::eSLERP)|(1<<PxD6Drive::eSWING)|(1<<PxD6Drive::eTWIST)))
	{
		const PxQuat d2cA_q = cB2cA.q.dot(data.drivePosition.q)>0.0f ? data.drivePosition.q : -data.drivePosition.q; 

		const PxVec3& v = data.driveAngularVelocity;
		const PxQuat delta = d2cA_q.getConjugate() * cB2cA.q;

		if(driving & (1<<PxD6Drive::eSLERP))
		{
			const PxVec3 velTarget = -cA2w.rotate(data.driveAngularVelocity);

			PxVec3 axis[3] = { PxVec3(1.0f, 0.0f, 0.0f), PxVec3(0.0f, 1.0f, 0.0f), PxVec3(0.0f, 0.0f, 1.0f) };
				
			if(drives[PxD6Drive::eSLERP].stiffness!=0.0f)
				joint::computeJacobianAxes(axis, cA2w.q * d2cA_q, cB2w.q);	// converges faster if there is only velocity drive

			for(PxU32 i=0; i<3; i++)
				ch.angular(axis[i], axis[i].dot(velTarget), -delta.getImaginaryPart()[i], drives[PxD6Drive::eSLERP], PxConstraintSolveHint::eSLERP_SPRING);
		}
		else 
		{
			if(driving & (1<<PxD6Drive::eTWIST))
				ch.angular(bX, v.x, -2.0f * delta.x, drives[PxD6Drive::eTWIST]); 

			if(driving & (1<<PxD6Drive::eSWING))
			{
				const PxVec3 err = delta.rotate(PxVec3(1.0f, 0.0f, 0.0f));

				if(!(locked & SWING1_FLAG))
					ch.angular(cB2w_m[1], v.y, err.z, drives[PxD6Drive::eSWING]);

				if(!(locked & SWING2_FLAG))
					ch.angular(cB2w_m[2], v.z, -err.y, drives[PxD6Drive::eSWING]);
			}
		}
	}

	if(limited & ANGULAR_MASK)
	{
		PxQuat swing, twist;
		Ps::separateSwingTwist(cB2cA.q, swing, twist);

		// swing limits: if just one is limited: if the other is free, we support 
		// (-pi/2, +pi/2) limit, using tan of the half-angle as the error measure parameter. 
		// If the other is locked, we support (-pi, +pi) limits using the tan of the quarter-angle
		// Notation: th == Ps::tanHalf, tq = tanQuarter

		if(limited & SWING1_FLAG && limited & SWING2_FLAG)
		{
			if(data.mUseConeLimit)
				setupConeSwingLimits(ch, data, swing, cA2w);
			// PT: no else here by design, we want to allow creating both the cone & the pyramid at the same time,
			// which can be useful to make the cone more robust against large velocities.
			if(data.mUsePyramidLimits)
				setupPyramidSwingLimits(ch, data, swing, cA2w, true, true);
		}
		else
		{
			if(limited & SWING1_FLAG)
			{
				if(locked & SWING2_FLAG)
				{
					if(data.mUsePyramidLimits)
						setupPyramidSwingLimits(ch, data, swing, cA2w, true, false);
					else
						setupSingleSwingLimit(ch, data, aY, swing.y, swing.w, data.swingLimit.yAngle);			// PT: swing Y limited, swing Z locked
				}
				else
				{
					if(!data.mUsePyramidLimits)
						setupDualConeSwingLimits(ch, data, aZ.cross(bX), -aZ.dot(bX), data.swingLimit.yAngle);	// PT: swing Y limited, swing Z free
					else
						Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "D6JointSolverPrep: invalid joint setup. Double pyramid mode not supported.");
				}
			}
			if(limited & SWING2_FLAG)
			{
				if(locked & SWING1_FLAG)
				{
					if(data.mUsePyramidLimits)
						setupPyramidSwingLimits(ch, data, swing, cA2w, false, true);
					else
						setupSingleSwingLimit(ch, data, aZ, swing.z, swing.w, data.swingLimit.zAngle);			// PT: swing Z limited, swing Y locked
				}
				else
					if(!data.mUsePyramidLimits)
						setupDualConeSwingLimits(ch, data, -aY.cross(bX), aY.dot(bX), data.swingLimit.zAngle);	// PT: swing Z limited, swing Y free
					else
						Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "D6JointSolverPrep: invalid joint setup. Double pyramid mode not supported.");
			}
		}

		if(limited & TWIST_FLAG)
		{
			ch.anglePair(computePhi(twist), data.twistLimit.lower, data.twistLimit.upper, data.twistLimit.contactDistance, cB2w_m.column0, data.twistLimit);
		}
	}

	if(limited & LINEAR_MASK)
	{
		if(data.mUseDistanceLimit)	// PT: old linear/distance limit
		{
			PxVec3 limitDir;

			const float distance = computeLimitedDistance(data, cB2cA, cA2w_m, limitDir);
			if(distance > data.distanceMinDist)
				ch.linearLimit(limitDir * (1.0f/distance), distance, data.distanceLimit.value, data.distanceLimit);
		}

		if(data.mUseNewLinearLimits)	// PT: new asymmetric linear limits
		{
			const PxVec3& bOriginInA = cB2cA.p;

			// PT: TODO: we check that the DOFs are not "locked" to be consistent with the prismatic joint, but it
			// doesn't look like this case is possible, since it would be caught by the "isValid" check when setting
			// the limits. And in fact the "distance" linear limit above doesn't do this check.
			if((limited & (1<<PxD6Axis::eX)) && data.linearLimitX.lower <= data.linearLimitX.upper)
				setupLinearLimit(ch, data.linearLimitX, bOriginInA.x, cA2w_m.column0);

			if((limited & (1<<PxD6Axis::eY)) && data.linearLimitY.lower <= data.linearLimitY.upper)
				setupLinearLimit(ch, data.linearLimitY, bOriginInA.y, cA2w_m.column1);

			if((limited & (1<<PxD6Axis::eZ)) && data.linearLimitZ.lower <= data.linearLimitZ.upper)
				setupLinearLimit(ch, data.linearLimitZ, bOriginInA.z, cA2w_m.column2);
		}
	}

	// we handle specially the case of just one swing dof locked

	const PxU32 angularLocked = locked & ANGULAR_MASK;

	if(angularLocked == SWING1_FLAG)
	{
		ch.angularHard(bX.cross(aZ), -bX.dot(aZ));
		locked &= ~SWING1_FLAG;
	}
	else if(angularLocked == SWING2_FLAG)
	{
		locked &= ~SWING2_FLAG;
		ch.angularHard(bX.cross(aY), -bX.dot(aY));
	}
	
	PxVec3 ra, rb;

	ch.prepareLockedAxes(cA2w.q, cB2w.q, cB2cA.p, locked&7, locked>>3, ra, rb);

	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;

	/*cA2wOut = cA2w.p;
	cB2wOut = cB2w.p;*/

	// PT: TODO: check the number cannot be too high now
	return ch.getCount();
}

PxConstraintShaderTable Ext::D6Joint::sShaders = { D6JointSolverPrep, D6JointProject, D6JointVisualize, /*PxConstraintFlag::Enum(0)*/PxConstraintFlag::eGPU_COMPATIBLE };
