/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_RIGID_STATE_H
#define _SCE_PFX_RIGID_STATE_H

#include "../base/pfx_common.h"
#include "../base/pfx_vec_utils.h"

namespace sce {
namespace PhysicsEffects {

// Motion Type
enum ePfxMotionType {
	kPfxMotionTypeFixed = 0,
	kPfxMotionTypeActive,
	kPfxMotionTypeKeyframe,
	kPfxMotionTypeOneWay,
	kPfxMotionTypeTrigger,
	kPfxMotionTypeCount
};

#define SCE_PFX_MOTION_MASK_DYNAMIC(motion)   ((1<<(motion))&0x0a) // Active,OneWay
#define SCE_PFX_MOTION_MASK_STATIC(motion)    ((1<<(motion))&0x95) // Fixed,Keyframe
#define SCE_PFX_MOTION_MASK_CAN_SLEEP(motion) ((1<<(motion))&0x0e) // Can sleep

#define SCE_PFX_MOTION_MASK_SLEEPING 0x80 // Is sleeping
#define SCE_PFX_MOTION_MASK_TYPE     0x7f // Motion Type

class SCE_PFX_ALIGNED(128) PfxRigidState
{
private:
	union {
		struct {
			PfxUInt8	m_useSleep  : 1;
			PfxUInt8	m_sleeping  : 1;
			PfxUInt8	m_reserved1 : 1;
			PfxUInt8	m_reserved2 : 1;
			PfxUInt8	m_reserved3 : 1;
			PfxUInt8	m_reserved4 : 1;
			PfxUInt8	m_reserved5 : 1;
			PfxUInt8	m_reserved6 : 1;
		};
		PfxUInt8 m_flags;
	};
	PfxUInt8	m_motionType;
	PfxUInt16	m_sleepCount;
	PfxUInt16	m_rigidBodyId;

	SCE_PFX_PADDING(1,2)

	PfxUInt32	m_contactFilterSelf;
	PfxUInt32	m_contactFilterTarget;

	PfxFloat	m_linearDamping;
	PfxFloat	m_angularDamping;

	PfxFloat	m_maxLinearVelocity;
	PfxFloat	m_maxAngularVelocity;
	
	PfxVector3	m_position;
	PfxQuat		m_orientation;
	PfxVector3	m_linearVelocity;
	PfxVector3	m_angularVelocity;

	void		*m_userData;
	PfxUInt32	m_userParam[4];
	
	SCE_PFX_PADDING(2,12)

public:
	inline void reset();

	PfxUInt16	getRigidBodyId() const {return m_rigidBodyId;}
	void		setRigidBodyId(PfxUInt16 i) {m_rigidBodyId = i;}

	PfxUInt32	getContactFilterSelf() const {return m_contactFilterSelf;}
	void		setContactFilterSelf(PfxUInt32 filter) {m_contactFilterSelf = filter;}

	PfxUInt32	getContactFilterTarget() const {return m_contactFilterTarget;}
	void		setContactFilterTarget(PfxUInt32 filter) {m_contactFilterTarget = filter;}

	ePfxMotionType	getMotionType() const {return (ePfxMotionType)m_motionType;}
	void		setMotionType(ePfxMotionType t) {SCE_PFX_ALWAYS_ASSERT(t<kPfxMotionTypeCount);m_motionType = (PfxUInt8)t;wakeup();}

	PfxUInt8	getMotionMask() const {return m_motionType|(m_sleeping<<7);}

	PfxBool		isAsleep() const {return m_sleeping==1;}
	PfxBool		isAwake() const {return m_sleeping==0;}

	void		wakeup() {m_sleeping=0;m_sleepCount=0;}
	void		sleep() {if(m_useSleep) {m_sleeping=1;m_sleepCount=0;}}

	PfxUInt8	getUseSleep() const {return m_useSleep;}
	void		setUseSleep(PfxUInt8 b) {m_useSleep=b;}

	void	 	incrementSleepCount() {if(m_sleepCount<0xffff)m_sleepCount++;}
	void		resetSleepCount() {m_sleepCount=0;}
	PfxUInt16	getSleepCount() const {return m_sleepCount;}

	PfxFloat getLinearDamping() const {return m_linearDamping;}
	PfxFloat getAngularDamping() const {return m_angularDamping;}

	PfxFloat getMaxLinearVelocity() const {return m_maxLinearVelocity;}
	PfxFloat getMaxAngularVelocity() const {return m_maxAngularVelocity;}

	void setLinearDamping(PfxFloat damping) {m_linearDamping=damping;}
	void setAngularDamping(PfxFloat damping) {m_angularDamping=damping;}

	void setMaxLinearVelocity(PfxFloat maxvelocity){m_maxLinearVelocity = maxvelocity;}
	void setMaxAngularVelocity(PfxFloat maxvelocity){m_maxAngularVelocity = maxvelocity;}

	PfxVector3 getPosition() const {return m_position;}
	PfxQuat    getOrientation() const {return m_orientation;}
	PfxVector3 getLinearVelocity() const {return m_linearVelocity;}
	PfxVector3 getAngularVelocity() const {return m_angularVelocity;}

	void setPosition(const PfxVector3 &pos) {m_position = pos;}
	void setOrientation(const PfxQuat &rot) {m_orientation = rot;}
	void setLinearVelocity(const PfxVector3 &vel) {m_linearVelocity = vel;}
	void setAngularVelocity(const PfxVector3 &vel) {m_angularVelocity = vel;}

	void		*getUserData() const {return m_userData;}
	void		setUserData(void *d) {m_userData=d;}

	inline void movePosition(const PfxVector3 &pos,PfxFloat timeStep);
	inline void moveOrientation(const PfxQuat &rot,PfxFloat timeStep);
};

inline
void PfxRigidState::reset()
{
	m_sleepCount = 0;
	m_motionType = kPfxMotionTypeActive;
	m_flags = 0;
	m_rigidBodyId = 0;
	m_contactFilterSelf = 0xffffffff;
	m_contactFilterTarget = 0xffffffff;
	m_linearDamping = 1.0f;
	m_angularDamping = 0.99f;
	m_maxLinearVelocity = 200.0f;
	m_maxAngularVelocity = SCE_PFX_PI * 100.0f;
	m_position = PfxVector3(0.0f);
	m_orientation = PfxQuat::identity();
	m_linearVelocity = PfxVector3(0.0f);
	m_angularVelocity = PfxVector3(0.0f);
	m_userData = NULL;
}

inline
void PfxRigidState::movePosition(const PfxVector3 &pos,PfxFloat timeStep)
{
	if(getMotionType()!=kPfxMotionTypeKeyframe) return;

	SCE_PFX_ASSERT(timeStep > 0.0f);

	setLinearVelocity((pos - getPosition()) / timeStep);
}

inline
void PfxRigidState::moveOrientation(const PfxQuat &rot,PfxFloat timeStep)
{
	if(getMotionType()!=kPfxMotionTypeKeyframe) return;

	SCE_PFX_ASSERT(timeStep > 0.0f);

	PfxQuat ori1 = getOrientation();
	PfxQuat ori2 = rot;

	if(dot(ori2,ori1) < 0.0f) {
		ori2 = -rot;
	}

	PfxQuat dq = ( ori2 - ori1 ) / timeStep;
	dq = dq * 2.0f * conj(ori1);
	PfxVector3 omega = dq.getXYZ();

	setAngularVelocity(omega);
}
} // namespace PhysicsEffects
} // namespace sce
#endif // _SCE_PFX_RIGID_STATE_H

