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

#ifndef _SCE_PFX_CONTACT_MANIFOLD_H
#define _SCE_PFX_CONTACT_MANIFOLD_H

#include "../base/pfx_common.h"
#include "../base/pfx_vec_utils.h"
#include "pfx_sub_data.h"
#include "../solver/pfx_constraint_row.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_NUMCONTACTS_PER_BODIES 4

///////////////////////////////////////////////////////////////////////////////
// Contact Point

struct PfxContactPoint
{
	PfxUInt8 m_duration;
	PfxUInt8 m_shapeIdA;
	PfxUInt8 m_shapeIdB;
	SCE_PFX_PADDING(1,1)
	PfxSubData m_subData;
	PfxFloat m_distance;
	PfxFloat m_localPointA[3];
	PfxFloat m_localPointB[3];
	SCE_PFX_PADDING(2,8)
	PfxConstraintRow m_constraintRow[3];

	void reset()
	{
		m_duration = 0;
		m_shapeIdA = m_shapeIdB = 0;
		m_subData = PfxSubData();
		m_distance = SCE_PFX_FLT_MAX;
		m_constraintRow[0].m_accumImpulse = 0.0f;
		m_constraintRow[1].m_accumImpulse = 0.0f;
		m_constraintRow[2].m_accumImpulse = 0.0f;
	}
};

///////////////////////////////////////////////////////////////////////////////
// Contact Manifold

//J	同一ペアの衝突が続く限り保持されるコンタクト情報
//E PfxContactManifold keeps contact information until two rigid bodies are separated.

class SCE_PFX_ALIGNED(128) PfxContactManifold
{
private:
	PfxUInt16 m_rigidBodyIdA,m_rigidBodyIdB;
	PfxUInt16 m_duration;
	PfxUInt16 m_numContacts;
	PfxFloat  m_compositeFriction;
	PfxUInt32 m_internalFlag;
	PfxContactPoint m_contactPoints[SCE_PFX_NUMCONTACTS_PER_BODIES];
	void		*m_userData;
	PfxUInt32	m_userParam[4];
	SCE_PFX_PADDING(1,28)

	int findNearestContactPoint(const PfxPoint3 &newPoint,const PfxVector3 &newNormal);
	int sort4ContactPoints(const PfxPoint3 &newPoint,PfxFloat newDistance);

public:
	// Internal method
	PfxFloat getCompositeFriction() const {return m_compositeFriction;}
	void setCompositeFriction(PfxFloat f) {m_compositeFriction = f;}

	PfxUInt32 getInternalFlag() const {return m_internalFlag;}
	void setInternalFlag(PfxUInt32 f) {m_internalFlag = f;}

public:
	void reset(PfxUInt16 rigidBodyIdA,PfxUInt16 rigidBodyIdB)
	{
		m_userData = 0;
		m_userParam[0] = m_userParam[1] = m_userParam[2] = m_userParam[3] = 0;
		m_numContacts = 0;
		m_duration = 0;
		m_rigidBodyIdA = rigidBodyIdA;
		m_rigidBodyIdB = rigidBodyIdB;
	}
	
	void addContactPoint(
		PfxFloat newDistance,
		const PfxVector3 &newNormal, // world coords
		const PfxPoint3 &newPointA, // local coords to the objectA
		const PfxPoint3 &newPointB, // local coords to the objectB
		PfxSubData subData);
	
	void addContactPoint(const PfxContactPoint &cp);
	
	void removeContactPoint(int i)
	{
		SCE_PFX_ASSERT(i>=0&&i<m_numContacts);
		m_contactPoints[i] = m_contactPoints[m_numContacts-1];
		m_numContacts--;
	}
	
	int getNumContacts() const {return (int)m_numContacts;}
	
	PfxContactPoint &getContactPoint(int i) {return m_contactPoints[i];}
	
	const PfxContactPoint &getContactPoint(int i) const {return m_contactPoints[i];}
	
	void refresh(const PfxVector3 &pA,const PfxQuat &qA,const PfxVector3 &pB,const PfxQuat &qB);
	
	void merge(const PfxContactManifold &contact);
	
	PfxUInt16 getDuration() const {return m_duration;}
	
	PfxUInt16 getRigidBodyIdA() const {return m_rigidBodyIdA;}
	
	PfxUInt16 getRigidBodyIdB() const {return m_rigidBodyIdB;}
};

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_CONTACT_MANIFOLD_H
