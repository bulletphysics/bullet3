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

#ifndef _SCE_PFX_CONTACT_CACHE_H
#define _SCE_PFX_CONTACT_CACHE_H

#include "../../../include/physics_effects/base_level/base/pfx_common.h"
#include "../../../include/physics_effects/base_level/base/pfx_vec_utils.h"
#include "../../../include/physics_effects/base_level/collision/pfx_sub_data.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_MAX_CACHED_CONTACT_POINTS 4

/*
	内部の衝突判定に使う軽量なコンタクトキャッシュ
*/

///////////////////////////////////////////////////////////////////////////////
// Contact Point

struct PfxCachedContactPoint
{
	PfxSubData m_subData;
	PfxUInt8 m_shapeIdA;
	PfxUInt8 m_shapeIdB;
	SCE_PFX_PADDING(1,2)
	PfxFloat m_distance;
	PfxVector3 m_normal;
	PfxPoint3 m_localPointA;
	PfxPoint3 m_localPointB;

	void reset()
	{
		m_shapeIdA = m_shapeIdB = 0;
		m_subData = PfxSubData();
		m_distance = SCE_PFX_FLT_MAX;
	}
};

///////////////////////////////////////////////////////////////////////////////
// Contact Point

class PfxContactCache
{
private:
	PfxUInt32 m_numContacts;
	SCE_PFX_PADDING(1,12)
	PfxCachedContactPoint m_cachedContactPoints[SCE_PFX_MAX_CACHED_CONTACT_POINTS];
	
	int findNearestContactPoint(const PfxPoint3 &newPoint,const PfxVector3 &newNormal);
	int sort4ContactPoints(const PfxPoint3 &newPoint,PfxFloat newDistance);
	
public:
	PfxContactCache() : m_numContacts(0) {}
	
	void addContactPoint(
		PfxFloat newDistance,
		const PfxVector3 &newNormal, // world normal vector
		const PfxPoint3 &newPointA, // local contact point to the objectA
		const PfxPoint3 &newPointB, // local contact point to the objectB
		PfxSubData m_subData);
	
	void addContactPoint(const PfxCachedContactPoint &cp);
	
	int getNumContacts() const {return (int)m_numContacts;}
	
	PfxCachedContactPoint &getContactPoint(int i) {return m_cachedContactPoints[i];}
	
	const PfxCachedContactPoint &getContactPoint(int i) const {return m_cachedContactPoints[i];}

	PfxFloat getDistance(int i) {return m_cachedContactPoints[i].m_distance;}
	
	const PfxVector3 &getNormal(int i) const {return m_cachedContactPoints[i].m_normal;}
	
	const PfxPoint3 &getLocalPointA(int i) const {return m_cachedContactPoints[i].m_localPointA;}
	
	const PfxPoint3 &getLocalPointB(int i) const {return m_cachedContactPoints[i].m_localPointB;}
	
	const PfxSubData &getSubData(int i) const {return m_cachedContactPoints[i].m_subData;}
};

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_CONTACT_CACHE_H
