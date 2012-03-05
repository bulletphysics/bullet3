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

#include "pfx_contact_cache.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_CONTACT_SAME_POINT 0.01f

int PfxContactCache::findNearestContactPoint(const PfxPoint3 &newPoint,const PfxVector3 &newNormal)
{
	int nearestIdx = -1;
	PfxFloat minDiff = SCE_PFX_CONTACT_SAME_POINT;
	for(PfxUInt32 i=0;i<m_numContacts;i++) {
		PfxVector3 dist = m_cachedContactPoints[i].m_localPointA-newPoint;
		PfxFloat diff = lengthSqr(dist);
		if(diff < minDiff && dot(newNormal,m_cachedContactPoints[i].m_normal) > 0.99f) {
			minDiff = diff;
			nearestIdx = i;
		}
	}
	return nearestIdx;
}

int PfxContactCache::sort4ContactPoints(const PfxPoint3 &newCP,PfxFloat newDistance)
{
	int maxPenetrationIndex = -1;
	PfxFloat maxPenetration = newDistance;

	// 最も深い衝突点は排除対象からはずす
	for(int i=0;i<SCE_PFX_MAX_CACHED_CONTACT_POINTS;i++) {
		if(m_cachedContactPoints[i].m_distance < maxPenetration) {
			maxPenetrationIndex = i;
			maxPenetration = m_cachedContactPoints[i].m_distance;
		}
	}
	
	PfxFloat res[4] = {0.0f};
	
	// 各点を除いたときの衝突点が作る面積のうち、最も大きくなるものを選択
	PfxVector3 newp(newCP);
	PfxVector3 p[4];
	p[0] = (PfxVector3)m_cachedContactPoints[0].m_localPointA;
	p[1] = (PfxVector3)m_cachedContactPoints[1].m_localPointA;
	p[2] = (PfxVector3)m_cachedContactPoints[2].m_localPointA;
	p[3] = (PfxVector3)m_cachedContactPoints[3].m_localPointA;

	if(maxPenetrationIndex != 0) {
		PfxVector3 a0 = newp-p[1];
		PfxVector3 b0 = p[3]-p[2];
		res[0] = lengthSqr(cross(a0,b0));
	}
 
	if(maxPenetrationIndex != 1) {
		PfxVector3 a1 = newp-p[0];
		PfxVector3 b1 = p[3]-p[2];
		res[1] = lengthSqr(cross(a1,b1));
	}

	if(maxPenetrationIndex != 2) {
		PfxVector3 a2 = newp-p[0];
		PfxVector3 b2 = p[3]-p[1];
		res[2] = lengthSqr(cross(a2,b2));
	}

	if(maxPenetrationIndex != 3) {
		PfxVector3 a3 = newp-p[0];
		PfxVector3 b3 = p[2]-p[1];
		res[3] = lengthSqr(cross(a3,b3));
	}

	int maxIndex = 0;
	PfxFloat maxVal = res[0];

	if (res[1] > maxVal) {
		maxIndex = 1;
		maxVal = res[1];
	}

	if (res[2] > maxVal) {
		maxIndex = 2;
		maxVal = res[2];
	}

	if (res[3] > maxVal) {
		maxIndex = 3;
		maxVal = res[3];
	}

	return maxIndex;
}

void PfxContactCache::addContactPoint(
		PfxFloat newDistance,
		const PfxVector3 &newNormal, // world coords
		const PfxPoint3 &newPointA, // local to the objectA
		const PfxPoint3 &newPointB, // local to the objectB
		PfxSubData subData)
{
	int id = findNearestContactPoint(newPointA,newNormal);

	if(id < 0 && m_numContacts < SCE_PFX_MAX_CACHED_CONTACT_POINTS) {
		// 衝突点を新規追加
		id = m_numContacts++;
		m_cachedContactPoints[id].reset();
	}
	else if(id < 0){
		// ソート
		id = sort4ContactPoints(newPointA,newDistance);
		m_cachedContactPoints[id].reset();
	}

	m_cachedContactPoints[id].m_distance = newDistance;
	m_cachedContactPoints[id].m_subData = subData;
	m_cachedContactPoints[id].m_normal = newNormal;
	m_cachedContactPoints[id].m_localPointA = newPointA;
	m_cachedContactPoints[id].m_localPointB = newPointB;
}

void PfxContactCache::addContactPoint(const PfxCachedContactPoint &cp)
{
	PfxPoint3 pA = cp.m_localPointA;
	
	int id = findNearestContactPoint(pA,cp.m_normal);
	
	if(id >= 0) {
		if(m_cachedContactPoints[id].m_distance > cp.m_distance) {
			// 同一点を発見、衝突点情報を更新
			m_cachedContactPoints[id].m_distance = cp.m_distance;
			m_cachedContactPoints[id].m_normal = cp.m_normal;
			m_cachedContactPoints[id].m_localPointA = cp.m_localPointA;
			m_cachedContactPoints[id].m_localPointB = cp.m_localPointB;
		}
	}
	else if(m_numContacts < SCE_PFX_MAX_CACHED_CONTACT_POINTS) {
		// 衝突点を新規追加
		m_cachedContactPoints[m_numContacts++] = cp;
	}
	else {
		// ソート
		id = sort4ContactPoints(pA,cp.m_distance);
		
		// コンタクトポイント入れ替え
		m_cachedContactPoints[id] = cp;
	}
}

} //namespace PhysicsEffects
} //namespace sce

