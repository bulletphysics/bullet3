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

#include "../../../include/physics_effects/base_level/collision/pfx_contact_manifold.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_CONTACT_SAME_POINT			0.01f
#define SCE_PFX_CONTACT_THRESHOLD_NORMAL	0.01f	// 衝突点の閾値（法線方向）
#define SCE_PFX_CONTACT_THRESHOLD_TANGENT	0.002f	// 衝突点の閾値（平面上）

int PfxContactManifold::findNearestContactPoint(const PfxPoint3 &newPoint,const PfxVector3 &newNormal)
{
	int nearestIdx = -1;
	PfxFloat minDiff = SCE_PFX_CONTACT_SAME_POINT;
	for(int i=0;i<m_numContacts;i++) {
		PfxFloat diff = lengthSqr(pfxReadVector3(m_contactPoints[i].m_localPointA) - PfxVector3(newPoint));
		if(diff < minDiff && dot(newNormal,pfxReadVector3(m_contactPoints[i].m_constraintRow[0].m_normal)) > 0.99f) {
			minDiff = diff;
			nearestIdx = i;
		}
	}
	return nearestIdx;
}

int PfxContactManifold::sort4ContactPoints(const PfxPoint3 &newCP,PfxFloat newDistance)
{
	int maxPenetrationIndex = -1;
	PfxFloat maxPenetration = newDistance;

	// 最も深い衝突点は排除対象からはずす
	for(int i=0;i<SCE_PFX_NUMCONTACTS_PER_BODIES;i++) {
		if(m_contactPoints[i].m_distance < maxPenetration) {
			maxPenetrationIndex = i;
			maxPenetration = m_contactPoints[i].m_distance;
		}
	}
	
	PfxFloat res[4] = {0.0f};
	
	// 各点を除いたときの衝突点が作る面積のうち、最も大きくなるものを選択
	PfxVector3 newp(newCP);
	PfxVector3 p[4];
	p[0] = pfxReadVector3(m_contactPoints[0].m_localPointA);
	p[1] = pfxReadVector3(m_contactPoints[1].m_localPointA);
	p[2] = pfxReadVector3(m_contactPoints[2].m_localPointA);
	p[3] = pfxReadVector3(m_contactPoints[3].m_localPointA);

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

void PfxContactManifold::addContactPoint(
		PfxFloat newDistance,
		const PfxVector3 &newNormal, // world coords
		const PfxPoint3 &newPointA, // local to the objectA
		const PfxPoint3 &newPointB, // local to the objectB
		PfxSubData subData)
{
	int id = findNearestContactPoint(newPointA,newNormal);

	if(id < 0 && m_numContacts < SCE_PFX_NUMCONTACTS_PER_BODIES) {
		// 衝突点を新規追加
		id = m_numContacts++;
		m_contactPoints[id].reset();
	}
	else if(id < 0){
		// ソート
		id = sort4ContactPoints(newPointA,newDistance);
		m_contactPoints[id].reset();
	}

	m_contactPoints[id].m_distance = newDistance;
	m_contactPoints[id].m_subData = subData;
	pfxStorePoint3(newPointA,m_contactPoints[id].m_localPointA);
	pfxStorePoint3(newPointB,m_contactPoints[id].m_localPointB);
	pfxStoreVector3(newNormal,m_contactPoints[id].m_constraintRow[0].m_normal);
}

void PfxContactManifold::addContactPoint(const PfxContactPoint &cp)
{
	PfxPoint3 pA = pfxReadPoint3(cp.m_localPointA);
	
	int id = findNearestContactPoint(pA,pfxReadVector3(cp.m_constraintRow[0].m_normal));
	
	if(id >= 0) {
#if 1
		PfxVector3 nml1(pfxReadVector3(m_contactPoints[id].m_constraintRow[0].m_normal));
		PfxVector3 nml2(pfxReadVector3(cp.m_constraintRow[0].m_normal));
		if(fabsf(dot(nml1,nml2)) > 0.99f ) {
			// 同一点を発見、蓄積された情報を継続
			m_contactPoints[id].m_distance = cp.m_distance;
			m_contactPoints[id].m_localPointA[0] = cp.m_localPointA[0];
			m_contactPoints[id].m_localPointA[1] = cp.m_localPointA[1];
			m_contactPoints[id].m_localPointA[2] = cp.m_localPointA[2];
			m_contactPoints[id].m_localPointB[0] = cp.m_localPointB[0];
			m_contactPoints[id].m_localPointB[1] = cp.m_localPointB[1];
			m_contactPoints[id].m_localPointB[2] = cp.m_localPointB[2];
			m_contactPoints[id].m_constraintRow[0].m_normal[0] = cp.m_constraintRow[0].m_normal[0];
			m_contactPoints[id].m_constraintRow[0].m_normal[1] = cp.m_constraintRow[0].m_normal[1];
			m_contactPoints[id].m_constraintRow[0].m_normal[2] = cp.m_constraintRow[0].m_normal[2];
		}
		else {
			// 同一点ではあるが法線が違うため更新
			m_contactPoints[id] = cp;
		}
#else
		if(m_contactPoints[id].m_distance > cp.m_distance) {
			// 同一点を発見、衝突点情報を更新
			m_contactPoints[id].m_distance = cp.m_distance;
			m_contactPoints[id].m_localPointA[0] = cp.m_localPointA[0];
			m_contactPoints[id].m_localPointA[1] = cp.m_localPointA[1];
			m_contactPoints[id].m_localPointA[2] = cp.m_localPointA[2];
			m_contactPoints[id].m_localPointB[0] = cp.m_localPointB[0];
			m_contactPoints[id].m_localPointB[1] = cp.m_localPointB[1];
			m_contactPoints[id].m_localPointB[2] = cp.m_localPointB[2];
			m_contactPoints[id].m_constraintRow[0].m_normal[0] = cp.m_constraintRow[0].m_normal[0];
			m_contactPoints[id].m_constraintRow[0].m_normal[1] = cp.m_constraintRow[0].m_normal[1];
			m_contactPoints[id].m_constraintRow[0].m_normal[2] = cp.m_constraintRow[0].m_normal[2];
		}
#endif
	}
	else if(m_numContacts < SCE_PFX_NUMCONTACTS_PER_BODIES) {
		// 衝突点を新規追加
		m_contactPoints[m_numContacts++] = cp;
	}
	else {
		// ソート
		id = sort4ContactPoints(pA,cp.m_distance);
		
		// コンタクトポイント入れ替え
		m_contactPoints[id] = cp;
	}
}

void PfxContactManifold::merge(const PfxContactManifold &contact)
{
	SCE_PFX_ALWAYS_ASSERT(m_rigidBodyIdA == contact.getRigidBodyIdA());
	SCE_PFX_ALWAYS_ASSERT(m_rigidBodyIdB == contact.getRigidBodyIdB());

	for(int i=0;i<contact.getNumContacts();i++) {
		addContactPoint(contact.getContactPoint(i));
	}
}

void PfxContactManifold::refresh(const PfxVector3 &pA,const PfxQuat &qA,const PfxVector3 &pB,const PfxQuat &qB)
{
	// 衝突点の更新
	// 両衝突点間の距離が閾値（CONTACT_THRESHOLD）を超えたら消去
	for(int i=0;i<(int)m_numContacts;i++) {
		if(m_contactPoints[i].m_duration > 0) {
			PfxVector3 normal = pfxReadVector3(m_contactPoints[i].m_constraintRow[0].m_normal);
			PfxVector3 cpA = pA + rotate(qA,pfxReadVector3(m_contactPoints[i].m_localPointA));
			PfxVector3 cpB = pB + rotate(qB,pfxReadVector3(m_contactPoints[i].m_localPointB));

			// 貫通深度がプラスに転じたかどうかをチェック
			PfxFloat distance = dot(normal,(cpA - cpB));
			if(distance > SCE_PFX_CONTACT_THRESHOLD_NORMAL) {
				removeContactPoint(i);
				i--;
				continue;
			}
			m_contactPoints[i].m_distance = distance;

			// 深度方向を除去して両点の距離をチェック
			cpA = cpA - m_contactPoints[i].m_distance * normal;
			PfxFloat distanceAB = lengthSqr(cpA - cpB);
			if(distanceAB > SCE_PFX_CONTACT_THRESHOLD_TANGENT) {
				removeContactPoint(i);
				i--;
				continue;
			}
		}
		if(m_contactPoints[i].m_duration < 255) m_contactPoints[i].m_duration++;
	}
	if(m_numContacts > 0 && m_duration < 65535) m_duration++;
}
} //namespace PhysicsEffects
} //namespace sce
