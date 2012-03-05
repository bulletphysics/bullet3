/*
Physics Effects Copyright(C) 2011 Sony Computer Entertainment Inc.
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

#include "btLowLevelBroadphase.h"


#include "BulletMultiThreaded/PlatformDefinitions.h"
#include "BulletCollision/NarrowphaseCollision/btPersistentManifold.h"

// Include base level headers
#include "physics_effects/base_level/pfx_base_level_include.h"

// Include low level headers
#include "physics_effects/low_level/broadphase/pfx_broadphase.h"

#include "physics_effects/low_level/sort/pfx_parallel_sort.h"

#include "BulletMultiThreaded/vectormath2bullet.h"

#include "physics_effects/base_level/base/pfx_vec_utils.h"
#include "physics_effects/base_level/collision/pfx_aabb.h"
#include "physics_effects/base_level/rigidbody/pfx_rigid_state.h"
#include "btLowLevelData.h"

using namespace sce::PhysicsEffects;

//E Temporary buffers
#define POOL_BYTES (5*1024*1024)
unsigned char SCE_PFX_ALIGNED(128) poolBuff[POOL_BYTES];

//E Stack allocator for temporary buffers
PfxHeapManager pool(poolBuff,POOL_BYTES);


///////////////////////////////////////////////////////////////////////////////
// Broadphase





//
btLowLevelBroadphase::btLowLevelBroadphase(btLowLevelData* lowLevelData, btOverlappingPairCache* paircache, int maxProxies)
:m_lowLevelData(lowLevelData)
{
	
	m_guidGenerator = 1;
	m_releasepaircache	=	(paircache!=0)?false:true;
	m_paircache			=	paircache? paircache	: new(btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16)) btHashedOverlappingPairCache();
	m_clientData.m_bp = this;
	m_clientData.m_dispatcher = 0;


	m_broadphaseAabbMin.setValue(1e30,1e30,1e30);
	m_broadphaseAabbMax.setValue(-1e30,-1e30,-1e30);

		// allocate handles buffer and put all handles on free list
	m_pHandlesRawPtr = btAlignedAlloc(sizeof(btLowLevelBroadphaseProxy)*maxProxies,16);
	m_pHandles = new(m_pHandlesRawPtr) btLowLevelBroadphaseProxy[maxProxies];
	m_maxHandles = maxProxies;
	m_numHandles = 0;
	m_firstFreeHandle = 0;
	m_LastHandleIndex = -1;
	

	{
		for (int i = m_firstFreeHandle; i < maxProxies; i++)
		{
			m_pHandles[i].SetNextFree(i + 1);
			m_pHandles[i].m_uniqueId = i;//start from zero, so we can re-use the uid for body ID
		}
		m_pHandles[maxProxies - 1].SetNextFree(0);
	
	}

}

//
btLowLevelBroadphase::~btLowLevelBroadphase()
{
	if(m_releasepaircache) 
	{
		m_paircache->~btOverlappingPairCache();
		btAlignedFree(m_paircache);
	}
}


btBroadphaseProxy*	btLowLevelBroadphase::createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* /*dispatcher*/,void* multiSapProxy)
{
	if (m_numHandles >= m_maxHandles)
	{
		btAssert(0);
		return 0; //should never happen, but don't let the game crash ;-)
	}
	btAssert(aabbMin[0]<= aabbMax[0] && aabbMin[1]<= aabbMax[1] && aabbMin[2]<= aabbMax[2]);

	int newHandleIndex = allocHandle();
	btLowLevelBroadphaseProxy* proxy = new (&m_pHandles[newHandleIndex])btLowLevelBroadphaseProxy(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy);
	m_uid2ptr.insert(proxy->m_uniqueId,proxy);
	return proxy;
}

void	btLowLevelBroadphase::destroyProxy(btBroadphaseProxy* proxyOrg,btDispatcher* dispatcher)
{
		m_uid2ptr.remove(proxyOrg->m_uniqueId);

		btLowLevelBroadphaseProxy* proxy0 = static_cast<btLowLevelBroadphaseProxy*>(proxyOrg);
		freeHandle(proxy0);

		m_paircache->removeOverlappingPairsContainingProxy(proxyOrg,dispatcher);

		//validate();
		
}

void	btLowLevelBroadphase::getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const
{
	const btLowLevelBroadphaseProxy* sbp = getLowLevelProxyFromProxy(proxy);
	aabbMin = sbp->m_aabbMin;
	aabbMax = sbp->m_aabbMax;
}

void	btLowLevelBroadphase::setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax, btDispatcher* /*dispatcher*/)
{
	btLowLevelBroadphaseProxy* sbp = getLowLevelProxyFromProxy(proxy);
	sbp->m_aabbMin = aabbMin;
	sbp->m_aabbMax = aabbMax;
}


bool	btLowLevelBroadphase::aabbOverlap(btLowLevelBroadphaseProxy* proxy0,btLowLevelBroadphaseProxy* proxy1)
{
	return proxy0->m_aabbMin[0] <= proxy1->m_aabbMax[0] && proxy1->m_aabbMin[0] <= proxy0->m_aabbMax[0] && 
		   proxy0->m_aabbMin[1] <= proxy1->m_aabbMax[1] && proxy1->m_aabbMin[1] <= proxy0->m_aabbMax[1] &&
		   proxy0->m_aabbMin[2] <= proxy1->m_aabbMax[2] && proxy1->m_aabbMin[2] <= proxy0->m_aabbMax[2];

}


PfxBroadphasePair*		btLowLevelBroadphase::getCurrentPairs()
{
	return &m_lowLevelData->m_pairsBuff[m_lowLevelData->m_pairSwap][0];
}

const PfxBroadphasePair*		btLowLevelBroadphase::getCurrentPairs() const
{
	return &m_lowLevelData->m_pairsBuff[m_lowLevelData->m_pairSwap][0];
}

int	btLowLevelBroadphase::getNumCurrentPairs() const
{
	return m_lowLevelData->m_numPairs[m_lowLevelData->m_pairSwap];
}

void btLowLevelBroadphase::broadphase(PfxSortData32* proxies, int numRigidBodies, int axis, btDispatcher* dispatcher)
{
	m_lowLevelData->m_pairSwap = 1-m_lowLevelData->m_pairSwap;

	unsigned int &numPreviousPairs = m_lowLevelData->m_numPairs[1-m_lowLevelData->m_pairSwap];
	unsigned int &numCurrentPairs = m_lowLevelData->m_numPairs[m_lowLevelData->m_pairSwap];
	PfxBroadphasePair *previousPairs = &m_lowLevelData->m_pairsBuff[1-m_lowLevelData->m_pairSwap][0];
	PfxBroadphasePair *currentPairs = &m_lowLevelData->m_pairsBuff[m_lowLevelData->m_pairSwap][0];


	//E Create broadpahse proxies
	{
//		for(int i=0;i<numRigidBodies;i++) {
//			pfxUpdateBroadphaseProxy(proxies[i],states[i],collidables[i],worldCenter,worldExtent,axis);
//		}

		int workBytes = sizeof(PfxBroadphaseProxy) * numRigidBodies;
		void *workBuff = pool.allocate(workBytes);
				
		pfxParallelSort(proxies,numRigidBodies,workBuff,workBytes);

		pool.deallocate(workBuff);
	}

	//E Find overlapped pairs
	{
		PfxFindPairsParam findPairsParam;
		findPairsParam.pairBytes = pfxGetPairBytesOfFindPairs(m_lowLevelData->m_maxPairs);
		findPairsParam.pairBuff = pool.allocate(findPairsParam.pairBytes);
		findPairsParam.workBytes = pfxGetWorkBytesOfFindPairs(m_lowLevelData->m_maxPairs);
		findPairsParam.workBuff = pool.allocate(findPairsParam.workBytes);
		findPairsParam.proxies = proxies;
		findPairsParam.numProxies = numRigidBodies;
		findPairsParam.maxPairs = m_lowLevelData->m_maxPairs;
		findPairsParam.axis = axis;

		PfxFindPairsResult findPairsResult;


		int ret = pfxFindPairs(findPairsParam,findPairsResult);
		if(ret != SCE_PFX_OK) 
			SCE_PFX_PRINTF("pfxFindPairs failed %d\n",ret);
		
		pool.deallocate(findPairsParam.workBuff);

		//E Decompose overlapped pairs into 3 arrays
		PfxDecomposePairsParam decomposePairsParam;
		decomposePairsParam.pairBytes = pfxGetPairBytesOfDecomposePairs(numPreviousPairs,findPairsResult.numPairs);
		decomposePairsParam.pairBuff = pool.allocate(decomposePairsParam.pairBytes);
		decomposePairsParam.workBytes = pfxGetWorkBytesOfDecomposePairs(numPreviousPairs,findPairsResult.numPairs);
		decomposePairsParam.workBuff = pool.allocate(decomposePairsParam.workBytes);
		decomposePairsParam.previousPairs = previousPairs;
		decomposePairsParam.numPreviousPairs = numPreviousPairs;
		decomposePairsParam.currentPairs = findPairsResult.pairs; // Set pairs from pfxFindPairs()
		decomposePairsParam.numCurrentPairs = findPairsResult.numPairs; // Set the number of pairs from pfxFindPairs()

		PfxDecomposePairsResult decomposePairsResult;

		ret = pfxDecomposePairs(decomposePairsParam,decomposePairsResult);
		if(ret != SCE_PFX_OK) 
			SCE_PFX_PRINTF("pfxDecomposePairs failed %d\n",ret);

		pool.deallocate(decomposePairsParam.workBuff);

		PfxBroadphasePair *outNewPairs = decomposePairsResult.outNewPairs;
		PfxBroadphasePair *outKeepPairs = decomposePairsResult.outKeepPairs;
		PfxBroadphasePair *outRemovePairs = decomposePairsResult.outRemovePairs;
		PfxUInt32 numOutNewPairs = decomposePairsResult.numOutNewPairs;
		PfxUInt32 numOutKeepPairs = decomposePairsResult.numOutKeepPairs;
		PfxUInt32 numOutRemovePairs = decomposePairsResult.numOutRemovePairs;

		for (int i=0;i<numOutRemovePairs;i++)
		{
			int idA = pfxGetObjectIdA(outRemovePairs[i]);
			int idB = pfxGetObjectIdB(outRemovePairs[i]);
			//use m_uid2ptr to get pointer
			
			btBroadphaseProxy* proxyA = (btBroadphaseProxy*)*m_uid2ptr[idA];
			btBroadphaseProxy* proxyB = (btBroadphaseProxy*)*m_uid2ptr[idB];
			m_paircache->removeOverlappingPair(proxyA,proxyB,dispatcher);

			//free low level contacts
			m_lowLevelData->m_contactIdPool[m_lowLevelData->m_numContactIdPool++] = pfxGetContactId(outRemovePairs[i]);

		}

		for (int i=0;i<numOutNewPairs;i++)
		{
			int idA = pfxGetObjectIdA(outNewPairs[i]);
			int idB = pfxGetObjectIdB(outNewPairs[i]);
			//use m_uid2ptr to get pointer
			
			btBroadphaseProxy* proxyA = (btBroadphaseProxy*)*m_uid2ptr[idA];
			btBroadphaseProxy* proxyB = (btBroadphaseProxy*)*m_uid2ptr[idB];
			btBroadphasePair* btpair = m_paircache->addOverlappingPair(proxyA,proxyB);

			//initialize low level contacts
			int cId = 0;
			if(m_lowLevelData->m_numContactIdPool > 0) {
				cId = m_lowLevelData->m_contactIdPool[--m_lowLevelData->m_numContactIdPool];
			}
			else {
				cId = m_lowLevelData->m_numContacts++;
			}
			if(cId >= m_lowLevelData->m_maxContacts) {
				cId = 0;
			}
			SCE_PFX_ASSERT(cId < m_lowLevelData->m_maxContacts);
			pfxSetContactId(outNewPairs[i],cId);
			PfxContactManifold &contact = m_lowLevelData->m_contacts[cId];
			int sz = sizeof(PfxContactManifold);
			int sz2 = sizeof(btPersistentManifold);
			int sz3 = 4*3*sizeof(btConstraintRow);
			contact.reset(pfxGetObjectIdA(outNewPairs[i]),pfxGetObjectIdB(outNewPairs[i]));
			contact.setCompositeFriction(0.1f);
			btpair->m_internalTmpValue = cId;
		}
		

		//E Merge 'new' and 'keep' pairs
		numCurrentPairs = 0;
		for(PfxUInt32 i=0;i<numOutKeepPairs;i++) {
			currentPairs[numCurrentPairs++] = outKeepPairs[i];
		}
		for(PfxUInt32 i=0;i<numOutNewPairs;i++) {
			currentPairs[numCurrentPairs++] = outNewPairs[i];
		}
		
		pool.deallocate(decomposePairsParam.pairBuff);
		pool.deallocate(findPairsParam.pairBuff);
	}
	
	{
		int workBytes = sizeof(PfxBroadphasePair) * numCurrentPairs;
		void *workBuff = pool.allocate(workBytes);
		
		pfxParallelSort(currentPairs,numCurrentPairs,workBuff,workBytes);
		
		pool.deallocate(workBuff);
	}
}


PfxInt32 MyUpdateBroadphaseProxy(PfxBroadphaseProxy& proxy,int rigidbodyId,const btBroadphaseProxy* bulletProxy, const PfxVector3& worldCenter,const PfxVector3& worldExtent,PfxUInt32 axis)
{
	SCE_PFX_ALWAYS_ASSERT(axis<3);
	
	PfxInt32 ret = SCE_PFX_OK;
	
	PfxVector3 minRig = getVmVector3(bulletProxy->m_aabbMin);
	PfxVector3 maxRig = getVmVector3(bulletProxy->m_aabbMax);
	
	PfxVecInt3 aabbMin,aabbMax;
	pfxConvertCoordWorldToLocal(worldCenter,worldExtent,minRig,maxRig,aabbMin,aabbMax);
	
	pfxSetXMin(proxy,aabbMin.getX());
	pfxSetXMax(proxy,aabbMax.getX());
	pfxSetYMin(proxy,aabbMin.getY());
	pfxSetYMax(proxy,aabbMax.getY());
	pfxSetZMin(proxy,aabbMin.getZ());
	pfxSetZMax(proxy,aabbMax.getZ());
	pfxSetKey(proxy,aabbMin.get(axis));
	pfxSetObjectId(proxy,rigidbodyId);
	pfxSetMotionMask(proxy, kPfxMotionTypeActive);
	pfxSetSelf(proxy,bulletProxy->m_collisionFilterGroup);
	pfxSetTarget(proxy,bulletProxy->m_collisionFilterMask);
	
	return ret;
}

void	btLowLevelBroadphase::calculateOverlappingPairs(btDispatcher* dispatcher)
{

	//set the broadphase proxies

	btAlignedObjectArray<PfxBroadphaseProxy> proxies;
	proxies.reserve(m_LastHandleIndex);

	//E Find the axis along which all rigid bodies are most widely positioned
	int axis = 0;
	int i;
	PfxVector3 s(0.0f),s2(0.0f);

	PfxVector3 worldMin(-1000);//PFX_FLT_MAX);
	PfxVector3 worldMax(1000);//-PFX_FLT_MAX);
	
	int numRigidBodies = 0;
	for (i=0; i <= m_LastHandleIndex; i++)
	{
		btLowLevelBroadphaseProxy* proxy0 = &m_pHandles[i];
		if(!proxy0->m_clientObject)
		{
			continue;
		}
		
		PfxVector3 pe_pos = getVmVector3(0.5f*(proxy0->m_aabbMax+proxy0->m_aabbMin));
		PfxVector3 pe_min = getVmVector3(proxy0->m_aabbMin);
		PfxVector3 pe_max = getVmVector3(proxy0->m_aabbMax);
		numRigidBodies++;
		//worldMin = minPerElem(worldMin,pe_min);
		//worldMax = maxPerElem(worldMax,pe_max);

		s += pe_pos;
		s2 += mulPerElem(pe_pos,pe_pos);
	}

	
	if (numRigidBodies)
	{
		PfxVector3 v = s2 - mulPerElem(s,s) / (float)numRigidBodies;
		if(v[1] > v[0]) 
			axis = 1;
		if(v[2] > v[axis]) 
			axis = 2;
	}

	PfxVector3 worldCenter = 0.5f*(worldMax+worldMin);
	PfxVector3 worldExtent = 0.5f*(worldMax-worldMin);

	for (i=0; i <= m_LastHandleIndex; i++)
	{
		btLowLevelBroadphaseProxy* proxy0 = &m_pHandles[i];
		if(!proxy0->m_clientObject)
		{
			continue;
		}

		PfxBroadphaseProxy& proxy = proxies.expandNonInitializing();
		MyUpdateBroadphaseProxy(proxy,proxy0->m_uniqueId,proxy0,worldCenter,worldExtent,axis);

	}

	//find pairs, and call 'addOverlappingPair' for new pairs and 'removeOverlappingPair' for removed pairs
	broadphase(&proxies[0],proxies.size(),axis, dispatcher);
}


//
btOverlappingPairCache*			btLowLevelBroadphase::getOverlappingPairCache()
{
	return(m_paircache);
}

//
const btOverlappingPairCache*	btLowLevelBroadphase::getOverlappingPairCache() const
{
	return(m_paircache);
}


void							btLowLevelBroadphase::getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const
{
	aabbMin = m_broadphaseAabbMin;
	aabbMax = m_broadphaseAabbMax;
}

void	btLowLevelBroadphase::printStats()
{
}

void	btLowLevelBroadphase::setNumTasks(int numTasks)
{
}
