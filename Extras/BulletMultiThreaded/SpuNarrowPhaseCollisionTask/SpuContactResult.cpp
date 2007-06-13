/* [SCE CONFIDENTIAL DOCUMENT]
 * PLAYSTATION(R)3 SPU Optimized Bullet Physics Library (http://bulletphysics.com)
 *                Copyright (C) 2007 Sony Computer Entertainment Inc.
 *                                                All Rights Reserved.
 */

#include "SpuContactResult.h"


//#define DEBUG_SPU_COLLISION_DETECTION 1


#include "SpuContactResult.h"


SpuContactResult::SpuContactResult()
{
	m_manifoldAddress = 0;
	m_spuManifold = NULL;
	m_RequiresWriteBack = false;
}

 SpuContactResult::~SpuContactResult()
{
	g_manifoldDmaExport.swapBuffers();
}

 void	SpuContactResult::setContactInfo(btPersistentManifold* spuManifold, uint64_t	manifoldAddress,const btTransform& worldTrans0,const btTransform& worldTrans1)
 {
//	spu_printf("SpuContactResult::setContactInfo\n");
	m_rootWorldTransform0 = worldTrans0;
	m_rootWorldTransform1 = worldTrans1;
	m_manifoldAddress = manifoldAddress;
    m_spuManifold = spuManifold;
 }

 void SpuContactResult::setShapeIdentifiers(int partId0,int index0,	int partId1,int index1)
 {
	
 }
	


 ///return true if it requires a dma transfer back
bool ManifoldResultAddContactPoint(const btVector3& normalOnBInWorld,
								   const btVector3& pointInWorld,
								   float depth,
								   btPersistentManifold* manifoldPtr,
								   btTransform& transA,
								   btTransform& transB
								   )
{
	
	float contactTreshold = manifoldPtr->getContactBreakingThreshold();

	//spu_printf("SPU: add contactpoint, depth:%f, contactTreshold %f, manifoldPtr %llx\n",depth,contactTreshold,manifoldPtr);

#ifdef DEBUG_SPU_COLLISION_DETECTION
	spu_printf("SPU: contactTreshold %f\n",contactTreshold);
#endif //DEBUG_SPU_COLLISION_DETECTION
	if (depth > manifoldPtr->getContactBreakingThreshold())
		return false;

	//provide inverses or just calculate?
	btTransform transAInv = transA.inverse();//m_body0->m_cachedInvertedWorldTransform;
	btTransform transBInv= transB.inverse();//m_body1->m_cachedInvertedWorldTransform;

	btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
	btVector3 localA = transAInv(pointA );
	btVector3 localB = transBInv(pointInWorld);
	btManifoldPoint newPt(localA,localB,normalOnBInWorld,depth);

	int insertIndex = manifoldPtr->getCacheEntry(newPt);
	if (insertIndex >= 0)
	{
//		manifoldPtr->replaceContactPoint(newPt,insertIndex);
//		return true;

#ifdef DEBUG_SPU_COLLISION_DETECTION
		spu_printf("SPU: same contact detected, nothing done\n");
#endif //DEBUG_SPU_COLLISION_DETECTION
		// This is not needed, just use the old info! saves a DMA transfer as well
	} else
	{

		newPt.m_combinedFriction = 0.25f;//calculateCombinedFriction(m_body0,m_body1);
		newPt.m_combinedRestitution = 0.0f;//calculateCombinedRestitution(m_body0,m_body1);

		/*
		//potential TODO: SPU callbacks, either immediate (local on the SPU), or deferred
		//User can override friction and/or restitution
		if (gContactAddedCallback &&
			//and if either of the two bodies requires custom material
			 ((m_body0->m_collisionFlags & btCollisionObject::customMaterialCallback) ||
			   (m_body1->m_collisionFlags & btCollisionObject::customMaterialCallback)))
		{
			//experimental feature info, for per-triangle material etc.
			(*gContactAddedCallback)(newPt,m_body0,m_partId0,m_index0,m_body1,m_partId1,m_index1);
		}
		*/
		manifoldPtr->AddManifoldPoint(newPt);
		return true;

	}
	return false;
	
}


void SpuContactResult::writeDoubleBufferedManifold(btPersistentManifold* lsManifold, btPersistentManifold* mmManifold)
{
    memcpy(g_manifoldDmaExport.getFront(),lsManifold,sizeof(btPersistentManifold));

    g_manifoldDmaExport.swapBuffers();
    g_manifoldDmaExport.backBufferDmaPut((uint64_t)mmManifold, sizeof(btPersistentManifold), DMA_TAG(9));
	// Should there be any kind of wait here?  What if somebody tries to use this tag again?  What if we call this function again really soon?
	//no, the swapBuffers does the wait
}

void SpuContactResult::addContactPoint(const btVector3& normalOnBInWorld,const btPoint3& pointInWorld,float depth)
{
//	spu_printf("*** SpuContactResult::addContactPoint: depth = %f\n",depth);

#ifdef DEBUG_SPU_COLLISION_DETECTION
 //   int sman = sizeof(rage::phManifold);
//	spu_printf("sizeof_manifold = %i\n",sman);
#endif //DEBUG_SPU_COLLISION_DETECTION

	btPersistentManifold* localManifold = m_spuManifold;

	btVector3	normalB(normalOnBInWorld.getX(),normalOnBInWorld.getY(),normalOnBInWorld.getZ());
	btVector3	pointWrld(pointInWorld.getX(),pointInWorld.getY(),pointInWorld.getZ());

	//process the contact point
	const bool retVal = ManifoldResultAddContactPoint(normalB,
		pointWrld,
		depth,
		localManifold,
		m_rootWorldTransform0,
		m_rootWorldTransform1
		);
	m_RequiresWriteBack = m_RequiresWriteBack || retVal;
}

void SpuContactResult::flush()
{
	if (m_RequiresWriteBack)
	{
#ifdef DEBUG_SPU_COLLISION_DETECTION
		spu_printf("SPU: Start rage::phManifold Write (Put) DMA\n");
#endif //DEBUG_SPU_COLLISION_DETECTION
	//	spu_printf("writeDoubleBufferedManifold\n");
		writeDoubleBufferedManifold(m_spuManifold, (btPersistentManifold*)m_manifoldAddress);
#ifdef DEBUG_SPU_COLLISION_DETECTION
		spu_printf("SPU: Finished (Put) DMA\n");
#endif //DEBUG_SPU_COLLISION_DETECTION
	}
	m_spuManifold = NULL;
	m_RequiresWriteBack = false;
}


