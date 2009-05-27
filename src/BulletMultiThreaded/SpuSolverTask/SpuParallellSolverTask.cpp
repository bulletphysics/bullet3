/*
Bullet Continuous Collision Detection and Physics Library - Parallel solver
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#define IN_PARALLELL_SOLVER 1


#include "SpuParallellSolverTask.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "../PlatformDefinitions.h"
#include "../SpuFakeDma.h"
#include "LinearMath/btMinMax.h"

// To setup constraints
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"

#ifndef offsetof
#define offsetof(s,m)   (size_t)&reinterpret_cast<const volatile char&>((((s *)0)->m))
#endif

//NOTE! When changing this, make sure the package sizes etc below are updated
#define TEMP_STORAGE_SIZE (150*1024)
#define CONSTRAINT_MAX_SIZE (60*16)

ATTRIBUTE_ALIGNED16(struct) SolverTask_LocalStoreMemory
{
	ATTRIBUTE_ALIGNED16(SpuSolverHash			m_localHash);

	// Data for temporary storage in situations where we just need very few
	ATTRIBUTE_ALIGNED16(btSolverConstraint	m_tempInternalConstr[4]);
	ATTRIBUTE_ALIGNED16(btSolverConstraint		m_tempConstraint[6]);
	ATTRIBUTE_ALIGNED16(btSolverBody			m_tempSPUBodies[2]);
	ATTRIBUTE_ALIGNED16(char					m_tempRBs[2][sizeof(btRigidBody)]);
	ATTRIBUTE_ALIGNED16(char					m_externalConstraint[CONSTRAINT_MAX_SIZE]);

	// The general temporary storage, "dynamically" allocated
	ATTRIBUTE_ALIGNED16(uint8_t					m_temporaryStorage[TEMP_STORAGE_SIZE]);
	size_t										m_temporaryStorageUsed;

	ATTRIBUTE_ALIGNED16(float					m_appliedImpulse[4]);
};



#if defined(__CELLOS_LV2__) || defined (LIBSPE2)

ATTRIBUTE_ALIGNED16(SolverTask_LocalStoreMemory	gLocalStoreMemory);

void* createSolverLocalStoreMemory()
{
	return &gLocalStoreMemory;
}
#else
void* createSolverLocalStoreMemory()
{
        return btAlignedAlloc(sizeof(SolverTask_LocalStoreMemory),16);
}

#endif






//-- MEMORY MANAGEMENT HELPERS
size_t memTemporaryStorage (SolverTask_LocalStoreMemory* lsmem)
{
	return TEMP_STORAGE_SIZE - lsmem->m_temporaryStorageUsed;
}

void setupTemporaryStorage (SolverTask_LocalStoreMemory* lsmem)
{
	lsmem->m_temporaryStorageUsed = 0;
}

void* allocTemporaryStorage (SolverTask_LocalStoreMemory* lsmem, size_t size)
{
	// Align size to even 16-byte interval to make it DMA-able
	size = (size+0xf)&~0xf;

	//btAssert(lsmem->m_temporaryStorageUsed + size <= TEMP_STORAGE_SIZE);

	void *res = &lsmem->m_temporaryStorage[lsmem->m_temporaryStorageUsed];
	lsmem->m_temporaryStorageUsed += size;
	return res;
}

void freeTemporaryStorage (SolverTask_LocalStoreMemory* lsmem, void* ptr, size_t size)
{
	// Align size to even 16-byte interval to make it DMA-able
	size = (size+0xf)&~0xf;

	// Only works if we free the last gotten allocation
	//btAssert(&lsmem->m_temporaryStorage[lsmem->m_temporaryStorageUsed - size] == ptr);

	lsmem->m_temporaryStorageUsed -= size;
}

btSolverBody* allocBodyStorage (SolverTask_LocalStoreMemory* lsmem, size_t numBodies)
{
	int sb = sizeof(btSolverBody);
	return static_cast<btSolverBody*> (allocTemporaryStorage(lsmem, sb*numBodies));
}

void freeBodyStorage (SolverTask_LocalStoreMemory* lsmem, btSolverBody* ptr, size_t numBodies)
{
	freeTemporaryStorage(lsmem, ptr, sizeof(btSolverBody)*numBodies);
}

btSolverConstraint* allocInternalConstraintStorage (SolverTask_LocalStoreMemory* lsmem, size_t numConstr)
{
	return static_cast<btSolverConstraint*> (allocTemporaryStorage(lsmem, sizeof(btSolverConstraint)*numConstr));
}

void freeInternalConstraintStorage (SolverTask_LocalStoreMemory* lsmem, btSolverConstraint* ptr, size_t numConstr)
{
	freeTemporaryStorage(lsmem, ptr, sizeof(btSolverConstraint)*numConstr);
}

btSolverConstraint* allocConstraintStorage (SolverTask_LocalStoreMemory* lsmem, size_t numConstr)
{
	return static_cast<btSolverConstraint*> (allocTemporaryStorage(lsmem, sizeof(btSolverConstraint)*numConstr));
}

void freeConstraintStorage (SolverTask_LocalStoreMemory* lsmem, btSolverConstraint* ptr, size_t numConstr)
{
	freeTemporaryStorage(lsmem, ptr, sizeof(btSolverConstraint)*numConstr);
}
//-- MEMORY MANAGEMENT HELPERS END










//-- INDEX SET HELPER
class SpuIndexSet
{
public:

	SIMD_FORCE_INLINE SpuIndexSet (uint32_t* a)
		: m_backingArray (a), m_size (0)
	{}

	SIMD_FORCE_INLINE int insert (uint32_t data)
	{
		int pos = 0;
		for (pos = 0; pos < m_size; ++pos)
		{
			if (m_backingArray[pos] == data)
			{
				return pos;
			}
		}
		//btAssert(m_size < SPU_MAX_BODIES_PER_CELL);

		m_backingArray[m_size] = data;
		return m_size++;
	}

	SIMD_FORCE_INLINE void clear ()
	{
		m_size = 0;
	}

	SIMD_FORCE_INLINE const uint32_t& operator[](int n) const
	{
		return m_backingArray[n];
	}

	SIMD_FORCE_INLINE uint32_t& operator[](int n)
	{
		return m_backingArray[n];
	}

	SIMD_FORCE_INLINE	int size() const
	{	// return length of sequence
		return m_size;
	}

private:
	uint32_t*	m_backingArray;
	int			m_size;
};
//-- INDEX SET HELPER END







#include "BulletDynamics/ConstraintSolver/btSolverBody.h"

//-- RB HANDLING
static void setupSpuBody (btCollisionObject* collisionObject, btSolverBody* solverBody)
{
	btRigidBody* rb = collisionObject? btRigidBody::upcast(collisionObject) : 0;

	solverBody->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
	solverBody->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);

	if (rb)
	{
		solverBody->m_invMass.setValue(rb->getInvMass(),rb->getInvMass(),rb->getInvMass());
		solverBody->m_originalBody = rb;
		solverBody->m_angularFactor = rb->getAngularFactor();
	} else
	{
		solverBody->m_invMass.setValue(0,0,0);
		solverBody->m_originalBody = 0;
		solverBody->m_angularFactor.setValue(1,1,1);
	}
	
}
//-- RB HANDLING END


//-- HASH HANDLING
static void writeTaskFlag(SpuSolverHash* hashRemote, uint32_t taskId, uint32_t* flags)
{
	int dmaSize = sizeof(uint32_t)*SPU_HASH_NUMCELLDWORDS;
	uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (hashRemote);
	dmaPpuAddress2 += offsetof(SpuSolverHash, m_currentMask);
	dmaPpuAddress2 += sizeof(uint32_t) * SPU_HASH_NUMCELLDWORDS * (taskId + 1);
	cellDmaLargePut(flags, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
	cellDmaWaitTagStatusAll(DMA_MASK(1));

}

static void updateLocalMask(SolverTask_LocalStoreMemory* localMemory, SpuSolverTaskDesc& taskDesc)
{
	int dmaSize = sizeof(uint32_t)*(SPU_MAX_SPUS+1)*SPU_HASH_NUMCELLDWORDS;
	uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverHash);
	dmaPpuAddress2 += offsetof(SpuSolverHash, m_currentMask);

	cellDmaLargeGet(&localMemory->m_localHash.m_currentMask, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
	cellDmaWaitTagStatusAll(DMA_MASK(1));
}

static unsigned int getZeroIndex(unsigned int start, uint32_t* mask, uint32_t* finished,  int numRegs)
{
	// Find the index of some zero within mask|finished 
	unsigned int index = start;

	int reg = (start >> 5);
	{
		unsigned int bit = 1 << (start & 31);

		uint32_t combinedMask = mask[reg] | finished[reg];
		for (int bitCnt = (start & 31); bitCnt < SPU_HASH_WORDWIDTH; ++bitCnt, ++index, bit <<= 1)
		{
			if ((combinedMask & bit) == 0)
			{
				return index;
			}
		}

		reg++;
	}

	for (; reg < numRegs; ++reg)
	{
		unsigned int bit = 1;
		uint32_t combinedMask = mask[reg] | finished[reg];

		for (int bitCnt = 0; bitCnt < SPU_HASH_WORDWIDTH; ++bitCnt, ++index, bit <<= 1)
		{
			if ((combinedMask & bit) == 0)
			{
				return index;
			}
		}
	}

	return SPU_HASH_NUMCELLS;
}

static bool isAllOne (uint32_t* mask, int numRegs)
{
	uint32_t totalMask = ~0;
	for (int reg = 0; reg < numRegs; ++reg)
	{
		totalMask &= mask[reg];
	}

	return totalMask == ~0;
}

static bool checkDependency( int tryIndex, uint32_t* mask, uint32_t matrix[SPU_HASH_NUMCELLS][SPU_HASH_NUMCELLDWORDS],  int numRegs)
{
	for (int reg = 0; reg < numRegs; ++reg)
	{
		if ((mask[reg] & matrix[tryIndex][reg]) != 0)
		{
			//Dependency conflict, no-go
			return false;
		}
	}

	return true;
}

static int getNextFreeCell(SolverTask_LocalStoreMemory* localMemory, SpuSolverTaskDesc& taskDesc, btSpinlock& lock)
{
	int cellIndex = SPU_HASH_NUMCELLS;

	uint32_t myMask[SPU_HASH_NUMCELLDWORDS] = {0};
	
	writeTaskFlag(taskDesc.m_solverData.m_solverHash, taskDesc.m_taskId, myMask);
	SpuSolverHash* hash = &localMemory->m_localHash;

	// locking
	lock.Lock();		

	bool stopLoop = false;
	while (!stopLoop)
	{

		// Try to find a free cell
		uint32_t tmpMask[SPU_HASH_NUMCELLDWORDS] = {0};

		updateLocalMask(localMemory, taskDesc);
		

		// Or together the masks of finished cells and all currently locked cells
		for (int row = 1; row <= SPU_MAX_SPUS; ++row)
		{
			for (int reg = 0; reg < SPU_HASH_NUMCELLDWORDS; ++reg)
			{
				tmpMask[reg] |= hash->m_currentMask[row][reg];
			}
		}

		// Find first zero, starting with offset
		int tryIndex;
		int start = 0;
		bool haveTry = false;
		while (!haveTry)
		{
			tryIndex = getZeroIndex(start, tmpMask, hash->m_currentMask[0], SPU_HASH_NUMCELLDWORDS);

			if (tryIndex >= SPU_HASH_NUMCELLS)
				break;

			haveTry = checkDependency(tryIndex, tmpMask, hash->m_dependencyMatrix, SPU_HASH_NUMCELLDWORDS);
			start = tryIndex+1;
		}
		
		if (tryIndex < SPU_HASH_NUMCELLS)
		{
			// If we get here there is no dependency conflict, so lets use it
			cellIndex = tryIndex;
			writeTaskFlag(taskDesc.m_solverData.m_solverHash, taskDesc.m_taskId, hash->m_dependencyMatrix[cellIndex]);

			hash->m_currentMask[0][cellIndex >> 5] |= (1 << (cellIndex & 31));

			{
				int dmaSize = sizeof(uint32_t)*SPU_HASH_NUMCELLDWORDS;
				uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverHash);
				dmaPpuAddress2 += offsetof(SpuSolverHash, m_currentMask);

				cellDmaLargePut(&hash->m_currentMask, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
			}

			stopLoop = true;
		}

		// Check if there are at all any cells left
		if (isAllOne (hash->m_currentMask[0], SPU_HASH_NUMCELLDWORDS))
		{
			//lock.Unlock();
			break;
		}
		
	}

	// unlock
	lock.Unlock();
	

	return cellIndex;
}
//-- HASH HANDLING END

#ifdef BT_USE_SSE
#include <emmintrin.h>
#define vec_splat(x, e) _mm_shuffle_ps(x, x, _MM_SHUFFLE(e,e,e,e))
static inline __m128 _vmathVfDot3( __m128 vec0, __m128 vec1 )
{
    __m128 result = _mm_mul_ps( vec0, vec1);
    return _mm_add_ps( vec_splat( result, 0 ), _mm_add_ps( vec_splat( result, 1 ), vec_splat( result, 2 ) ) );
}
#endif//USE_SIMD


static void SpuResolveSingleConstraintRowGeneric(btSolverBody& body1,btSolverBody& body2,const btSolverConstraint& c)
{
#ifdef BT_USE_SSE
	__m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
	__m128	lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
	__m128	upperLimit1 = _mm_set1_ps(c.m_upperLimit);
	__m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse),_mm_set1_ps(c.m_cfm)));
	__m128 deltaVel1Dotn	=	_mm_add_ps(_vmathVfDot3(c.m_contactNormal.mVec128,body1.m_deltaLinearVelocity.mVec128), _vmathVfDot3(c.m_relpos1CrossNormal.mVec128,body1.m_deltaAngularVelocity.mVec128));
	__m128 deltaVel2Dotn	=	_mm_add_ps(_vmathVfDot3(c.m_contactNormal.mVec128,body2.m_deltaLinearVelocity.mVec128) ,_vmathVfDot3(c.m_relpos2CrossNormal.mVec128,body2.m_deltaAngularVelocity.mVec128));
	__m128 delta_rel_vel	=	_mm_sub_ps(deltaVel1Dotn,deltaVel2Dotn);
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	deltaImpulse	=	_mm_add_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
	btSimdScalar resultLowerLess,resultUpperLess;
	resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
	resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
	__m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
	deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
	c.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
	__m128 upperMinApplied = _mm_sub_ps(upperLimit1,cpAppliedImp);
	deltaImpulse = _mm_or_ps( _mm_and_ps(resultUpperLess, deltaImpulse), _mm_andnot_ps(resultUpperLess, upperMinApplied) );
	c.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultUpperLess, c.m_appliedImpulse), _mm_andnot_ps(resultUpperLess, upperLimit1) );
	__m128	linearComponentA = _mm_mul_ps(c.m_contactNormal.mVec128,body1.m_invMass.mVec128);
	__m128	linearComponentB = _mm_mul_ps(c.m_contactNormal.mVec128,body2.m_invMass.mVec128);
	__m128 impulseMagnitude = deltaImpulse;
	body1.m_deltaLinearVelocity.mVec128 = _mm_add_ps(body1.m_deltaLinearVelocity.mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
	body1.m_deltaAngularVelocity.mVec128 = _mm_add_ps(body1.m_deltaAngularVelocity.mVec128 ,_mm_mul_ps(c.m_angularComponentA.mVec128,impulseMagnitude));
	body2.m_deltaLinearVelocity.mVec128 = _mm_sub_ps(body2.m_deltaLinearVelocity.mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
	body2.m_deltaAngularVelocity.mVec128 = _mm_sub_ps(body2.m_deltaAngularVelocity.mVec128 ,_mm_mul_ps(c.m_angularComponentB.mVec128,impulseMagnitude));
#else
	btScalar deltaImpulse = c.m_rhs-btScalar(c.m_appliedImpulse)*c.m_cfm;
	const btScalar deltaVel1Dotn	=	c.m_contactNormal.dot(body1.m_deltaLinearVelocity) 	+ c.m_relpos1CrossNormal.dot(body1.m_deltaAngularVelocity);
	const btScalar deltaVel2Dotn	=	c.m_contactNormal.dot(body2.m_deltaLinearVelocity) 	+ c.m_relpos2CrossNormal.dot(body2.m_deltaAngularVelocity);
	const btScalar delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
	deltaImpulse	-=	deltaVel1Dotn*c.m_jacDiagABInv;
	deltaImpulse	+=	deltaVel2Dotn*c.m_jacDiagABInv;
	const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
	if (sum < c.m_lowerLimit)
	{
		deltaImpulse = c.m_lowerLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_lowerLimit;
	}
	else if (sum > c.m_upperLimit) 
	{
		deltaImpulse = c.m_upperLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_upperLimit;
	}
	else
	{
		c.m_appliedImpulse = sum;
	}
	if (body1.m_invMass)
		body1.applyImpulse(c.m_contactNormal*body1.m_invMass,c.m_angularComponentA,deltaImpulse);
	if (body2.m_invMass)
		body2.applyImpulse(c.m_contactNormal*body2.m_invMass,c.m_angularComponentB,-deltaImpulse);
#endif
}




#ifdef NOT_YET
// Constraint solving
static void solveConstraint (btSolverConstraint& constraint, btSolverBody& bodyA, btSolverBody& bodyB)
{
	// All but D6 use worldspace normals, use same code
	if (constraint.m_flags.m_useLinear)
	{
		if (constraint.m_constraintType == POINT2POINT_CONSTRAINT_TYPE ||
			constraint.m_constraintType == HINGE_CONSTRAINT_TYPE ||
			constraint.m_constraintType == CONETWIST_CONSTRAINT_TYPE)
		{
			btVector3 normal (0,0,0);

			const btVector3& bias =constraint.m_linearBias;
			const btVector3& jacInv =constraint.m_jacdiagABInv;

			for (int i = 0; i < 3; ++i)
			{
				normal[i] = 1;

				// Compute relative velocity
				btVector3 vel1 = bodyA.m_linearVelocity + bodyA.m_angularVelocity.cross(constraint.m_relPos1);
				btVector3 vel2 = bodyB.m_linearVelocity + bodyB.m_angularVelocity.cross(constraint.m_relPos2);
				btVector3 vel = vel1 - vel2;

				float relVelNormal = normal.dot(vel);

				// Compute impulse
				float impulse = (bias[i] - relVelNormal) * jacInv[i];

				btVector3 impNormal = normal*impulse;

				// Apply
				if (bodyA.m_invMass > 0)
				{
					bodyA.m_linearVelocity += impNormal*bodyA.m_invMass;
					bodyA.m_angularVelocity += bodyA.m_angularFactor * (bodyA.m_worldInvInertiaTensor * (btVector3(constraint.m_relPos1).cross(impNormal)));
				}
				if (bodyB.m_invMass > 0)
				{
					bodyB.m_linearVelocity -= impNormal*bodyB.m_invMass;
					bodyB.m_angularVelocity -=  bodyB.m_angularFactor * (bodyB.m_worldInvInertiaTensor * (btVector3(constraint.m_relPos2).cross(impNormal)));
				}

				normal[i] = 0;
			}
		}
		else
		{
			//D6
		}

	}

	switch (constraint.m_constraintType)
	{
	case POINT2POINT_CONSTRAINT_TYPE:
		break; // Nothing special to do
	case HINGE_CONSTRAINT_TYPE:
		{
			// Angular solving for the two first axes
			const btVector3& bias =constraint.hinge.m_angularBias;
			const btVector3& jacInv =constraint.hinge.m_angJacdiagABInv;

			for (int i = 0; i < 2; ++i)
			{
				const btVector3& axis =constraint.hinge.m_frameAinW[i];
				
				// Compute relative velocity
				btVector3 relVel = bodyA.m_angularVelocity - bodyB.m_angularVelocity;

				float relVelAxis = axis.dot(relVel);

				// Compute impulse
				float impulse = (bias[i] - relVelAxis) * jacInv[i];
				btVector3 impAxis = axis*impulse;

				// Apply
				if (bodyA.m_invMass > 0)
				{
					bodyA.m_angularVelocity +=  bodyA.m_angularFactor * (bodyA.m_worldInvInertiaTensor * impAxis);
				}
				if (bodyB.m_invMass > 0)
				{
					bodyB.m_angularVelocity -= bodyB.m_angularFactor * (bodyB.m_worldInvInertiaTensor * impAxis);
				}
			}

			// Limit
			if (constraint.m_flags.m_limit1)
			{
				const btVector3& axis =constraint.hinge.m_frameAinW[2];

				// Compute relative velocity
				btVector3 relVel = bodyA.m_angularVelocity - bodyB.m_angularVelocity;
				float relVelAxis = axis.dot(relVel);
				
				// Compute impulse
				float impulse = (bias[2] - relVelAxis) * jacInv[2] * constraint.hinge.m_limitJacFactor;

				// Clamp it
				float temp = constraint.hinge.m_limitAccumulatedImpulse;
				constraint.hinge.m_limitAccumulatedImpulse = btMax (constraint.hinge.m_limitAccumulatedImpulse + impulse, 0.0f);
				impulse = constraint.hinge.m_limitAccumulatedImpulse - temp;

				btVector3 impAxis = axis*impulse* (constraint.hinge.m_limitJacFactor/btFabs (constraint.hinge.m_limitJacFactor));

				// Apply
				if (bodyA.m_invMass > 0)
				{
					bodyA.m_angularVelocity += bodyA.m_angularFactor * (bodyA.m_worldInvInertiaTensor * impAxis);
				}
				if (bodyB.m_invMass > 0)
				{
					bodyB.m_angularVelocity -= bodyB.m_angularFactor * (bodyB.m_worldInvInertiaTensor * impAxis);
				}				
			}

			// Motor
			if (constraint.m_flags.m_motor1)
			{
				const btVector3& axis =constraint.hinge.m_frameAinW[2];

				// Compute relative velocity
				btVector3 relVel = bodyA.m_angularVelocity - bodyB.m_angularVelocity;
				float relVelAxis = axis.dot(relVel);

				// Compute impulse
				float impulse = (constraint.hinge.m_motorVelocity - relVelAxis) * jacInv[2];

				// Clamp it
				float clampedImpulse = impulse > constraint.hinge.m_motorImpulse ? constraint.hinge.m_motorImpulse : impulse;
				clampedImpulse = impulse < -constraint.hinge.m_motorImpulse ? -constraint.hinge.m_motorImpulse : clampedImpulse;
				

				btVector3 impAxis = axis*clampedImpulse;

				// Apply
				if (bodyA.m_invMass > 0)
				{
					bodyA.m_angularVelocity += bodyA.m_angularFactor * (bodyA.m_worldInvInertiaTensor * impAxis);
				}
				if (bodyB.m_invMass > 0)
				{
					bodyB.m_angularVelocity -= bodyB.m_angularFactor * (bodyB.m_worldInvInertiaTensor * impAxis);
				}
			}
		}
		break;
	case CONETWIST_CONSTRAINT_TYPE:
		{
			// Swing
			if (constraint.m_flags.m_limit1)
			{
				const btVector3& axis =constraint.conetwist.m_swingAxis;

				// Compute relative velocity
				btVector3 relVel = bodyA.m_angularVelocity - bodyB.m_angularVelocity;
				float relVelAxis = axis.dot(relVel);

				// Compute impulse
				float impulse = (constraint.conetwist.m_swingError - relVelAxis) * constraint.conetwist.m_swingJacInv;

				// Clamp it
				float temp = constraint.conetwist.m_swingLimitImpulse;
				constraint.conetwist.m_swingLimitImpulse = btMax (constraint.conetwist.m_swingLimitImpulse + impulse, 0.0f);
				impulse = constraint.conetwist.m_swingLimitImpulse - temp;

				btVector3 impAxis = axis*impulse;

				// Apply
				if (bodyA.m_invMass > 0)
				{
					bodyA.m_angularVelocity += bodyA.m_angularFactor * (bodyA.m_worldInvInertiaTensor * impAxis);
				}
				if (bodyB.m_invMass > 0)
				{
					bodyB.m_angularVelocity -= bodyB.m_angularFactor * (bodyB.m_worldInvInertiaTensor * impAxis);
				}
			}

			// Twist
			if (constraint.m_flags.m_limit2)
			{
				const btVector3& axis =constraint.conetwist.m_twistAxis;

				// Compute relative velocity
				btVector3 relVel = bodyA.m_angularVelocity - bodyB.m_angularVelocity;
				float relVelAxis = axis.dot(relVel);

				// Compute impulse
				float impulse = (constraint.conetwist.m_twistError - relVelAxis) * constraint.conetwist.m_twistJacInv;

				// Clamp it
				float temp = constraint.conetwist.m_twistLimitImpulse;
				constraint.conetwist.m_twistLimitImpulse = btMax (constraint.conetwist.m_twistLimitImpulse + impulse, 0.0f);
				impulse = constraint.conetwist.m_twistLimitImpulse - temp;

				btVector3 impAxis = axis*impulse;

				// Apply
				if (bodyA.m_invMass > 0)
				{
					bodyA.m_angularVelocity += bodyA.m_angularFactor * (bodyA.m_worldInvInertiaTensor * impAxis);
				}
				if (bodyB.m_invMass > 0)
				{
					bodyB.m_angularVelocity -= bodyB.m_angularFactor * (bodyB.m_worldInvInertiaTensor * impAxis);
				}	
			}
		}
		break;
	default:
		;
	}
}
//-- SOLVER METHODS END


#endif //NOT_YET





//-- CONSTRAINT SETUP METHODS
/// Compute the jacobian inverse 
///@todo: Optimize
static float computeJacobianInverse (const btRigidBody* rb0, const btRigidBody* rb1,
							  const btVector3& anchorAinW, const btVector3& anchorBinW, const btVector3& normal)
{
	float jacobian = rb0->computeImpulseDenominator(anchorAinW, normal);
	jacobian += rb1->computeImpulseDenominator(anchorBinW, normal);

	return 1.0f/jacobian;
}

static float computeAngularJacobianInverse (const btRigidBody* rb0, const btRigidBody* rb1,
											const btVector3& normal)
{
	float jacobian = rb0->computeAngularImpulseDenominator(normal);
	jacobian += rb1->computeAngularImpulseDenominator(normal);

	return 1.0f/jacobian;
}

/*static void setupLinearConstraintWorld (btSolverConstraint& constraint, const btRigidBody* rb0, const btRigidBody* rb1,
										const btVector3& anchorAinW, const btVector3& anchorBinW, const btContactSolverInfoData& solverInfo)
{
	btVector3 relPos1 = anchorAinW - rb0->getCenterOfMassPosition();
	btVector3 relPos2 = anchorBinW - rb1->getCenterOfMassPosition();


	btVector3 error = anchorAinW - anchorBinW;

	// Setup the three axes
	btVector3 normal (0,0,0);
	btVector3 jacInv, bias;
	const float errorFactor = solverInfo.m_tau / (solverInfo.m_timeStep * solverInfo.m_damping);

	for (int i = 0; i < 3; ++i)
	{
		normal[i] = 1;

		jacInv[i] = solverInfo.m_damping * computeJacobianInverse (rb0, rb1, anchorAinW, anchorBinW, normal);

		// Compute the depth
		float depth = -error[i]*errorFactor;
		bias[i] = depth;

		normal[i] = 0;
	}

	constraint.m_jacdiagABInv = jacInv;
	constraint.m_linearBias = bias;
	constraint.m_flags.m_useLinear = 1;
}
//-- CONSTRAINT SETUP METHODS END

*/


#ifdef NOT_YET
void	setupConstraint(btSolverConstraint* currentConstraintRow,btRigidBody* rb0,btSolverBody& bodyA,btRigidBody* rb1,btSolverBody& bodyB,const btVector3& pivotAinW,const btVector3& pivotBinW,const btContactSolverInfoData& infoGlobal)
{
	return;
	int j;

	for (j=0;j<3;j++)
	{
		memset(&currentConstraintRow[j],0,sizeof(btSolverConstraint));
		currentConstraintRow[j].m_lowerLimit = -FLT_MAX;
		currentConstraintRow[j].m_upperLimit = FLT_MAX;
		currentConstraintRow[j].m_appliedImpulse = 0.f;
		currentConstraintRow[j].m_penetration = 0.f;
		currentConstraintRow[j].m_appliedPushImpulse = 0.f;

	}

	bodyA.m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
	bodyA.m_deltaAngularVelocity.setValue(0.f,0.f,0.f);
	bodyB.m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
	bodyB.m_deltaAngularVelocity.setValue(0.f,0.f,0.f);
					


	btTypedConstraint::btConstraintInfo2 info2;
	info2.fps = 1.f/infoGlobal.m_timeStep;
	info2.erp = infoGlobal.m_erp;
	info2.m_J1linearAxis = currentConstraintRow->m_contactNormal;
	info2.m_J1angularAxis = currentConstraintRow->m_relpos1CrossNormal;
	info2.m_J2linearAxis = 0;
	info2.m_J2angularAxis = currentConstraintRow->m_relpos2CrossNormal;
	info2.rowskip = sizeof(btSolverConstraint)/sizeof(btScalar);//check this
	info2.m_constraintError = &currentConstraintRow->m_rhs;
	info2.cfm = &currentConstraintRow->m_cfm;
	info2.m_lowerLimit = &currentConstraintRow->m_lowerLimit;
	info2.m_upperLimit = &currentConstraintRow->m_upperLimit;

	btTypedConstraint::btConstraintInfo2* info = &info2;
	
	 //retrieve matrices
	btTransform body0_trans;
	body0_trans = rb0->getCenterOfMassTransform();
    btTransform body1_trans;
	body1_trans = rb1->getCenterOfMassTransform();

	// anchor points in global coordinates with respect to body PORs.
   
    // set jacobian
    info->m_J1linearAxis[0] = 1;
    info->m_J1linearAxis[info->rowskip+1] = 1;
    info->m_J1linearAxis[2*info->rowskip+2] = 1;

	btVector3 a1 = body0_trans.getBasis()*pivotAinW;
	{
		btVector3* angular0 = (btVector3*)(info->m_J1angularAxis);
		btVector3* angular1 = (btVector3*)(info->m_J1angularAxis+info->rowskip);
		btVector3* angular2 = (btVector3*)(info->m_J1angularAxis+2*info->rowskip);
		btVector3 a1neg = -a1;
		a1neg.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
    
	/*info->m_J2linearAxis[0] = -1;
    info->m_J2linearAxis[s+1] = -1;
    info->m_J2linearAxis[2*s+2] = -1;
	*/
	
	btVector3 a2 = body1_trans.getBasis()*pivotBinW;
   
	{
		btVector3 a2n = -a2;
		btVector3* angular0 = (btVector3*)(info->m_J2angularAxis);
		btVector3* angular1 = (btVector3*)(info->m_J2angularAxis+info->rowskip);
		btVector3* angular2 = (btVector3*)(info->m_J2angularAxis+2*info->rowskip);
		a2.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
    


    // set right hand side
    btScalar k = info->fps * info->erp;

	for (j=0; j<3; j++)
    {
        info->m_constraintError[j*info->rowskip] = k * (a2[j] + body1_trans.getOrigin()[j] -                     a1[j] - body0_trans.getOrigin()[j]);
		//printf("info->m_constraintError[%d]=%f\n",j,info->m_constraintError[j]);
    }

/*	btScalar impulseClamp = m_setting.m_impulseClamp;//
	for (j=0; j<3; j++)
    {
		if (m_setting.m_impulseClamp > 0)
		{
			info->m_lowerLimit[j*info->rowskip] = -impulseClamp;
			info->m_upperLimit[j*info->rowskip] = impulseClamp;
		}
	}
	*/



	///finalize the constraint setup
	for (int j=0;j<3;j++)
	{
		btSolverConstraint& solverConstraint = currentConstraintRow[j];

		{
			const btVector3& ftorqueAxis1 = solverConstraint.m_relpos1CrossNormal;
			solverConstraint.m_angularComponentA = rb0->getInvInertiaTensorWorld()*ftorqueAxis1;
		}
		{
			const btVector3& ftorqueAxis2 = solverConstraint.m_relpos2CrossNormal;
			solverConstraint.m_angularComponentB = rb1->getInvInertiaTensorWorld()*ftorqueAxis2;
		}

		{
			btVector3 iMJlA = solverConstraint.m_contactNormal*rb0->getInvMass();
			btVector3 iMJaA = rb0->getInvInertiaTensorWorld()*solverConstraint.m_relpos1CrossNormal;
			btVector3 iMJlB = solverConstraint.m_contactNormal*rb1->getInvMass();//sign of normal?
			btVector3 iMJaB = rb1->getInvInertiaTensorWorld()*solverConstraint.m_relpos2CrossNormal;
			
			btScalar sum = iMJlA.dot(solverConstraint.m_contactNormal);
			sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
			sum += iMJlB.dot(solverConstraint.m_contactNormal);
			sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);

			solverConstraint.m_jacDiagABInv = btScalar(1.)/sum;
		}


		///fix rhs
		///todo: add force/torque accelerators
		{
			btScalar rel_vel;
			btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(rb0->getLinearVelocity()) + solverConstraint.m_relpos1CrossNormal.dot(rb0->getAngularVelocity());
			btScalar vel2Dotn = solverConstraint.m_contactNormal.dot(rb1->getLinearVelocity()) + solverConstraint.m_relpos2CrossNormal.dot(rb1->getAngularVelocity());

			rel_vel = vel1Dotn-vel2Dotn;

			btScalar positionalError = solverConstraint.m_rhs;//already filled in by getConstraintInfo2
			solverConstraint.m_restitution  = 0.f;
			btScalar	velocityError = solverConstraint.m_restitution - rel_vel;// * damping;
			btScalar	penetrationImpulse = positionalError*solverConstraint.m_jacDiagABInv;
			btScalar	velocityImpulse = velocityError *solverConstraint.m_jacDiagABInv;
			solverConstraint.m_rhs = penetrationImpulse+velocityImpulse;
			solverConstraint.m_appliedImpulse = 0.f;
		
		}
	}
}
#endif //NOT_YET

static int getConstraintSize (btTypedConstraintType type)
{
	switch (type)
	{
	case POINT2POINT_CONSTRAINT_TYPE:
		return sizeof(btPoint2PointConstraint);
	case HINGE_CONSTRAINT_TYPE:
		return sizeof(btHingeConstraint);
	case CONETWIST_CONSTRAINT_TYPE:
		return sizeof(btConeTwistConstraint);
	case D6_CONSTRAINT_TYPE:
		return sizeof(btGeneric6DofConstraint);
	default:
		;
		//btAssert(0);
	}

	return 0;
}









//-- MAIN METHOD
void processSolverTask(void* userPtr, void* lsMemory)
{
//	BT_PROFILE("processSolverTask");

	SolverTask_LocalStoreMemory* localMemory = (SolverTask_LocalStoreMemory*)lsMemory;

	SpuSolverTaskDesc* taskDescPtr = (SpuSolverTaskDesc*)userPtr;
	SpuSolverTaskDesc& taskDesc = *taskDescPtr;

	setupTemporaryStorage(localMemory);
	
	switch (taskDesc.m_solverCommand)
	{
	case CMD_SOLVER_SETUP_BODIES:
		{
			int bodiesToProcess = taskDesc.m_commandData.m_bodySetup.m_numBodies;
			int bodyPackageOffset = taskDesc.m_commandData.m_bodySetup.m_startBody;
			const int bodiesPerPackage = 256;

			btRigidBody** bodyPtrList = (btRigidBody**)allocTemporaryStorage(localMemory, bodiesPerPackage*sizeof(btRigidBody*));
			btRigidBody* bodyList = (btRigidBody*)allocTemporaryStorage(localMemory, bodiesPerPackage*sizeof(btRigidBody));
			btSolverBody* spuBodyList = allocBodyStorage(localMemory, bodiesPerPackage);


			while (bodiesToProcess > 0)
			{
				const int packageSize = bodiesToProcess > bodiesPerPackage ? bodiesPerPackage : bodiesToProcess;

				// DMA the body pointers
				{
					int dmaSize = sizeof(btRigidBody*)*packageSize;
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_commandData.m_bodySetup.m_rbList + bodyPackageOffset);
					cellDmaLargeGet(bodyPtrList, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
					cellDmaWaitTagStatusAll(DMA_MASK(1));
				}

				int b;
				// DMA the rigid bodies
				for ( b = 0; b < packageSize; ++b)
				{
					btRigidBody* body = bodyPtrList[b];
					int dmaSize = sizeof(btRigidBody);
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (body);
					cellDmaLargeGet(&bodyList[b], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);					
				}
				cellDmaWaitTagStatusAll(DMA_MASK(1));

				for ( b = 0; b < packageSize; ++b)
				{					
					btRigidBody* localBody = bodyList+b;
					btSolverBody* spuBody = spuBodyList + b;
					//Set it up solver body
					setupSpuBody(localBody, spuBody);

					int spuBodyIndex = bodyPackageOffset + b;
					localBody->setCompanionId(spuBodyIndex);
				}

				// DMA the rigid bodies back
				for ( b = 0; b < packageSize; ++b)
				{
					btRigidBody* body = bodyPtrList[b];
					int dmaSize = sizeof(btRigidBody);
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (body);
					cellDmaLargePut(&bodyList[b], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);					
				}

				// DMA the list of SPU bodies
				{
					int dmaSize = sizeof(btSolverBody)*packageSize;
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + bodyPackageOffset);
					cellDmaLargePut(spuBodyList, dmaPpuAddress2, dmaSize, DMA_TAG(2), 0, 0);
				}


				cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
				bodiesToProcess -= packageSize;
				bodyPackageOffset += packageSize;				
			}

		}
		break;
	case CMD_SOLVER_MANIFOLD_SETUP:
		{			
			// DMA the hash
			{
				int dmaSize = sizeof(SpuSolverHash);
				uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverHash);
				cellDmaLargeGet(&localMemory->m_localHash, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
			}

			// Iterate over our cells
			const int manifoldsPerPackage = 8;
			const int constraintsPerPackage = 8;

			ManifoldCellHolder* manifoldHolderList = (ManifoldCellHolder*)allocTemporaryStorage(localMemory, sizeof(ManifoldCellHolder)*manifoldsPerPackage);
			btPersistentManifold* manifoldList = (btPersistentManifold*)allocTemporaryStorage(localMemory, sizeof(btPersistentManifold)*manifoldsPerPackage);			

			ConstraintCellHolder* constraintHolderList = (ConstraintCellHolder*)allocTemporaryStorage(localMemory, sizeof(ConstraintCellHolder)*constraintsPerPackage);
			uint8_t* constraintList = (uint8_t*)allocTemporaryStorage(localMemory, CONSTRAINT_MAX_SIZE*constraintsPerPackage);

			uint32_t* indexArray = (uint32_t*)allocTemporaryStorage(localMemory, sizeof(uint32_t)*SPU_MAX_BODIES_PER_CELL);						

			for (unsigned int c = 0; c < taskDesc.m_commandData.m_manifoldSetup.m_numCells; ++c)
			{
				int cellIdx = taskDesc.m_commandData.m_manifoldSetup.m_startCell + c;
				SpuSolverHashCell& hashCell = localMemory->m_localHash.m_Hash[cellIdx];
				
				SpuIndexSet localRBs (indexArray);

				{
					int constraintIndex = hashCell.m_internalConstraintListOffset;
					int manifoldsToProcess = hashCell.m_numManifolds;
					int manifoldPackageOffset = hashCell.m_manifoldListOffset;								

					while (manifoldsToProcess > 0)
					{
						const int packageSize = manifoldsToProcess > manifoldsPerPackage ? manifoldsPerPackage : manifoldsToProcess;

						// DMA the holder list
						{						
							int dmaSize = sizeof(ManifoldCellHolder)*packageSize;						
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_commandData.m_manifoldSetup.m_manifoldHolders + manifoldPackageOffset);
							cellDmaLargeGet(manifoldHolderList, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
							cellDmaWaitTagStatusAll(DMA_MASK(1));
						}

						int m;
						// DMA the manifold list
						for ( m = 0; m < packageSize; ++m)
						{
							int dmaSize = sizeof(btPersistentManifold);
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (manifoldHolderList[m].m_manifold);
							cellDmaLargeGet(manifoldList + m, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);						
						}
						cellDmaWaitTagStatusAll(DMA_MASK(1));

						for ( m = 0; m < packageSize; ++m)
						{
							btPersistentManifold* currManifold = manifoldList + m;

							btRigidBody* rb0Ptr = (btRigidBody*)currManifold->getBody0();
							btRigidBody* rb1Ptr = (btRigidBody*)currManifold->getBody1();

							int numContacts = currManifold->getNumContacts();

							if (!numContacts)
							{
								// No need to DMA anything more or so, so quit							
								continue;
							}

							unsigned int solverBodyIdA = ~0, solverBodyIdB = ~0;

							// DMA the bodies
							{
								int dmaSize = sizeof(btRigidBody);
								uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (rb0Ptr);
								cellDmaLargeGet(&localMemory->m_tempRBs[0], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
							}
							{
								int dmaSize = sizeof(btRigidBody);
								uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (rb1Ptr);
								cellDmaLargeGet(&localMemory->m_tempRBs[1], dmaPpuAddress2, dmaSize, DMA_TAG(2), 0, 0);							
							}
							cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));


							btRigidBody* rb0readonly = (btRigidBody*)&localMemory->m_tempRBs[0];
							btRigidBody* rb1readonly = (btRigidBody*)&localMemory->m_tempRBs[1];

							if (rb0readonly->getIslandTag() >= 0)
							{
								solverBodyIdA = rb0readonly->getCompanionId();

								///DMA back bodyA (with applied impulse)
								{
									int dmaSize = sizeof(btSolverBody);
									uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + solverBodyIdA);
									cellDmaLargeGet(&localMemory->m_tempSPUBodies[0], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
									cellDmaWaitTagStatusAll(DMA_MASK(1));
								}
							} 
							else
							{
								//create a static body
								solverBodyIdA = taskDesc.m_commandData.m_manifoldSetup.m_numBodies + hashCell.m_manifoldListOffset;
								setupSpuBody(rb0readonly, &localMemory->m_tempSPUBodies[0]);
							}

							btSolverBody* solverBodyA = &localMemory->m_tempSPUBodies[0];

							

							if (rb1readonly->getIslandTag() >= 0)
							{
								solverBodyIdB = rb1readonly->getCompanionId();
								///DMA back bodyB (with applied impulse)
								{
									int dmaSize = sizeof(btSolverBody);
									uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + solverBodyIdB);
									cellDmaLargeGet(&localMemory->m_tempSPUBodies[1], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
									cellDmaWaitTagStatusAll(DMA_MASK(1));
								}

							} 
							else
							{
								//create a static body
								solverBodyIdB = taskDesc.m_commandData.m_manifoldSetup.m_numBodies + hashCell.m_manifoldListOffset;					
								setupSpuBody(rb1readonly, &localMemory->m_tempSPUBodies[1]);
							}

							btSolverBody* solverBodyB = &localMemory->m_tempSPUBodies[1];


							// Setup the pointer table
							int offsA = localRBs.insert(solverBodyIdA);		
							int offsB = localRBs.insert(solverBodyIdB);

							// Setup all the contacts
							for (int c = 0; c < numContacts; ++c)
							{
								btManifoldPoint& cp = currManifold->getContactPoint(c);

								btVector3 pos1 = cp.getPositionWorldOnA();
								btVector3 pos2 = cp.getPositionWorldOnB();

								btVector3 rel_pos1 = pos1 - rb0readonly->getCenterOfMassPosition(); 
								btVector3 rel_pos2 = pos2 - rb1readonly->getCenterOfMassPosition();

								btScalar rel_vel;
								btVector3 vel;

								// De-penetration
								{
									btSolverConstraint& constraint = localMemory->m_tempInternalConstr[0];

									{
										uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (manifoldHolderList[m].m_manifold);
										//btManifoldPoint
										int index = offsetof(btManifoldPoint,m_appliedImpulse)+ c* sizeof(btManifoldPoint);
										dmaPpuAddress2+=index;
										constraint.m_originalContactPoint = (void*)dmaPpuAddress2;
									}


									constraint.m_solverBodyIdA = offsA;
									constraint.m_solverBodyIdB = offsB;

									constraint.m_contactNormal = cp.m_normalWorldOnB;
									{
										//can be optimized, the cross products are already calculated										
										//constraint.m_jacDiagABInv = computeJacobianInverse (rb0, rb1, pos1, pos2, cp.m_normalWorldOnB);
									}

									constraint.m_relpos1CrossNormal = rel_pos1.cross(cp.m_normalWorldOnB);
									constraint.m_relpos2CrossNormal = rel_pos2.cross(cp.m_normalWorldOnB);
									btVector3 torqueAxis0 = rel_pos1.cross(cp.m_normalWorldOnB);
									constraint.m_angularComponentA = rb0readonly->getInvInertiaTensorWorld()*torqueAxis0;
									btVector3 torqueAxis1 = rel_pos2.cross(cp.m_normalWorldOnB);		
									constraint.m_angularComponentB = rb1readonly->getInvInertiaTensorWorld()*torqueAxis1;


									{
										btVector3 vec;
										btScalar denom0 = 0.f;
										btScalar denom1 = 0.f;
										if (rb0readonly)
										{
											vec = ( constraint.m_angularComponentA).cross(rel_pos1);
											denom0 = rb0readonly->getInvMass() + cp.m_normalWorldOnB.dot(vec);
										}
										if (rb1readonly)
										{
											vec = ( constraint.m_angularComponentB).cross(rel_pos2);
											denom1 = rb1readonly->getInvMass() + cp.m_normalWorldOnB.dot(vec);
										}
										
										btScalar denom = 1/(denom0+denom1);
										constraint.m_jacDiagABInv = denom;
									}


									

									//btVector3 vel1 = rb0readonly->getVelocityInLocalPoint(rel_pos1);
									//btVector3 vel2 = rb1readonly->getVelocityInLocalPoint(rel_pos2);
									btVector3 vel1;
									solverBodyA->getVelocityInLocalPointObsolete(rel_pos1,vel1);
									btVector3 vel2;
									solverBodyB->getVelocityInLocalPointObsolete(rel_pos2,vel2);


									vel = vel1 - vel2;
									rel_vel = cp.m_normalWorldOnB.dot(vel);

									btScalar penetration = cp.getDistance();///btScalar(infoGlobal.m_numIterations);
									constraint.m_friction = cp.m_combinedFriction;
									float rest =  - rel_vel * cp.m_combinedRestitution;
									if (rest <= btScalar(0.))
									{
										rest = 0.f;
									};

									 

									btScalar erp = taskDesc.m_commandData.m_manifoldSetup.m_solverInfo.m_erp;
									btScalar timeStep = taskDesc.m_commandData.m_manifoldSetup.m_solverInfo.m_timeStep;
																	
									btScalar restitution = rest;
									constraint.m_appliedImpulse = cp.m_appliedImpulse*taskDesc.m_commandData.m_manifoldSetup.m_solverInfo.m_warmstartingFactor;
									if (constraint.m_appliedImpulse!= 0.f)
									{
										if (solverBodyA)
											solverBodyA->applyImpulse(constraint.m_contactNormal*rb0readonly->getInvMass(),constraint.m_angularComponentA,constraint.m_appliedImpulse);
										if (solverBodyB)
											solverBodyB->applyImpulse(constraint.m_contactNormal*rb1readonly->getInvMass(),constraint.m_angularComponentB,-constraint.m_appliedImpulse);
									}

									{
										btScalar rel_vel;
										btScalar vel1Dotn = constraint.m_contactNormal.dot(rb0readonly?rb0readonly->getLinearVelocity():btVector3(0,0,0)) 
											+ constraint.m_relpos1CrossNormal.dot(rb0readonly?rb0readonly->getAngularVelocity():btVector3(0,0,0));
										btScalar vel2Dotn = constraint.m_contactNormal.dot(rb1readonly?rb1readonly->getLinearVelocity():btVector3(0,0,0)) 
											+ constraint.m_relpos2CrossNormal.dot(rb1readonly?rb1readonly->getAngularVelocity():btVector3(0,0,0));

										rel_vel = vel1Dotn-vel2Dotn;

										btScalar positionalError = 0.f;
										positionalError = -penetration * erp/timeStep;
										btScalar	velocityError = restitution - rel_vel;// * damping;
										btScalar  penetrationImpulse = positionalError*constraint.m_jacDiagABInv;
										btScalar velocityImpulse = velocityError *constraint.m_jacDiagABInv;
										constraint.m_rhs = penetrationImpulse+velocityImpulse;
										constraint.m_cfm = 0.f;
										constraint.m_lowerLimit = 0;
										constraint.m_upperLimit = 1e10f;
									}


									
								}

								// Friction

								btVector3 frictionTangential0a, frictionTangential1b;

								frictionTangential0a = vel - cp.m_normalWorldOnB * rel_vel;
								btScalar lat_rel_vel = frictionTangential0a.length2();
								if (lat_rel_vel > SIMD_EPSILON)//0.0f)
								{
									frictionTangential0a /= btSqrt(lat_rel_vel);
									frictionTangential1b = frictionTangential0a.cross(cp.m_normalWorldOnB);
									frictionTangential1b.normalize();
								} else
								{
									btPlaneSpace1(cp.m_normalWorldOnB,frictionTangential0a,frictionTangential1b);
								}


								{
									btSolverConstraint& constraint = localMemory->m_tempInternalConstr[1];
									constraint.m_originalContactPoint = 0;

									constraint.m_contactNormal = frictionTangential0a;

									constraint.m_solverBodyIdA = offsA;
									constraint.m_solverBodyIdB = offsB;

									constraint.m_friction = cp.m_combinedFriction;

									constraint.m_appliedImpulse = 0;//cp.m_appliedImpulse;//btScalar(0.);

									constraint.m_jacDiagABInv = computeJacobianInverse (rb0readonly, rb1readonly, pos1, pos2, constraint.m_contactNormal);

									{
										btVector3 ftorqueAxis0 = rel_pos1.cross(constraint.m_contactNormal);
										constraint.m_relpos1CrossNormal = ftorqueAxis0;
										constraint.m_angularComponentA = rb0readonly->getInvInertiaTensorWorld()*ftorqueAxis0;
									}
									{
										btVector3 ftorqueAxis0 = rel_pos2.cross(constraint.m_contactNormal);
										constraint.m_relpos2CrossNormal = ftorqueAxis0;
										constraint.m_angularComponentB = rb1readonly->getInvInertiaTensorWorld()*ftorqueAxis0;
									}

									{
										btScalar rel_vel;
										btScalar vel1Dotn = constraint.m_contactNormal.dot(rb0readonly?rb0readonly->getLinearVelocity():btVector3(0,0,0)) 
											+ constraint.m_relpos1CrossNormal.dot(rb0readonly?rb0readonly->getAngularVelocity():btVector3(0,0,0));
										btScalar vel2Dotn = constraint.m_contactNormal.dot(rb1readonly?rb1readonly->getLinearVelocity():btVector3(0,0,0)) 
											+ constraint.m_relpos2CrossNormal.dot(rb1readonly?rb1readonly->getAngularVelocity():btVector3(0,0,0));

										rel_vel = vel1Dotn-vel2Dotn;

										btScalar positionalError = 0.f;
										positionalError = 0;
										btScalar restitution=0.f;

										btSimdScalar velocityError = restitution - rel_vel;
										btSimdScalar	velocityImpulse = velocityError * btSimdScalar(constraint.m_jacDiagABInv);
										constraint.m_rhs = velocityImpulse;
										constraint.m_cfm = 0.f;
										constraint.m_lowerLimit = 0;
										constraint.m_upperLimit = 1e10f;
									}
								}

								{
									btSolverConstraint& constraint = localMemory->m_tempInternalConstr[2];
									constraint.m_originalContactPoint = 0;
									constraint.m_contactNormal = frictionTangential1b;

									constraint.m_solverBodyIdA = offsA;
									constraint.m_solverBodyIdB = offsB;

									constraint.m_friction = cp.m_combinedFriction;

									constraint.m_appliedImpulse = btScalar(0.);

									constraint.m_jacDiagABInv = computeJacobianInverse (rb0readonly, rb1readonly, pos1, pos2, constraint.m_contactNormal);

									{
										btVector3 ftorqueAxis0 = rel_pos1.cross(constraint.m_contactNormal);
										constraint.m_relpos1CrossNormal = ftorqueAxis0;
										constraint.m_angularComponentA = rb0readonly->getInvInertiaTensorWorld()*ftorqueAxis0;
									}
									{
										btVector3 ftorqueAxis0 = rel_pos2.cross(constraint.m_contactNormal);
										constraint.m_relpos2CrossNormal = ftorqueAxis0;
										constraint.m_angularComponentB = rb1readonly->getInvInertiaTensorWorld()*ftorqueAxis0;
									}

									{
										btScalar rel_vel;
										btScalar vel1Dotn = constraint.m_contactNormal.dot(rb0readonly?rb0readonly->getLinearVelocity():btVector3(0,0,0)) 
											+ constraint.m_relpos1CrossNormal.dot(rb0readonly?rb0readonly->getAngularVelocity():btVector3(0,0,0));
										btScalar vel2Dotn = constraint.m_contactNormal.dot(rb1readonly?rb1readonly->getLinearVelocity():btVector3(0,0,0)) 
											+ constraint.m_relpos2CrossNormal.dot(rb1readonly?rb1readonly->getAngularVelocity():btVector3(0,0,0));

										rel_vel = vel1Dotn-vel2Dotn;

										btScalar positionalError = 0.f;
										positionalError = 0;
										btScalar restitution=0.f;

										btSimdScalar velocityError = restitution - rel_vel;
										btSimdScalar	velocityImpulse = velocityError * btSimdScalar(constraint.m_jacDiagABInv);
										constraint.m_rhs = velocityImpulse;
										constraint.m_cfm = 0.f;
										constraint.m_lowerLimit = 0;
										constraint.m_upperLimit = 1e10f;
									}
								}

								// DMA the three constraints
								{
									int dmaSize = sizeof(btSolverConstraint)*3;
									uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverInternalConstraintList + constraintIndex);
									cellDmaLargePut(&localMemory->m_tempInternalConstr, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);		
									cellDmaWaitTagStatusAll(DMA_MASK(1));
								}

								constraintIndex += 3;

							}
							if (1)
							{
								///DMA back bodyA (with applied impulse)
								{
									int dmaSize = sizeof(btSolverBody);
									uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + solverBodyIdA);
									cellDmaLargePut(&localMemory->m_tempSPUBodies[0], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
									cellDmaWaitTagStatusAll(DMA_MASK(1));
								}
								///DMA back bodyB (with applied impulse)
								{
									int dmaSize = sizeof(btSolverBody);
									uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + solverBodyIdB);
									cellDmaLargePut(&localMemory->m_tempSPUBodies[1], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
									cellDmaWaitTagStatusAll(DMA_MASK(1));
								}
							}

						}

						

						manifoldsToProcess -= packageSize;
						manifoldPackageOffset += packageSize;
					}
				}
				int numOutConstraints = 0;
				// Setup constraints
				{
					const btContactSolverInfoData& solverInfo = taskDesc.m_commandData.m_manifoldSetup.m_solverInfo;

					int constraintIndex = hashCell.m_constraintListOffset;
					
					int constraintsToProcess = hashCell.m_numConstraints;
					int constraintPackageOffset = hashCell.m_constraintListOffset;

					while (constraintsToProcess)
					{
						const int packageSize = constraintsToProcess > constraintsPerPackage ? constraintsPerPackage : constraintsToProcess;

						// DMA the holder list
						{
							int dmaSize = sizeof(ConstraintCellHolder)*packageSize;						
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_commandData.m_manifoldSetup.m_constraintHolders + constraintPackageOffset);
							cellDmaLargeGet(constraintHolderList, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
							cellDmaWaitTagStatusAll(DMA_MASK(1));
						}

						int c;
						// DMA the constraint list
						for ( c = 0; c < packageSize; ++c)
						{
							//int dmaSize = CONSTRAINT_MAX_SIZE;
							int dmaSize = getConstraintSize((btTypedConstraintType)constraintHolderList[c].m_constraintType);
							btAssert(dmaSize<CONSTRAINT_MAX_SIZE);
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (constraintHolderList[c].m_constraint);
							cellDmaLargeGet(constraintList + CONSTRAINT_MAX_SIZE*c, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
						}
						cellDmaWaitTagStatusAll(DMA_MASK(1));

						for ( c = 0; c < packageSize; ++c)
						{
							btTypedConstraint* currConstraint = (btTypedConstraint*)(constraintList + CONSTRAINT_MAX_SIZE*c);
							btTypedConstraintType type = currConstraint->getConstraintType();

							btRigidBody* rb0Ptr = (btRigidBody*)&currConstraint->getRigidBodyA();
							btRigidBody* rb1Ptr = (btRigidBody*)&currConstraint->getRigidBodyB();

							// DMA the bodies
							{
								int dmaSize = sizeof(btRigidBody);
								uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (rb0Ptr);
								cellDmaLargeGet(&localMemory->m_tempRBs[0], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
							}
							{
								int dmaSize = sizeof(btRigidBody);
								uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (rb1Ptr);
								cellDmaLargeGet(&localMemory->m_tempRBs[1], dmaPpuAddress2, dmaSize, DMA_TAG(2), 0, 0);							
							}
							cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));


							btRigidBody* rb0 = (btRigidBody*)&localMemory->m_tempRBs[0];
							btRigidBody* rb1 = (btRigidBody*)&localMemory->m_tempRBs[1];

							unsigned int solverBodyIdA = ~0, solverBodyIdB = ~0;
							if (rb0->getIslandTag() >= 0)
							{
								solverBodyIdA = rb0->getCompanionId();
							} 
							else
							{
								//create a static body
								solverBodyIdA = taskDesc.m_commandData.m_manifoldSetup.m_numBodies + taskDesc.m_commandData.m_manifoldSetup.m_numBodies + 
									hashCell.m_constraintListOffset;
								setupSpuBody(rb0, &localMemory->m_tempSPUBodies[0]);
								{
									int dmaSize = sizeof(btSolverBody);
									uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + solverBodyIdA);
									cellDmaLargePut(&localMemory->m_tempSPUBodies[0], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
									cellDmaWaitTagStatusAll(DMA_MASK(1));
								}							
							}

							if (rb1->getIslandTag() >= 0)
							{
								solverBodyIdB = rb1->getCompanionId();
							} 
							else
							{
								//create a static body
								solverBodyIdB = taskDesc.m_commandData.m_manifoldSetup.m_numBodies + taskDesc.m_commandData.m_manifoldSetup.m_numManifolds + 
									hashCell.m_constraintListOffset;					
								setupSpuBody(rb1, &localMemory->m_tempSPUBodies[0]);
								{
									int dmaSize = sizeof(btSolverBody);
									uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + solverBodyIdB);
									cellDmaLargePut(&localMemory->m_tempSPUBodies[0], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
									cellDmaWaitTagStatusAll(DMA_MASK(1));
								}							
							}

							// Setup the pointer table
							int offsA = localRBs.insert(solverBodyIdA);		
							int offsB = localRBs.insert(solverBodyIdB);

							int numConstraintRows = 0;
#ifdef NOT_YET
							// Setup the constraint
							switch (type)
							{
							case POINT2POINT_CONSTRAINT_TYPE:
								{
									btSolverConstraint* spuConstraint = &localMemory->m_tempConstraint[0];
									btPoint2PointConstraint* p2pC = (btPoint2PointConstraint*)currConstraint;

									spuConstraint->m_solverBodyIdA = offsA;
									spuConstraint->m_solverBodyIdB = offsB;
									spuConstraint->m_constraintType = type;

									// Compute the anchor positions
									btVector3 pivotAinW = rb0->getCenterOfMassTransform()*p2pC->m_pivotInA;
									btVector3 pivotBinW = rb1->getCenterOfMassTransform()*p2pC->m_pivotInB;
	
									//setupLinearConstraintWorld(spuConstraint, rb0, rb1, pivotAinW, pivotBinW, solverInfo);
									//setupConstraint(spuConstraint,rb0,localMemory->m_tempSPUBodies[0],rb1,localMemory->m_tempSPUBodies[1],pivotAinW,pivotBinW,solverInfo);
									
									numConstraintRows = 3; //We have 3 constraint rows
								}
								break;

							case HINGE_CONSTRAINT_TYPE:
								{
									btSolverConstraint& spuConstraint = localMemory->m_tempConstraint[0];
									btHingeConstraint* hC = (btHingeConstraint*)currConstraint;

									spuConstraint.m_localOffsetBodyA = offsA;
									spuConstraint.m_localOffsetBodyB = offsB;
									spuConstraint.m_constraintType = type;									

									// Compute the transforms
									btTransform frameAinW = rb0->getCenterOfMassTransform()*hC->m_rbAFrame;
									btTransform frameBinW = rb1->getCenterOfMassTransform()*hC->m_rbBFrame;

									// Setup the linear part
									//setupLinearConstraintWorld(spuConstraint, rb0, rb1, frameAinW.getOrigin(), frameBinW.getOrigin(), solverInfo);

									// Setup angular part
									btVector3 jacInv; 

									// Setup the jacobian inverses
									for (int i = 0; i < 3; ++i)
									{
										const btVector3 axisA = frameAinW.getBasis().getColumn(i);
										const btVector3 axisB = frameBinW.getBasis().getColumn(i);
										
										spuConstraint.hinge.m_frameAinW[i] = axisA;
										spuConstraint.hinge.m_frameBinW[i] = axisB;									

										jacInv[i] = computeAngularJacobianInverse(rb0, rb1, axisA);
									}

									// Compute position error along the two secondary axes & limit
									{
										btVector3 angularBias (0,0,0);

										const btVector3 axisA = frameAinW.getBasis().getColumn(2);
										const btVector3 axisB = frameBinW.getBasis().getColumn(2); 
										
										btVector3 error = -axisA.cross(axisB) / solverInfo.m_timeStep;
										
										angularBias[0] = error.dot(frameAinW.getBasis().getColumn(0));
										angularBias[1] = error.dot(frameAinW.getBasis().getColumn(1));

										spuConstraint.m_flags.m_limit1 = 0;
										
										if (hC->m_lowerLimit < hC->m_upperLimit)
										{
											// Compute hinge axis
											const btVector3& refAxis0 = frameAinW.getBasis().getColumn(0);
											const btVector3& refAxis1 = frameAinW.getBasis().getColumn(1);
											const btVector3& swingAxis = frameBinW.getBasis().getColumn(1);

											float hingeAngle = btAtan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
											float correction, sign;

											spuConstraint.hinge.m_limitAccumulatedImpulse = 0;

											if (hingeAngle <= hC->m_lowerLimit*hC->m_limitSoftness)
											{
												correction = (hC->m_lowerLimit - hingeAngle);
												sign = 1.0f;
												spuConstraint.m_flags.m_limit1 = 1;
											} 
											else if (hingeAngle >= hC->m_upperLimit*hC->m_limitSoftness)
											{
												correction = (hC->m_upperLimit - hingeAngle);
												sign = -1.0f;
												spuConstraint.m_flags.m_limit1 = 1;
											}

											angularBias[2] = correction * hC->m_biasFactor / (solverInfo.m_timeStep * hC->m_relaxationFactor);
											spuConstraint.hinge.m_limitJacFactor = hC->m_relaxationFactor * sign;
										}										

										spuConstraint.hinge.m_angularBias = angularBias;
									}
									
									// Setup motor
									spuConstraint.m_flags.m_motor1 = 0;
									if (hC->m_enableAngularMotor)
									{
										spuConstraint.m_flags.m_motor1 = 1;
										spuConstraint.hinge.m_motorVelocity = hC->m_motorTargetVelocity;
										spuConstraint.hinge.m_motorImpulse = hC->m_maxMotorImpulse;
									}

									spuConstraint.hinge.m_angJacdiagABInv = jacInv;

									haveConstraint = true;
								}
								break;
							case CONETWIST_CONSTRAINT_TYPE:
								{
									btSolverConstraint& spuConstraint = localMemory->m_tempConstraint[0];
									btConeTwistConstraint* ctC = (btConeTwistConstraint*)currConstraint;

									spuConstraint.m_localOffsetBodyA = offsA;
									spuConstraint.m_localOffsetBodyB = offsB;
									spuConstraint.m_constraintType = type;

									// Compute the transforms
									btTransform frameAinW = rb0->getCenterOfMassTransform()*ctC->m_rbAFrame;
									btTransform frameBinW = rb1->getCenterOfMassTransform()*ctC->m_rbBFrame;

									// Setup the linear part
									setupLinearConstraintWorld(spuConstraint, rb0, rb1, frameAinW.getOrigin(), frameBinW.getOrigin(), solverInfo);

									// Setup the swing limits
									const btVector3& b1Axis1 = frameAinW.getBasis().getColumn(0);
									const btVector3& b2Axis1 = frameBinW.getBasis().getColumn(0);
									const btVector3& b1Axis2 = frameAinW.getBasis().getColumn(1);
									const btVector3& b1Axis3 = frameAinW.getBasis().getColumn(2);

									float swing1 = 0.0f, swing2 = 0.0f;

									if (ctC->m_swingSpan1 >= 0.05f)
									{
										swing1 = btAtan2Fast(b2Axis1.dot(b1Axis2),b2Axis1.dot(b1Axis1));
									}
									if (ctC->m_swingSpan2 >= 0.05f)
									{
										swing2 = btAtan2Fast(b2Axis1.dot(b1Axis3),b2Axis1.dot(b1Axis1));
									}

									float rMaxAngle1Sq = 1.0f / (ctC->m_swingSpan1*ctC->m_swingSpan1);		
									float rMaxAngle2Sq = 1.0f / (ctC->m_swingSpan2*ctC->m_swingSpan2);	
									float ellipseAngle = btFabs(swing1)* rMaxAngle1Sq + btFabs(swing2) * rMaxAngle2Sq;

									spuConstraint.m_flags.m_limit1 = 0;
									spuConstraint.m_flags.m_limit2 = 0;

									spuConstraint.conetwist.m_swingLimitImpulse = 0;
									spuConstraint.conetwist.m_twistLimitImpulse = 0;

									float relFactorSq = ctC->m_relaxationFactor*ctC->m_relaxationFactor;
	
									if (ellipseAngle > 1.0f)
									{
										spuConstraint.conetwist.m_swingError = ellipseAngle - 1.0f;
										spuConstraint.conetwist.m_swingError *= ctC->m_biasFactor;
										spuConstraint.conetwist.m_swingError /= solverInfo.m_timeStep * relFactorSq;

										spuConstraint.m_flags.m_limit1 = 1;

										btVector3 axis = b2Axis1.cross(b1Axis2* b2Axis1.dot(b1Axis2) + b1Axis3* b2Axis1.dot(b1Axis3));
										axis.normalize();

										float swingAxisSign = (b2Axis1.dot(b1Axis1) >= 0.0f) ? 1.0f : -1.0f;
										axis *= swingAxisSign;

										spuConstraint.conetwist.m_swingAxis = axis;

										float jacobian = computeAngularJacobianInverse(rb0, rb1, axis);
										spuConstraint.conetwist.m_swingJacInv = relFactorSq	 * jacobian;
									}

									// Setup twist limits
									if (ctC->m_twistSpan >= 0.0f)										
									{
										const btVector3& b2Axis2 = frameBinW.getBasis().getColumn(1);

										btQuaternion rotationArc = shortestArcQuat(b2Axis1,b1Axis1);
										btVector3 TwistRef = quatRotate(rotationArc,b2Axis2); 
										float twist = btAtan2Fast(TwistRef.dot(b1Axis3), TwistRef.dot(b1Axis2));

										float lockedFreeFactor = (ctC->m_twistSpan > btScalar(0.05f)) ? ctC->m_limitSoftness : btScalar(0.);
										if (twist <= -ctC->m_twistSpan*lockedFreeFactor)
										{
											spuConstraint.conetwist.m_twistError = -(twist + ctC->m_twistSpan);
											spuConstraint.conetwist.m_twistError *= ctC->m_biasFactor;
											spuConstraint.conetwist.m_twistError /= solverInfo.m_timeStep * relFactorSq;

											spuConstraint.m_flags.m_limit2 = 1;

											btVector3 axis = -(b1Axis1 + b2Axis1);
											axis.normalize();
											spuConstraint.conetwist.m_twistAxis = axis;

											float jacobian = computeAngularJacobianInverse(rb0, rb1, axis);
											spuConstraint.conetwist.m_twistJacInv = relFactorSq * jacobian;
										}
										else if (twist >= ctC->m_twistSpan*lockedFreeFactor)
										{
											spuConstraint.conetwist.m_twistError = twist - ctC->m_twistSpan;
											spuConstraint.conetwist.m_twistError *= ctC->m_biasFactor;
											spuConstraint.conetwist.m_twistError /= solverInfo.m_timeStep * relFactorSq;

											spuConstraint.m_flags.m_limit2 = 1;

											btVector3 axis = b1Axis1 + b2Axis1;
											axis.normalize();
											spuConstraint.conetwist.m_twistAxis = axis;

											float jacobian = computeAngularJacobianInverse(rb0, rb1, axis);
											spuConstraint.conetwist.m_twistJacInv = relFactorSq * jacobian;
										}

									}

									haveConstraint = true; //We have one constraint
								}
								
								break;

							default:
								;
							}
#endif //NOT_YET

							if (numConstraintRows)
							{
								//DMA it
								int dmaSize = sizeof(btSolverConstraint)*numConstraintRows;
								uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverConstraintList + 
									hashCell.m_constraintListOffset + numOutConstraints);
								cellDmaLargePut(&localMemory->m_tempConstraint[0], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
								cellDmaWaitTagStatusAll(DMA_MASK(1));

								numOutConstraints+=numConstraintRows;
							}

						}

						constraintsToProcess -= packageSize;
						constraintPackageOffset += packageSize;
					}
				}

				// Write back some data, if needed
				if (localRBs.size() > 0)
				{
					{
						// DMA the local body list
						int dmaSize = sizeof(uint32_t)*localRBs.size();
						uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyOffsetList + hashCell.m_solverBodyOffsetListOffset);
						cellDmaLargePut(indexArray, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);								
					}
					hashCell.m_numLocalBodies = localRBs.size();
					hashCell.m_numConstraints = numOutConstraints;
					{
						// DMA the hash cell
						int dmaSize = sizeof(SpuSolverHashCell);
						uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverHash);
						dmaPpuAddress2 += offsetof(SpuSolverHash,m_Hash);
						dmaPpuAddress2 += sizeof(SpuSolverHashCell) * cellIdx;

						cellDmaLargePut(&hashCell, dmaPpuAddress2, dmaSize, DMA_TAG(2), 0, 0);
					}
					cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));
				}				
			}
		}
		break;

	case CMD_SOLVER_SOLVE_ITERATE:
		{			
			// DMA the hash
			{
				int dmaSize = sizeof(SpuSolverHash);
				uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverHash);
				cellDmaLargeGet(&localMemory->m_localHash, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
			}
			
			btSpinlock hashLock (taskDesc.m_commandData.m_iterate.m_spinLockVar);			

			int cellToProcess;
			while (1)
			{
				cellToProcess = getNextFreeCell(localMemory, taskDesc, hashLock);

				if (cellToProcess >= SPU_HASH_NUMCELLS)
					break;

				// Now process that one cell
				SpuSolverHashCell& hashCell = localMemory->m_localHash.m_Hash[cellToProcess];
				
				if (hashCell.m_numContacts == 0 && hashCell.m_numConstraints == 0)
					continue;

				// DMA the local bodies and constraints

				// Get the body list
				uint32_t* indexList = (uint32_t*)allocTemporaryStorage(localMemory, sizeof(uint32_t)*hashCell.m_numLocalBodies);
				btSolverBody* bodyList = allocBodyStorage(localMemory, hashCell.m_numLocalBodies);
				int b;
				{
					int dmaSize = sizeof(uint32_t)*hashCell.m_numLocalBodies;
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyOffsetList + hashCell.m_solverBodyOffsetListOffset);
					cellDmaLargeGet(indexList, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);		
					cellDmaWaitTagStatusAll(DMA_MASK(1));
				}

				// DMA the bodies
				for ( b = 0; b < hashCell.m_numLocalBodies; ++b)
				{
					int dmaSize = sizeof(btSolverBody);
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + indexList[b]);
					cellDmaLargeGet(bodyList+b, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);						
				}
				cellDmaWaitTagStatusAll(DMA_MASK(1));

				// Process the constraints in packets
				if (hashCell.m_numConstraints)
				{
					const size_t maxConstraintsPerPacket = memTemporaryStorage(localMemory) / sizeof(btSolverConstraint);
					size_t constraintsToProcess = hashCell.m_numConstraints;
					size_t constraintListOffset = hashCell.m_constraintListOffset;

					btSolverConstraint* constraints = allocConstraintStorage(localMemory, maxConstraintsPerPacket);

					while (constraintsToProcess > 0)
					{
						size_t packetSize = constraintsToProcess > maxConstraintsPerPacket ? maxConstraintsPerPacket : constraintsToProcess;

						// DMA the constraints
						{
							int dmaSize = sizeof(btSolverConstraint)*(int)packetSize;
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverConstraintList + constraintListOffset);
							cellDmaLargeGet(constraints, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
						}
						cellDmaWaitTagStatusAll(DMA_MASK(1));


						// Solve
						for (size_t j = 0; j < packetSize; ++j)
						{
							btSolverConstraint& constraint = constraints[j];
							btSolverBody& bodyA = bodyList[constraint.m_solverBodyIdA];
							btSolverBody& bodyB = bodyList[constraint.m_solverBodyIdB];

							//solveConstraint(constraint, bodyA, bodyB);
						}
						
						// Write back the constraints for accumulated stuff
						{
							int dmaSize = sizeof(btSolverConstraint)*(int)packetSize;
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverConstraintList + constraintListOffset);					
							cellDmaLargePut(constraints, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
						}
						cellDmaWaitTagStatusAll(DMA_MASK(1));

						constraintListOffset += packetSize;
						constraintsToProcess -= packetSize;
					}

					freeConstraintStorage (localMemory, constraints, maxConstraintsPerPacket);
				} 

				// Now process the contacts
				if (hashCell.m_numContacts)
				{
					const size_t maxContactsPerPacket = memTemporaryStorage(localMemory) / (sizeof(btSolverConstraint)*3);
					size_t contactsToProcess = hashCell.m_numContacts;
					size_t constraintListOffset = hashCell.m_internalConstraintListOffset;

					btSolverConstraint* internalConstraints = allocInternalConstraintStorage(localMemory, maxContactsPerPacket*3);

					while (contactsToProcess > 0)
					{
						size_t packetSize = contactsToProcess > maxContactsPerPacket ? maxContactsPerPacket : contactsToProcess;

						// DMA the constraints
						{
							int dmaSize = sizeof(btSolverConstraint)*(int)packetSize*3;
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverInternalConstraintList + constraintListOffset);
							cellDmaLargeGet(internalConstraints, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
						}
						cellDmaWaitTagStatusAll(DMA_MASK(1));


						size_t j;
						// Solve
						
						{
							for ( j = 0; j < packetSize*3; j += 3)
							{
								btSolverConstraint& contact = internalConstraints[j];
								btSolverBody& bodyA = bodyList[contact.m_solverBodyIdA];
								btSolverBody& bodyB = bodyList[contact.m_solverBodyIdB];

								SpuResolveSingleConstraintRowGeneric(bodyA, bodyB,contact);
							}
						}

						{
							for ( j = 0; j < packetSize*3; j += 3)
							{
								btSolverConstraint& contact = internalConstraints[j];
								btSolverBody& bodyA = bodyList[contact.m_solverBodyIdA];
								btSolverBody& bodyB = bodyList[contact.m_solverBodyIdB];

								btSolverConstraint& frictionConstraint1 = internalConstraints[j + 1];
								frictionConstraint1.m_lowerLimit = frictionConstraint1.m_friction*btScalar(-contact.m_appliedImpulse);
								frictionConstraint1.m_upperLimit = frictionConstraint1.m_friction*btScalar(contact.m_appliedImpulse);
								SpuResolveSingleConstraintRowGeneric(bodyA, bodyB, frictionConstraint1);

								btSolverConstraint& frictionConstraint2 = internalConstraints[j + 2];
								frictionConstraint2.m_lowerLimit = frictionConstraint2.m_friction*btScalar(-contact.m_appliedImpulse);
								frictionConstraint2.m_upperLimit = frictionConstraint2.m_friction*btScalar(contact.m_appliedImpulse);
								SpuResolveSingleConstraintRowGeneric(bodyA, bodyB, frictionConstraint2);
							}
						}


						// Write back the constraints for accumulated stuff
						{
							int dmaSize = sizeof(btSolverConstraint)*(int)packetSize*3;
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverInternalConstraintList + constraintListOffset);					
							cellDmaLargePut(internalConstraints, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
						}
						cellDmaWaitTagStatusAll(DMA_MASK(1));

						constraintListOffset += packetSize*3;
						contactsToProcess -= packetSize;
					}

					freeInternalConstraintStorage (localMemory, internalConstraints, maxContactsPerPacket*3);
				}

								
				// DMA the bodies back to main memory
				for ( b = 0; b < hashCell.m_numLocalBodies; ++b)
				{					
					int dmaSize = sizeof(btSolverBody);
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + indexList[b]);
					cellDmaLargePut(bodyList + b, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);						
				}
				cellDmaWaitTagStatusAll(DMA_MASK(1));

				freeBodyStorage(localMemory, bodyList, hashCell.m_numLocalBodies);
				freeTemporaryStorage(localMemory, indexList, sizeof(uint32_t)*hashCell.m_numLocalBodies);

			};
		}
		break;
	case CMD_SOLVER_COPYBACK_BODIES:
		{
			int bodiesToProcess = taskDesc.m_commandData.m_bodyCopyback.m_numBodies;
			int bodyPackageOffset = taskDesc.m_commandData.m_bodyCopyback.m_startBody;
			const int bodiesPerPackage = 256;

			btRigidBody** bodyPtrList = (btRigidBody**)allocTemporaryStorage(localMemory, bodiesPerPackage*sizeof(btRigidBody*));
			btRigidBody* bodyList = (btRigidBody*)allocTemporaryStorage(localMemory, bodiesPerPackage*sizeof(btRigidBody));
			btSolverBody* spuBodyList = allocBodyStorage(localMemory, bodiesPerPackage);

			while (bodiesToProcess > 0)
			{
				const int packageSize = bodiesToProcess > bodiesPerPackage ? bodiesPerPackage : bodiesToProcess;

				// DMA the body pointers
				{
					int dmaSize = sizeof(btRigidBody*)*packageSize;
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_commandData.m_bodySetup.m_rbList + bodyPackageOffset);
					cellDmaLargeGet(bodyPtrList, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
					cellDmaWaitTagStatusAll(DMA_MASK(1));
				}

				int b;
				// DMA the rigid bodies
				for ( b = 0; b < packageSize; ++b)
				{
					btRigidBody* body = bodyPtrList[b];
					int dmaSize = sizeof(btRigidBody);
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (body);
					cellDmaLargeGet(&bodyList[b], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);					
				}

				// DMA the list of SPU bodies
				{
					int dmaSize = sizeof(btSolverBody)*packageSize;
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverBodyList + bodyPackageOffset);
					cellDmaLargeGet(spuBodyList, dmaPpuAddress2, dmaSize, DMA_TAG(2), 0, 0);
				}
				cellDmaWaitTagStatusAll(DMA_MASK(1) | DMA_MASK(2));


				for ( b = 0; b < packageSize; ++b)
				{
					btRigidBody* localBody = bodyList + b;
					btSolverBody* solverBody = spuBodyList + b;
				
					if (solverBody->m_invMass > 0)
					{
						localBody->setLinearVelocity(localBody->getLinearVelocity()+solverBody->m_deltaLinearVelocity);
						localBody->setAngularVelocity(localBody->getAngularVelocity()+solverBody->m_deltaAngularVelocity);
					}
					localBody->setCompanionId(-1);
				}

				// DMA the rigid bodies
				for ( b = 0; b < packageSize; ++b)
				{
					btRigidBody* body = bodyPtrList[b];
					int dmaSize = sizeof(btRigidBody);
					uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (body);
					cellDmaLargePut(&bodyList[b], dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);					
				}


				bodiesToProcess -= packageSize;
				bodyPackageOffset += packageSize;				
			}

		}
		break;

		case CMD_SOLVER_MANIFOLD_WARMSTART_WRITEBACK:
		{			
			// DMA the hash
			{
				int dmaSize = sizeof(SpuSolverHash);
				uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverHash);
				cellDmaLargeGet(&localMemory->m_localHash, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
				cellDmaWaitTagStatusAll(DMA_MASK(1));
			}
			
			btSpinlock hashLock (taskDesc.m_commandData.m_iterate.m_spinLockVar);			

			int cellToProcess;
			while (1)
			{
				cellToProcess = getNextFreeCell(localMemory, taskDesc, hashLock);

				if (cellToProcess >= SPU_HASH_NUMCELLS)
					break;

				// Now process that one cell
				SpuSolverHashCell& hashCell = localMemory->m_localHash.m_Hash[cellToProcess];
				
				if (hashCell.m_numContacts == 0 && hashCell.m_numConstraints == 0)
					continue;

				// Now process the contacts
				if (hashCell.m_numContacts)
				{
					const size_t maxContactsPerPacket = memTemporaryStorage(localMemory) / (sizeof(btSolverConstraint)*3);
					size_t contactsToProcess = hashCell.m_numContacts;
					size_t constraintListOffset = hashCell.m_internalConstraintListOffset;

					btSolverConstraint* internalConstraints = allocInternalConstraintStorage(localMemory, maxContactsPerPacket*3);

					while (contactsToProcess > 0)
					{
						size_t packetSize = contactsToProcess > maxContactsPerPacket ? maxContactsPerPacket : contactsToProcess;

						// DMA the constraints
						{
							int dmaSize = sizeof(btSolverConstraint)*(int)packetSize*3;
							uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (taskDesc.m_solverData.m_solverInternalConstraintList + constraintListOffset);
							cellDmaLargeGet(internalConstraints, dmaPpuAddress2, dmaSize, DMA_TAG(1), 0, 0);
						}
						cellDmaWaitTagStatusAll(DMA_MASK(1));

						int j;
						for ( j = 0; j < packetSize*3; j += 3)
						{
								btSolverConstraint& contact = internalConstraints[j];
								{
									//DMA in
									uint64_t dmaPpuAddress2 = reinterpret_cast<uint64_t> (contact.m_originalContactPoint);
									int dmasize = 4*sizeof(float);
									float* tmpMem = &localMemory->m_appliedImpulse[0];
									cellDmaGet(tmpMem,dmaPpuAddress2,dmasize,DMA_TAG(1),0,0);
									cellDmaWaitTagStatusAll(DMA_MASK(1));

									
									*tmpMem = btMin(btScalar(3.),btScalar(contact.m_appliedImpulse));

									///DMA out
									cellDmaLargePut(tmpMem,dmaPpuAddress2,dmasize,DMA_TAG(1),0,0);
									cellDmaWaitTagStatusAll(DMA_MASK(1));
								}
						}
						constraintListOffset += packetSize*3;
						contactsToProcess -= packetSize;
					}

					freeInternalConstraintStorage (localMemory, internalConstraints, maxContactsPerPacket*3);
				}

				
			}
		}
		break;

	default:
		//.. nothing
		;
//		btAssert(0);
	}
}
