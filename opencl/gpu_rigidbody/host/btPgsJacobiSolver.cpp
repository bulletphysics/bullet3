/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2012 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//enable BT_SOLVER_DEBUG if you experience solver crashes
//#define BT_SOLVER_DEBUG
//#define COMPUTE_IMPULSE_DENOM 1
//It is not necessary (redundant) to refresh contact manifolds, this refresh has been moved to the collision algorithms.

#define DISABLE_JOINTS

#include "btPgsJacobiSolver.h"
#include "BulletCommon/btMinMax.h"
#include "btTypedConstraint.h"
#include <new>
#include "BulletCommon/btStackAlloc.h"
#include "BulletCommon/btQuickprof.h"
//#include "btSolverBody.h"
//#include "btSolverConstraint.h"
#include "BulletCommon/btAlignedObjectArray.h"
#include <string.h> //for memset
//#include "../../dynamics/basic_demo/Stubs/AdlContact4.h"
#include "../../gpu_sat/host/btContact4.h"

bool usePgs = false;//true;
int		gNumSplitImpulseRecoveries2 = 0;

#include "btRigidBody.h"
//#include "../../dynamics/basic_demo/Stubs/AdlRigidBody.h"
#include "../../gpu_sat/host/btRigidBodyCL.h"

btTransform	getWorldTransform(btRigidBodyCL* rb)
{
	btTransform newTrans;
	newTrans.setOrigin(rb->m_pos);
	newTrans.setRotation(rb->m_quat);
	return newTrans;
}

const btMatrix3x3&	getInvInertiaTensorWorld(btInertiaCL* inertia)
{
	return inertia->m_invInertiaWorld;
}



const btVector3&	getLinearVelocity(btRigidBodyCL* rb)
{
	return rb->m_linVel;
}

const btVector3&	getAngularVelocity(btRigidBodyCL* rb)
{
	return rb->m_angVel;
}

btVector3 getVelocityInLocalPoint(btRigidBodyCL* rb, const btVector3& rel_pos)
{
	//we also calculate lin/ang velocity for kinematic objects
	return getLinearVelocity(rb) + getAngularVelocity(rb).cross(rel_pos);
	
}

struct	btContactPoint
{
	btVector3	m_positionWorldOnA;
	btVector3	m_positionWorldOnB;
	btVector3	m_normalWorldOnB;
	btScalar	m_appliedImpulse;
	btScalar	m_distance;
	btScalar	m_combinedRestitution;

	///information related to friction
	btScalar	m_combinedFriction;
	btVector3	m_lateralFrictionDir1;
	btVector3	m_lateralFrictionDir2;
	btScalar	m_appliedImpulseLateral1;
	btScalar	m_appliedImpulseLateral2;	
	btScalar	m_combinedRollingFriction;
	btScalar	m_contactMotion1;
	btScalar	m_contactMotion2;
	btScalar	m_contactCFM1;
	btScalar	m_contactCFM2;
	
	bool		m_lateralFrictionInitialized;

	btVector3	getPositionWorldOnA()
	{
		return m_positionWorldOnA;
	}
	btVector3	getPositionWorldOnB()
	{
		return m_positionWorldOnB;
	}
	btScalar	getDistance()
	{
		return m_distance;
	}
};

void	getContactPoint(btContact4* contact, int contactIndex, btContactPoint& pointOut)
{
	pointOut.m_appliedImpulse = 0.f;
	pointOut.m_appliedImpulseLateral1 = 0.f;
	pointOut.m_appliedImpulseLateral2 = 0.f;
	pointOut.m_combinedFriction = contact->getFrictionCoeff();
	pointOut.m_combinedRestitution = contact->getRestituitionCoeff();
	pointOut.m_combinedRollingFriction = 0.f;
	pointOut.m_contactCFM1 = 0.f;
	pointOut.m_contactCFM2 = 0.f;
	pointOut.m_contactMotion1 = 0.f;
	pointOut.m_contactMotion2 = 0.f;
	pointOut.m_distance = contact->getPenetration(contactIndex)+0.01;
	btVector3 n = contact->m_worldNormal;
	btVector3 normalOnB(-n);
	normalOnB.normalize();

	btVector3 l1,l2;
	btPlaneSpace1(normalOnB,l1,l2);

	pointOut.m_normalWorldOnB = normalOnB;
	//printf("normalOnB = %f,%f,%f\n",normalOnB.getX(),normalOnB.getY(),normalOnB.getZ());
	pointOut.m_lateralFrictionDir1 = l1;
	pointOut.m_lateralFrictionDir2 = l2;
	pointOut.m_lateralFrictionInitialized = true;
	
	
	btVector3 worldPosB = contact->m_worldPos[contactIndex];
	pointOut.m_positionWorldOnB = worldPosB;
	pointOut.m_positionWorldOnA = worldPosB+normalOnB*pointOut.m_distance;
}

int	getNumContacts(btContact4* contact)
{
	return contact->getNPoints();
}

btPgsJacobiSolver::btPgsJacobiSolver()
:m_btSeed2(0),m_usePgs(usePgs)
{

}

btPgsJacobiSolver::~btPgsJacobiSolver()
{
}

void	btPgsJacobiSolver::solveContacts(int numBodies, btRigidBodyCL* bodies, btInertiaCL* inertias, int numContacts, btContact4* contacts)
{
	btContactSolverInfo infoGlobal;
	infoGlobal.m_splitImpulse = false;
	infoGlobal.m_timeStep = 1.f/60.f;
	infoGlobal.m_numIterations = 4;//4;
//	infoGlobal.m_solverMode|=SOLVER_USE_2_FRICTION_DIRECTIONS|SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS|SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION;
	//infoGlobal.m_solverMode|=SOLVER_USE_2_FRICTION_DIRECTIONS|SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS;
	infoGlobal.m_solverMode|=SOLVER_USE_2_FRICTION_DIRECTIONS;

	//if (infoGlobal.m_solverMode & SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS)
	//if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) && (infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION))
				

	solveGroup(bodies,inertias,numBodies,contacts,numContacts,0,0,infoGlobal);

	if (!numContacts)
		return;
}




/// btPgsJacobiSolver Sequentially applies impulses
btScalar btPgsJacobiSolver::solveGroup(btRigidBodyCL* bodies,
										btInertiaCL* inertias, 
										int numBodies,
										btContact4* manifoldPtr, 
										int numManifolds,
										btTypedConstraint** constraints,
										int numConstraints,
										const btContactSolverInfo& infoGlobal)
{

	BT_PROFILE("solveGroup");
	//you need to provide at least some bodies
	
	solveGroupCacheFriendlySetup( bodies, inertias,numBodies, manifoldPtr,  numManifolds,constraints, numConstraints,infoGlobal);

	solveGroupCacheFriendlyIterations(constraints, numConstraints,infoGlobal);

	solveGroupCacheFriendlyFinish(bodies, inertias,numBodies, infoGlobal);
	
	return 0.f;
}









#ifdef USE_SIMD
#include <emmintrin.h>
#define btVecSplat(x, e) _mm_shuffle_ps(x, x, _MM_SHUFFLE(e,e,e,e))
static inline __m128 btSimdDot3( __m128 vec0, __m128 vec1 )
{
	__m128 result = _mm_mul_ps( vec0, vec1);
	return _mm_add_ps( btVecSplat( result, 0 ), _mm_add_ps( btVecSplat( result, 1 ), btVecSplat( result, 2 ) ) );
}
#endif//USE_SIMD

// Project Gauss Seidel or the equivalent Sequential Impulse
void btPgsJacobiSolver::resolveSingleConstraintRowGenericSIMD(btSolverBody& body1,btSolverBody& body2,const btSolverConstraint& c)
{
#ifdef USE_SIMD
	__m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
	__m128	lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
	__m128	upperLimit1 = _mm_set1_ps(c.m_upperLimit);
	__m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse),_mm_set1_ps(c.m_cfm)));
	__m128 deltaVel1Dotn	=	_mm_add_ps(btSimdDot3(c.m_contactNormal.mVec128,body1.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128,body1.internalGetDeltaAngularVelocity().mVec128));
	__m128 deltaVel2Dotn	=	_mm_sub_ps(btSimdDot3(c.m_relpos2CrossNormal.mVec128,body2.internalGetDeltaAngularVelocity().mVec128),btSimdDot3((c.m_contactNormal).mVec128,body2.internalGetDeltaLinearVelocity().mVec128));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
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
	__m128	linearComponentA = _mm_mul_ps(c.m_contactNormal.mVec128,body1.internalGetInvMass().mVec128);
	__m128	linearComponentB = _mm_mul_ps((c.m_contactNormal).mVec128,body2.internalGetInvMass().mVec128);
	__m128 impulseMagnitude = deltaImpulse;
	body1.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(body1.internalGetDeltaLinearVelocity().mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
	body1.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(body1.internalGetDeltaAngularVelocity().mVec128 ,_mm_mul_ps(c.m_angularComponentA.mVec128,impulseMagnitude));
	body2.internalGetDeltaLinearVelocity().mVec128 = _mm_sub_ps(body2.internalGetDeltaLinearVelocity().mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
	body2.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(body2.internalGetDeltaAngularVelocity().mVec128 ,_mm_mul_ps(c.m_angularComponentB.mVec128,impulseMagnitude));
#else
	resolveSingleConstraintRowGeneric(body1,body2,c);
#endif
}

// Project Gauss Seidel or the equivalent Sequential Impulse
 void btPgsJacobiSolver::resolveSingleConstraintRowGeneric(btSolverBody& body1,btSolverBody& body2,const btSolverConstraint& c)
{
	btScalar deltaImpulse = c.m_rhs-btScalar(c.m_appliedImpulse)*c.m_cfm;
	const btScalar deltaVel1Dotn	=	c.m_contactNormal.dot(body1.internalGetDeltaLinearVelocity()) 	+ c.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocity());
	const btScalar deltaVel2Dotn	=	-c.m_contactNormal.dot(body2.internalGetDeltaLinearVelocity()) + c.m_relpos2CrossNormal.dot(body2.internalGetDeltaAngularVelocity());

//	const btScalar delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
	deltaImpulse	-=	deltaVel1Dotn*c.m_jacDiagABInv;
	deltaImpulse	-=	deltaVel2Dotn*c.m_jacDiagABInv;

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

	body1.internalApplyImpulse(c.m_contactNormal*body1.internalGetInvMass(),c.m_angularComponentA,deltaImpulse);
	body2.internalApplyImpulse(-c.m_contactNormal*body2.internalGetInvMass(),c.m_angularComponentB,deltaImpulse);
}

 void btPgsJacobiSolver::resolveSingleConstraintRowLowerLimitSIMD(btSolverBody& body1,btSolverBody& body2,const btSolverConstraint& c)
{
#ifdef USE_SIMD
	__m128 cpAppliedImp = _mm_set1_ps(c.m_appliedImpulse);
	__m128	lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
	__m128	upperLimit1 = _mm_set1_ps(c.m_upperLimit);
	__m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhs), _mm_mul_ps(_mm_set1_ps(c.m_appliedImpulse),_mm_set1_ps(c.m_cfm)));
	__m128 deltaVel1Dotn	=	_mm_add_ps(btSimdDot3(c.m_contactNormal.mVec128,body1.internalGetDeltaLinearVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128,body1.internalGetDeltaAngularVelocity().mVec128));
	__m128 deltaVel2Dotn	=	_mm_sub_ps(btSimdDot3(c.m_relpos2CrossNormal.mVec128,body2.internalGetDeltaAngularVelocity().mVec128),btSimdDot3((c.m_contactNormal).mVec128,body2.internalGetDeltaLinearVelocity().mVec128));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
	btSimdScalar resultLowerLess,resultUpperLess;
	resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
	resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
	__m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
	deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
	c.m_appliedImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
	__m128	linearComponentA = _mm_mul_ps(c.m_contactNormal.mVec128,body1.internalGetInvMass().mVec128);
	__m128	linearComponentB = _mm_mul_ps((c.m_contactNormal).mVec128,body2.internalGetInvMass().mVec128);
	__m128 impulseMagnitude = deltaImpulse;
	body1.internalGetDeltaLinearVelocity().mVec128 = _mm_add_ps(body1.internalGetDeltaLinearVelocity().mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
	body1.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(body1.internalGetDeltaAngularVelocity().mVec128 ,_mm_mul_ps(c.m_angularComponentA.mVec128,impulseMagnitude));
	body2.internalGetDeltaLinearVelocity().mVec128 = _mm_sub_ps(body2.internalGetDeltaLinearVelocity().mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
	body2.internalGetDeltaAngularVelocity().mVec128 = _mm_add_ps(body2.internalGetDeltaAngularVelocity().mVec128 ,_mm_mul_ps(c.m_angularComponentB.mVec128,impulseMagnitude));
#else
	resolveSingleConstraintRowLowerLimit(body1,body2,c);
#endif
}

// Project Gauss Seidel or the equivalent Sequential Impulse
 void btPgsJacobiSolver::resolveSingleConstraintRowLowerLimit(btSolverBody& body1,btSolverBody& body2,const btSolverConstraint& c)
{
	btScalar deltaImpulse = c.m_rhs-btScalar(c.m_appliedImpulse)*c.m_cfm;
	const btScalar deltaVel1Dotn	=	c.m_contactNormal.dot(body1.internalGetDeltaLinearVelocity()) 	+ c.m_relpos1CrossNormal.dot(body1.internalGetDeltaAngularVelocity());
	const btScalar deltaVel2Dotn	=	-c.m_contactNormal.dot(body2.internalGetDeltaLinearVelocity()) + c.m_relpos2CrossNormal.dot(body2.internalGetDeltaAngularVelocity());

	deltaImpulse	-=	deltaVel1Dotn*c.m_jacDiagABInv;
	deltaImpulse	-=	deltaVel2Dotn*c.m_jacDiagABInv;
	const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;
	if (sum < c.m_lowerLimit)
	{
		deltaImpulse = c.m_lowerLimit-c.m_appliedImpulse;
		c.m_appliedImpulse = c.m_lowerLimit;
	}
	else
	{
		c.m_appliedImpulse = sum;
	}
	body1.internalApplyImpulse(c.m_contactNormal*body1.internalGetInvMass(),c.m_angularComponentA,deltaImpulse);
	body2.internalApplyImpulse(-c.m_contactNormal*body2.internalGetInvMass(),c.m_angularComponentB,deltaImpulse);
}


void	btPgsJacobiSolver::resolveSplitPenetrationImpulseCacheFriendly(
        btSolverBody& body1,
        btSolverBody& body2,
        const btSolverConstraint& c)
{
		if (c.m_rhsPenetration)
        {
			gNumSplitImpulseRecoveries2++;
			btScalar deltaImpulse = c.m_rhsPenetration-btScalar(c.m_appliedPushImpulse)*c.m_cfm;
			const btScalar deltaVel1Dotn	=	c.m_contactNormal.dot(body1.internalGetPushVelocity()) 	+ c.m_relpos1CrossNormal.dot(body1.internalGetTurnVelocity());
			const btScalar deltaVel2Dotn	=	-c.m_contactNormal.dot(body2.internalGetPushVelocity()) + c.m_relpos2CrossNormal.dot(body2.internalGetTurnVelocity());

			deltaImpulse	-=	deltaVel1Dotn*c.m_jacDiagABInv;
			deltaImpulse	-=	deltaVel2Dotn*c.m_jacDiagABInv;
			const btScalar sum = btScalar(c.m_appliedPushImpulse) + deltaImpulse;
			if (sum < c.m_lowerLimit)
			{
				deltaImpulse = c.m_lowerLimit-c.m_appliedPushImpulse;
				c.m_appliedPushImpulse = c.m_lowerLimit;
			}
			else
			{
				c.m_appliedPushImpulse = sum;
			}
			body1.internalApplyPushImpulse(c.m_contactNormal*body1.internalGetInvMass(),c.m_angularComponentA,deltaImpulse);
			body2.internalApplyPushImpulse(-c.m_contactNormal*body2.internalGetInvMass(),c.m_angularComponentB,deltaImpulse);
        }
}

 void btPgsJacobiSolver::resolveSplitPenetrationSIMD(btSolverBody& body1,btSolverBody& body2,const btSolverConstraint& c)
{
#ifdef USE_SIMD
	if (!c.m_rhsPenetration)
		return;

	gNumSplitImpulseRecoveries2++;

	__m128 cpAppliedImp = _mm_set1_ps(c.m_appliedPushImpulse);
	__m128	lowerLimit1 = _mm_set1_ps(c.m_lowerLimit);
	__m128	upperLimit1 = _mm_set1_ps(c.m_upperLimit);
	__m128 deltaImpulse = _mm_sub_ps(_mm_set1_ps(c.m_rhsPenetration), _mm_mul_ps(_mm_set1_ps(c.m_appliedPushImpulse),_mm_set1_ps(c.m_cfm)));
	__m128 deltaVel1Dotn	=	_mm_add_ps(btSimdDot3(c.m_contactNormal.mVec128,body1.internalGetPushVelocity().mVec128), btSimdDot3(c.m_relpos1CrossNormal.mVec128,body1.internalGetTurnVelocity().mVec128));
	__m128 deltaVel2Dotn	=	_mm_sub_ps(btSimdDot3(c.m_relpos2CrossNormal.mVec128,body2.internalGetTurnVelocity().mVec128),btSimdDot3((c.m_contactNormal).mVec128,body2.internalGetPushVelocity().mVec128));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel1Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	deltaImpulse	=	_mm_sub_ps(deltaImpulse,_mm_mul_ps(deltaVel2Dotn,_mm_set1_ps(c.m_jacDiagABInv)));
	btSimdScalar sum = _mm_add_ps(cpAppliedImp,deltaImpulse);
	btSimdScalar resultLowerLess,resultUpperLess;
	resultLowerLess = _mm_cmplt_ps(sum,lowerLimit1);
	resultUpperLess = _mm_cmplt_ps(sum,upperLimit1);
	__m128 lowMinApplied = _mm_sub_ps(lowerLimit1,cpAppliedImp);
	deltaImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowMinApplied), _mm_andnot_ps(resultLowerLess, deltaImpulse) );
	c.m_appliedPushImpulse = _mm_or_ps( _mm_and_ps(resultLowerLess, lowerLimit1), _mm_andnot_ps(resultLowerLess, sum) );
	__m128	linearComponentA = _mm_mul_ps(c.m_contactNormal.mVec128,body1.internalGetInvMass().mVec128);
	__m128	linearComponentB = _mm_mul_ps((c.m_contactNormal).mVec128,body2.internalGetInvMass().mVec128);
	__m128 impulseMagnitude = deltaImpulse;
	body1.internalGetPushVelocity().mVec128 = _mm_add_ps(body1.internalGetPushVelocity().mVec128,_mm_mul_ps(linearComponentA,impulseMagnitude));
	body1.internalGetTurnVelocity().mVec128 = _mm_add_ps(body1.internalGetTurnVelocity().mVec128 ,_mm_mul_ps(c.m_angularComponentA.mVec128,impulseMagnitude));
	body2.internalGetPushVelocity().mVec128 = _mm_sub_ps(body2.internalGetPushVelocity().mVec128,_mm_mul_ps(linearComponentB,impulseMagnitude));
	body2.internalGetTurnVelocity().mVec128 = _mm_add_ps(body2.internalGetTurnVelocity().mVec128 ,_mm_mul_ps(c.m_angularComponentB.mVec128,impulseMagnitude));
#else
	resolveSplitPenetrationImpulseCacheFriendly(body1,body2,c);
#endif
}



unsigned long btPgsJacobiSolver::btRand2()
{
	m_btSeed2 = (1664525L*m_btSeed2 + 1013904223L) & 0xffffffff;
	return m_btSeed2;
}



//See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)
int btPgsJacobiSolver::btRandInt2 (int n)
{
	// seems good; xor-fold and modulus
	const unsigned long un = static_cast<unsigned long>(n);
	unsigned long r = btRand2();

	// note: probably more aggressive than it needs to be -- might be
	//       able to get away without one or two of the innermost branches.
	if (un <= 0x00010000UL) {
		r ^= (r >> 16);
		if (un <= 0x00000100UL) {
			r ^= (r >> 8);
			if (un <= 0x00000010UL) {
				r ^= (r >> 4);
				if (un <= 0x00000004UL) {
					r ^= (r >> 2);
					if (un <= 0x00000002UL) {
						r ^= (r >> 1);
					}
				}
			}
		}
	}

	return (int) (r % un);
}



void	btPgsJacobiSolver::initSolverBody(int bodyIndex, btSolverBody* solverBody, btRigidBodyCL* rb)
{

	solverBody->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
	solverBody->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);
	solverBody->internalGetPushVelocity().setValue(0.f,0.f,0.f);
	solverBody->internalGetTurnVelocity().setValue(0.f,0.f,0.f);

	if (rb)
	{
		solverBody->m_worldTransform = getWorldTransform(rb);
		solverBody->internalSetInvMass(btVector3(rb->getInvMass(),rb->getInvMass(),rb->getInvMass()));
		solverBody->m_originalBodyIndex = bodyIndex;
		solverBody->m_angularFactor = btVector3(1,1,1);
		solverBody->m_linearFactor = btVector3(1,1,1);
		solverBody->m_linearVelocity = getLinearVelocity(rb);
		solverBody->m_angularVelocity = getAngularVelocity(rb);
	} else
	{
		solverBody->m_worldTransform.setIdentity();
		solverBody->internalSetInvMass(btVector3(0,0,0));
		solverBody->m_originalBodyIndex = bodyIndex;
		solverBody->m_angularFactor.setValue(1,1,1);
		solverBody->m_linearFactor.setValue(1,1,1);
		solverBody->m_linearVelocity.setValue(0,0,0);
		solverBody->m_angularVelocity.setValue(0,0,0);
	}


}






btScalar btPgsJacobiSolver::restitutionCurve(btScalar rel_vel, btScalar restitution)
{
	btScalar rest = restitution * -rel_vel;
	return rest;
}






void btPgsJacobiSolver::setupFrictionConstraint(btRigidBodyCL* bodies,btInertiaCL* inertias, btSolverConstraint& solverConstraint, const btVector3& normalAxis,int  solverBodyIdA,int solverBodyIdB,btContactPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,btRigidBodyCL* colObj0,btRigidBodyCL* colObj1, btScalar relaxation, btScalar desiredVelocity, btScalar cfmSlip)
{

	
	solverConstraint.m_contactNormal = normalAxis;
	btSolverBody& solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody& solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];

	btRigidBodyCL* body0 = &bodies[solverBodyA.m_originalBodyIndex];
	btRigidBodyCL* body1 = &bodies[solverBodyB.m_originalBodyIndex];


	solverConstraint.m_solverBodyIdA = solverBodyIdA;
	solverConstraint.m_solverBodyIdB = solverBodyIdB;

	solverConstraint.m_friction = cp.m_combinedFriction;
	solverConstraint.m_originalContactPoint = 0;

	solverConstraint.m_appliedImpulse = 0.f;
	solverConstraint.m_appliedPushImpulse = 0.f;

	{
		btVector3 ftorqueAxis1 = rel_pos1.cross(solverConstraint.m_contactNormal);
		solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentA = body0 ? getInvInertiaTensorWorld(&inertias[solverBodyA.m_originalBodyIndex])*ftorqueAxis1 : btVector3(0,0,0);
	}
	{
		btVector3 ftorqueAxis1 = rel_pos2.cross(-solverConstraint.m_contactNormal);
		solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentB = body1 ? getInvInertiaTensorWorld(&inertias[solverBodyB.m_originalBodyIndex])*ftorqueAxis1 : btVector3(0,0,0);
	}

	btScalar scaledDenom;

	{
		btVector3 vec;
		btScalar denom0 = 0.f;
		btScalar denom1 = 0.f;
		if (body0)
		{
			vec = ( solverConstraint.m_angularComponentA).cross(rel_pos1);
			denom0 = body0->getInvMass() + normalAxis.dot(vec);
		}
		if (body1)
		{
			vec = ( -solverConstraint.m_angularComponentB).cross(rel_pos2);
			denom1 = body1->getInvMass() + normalAxis.dot(vec);
		}

		btScalar denom;
		if (m_usePgs)
		{
			scaledDenom = denom = relaxation/(denom0+denom1);
		} else
		{
			denom = relaxation/(denom0+denom1);
			btScalar countA = body0->getInvMass() ? btScalar(m_bodyCount[solverBodyA.m_originalBodyIndex]): 1.f;
			btScalar countB = body1->getInvMass() ? btScalar(m_bodyCount[solverBodyB.m_originalBodyIndex]): 1.f;

			scaledDenom = relaxation/(denom0*countA+denom1*countB);
		}

		solverConstraint.m_jacDiagABInv = denom;
	}

	{
		

		btScalar rel_vel;
		btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(body0?solverBodyA.m_linearVelocity:btVector3(0,0,0)) 
			+ solverConstraint.m_relpos1CrossNormal.dot(body0?solverBodyA.m_angularVelocity:btVector3(0,0,0));
		btScalar vel2Dotn = -solverConstraint.m_contactNormal.dot(body1?solverBodyB.m_linearVelocity:btVector3(0,0,0)) 
			+ solverConstraint.m_relpos2CrossNormal.dot(body1?solverBodyB.m_angularVelocity:btVector3(0,0,0));

		rel_vel = vel1Dotn+vel2Dotn;

//		btScalar positionalError = 0.f;

		btSimdScalar velocityError =  desiredVelocity - rel_vel;
		btSimdScalar	velocityImpulse = velocityError * btSimdScalar(scaledDenom);//solverConstraint.m_jacDiagABInv);
		solverConstraint.m_rhs = velocityImpulse;
		solverConstraint.m_cfm = cfmSlip;
		solverConstraint.m_lowerLimit = 0;
		solverConstraint.m_upperLimit = 1e10f;
		
	}
}

btSolverConstraint&	btPgsJacobiSolver::addFrictionConstraint(btRigidBodyCL* bodies,btInertiaCL* inertias, const btVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btContactPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,btRigidBodyCL* colObj0,btRigidBodyCL* colObj1, btScalar relaxation, btScalar desiredVelocity, btScalar cfmSlip)
{
	btSolverConstraint& solverConstraint = m_tmpSolverContactFrictionConstraintPool.expandNonInitializing();
	solverConstraint.m_frictionIndex = frictionIndex;
	setupFrictionConstraint(bodies,inertias,solverConstraint, normalAxis, solverBodyIdA, solverBodyIdB, cp, rel_pos1, rel_pos2, 
							colObj0, colObj1, relaxation, desiredVelocity, cfmSlip);
	return solverConstraint;
}


void btPgsJacobiSolver::setupRollingFrictionConstraint(btRigidBodyCL* bodies,btInertiaCL* inertias,	btSolverConstraint& solverConstraint, const btVector3& normalAxis1,int solverBodyIdA,int  solverBodyIdB,
									btContactPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,
									btRigidBodyCL* colObj0,btRigidBodyCL* colObj1, btScalar relaxation, 
									btScalar desiredVelocity, btScalar cfmSlip)

{
	btVector3 normalAxis(0,0,0);


	solverConstraint.m_contactNormal = normalAxis;
	btSolverBody& solverBodyA = m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody& solverBodyB = m_tmpSolverBodyPool[solverBodyIdB];

	btRigidBodyCL* body0 = &bodies[m_tmpSolverBodyPool[solverBodyIdA].m_originalBodyIndex];
	btRigidBodyCL* body1 = &bodies[m_tmpSolverBodyPool[solverBodyIdB].m_originalBodyIndex];

	solverConstraint.m_solverBodyIdA = solverBodyIdA;
	solverConstraint.m_solverBodyIdB = solverBodyIdB;

	solverConstraint.m_friction = cp.m_combinedRollingFriction;
	solverConstraint.m_originalContactPoint = 0;

	solverConstraint.m_appliedImpulse = 0.f;
	solverConstraint.m_appliedPushImpulse = 0.f;

	{
		btVector3 ftorqueAxis1 = -normalAxis1;
		solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentA = body0 ? getInvInertiaTensorWorld(&inertias[solverBodyA.m_originalBodyIndex])*ftorqueAxis1 : btVector3(0,0,0);
	}
	{
		btVector3 ftorqueAxis1 = normalAxis1;
		solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentB = body1 ? getInvInertiaTensorWorld(&inertias[solverBodyB.m_originalBodyIndex])*ftorqueAxis1 : btVector3(0,0,0);
	}


	{
		btVector3 iMJaA = body0?getInvInertiaTensorWorld(&inertias[solverBodyA.m_originalBodyIndex])*solverConstraint.m_relpos1CrossNormal:btVector3(0,0,0);
		btVector3 iMJaB = body1?getInvInertiaTensorWorld(&inertias[solverBodyB.m_originalBodyIndex])*solverConstraint.m_relpos2CrossNormal:btVector3(0,0,0);
		btScalar sum = 0;
		sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
		sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);
		solverConstraint.m_jacDiagABInv = btScalar(1.)/sum;
	}

	{
		

		btScalar rel_vel;
		btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(body0?solverBodyA.m_linearVelocity:btVector3(0,0,0)) 
			+ solverConstraint.m_relpos1CrossNormal.dot(body0?solverBodyA.m_angularVelocity:btVector3(0,0,0));
		btScalar vel2Dotn = -solverConstraint.m_contactNormal.dot(body1?solverBodyB.m_linearVelocity:btVector3(0,0,0)) 
			+ solverConstraint.m_relpos2CrossNormal.dot(body1?solverBodyB.m_angularVelocity:btVector3(0,0,0));

		rel_vel = vel1Dotn+vel2Dotn;

//		btScalar positionalError = 0.f;

		btSimdScalar velocityError =  desiredVelocity - rel_vel;
		btSimdScalar	velocityImpulse = velocityError * btSimdScalar(solverConstraint.m_jacDiagABInv);
		solverConstraint.m_rhs = velocityImpulse;
		solverConstraint.m_cfm = cfmSlip;
		solverConstraint.m_lowerLimit = 0;
		solverConstraint.m_upperLimit = 1e10f;
		
	}
}








btSolverConstraint&	btPgsJacobiSolver::addRollingFrictionConstraint(btRigidBodyCL* bodies,btInertiaCL* inertias,const btVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btContactPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,btRigidBodyCL* colObj0,btRigidBodyCL* colObj1, btScalar relaxation, btScalar desiredVelocity, btScalar cfmSlip)
{
	btSolverConstraint& solverConstraint = m_tmpSolverContactRollingFrictionConstraintPool.expandNonInitializing();
	solverConstraint.m_frictionIndex = frictionIndex;
	setupRollingFrictionConstraint(bodies,inertias,solverConstraint, normalAxis, solverBodyIdA, solverBodyIdB, cp, rel_pos1, rel_pos2, 
							colObj0, colObj1, relaxation, desiredVelocity, cfmSlip);
	return solverConstraint;
}


int	btPgsJacobiSolver::getOrInitSolverBody(int bodyIndex, btRigidBodyCL* bodies,btInertiaCL* inertias)
{
	//btAssert(bodyIndex< m_tmpSolverBodyPool.size());

	btRigidBodyCL& body = bodies[bodyIndex];
	int curIndex = -1;
	if (m_usePgs || body.getInvMass()==0.f)
	{
		if (m_bodyCount[bodyIndex]<0)
		{
			curIndex = m_tmpSolverBodyPool.size();
			btSolverBody& solverBody = m_tmpSolverBodyPool.expand();
			initSolverBody(bodyIndex,&solverBody,&body);
			solverBody.m_originalBodyIndex = bodyIndex;
			m_bodyCount[bodyIndex] = curIndex;
		} else
		{
			curIndex = m_bodyCount[bodyIndex];
		}
	} else
	{
		btAssert(m_bodyCount[bodyIndex]>0);
		m_bodyCountCheck[bodyIndex]++;
		curIndex = m_tmpSolverBodyPool.size();
		btSolverBody& solverBody = m_tmpSolverBodyPool.expand();
		initSolverBody(bodyIndex,&solverBody,&body);
		solverBody.m_originalBodyIndex = bodyIndex;
	}

	btAssert(curIndex>=0);
	return curIndex;

}
#include <stdio.h>


void btPgsJacobiSolver::setupContactConstraint(btRigidBodyCL* bodies, btInertiaCL* inertias,btSolverConstraint& solverConstraint, 
																 int solverBodyIdA, int solverBodyIdB,
																 btContactPoint& cp, const btContactSolverInfo& infoGlobal,
																 btVector3& vel, btScalar& rel_vel, btScalar& relaxation,
																 btVector3& rel_pos1, btVector3& rel_pos2)
{
			
			const btVector3& pos1 = cp.getPositionWorldOnA();
			const btVector3& pos2 = cp.getPositionWorldOnB();

			btSolverBody* bodyA = &m_tmpSolverBodyPool[solverBodyIdA];
			btSolverBody* bodyB = &m_tmpSolverBodyPool[solverBodyIdB];

			btRigidBodyCL* rb0 = &bodies[bodyA->m_originalBodyIndex];
			btRigidBodyCL* rb1 = &bodies[bodyB->m_originalBodyIndex];

//			btVector3 rel_pos1 = pos1 - colObj0->getWorldTransform().getOrigin(); 
//			btVector3 rel_pos2 = pos2 - colObj1->getWorldTransform().getOrigin();
			rel_pos1 = pos1 - bodyA->getWorldTransform().getOrigin(); 
			rel_pos2 = pos2 - bodyB->getWorldTransform().getOrigin();

			relaxation = 1.f;

			btVector3 torqueAxis0 = rel_pos1.cross(cp.m_normalWorldOnB);
			solverConstraint.m_angularComponentA = rb0 ? getInvInertiaTensorWorld(&inertias[bodyA->m_originalBodyIndex])*torqueAxis0 : btVector3(0,0,0);
			btVector3 torqueAxis1 = rel_pos2.cross(cp.m_normalWorldOnB);		
			solverConstraint.m_angularComponentB = rb1 ? getInvInertiaTensorWorld(&inertias[bodyB->m_originalBodyIndex])*-torqueAxis1 : btVector3(0,0,0);

			btScalar scaledDenom;
				{
#ifdef COMPUTE_IMPULSE_DENOM
					btScalar denom0 = rb0->computeImpulseDenominator(pos1,cp.m_normalWorldOnB);
					btScalar denom1 = rb1->computeImpulseDenominator(pos2,cp.m_normalWorldOnB);
#else							
					btVector3 vec;
					btScalar denom0 = 0.f;
					btScalar denom1 = 0.f;
					if (rb0)
					{
						vec = ( solverConstraint.m_angularComponentA).cross(rel_pos1);
						denom0 = rb0->getInvMass() + cp.m_normalWorldOnB.dot(vec);
					}
					if (rb1)
					{
						vec = ( -solverConstraint.m_angularComponentB).cross(rel_pos2);
						denom1 = rb1->getInvMass() + cp.m_normalWorldOnB.dot(vec);
					}
#endif //COMPUTE_IMPULSE_DENOM		

					
					btScalar denom;
					if (m_usePgs)
					{
						scaledDenom = denom = relaxation/(denom0+denom1);
					} else
					{
						denom = relaxation/(denom0+denom1);

						btScalar countA = rb0->m_invMass? btScalar(m_bodyCount[bodyA->m_originalBodyIndex]) : 1.f;
						btScalar countB = rb1->m_invMass? btScalar(m_bodyCount[bodyB->m_originalBodyIndex]) : 1.f;
						scaledDenom = relaxation/(denom0*countA+denom1*countB);
					}
					solverConstraint.m_jacDiagABInv = denom;
				}

				solverConstraint.m_contactNormal = cp.m_normalWorldOnB;
				solverConstraint.m_relpos1CrossNormal = torqueAxis0;
				solverConstraint.m_relpos2CrossNormal = -torqueAxis1;

				btScalar restitution = 0.f;
				btScalar penetration = cp.getDistance()+infoGlobal.m_linearSlop;

				{
					btVector3 vel1,vel2;

					vel1 = rb0? getVelocityInLocalPoint(rb0,rel_pos1) : btVector3(0,0,0);
					vel2 = rb1? getVelocityInLocalPoint(rb1, rel_pos2) : btVector3(0,0,0);

	//			btVector3 vel2 = rb1 ? rb1->getVelocityInLocalPoint(rel_pos2) : btVector3(0,0,0);
					vel  = vel1 - vel2;
					rel_vel = cp.m_normalWorldOnB.dot(vel);

					

					solverConstraint.m_friction = cp.m_combinedFriction;

				
					restitution =  restitutionCurve(rel_vel, cp.m_combinedRestitution);
					if (restitution <= btScalar(0.))
					{
						restitution = 0.f;
					};
				}


				///warm starting (or zero if disabled)
				if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
				{
					solverConstraint.m_appliedImpulse = cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;
					if (rb0)
						bodyA->internalApplyImpulse(solverConstraint.m_contactNormal*bodyA->internalGetInvMass(),solverConstraint.m_angularComponentA,solverConstraint.m_appliedImpulse);
					if (rb1)
						bodyB->internalApplyImpulse(solverConstraint.m_contactNormal*bodyB->internalGetInvMass(),-solverConstraint.m_angularComponentB,-(btScalar)solverConstraint.m_appliedImpulse);
				} else
				{
					solverConstraint.m_appliedImpulse = 0.f;
				}

				solverConstraint.m_appliedPushImpulse = 0.f;

				{
					btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(rb0?bodyA->m_linearVelocity:btVector3(0,0,0)) 
						+ solverConstraint.m_relpos1CrossNormal.dot(rb0?bodyA->m_angularVelocity:btVector3(0,0,0));
					btScalar vel2Dotn = -solverConstraint.m_contactNormal.dot(rb1?bodyB->m_linearVelocity:btVector3(0,0,0)) 
						+ solverConstraint.m_relpos2CrossNormal.dot(rb1?bodyB->m_angularVelocity:btVector3(0,0,0));
					btScalar rel_vel = vel1Dotn+vel2Dotn;

					btScalar positionalError = 0.f;
					btScalar	velocityError = restitution - rel_vel;// * damping;
					

					btScalar erp = infoGlobal.m_erp2;
					if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
					{
						erp = infoGlobal.m_erp;
					}

					if (penetration>0)
					{
						positionalError = 0;

						velocityError -= penetration / infoGlobal.m_timeStep;
					} else
					{
						positionalError = -penetration * erp/infoGlobal.m_timeStep;
					}

					btScalar  penetrationImpulse = positionalError*scaledDenom;//solverConstraint.m_jacDiagABInv;
					btScalar velocityImpulse = velocityError *scaledDenom;//solverConstraint.m_jacDiagABInv;

					if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
					{
						//combine position and velocity into rhs
						solverConstraint.m_rhs = penetrationImpulse+velocityImpulse;
						solverConstraint.m_rhsPenetration = 0.f;

					} else
					{
						//split position and velocity into rhs and m_rhsPenetration
						solverConstraint.m_rhs = velocityImpulse;
						solverConstraint.m_rhsPenetration = penetrationImpulse;
					}
					solverConstraint.m_cfm = 0.f;
					solverConstraint.m_lowerLimit = 0;
					solverConstraint.m_upperLimit = 1e10f;
				}




}



void btPgsJacobiSolver::setFrictionConstraintImpulse( btRigidBodyCL* bodies, btInertiaCL* inertias,btSolverConstraint& solverConstraint, 
																		int solverBodyIdA, int solverBodyIdB,
																 btContactPoint& cp, const btContactSolverInfo& infoGlobal)
{

	btSolverBody* bodyA = &m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody* bodyB = &m_tmpSolverBodyPool[solverBodyIdB];


	{
		btSolverConstraint& frictionConstraint1 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex];
		if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
		{
			frictionConstraint1.m_appliedImpulse = cp.m_appliedImpulseLateral1 * infoGlobal.m_warmstartingFactor;
			if (bodies[bodyA->m_originalBodyIndex].m_invMass)
				bodyA->internalApplyImpulse(frictionConstraint1.m_contactNormal*bodies[bodyA->m_originalBodyIndex].m_invMass,frictionConstraint1.m_angularComponentA,frictionConstraint1.m_appliedImpulse);
			if (bodies[bodyB->m_originalBodyIndex].m_invMass)
				bodyB->internalApplyImpulse(frictionConstraint1.m_contactNormal*bodies[bodyB->m_originalBodyIndex].m_invMass,-frictionConstraint1.m_angularComponentB,-(btScalar)frictionConstraint1.m_appliedImpulse);
		} else
		{
			frictionConstraint1.m_appliedImpulse = 0.f;
		}
	}

	if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
	{
		btSolverConstraint& frictionConstraint2 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex+1];
		if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
		{
			frictionConstraint2.m_appliedImpulse = cp.m_appliedImpulseLateral2  * infoGlobal.m_warmstartingFactor;
			if (bodies[bodyA->m_originalBodyIndex].m_invMass)
				bodyA->internalApplyImpulse(frictionConstraint2.m_contactNormal*bodies[bodyA->m_originalBodyIndex].m_invMass,frictionConstraint2.m_angularComponentA,frictionConstraint2.m_appliedImpulse);
			if (bodies[bodyB->m_originalBodyIndex].m_invMass)
				bodyB->internalApplyImpulse(frictionConstraint2.m_contactNormal*bodies[bodyB->m_originalBodyIndex].m_invMass,-frictionConstraint2.m_angularComponentB,-(btScalar)frictionConstraint2.m_appliedImpulse);
		} else
		{
			frictionConstraint2.m_appliedImpulse = 0.f;
		}
	}
}




void	btPgsJacobiSolver::convertContact(btRigidBodyCL* bodies, btInertiaCL* inertias,btContact4* manifold,const btContactSolverInfo& infoGlobal)
{
	btRigidBodyCL* colObj0=0,*colObj1=0;

	
	int solverBodyIdA = getOrInitSolverBody(manifold->getBodyA(),bodies,inertias);
	int solverBodyIdB = getOrInitSolverBody(manifold->getBodyB(),bodies,inertias);

//	btRigidBody* bodyA = btRigidBody::upcast(colObj0);
//	btRigidBody* bodyB = btRigidBody::upcast(colObj1);

	btSolverBody* solverBodyA = &m_tmpSolverBodyPool[solverBodyIdA];
	btSolverBody* solverBodyB = &m_tmpSolverBodyPool[solverBodyIdB];



	///avoid collision response between two static objects
	if (solverBodyA->m_invMass.isZero() && solverBodyB->m_invMass.isZero())
		return;

	int rollingFriction=1;
	int numContacts = getNumContacts(manifold);
	for (int j=0;j<numContacts;j++)
	{

		btContactPoint cp;
		getContactPoint(manifold,j,cp);

		if (cp.getDistance() <= getContactProcessingThreshold(manifold))
		{
			btVector3 rel_pos1;
			btVector3 rel_pos2;
			btScalar relaxation;
			btScalar rel_vel;
			btVector3 vel;

			int frictionIndex = m_tmpSolverContactConstraintPool.size();
			btSolverConstraint& solverConstraint = m_tmpSolverContactConstraintPool.expandNonInitializing();
//			btRigidBody* rb0 = btRigidBody::upcast(colObj0);
//			btRigidBody* rb1 = btRigidBody::upcast(colObj1);
			solverConstraint.m_solverBodyIdA = solverBodyIdA;
			solverConstraint.m_solverBodyIdB = solverBodyIdB;

			solverConstraint.m_originalContactPoint = &cp;

			setupContactConstraint(bodies,inertias,solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal, vel, rel_vel, relaxation, rel_pos1, rel_pos2);

//			const btVector3& pos1 = cp.getPositionWorldOnA();
//			const btVector3& pos2 = cp.getPositionWorldOnB();

			/////setup the friction constraints

			solverConstraint.m_frictionIndex = m_tmpSolverContactFrictionConstraintPool.size();

			btVector3 angVelA,angVelB;
			solverBodyA->getAngularVelocity(angVelA);
			solverBodyB->getAngularVelocity(angVelB);			
			btVector3 relAngVel = angVelB-angVelA;

			if ((cp.m_combinedRollingFriction>0.f) && (rollingFriction>0))
			{
				//only a single rollingFriction per manifold
				rollingFriction--;
				if (relAngVel.length()>infoGlobal.m_singleAxisRollingFrictionThreshold)
				{
					relAngVel.normalize();
					if (relAngVel.length()>0.001)
						addRollingFrictionConstraint(bodies,inertias,relAngVel,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);

				} else
				{
					addRollingFrictionConstraint(bodies,inertias,cp.m_normalWorldOnB,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
					btVector3 axis0,axis1;
					btPlaneSpace1(cp.m_normalWorldOnB,axis0,axis1);
					if (axis0.length()>0.001)
						addRollingFrictionConstraint(bodies,inertias,axis0,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
					if (axis1.length()>0.001)
						addRollingFrictionConstraint(bodies,inertias,axis1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
		
				}
			}

			///Bullet has several options to set the friction directions
			///By default, each contact has only a single friction direction that is recomputed automatically very frame 
			///based on the relative linear velocity.
			///If the relative velocity it zero, it will automatically compute a friction direction.
			
			///You can also enable two friction directions, using the SOLVER_USE_2_FRICTION_DIRECTIONS.
			///In that case, the second friction direction will be orthogonal to both contact normal and first friction direction.
			///
			///If you choose SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION, then the friction will be independent from the relative projected velocity.
			///
			///The user can manually override the friction directions for certain contacts using a contact callback, 
			///and set the cp.m_lateralFrictionInitialized to true
			///In that case, you can set the target relative motion in each friction direction (cp.m_contactMotion1 and cp.m_contactMotion2)
			///this will give a conveyor belt effect
			///
			if (!(infoGlobal.m_solverMode & SOLVER_ENABLE_FRICTION_DIRECTION_CACHING) || !cp.m_lateralFrictionInitialized)
			{
				cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
				btScalar lat_rel_vel = cp.m_lateralFrictionDir1.length2();
				if (!(infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION) && lat_rel_vel > SIMD_EPSILON)
				{
					cp.m_lateralFrictionDir1 *= 1.f/btSqrt(lat_rel_vel);
					if((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
					{
						cp.m_lateralFrictionDir2 = cp.m_lateralFrictionDir1.cross(cp.m_normalWorldOnB);
						cp.m_lateralFrictionDir2.normalize();//??
						addFrictionConstraint(bodies,inertias,cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);

					}

					addFrictionConstraint(bodies,inertias,cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);

				} else
				{
					btPlaneSpace1(cp.m_normalWorldOnB,cp.m_lateralFrictionDir1,cp.m_lateralFrictionDir2);

					if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
					{
						addFrictionConstraint(bodies,inertias,cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
					}

					addFrictionConstraint(bodies,inertias,cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);

					if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) && (infoGlobal.m_solverMode & SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION))
					{
						cp.m_lateralFrictionInitialized = true;
					}
				}

			} else
			{
				addFrictionConstraint(bodies,inertias,cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation,cp.m_contactMotion1, cp.m_contactCFM1);

				if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
					addFrictionConstraint(bodies,inertias,cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation, cp.m_contactMotion2, cp.m_contactCFM2);

				setFrictionConstraintImpulse( bodies,inertias,solverConstraint, solverBodyIdA, solverBodyIdB, cp, infoGlobal);
			}
		

			

		}
	}
}

btScalar btPgsJacobiSolver::solveGroupCacheFriendlySetup(btRigidBodyCL* bodies, btInertiaCL* inertias, int numBodies, btContact4* manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal)
{
	BT_PROFILE("solveGroupCacheFriendlySetup");


	m_maxOverrideNumSolverIterations = 0;



	m_tmpSolverBodyPool.resize(0);
	
	
	m_bodyCount.resize(0);
	m_bodyCount.resize(numBodies,0);
	m_bodyCountCheck.resize(0);
	m_bodyCountCheck.resize(numBodies,0);
	
	m_deltaLinearVelocities.resize(0);
	m_deltaLinearVelocities.resize(numBodies,btVector3(0,0,0));
	m_deltaAngularVelocities.resize(0);
	m_deltaAngularVelocities.resize(numBodies,btVector3(0,0,0));
	
	int totalBodies = 0;

	for (int i=0;i<numManifolds;i++)
	{
		int bodyIndexA = manifoldPtr[i].getBodyA();
		int bodyIndexB = manifoldPtr[i].getBodyB();
		if (m_usePgs)
		{
			m_bodyCount[bodyIndexA]=-1;
			m_bodyCount[bodyIndexB]=-1;
		} else
		{
			if (bodies[bodyIndexA].getInvMass())
			{
				//m_bodyCount[bodyIndexA]+=manifoldPtr[i].getNPoints();
				m_bodyCount[bodyIndexA]++;
			}
			else
				m_bodyCount[bodyIndexA]=-1;

			if (bodies[bodyIndexB].getInvMass())
			//	m_bodyCount[bodyIndexB]+=manifoldPtr[i].getNPoints();
				m_bodyCount[bodyIndexB]++;
			else
				m_bodyCount[bodyIndexB]=-1;
		}

	}


	
	if (1)
	{
		int j;
		for (j=0;j<numConstraints;j++)
		{
			btTypedConstraint* constraint = constraints[j];
			constraint->buildJacobian();
			constraint->internalSetAppliedImpulse(0.0f);
		}
	}

	//btRigidBody* rb0=0,*rb1=0;
	//if (1)
	{
		{

			int totalNumRows = 0;
			int i;
			
			m_tmpConstraintSizesPool.resizeNoInitialize(numConstraints);
			//calculate the total number of contraint rows
			for (i=0;i<numConstraints;i++)
			{
				btTypedConstraint::btConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];
				btJointFeedback* fb = constraints[i]->getJointFeedback();
				if (fb)
				{
					fb->m_appliedForceBodyA.setZero();
					fb->m_appliedTorqueBodyA.setZero();
					fb->m_appliedForceBodyB.setZero();
					fb->m_appliedTorqueBodyB.setZero();
				}

				if (constraints[i]->isEnabled())
				{
				}
				if (constraints[i]->isEnabled())
				{
					constraints[i]->getInfo1(&info1);
				} else
				{
					info1.m_numConstraintRows = 0;
					info1.nub = 0;
				}
				totalNumRows += info1.m_numConstraintRows;
			}
			m_tmpSolverNonContactConstraintPool.resizeNoInitialize(totalNumRows);

			
#ifndef DISABLE_JOINTS
			///setup the btSolverConstraints
			int currentRow = 0;

			for (i=0;i<numConstraints;i++)
			{
				const btTypedConstraint::btConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];
				
				if (info1.m_numConstraintRows)
				{
					btAssert(currentRow<totalNumRows);

					btSolverConstraint* currentConstraintRow = &m_tmpSolverNonContactConstraintPool[currentRow];
					btTypedConstraint* constraint = constraints[i];
					btRigidBody& rbA = constraint->getRigidBodyA();
					btRigidBody& rbB = constraint->getRigidBodyB();

                    int solverBodyIdA = getOrInitSolverBody(rbA);
                    int solverBodyIdB = getOrInitSolverBody(rbB);

                    btSolverBody* bodyAPtr = &m_tmpSolverBodyPool[solverBodyIdA];
                    btSolverBody* bodyBPtr = &m_tmpSolverBodyPool[solverBodyIdB];




					int overrideNumSolverIterations = constraint->getOverrideNumSolverIterations() > 0 ? constraint->getOverrideNumSolverIterations() : infoGlobal.m_numIterations;
					if (overrideNumSolverIterations>m_maxOverrideNumSolverIterations)
						m_maxOverrideNumSolverIterations = overrideNumSolverIterations;


					int j;
					for ( j=0;j<info1.m_numConstraintRows;j++)
					{
						memset(&currentConstraintRow[j],0,sizeof(btSolverConstraint));
						currentConstraintRow[j].m_lowerLimit = -SIMD_INFINITY;
						currentConstraintRow[j].m_upperLimit = SIMD_INFINITY;
						currentConstraintRow[j].m_appliedImpulse = 0.f;
						currentConstraintRow[j].m_appliedPushImpulse = 0.f;
						currentConstraintRow[j].m_solverBodyIdA = solverBodyIdA;
						currentConstraintRow[j].m_solverBodyIdB = solverBodyIdB;
						currentConstraintRow[j].m_overrideNumSolverIterations = overrideNumSolverIterations;
					}

					bodyAPtr->internalGetDeltaLinearVelocity().setValue(0.f,0.f,0.f);
					bodyAPtr->internalGetDeltaAngularVelocity().setValue(0.f,0.f,0.f);
					bodyAPtr->internalGetPushVelocity().setValue(0.f,0.f,0.f);
					bodyAPtr->internalGetTurnVelocity().setValue(0.f,0.f,0.f);
					bodyBPtr->internalGetDeltaLinearVelocity().setValue(0.f,0.f,0.f);
					bodyBPtr->internalGetDeltaAngularVelocity().setValue(0.f,0.f,0.f);
					bodyBPtr->internalGetPushVelocity().setValue(0.f,0.f,0.f);
					bodyBPtr->internalGetTurnVelocity().setValue(0.f,0.f,0.f);


					btTypedConstraint::btConstraintInfo2 info2;
					info2.fps = 1.f/infoGlobal.m_timeStep;
					info2.erp = infoGlobal.m_erp;
					info2.m_J1linearAxis = currentConstraintRow->m_contactNormal;
					info2.m_J1angularAxis = currentConstraintRow->m_relpos1CrossNormal;
					info2.m_J2linearAxis = 0;
					info2.m_J2angularAxis = currentConstraintRow->m_relpos2CrossNormal;
					info2.rowskip = sizeof(btSolverConstraint)/sizeof(btScalar);//check this
					///the size of btSolverConstraint needs be a multiple of btScalar
		            btAssert(info2.rowskip*sizeof(btScalar)== sizeof(btSolverConstraint));
					info2.m_constraintError = &currentConstraintRow->m_rhs;
					currentConstraintRow->m_cfm = infoGlobal.m_globalCfm;
					info2.m_damping = infoGlobal.m_damping;
					info2.cfm = &currentConstraintRow->m_cfm;
					info2.m_lowerLimit = &currentConstraintRow->m_lowerLimit;
					info2.m_upperLimit = &currentConstraintRow->m_upperLimit;
					info2.m_numIterations = infoGlobal.m_numIterations;
					constraints[i]->getInfo2(&info2);

					///finalize the constraint setup
					for ( j=0;j<info1.m_numConstraintRows;j++)
					{
						btSolverConstraint& solverConstraint = currentConstraintRow[j];

						if (solverConstraint.m_upperLimit>=constraints[i]->getBreakingImpulseThreshold())
						{
							solverConstraint.m_upperLimit = constraints[i]->getBreakingImpulseThreshold();
						}

						if (solverConstraint.m_lowerLimit<=-constraints[i]->getBreakingImpulseThreshold())
						{
							solverConstraint.m_lowerLimit = -constraints[i]->getBreakingImpulseThreshold();
						}

						solverConstraint.m_originalContactPoint = constraint;

						{
							const btVector3& ftorqueAxis1 = solverConstraint.m_relpos1CrossNormal;
							solverConstraint.m_angularComponentA = constraint->getRigidBodyA().getInvInertiaTensorWorld()*ftorqueAxis1*constraint->getRigidBodyA().getAngularFactor();
						}
						{
							const btVector3& ftorqueAxis2 = solverConstraint.m_relpos2CrossNormal;
							solverConstraint.m_angularComponentB = constraint->getRigidBodyB().getInvInertiaTensorWorld()*ftorqueAxis2*constraint->getRigidBodyB().getAngularFactor();
						}

						{
							btVector3 iMJlA = solverConstraint.m_contactNormal*rbA.getInvMass();
							btVector3 iMJaA = rbA.getInvInertiaTensorWorld()*solverConstraint.m_relpos1CrossNormal;
							btVector3 iMJlB = solverConstraint.m_contactNormal*rbB.getInvMass();//sign of normal?
							btVector3 iMJaB = rbB.getInvInertiaTensorWorld()*solverConstraint.m_relpos2CrossNormal;

							btScalar sum = iMJlA.dot(solverConstraint.m_contactNormal);
							sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
							sum += iMJlB.dot(solverConstraint.m_contactNormal);
							sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);
							btScalar fsum = btFabs(sum);
							btAssert(fsum > SIMD_EPSILON);
							solverConstraint.m_jacDiagABInv = fsum>SIMD_EPSILON?btScalar(1.)/sum : 0.f;
						}


						///fix rhs
						///todo: add force/torque accelerators
						{
							btScalar rel_vel;
							btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(rbA.getLinearVelocity()) + solverConstraint.m_relpos1CrossNormal.dot(rbA.getAngularVelocity());
							btScalar vel2Dotn = -solverConstraint.m_contactNormal.dot(rbB.getLinearVelocity()) + solverConstraint.m_relpos2CrossNormal.dot(rbB.getAngularVelocity());

							rel_vel = vel1Dotn+vel2Dotn;

							btScalar restitution = 0.f;
							btScalar positionalError = solverConstraint.m_rhs;//already filled in by getConstraintInfo2
							btScalar	velocityError = restitution - rel_vel * info2.m_damping;
							btScalar	penetrationImpulse = positionalError*solverConstraint.m_jacDiagABInv;
							btScalar	velocityImpulse = velocityError *solverConstraint.m_jacDiagABInv;
							solverConstraint.m_rhs = penetrationImpulse+velocityImpulse;
							solverConstraint.m_appliedImpulse = 0.f;

						}
					}
				}
				currentRow+=m_tmpConstraintSizesPool[i].m_numConstraintRows;
			}
#endif //DISABLE_JOINTS
		}


		{
			int i;

			for (i=0;i<numManifolds;i++)
			{
				btContact4& manifold = manifoldPtr[i];
				convertContact(bodies,inertias,&manifold,infoGlobal);
			}
		}
	}

//	btContactSolverInfo info = infoGlobal;


	int numNonContactPool = m_tmpSolverNonContactConstraintPool.size();
	int numConstraintPool = m_tmpSolverContactConstraintPool.size();
	int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.size();

	///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
	m_orderNonContactConstraintPool.resizeNoInitialize(numNonContactPool);
	if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
		m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool*2);
	else
		m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool);

	m_orderFrictionConstraintPool.resizeNoInitialize(numFrictionPool);
	{
		int i;
		for (i=0;i<numNonContactPool;i++)
		{
			m_orderNonContactConstraintPool[i] = i;
		}
		for (i=0;i<numConstraintPool;i++)
		{
			m_orderTmpConstraintPool[i] = i;
		}
		for (i=0;i<numFrictionPool;i++)
		{
			m_orderFrictionConstraintPool[i] = i;
		}
	}

	return 0.f;

}


btScalar btPgsJacobiSolver::solveSingleIteration(int iteration,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal)
{

	int numNonContactPool = m_tmpSolverNonContactConstraintPool.size();
	int numConstraintPool = m_tmpSolverContactConstraintPool.size();
	int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.size();
	
	if (infoGlobal.m_solverMode & SOLVER_RANDMIZE_ORDER)
	{
		if (1)			// uncomment this for a bit less random ((iteration & 7) == 0)
		{

			for (int j=0; j<numNonContactPool; ++j) {
				int tmp = m_orderNonContactConstraintPool[j];
				int swapi = btRandInt2(j+1);
				m_orderNonContactConstraintPool[j] = m_orderNonContactConstraintPool[swapi];
				m_orderNonContactConstraintPool[swapi] = tmp;
			}

			//contact/friction constraints are not solved more than 
			if (iteration< infoGlobal.m_numIterations)
			{
				for (int j=0; j<numConstraintPool; ++j) {
					int tmp = m_orderTmpConstraintPool[j];
					int swapi = btRandInt2(j+1);
					m_orderTmpConstraintPool[j] = m_orderTmpConstraintPool[swapi];
					m_orderTmpConstraintPool[swapi] = tmp;
				}

				for (int j=0; j<numFrictionPool; ++j) {
					int tmp = m_orderFrictionConstraintPool[j];
					int swapi = btRandInt2(j+1);
					m_orderFrictionConstraintPool[j] = m_orderFrictionConstraintPool[swapi];
					m_orderFrictionConstraintPool[swapi] = tmp;
				}
			}
		}
	}

	if (infoGlobal.m_solverMode & SOLVER_SIMD)
	{
		///solve all joint constraints, using SIMD, if available
		for (int j=0;j<m_tmpSolverNonContactConstraintPool.size();j++)
		{
			btSolverConstraint& constraint = m_tmpSolverNonContactConstraintPool[m_orderNonContactConstraintPool[j]];
			if (iteration < constraint.m_overrideNumSolverIterations)
				resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[constraint.m_solverBodyIdA],m_tmpSolverBodyPool[constraint.m_solverBodyIdB],constraint);
		}

		if (iteration< infoGlobal.m_numIterations)
		{
#ifndef DISABLE_JOINTS
			for (int j=0;j<numConstraints;j++)
			{
                if (constraints[j]->isEnabled())
                {
                    int bodyAid = getOrInitSolverBody(constraints[j]->getRigidBodyA());
                    int bodyBid = getOrInitSolverBody(constraints[j]->getRigidBodyB());
                    btSolverBody& bodyA = m_tmpSolverBodyPool[bodyAid];
                    btSolverBody& bodyB = m_tmpSolverBodyPool[bodyBid];
                    constraints[j]->solveConstraintObsolete(bodyA,bodyB,infoGlobal.m_timeStep);
                }
			}
#endif


			///solve all contact constraints using SIMD, if available
			if (infoGlobal.m_solverMode & SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS)
			{
				int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
				int multiplier = (infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS)? 2 : 1;

				for (int c=0;c<numPoolConstraints;c++)
				{
					btScalar totalImpulse =0;

					{
						const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[c]];
						resolveSingleConstraintRowLowerLimitSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
						totalImpulse = solveManifold.m_appliedImpulse;
					}
					bool applyFriction = true;
					if (applyFriction)
					{
						{

							btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[c*multiplier]];

							if (totalImpulse>btScalar(0))
							{
								solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
								solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;

								resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
							}
						}

						if (infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS)
						{

							btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[c*multiplier+1]];
				
							if (totalImpulse>btScalar(0))
							{
								solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
								solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;

								resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
							}
						}
					}
				}

			}
			else//SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS
			{
				//solve the friction constraints after all contact constraints, don't interleave them
				int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
				int j;

				for (j=0;j<numPoolConstraints;j++)
				{
					const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];
					resolveSingleConstraintRowLowerLimitSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);

				}

				if (!m_usePgs)
					averageVelocities();
				

				///solve all friction constraints, using SIMD, if available

				int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.size();
				for (j=0;j<numFrictionPoolConstraints;j++)
				{
					btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
					btScalar totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;

					if (totalImpulse>btScalar(0))
					{
						solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
						solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;

						resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
					}
				}

				
				int numRollingFrictionPoolConstraints = m_tmpSolverContactRollingFrictionConstraintPool.size();
				for (j=0;j<numRollingFrictionPoolConstraints;j++)
				{

					btSolverConstraint& rollingFrictionConstraint = m_tmpSolverContactRollingFrictionConstraintPool[j];
					btScalar totalImpulse = m_tmpSolverContactConstraintPool[rollingFrictionConstraint.m_frictionIndex].m_appliedImpulse;
					if (totalImpulse>btScalar(0))
					{
						btScalar rollingFrictionMagnitude = rollingFrictionConstraint.m_friction*totalImpulse;
						if (rollingFrictionMagnitude>rollingFrictionConstraint.m_friction)
							rollingFrictionMagnitude = rollingFrictionConstraint.m_friction;

						rollingFrictionConstraint.m_lowerLimit = -rollingFrictionMagnitude;
						rollingFrictionConstraint.m_upperLimit = rollingFrictionMagnitude;

						resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[rollingFrictionConstraint.m_solverBodyIdA],m_tmpSolverBodyPool[rollingFrictionConstraint.m_solverBodyIdB],rollingFrictionConstraint);
					}
				}
				

			}			
		}
	} else
	{
		//non-SIMD version
		///solve all joint constraints
		for (int j=0;j<m_tmpSolverNonContactConstraintPool.size();j++)
		{
			btSolverConstraint& constraint = m_tmpSolverNonContactConstraintPool[m_orderNonContactConstraintPool[j]];
			if (iteration < constraint.m_overrideNumSolverIterations)
				resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[constraint.m_solverBodyIdA],m_tmpSolverBodyPool[constraint.m_solverBodyIdB],constraint);
		}

		if (iteration< infoGlobal.m_numIterations)
		{
#ifndef DISABLE_JOINTS
			for (int j=0;j<numConstraints;j++)
			{
                if (constraints[j]->isEnabled())
                {
                    int bodyAid = getOrInitSolverBody(constraints[j]->getRigidBodyA());
                    int bodyBid = getOrInitSolverBody(constraints[j]->getRigidBodyB());
                    btSolverBody& bodyA = m_tmpSolverBodyPool[bodyAid];
                    btSolverBody& bodyB = m_tmpSolverBodyPool[bodyBid];
                    constraints[j]->solveConstraintObsolete(bodyA,bodyB,infoGlobal.m_timeStep);
                }
			}
#endif //DISABLE_JOINTS

			///solve all contact constraints
			int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
			for (int j=0;j<numPoolConstraints;j++)
			{
				const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];
				resolveSingleConstraintRowLowerLimit(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
			}
			///solve all friction constraints
			int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.size();
			for (int j=0;j<numFrictionPoolConstraints;j++)
			{
				btSolverConstraint& solveManifold = m_tmpSolverContactFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
				btScalar totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;

				if (totalImpulse>btScalar(0))
				{
					solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
					solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;

					resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
				}
			}

			int numRollingFrictionPoolConstraints = m_tmpSolverContactRollingFrictionConstraintPool.size();
			for (int j=0;j<numRollingFrictionPoolConstraints;j++)
			{
				btSolverConstraint& rollingFrictionConstraint = m_tmpSolverContactRollingFrictionConstraintPool[j];
				btScalar totalImpulse = m_tmpSolverContactConstraintPool[rollingFrictionConstraint.m_frictionIndex].m_appliedImpulse;
				if (totalImpulse>btScalar(0))
				{
					btScalar rollingFrictionMagnitude = rollingFrictionConstraint.m_friction*totalImpulse;
					if (rollingFrictionMagnitude>rollingFrictionConstraint.m_friction)
						rollingFrictionMagnitude = rollingFrictionConstraint.m_friction;

					rollingFrictionConstraint.m_lowerLimit = -rollingFrictionMagnitude;
					rollingFrictionConstraint.m_upperLimit = rollingFrictionMagnitude;

					resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[rollingFrictionConstraint.m_solverBodyIdA],m_tmpSolverBodyPool[rollingFrictionConstraint.m_solverBodyIdB],rollingFrictionConstraint);
				}
			}
		}
	}
	return 0.f;
}


void btPgsJacobiSolver::solveGroupCacheFriendlySplitImpulseIterations(btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal)
{
	int iteration;
	if (infoGlobal.m_splitImpulse)
	{
		if (infoGlobal.m_solverMode & SOLVER_SIMD)
		{
			for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
			{
				{
					int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
					int j;
					for (j=0;j<numPoolConstraints;j++)
					{
						const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];

						resolveSplitPenetrationSIMD(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
					}
				}
			}
		}
		else
		{
			for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
			{
				{
					int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
					int j;
					for (j=0;j<numPoolConstraints;j++)
					{
						const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];

						resolveSplitPenetrationImpulseCacheFriendly(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
					}
				}
			}
		}
	}
}

btScalar btPgsJacobiSolver::solveGroupCacheFriendlyIterations(btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal)
{
	BT_PROFILE("solveGroupCacheFriendlyIterations");

	{
		///this is a special step to resolve penetrations (just for contacts)
		solveGroupCacheFriendlySplitImpulseIterations(constraints,numConstraints,infoGlobal);

		int maxIterations = m_maxOverrideNumSolverIterations > infoGlobal.m_numIterations? m_maxOverrideNumSolverIterations : infoGlobal.m_numIterations;

		for ( int iteration = 0 ; iteration< maxIterations ; iteration++)
		//for ( int iteration = maxIterations-1  ; iteration >= 0;iteration--)
		{			
			
			solveSingleIteration(iteration, constraints,numConstraints,infoGlobal);


			if (!m_usePgs)
			{
				averageVelocities();
			}
		}
		
	}
	return 0.f;
}

void	btPgsJacobiSolver::averageVelocities()
{
	BT_PROFILE("averaging");
	//average the velocities
	int numBodies = m_bodyCount.size();

	m_deltaLinearVelocities.resize(0);
	m_deltaLinearVelocities.resize(numBodies,btVector3(0,0,0));
	m_deltaAngularVelocities.resize(0);
	m_deltaAngularVelocities.resize(numBodies,btVector3(0,0,0));

	for (int i=0;i<m_tmpSolverBodyPool.size();i++)
	{
		if (!m_tmpSolverBodyPool[i].m_invMass.isZero())
		{
			int orgBodyIndex = m_tmpSolverBodyPool[i].m_originalBodyIndex;
			m_deltaLinearVelocities[orgBodyIndex]+=m_tmpSolverBodyPool[i].getDeltaLinearVelocity();
			m_deltaAngularVelocities[orgBodyIndex]+=m_tmpSolverBodyPool[i].getDeltaAngularVelocity();
		}
	}
				
	for (int i=0;i<m_tmpSolverBodyPool.size();i++)
	{
		int orgBodyIndex = m_tmpSolverBodyPool[i].m_originalBodyIndex;

		if (!m_tmpSolverBodyPool[i].m_invMass.isZero())
		{
						
			btAssert(m_bodyCount[orgBodyIndex] == m_bodyCountCheck[orgBodyIndex]);
						
			btScalar factor = 1.f/btScalar(m_bodyCount[orgBodyIndex]);
						

			m_tmpSolverBodyPool[i].m_deltaLinearVelocity = m_deltaLinearVelocities[orgBodyIndex]*factor;
			m_tmpSolverBodyPool[i].m_deltaAngularVelocity = m_deltaAngularVelocities[orgBodyIndex]*factor;
		}
	}
}

btScalar btPgsJacobiSolver::solveGroupCacheFriendlyFinish(btRigidBodyCL* bodies,btInertiaCL* inertias,int numBodies,const btContactSolverInfo& infoGlobal)
{
	int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
	int i,j;

	if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
	{
		for (j=0;j<numPoolConstraints;j++)
		{
			const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[j];
			btContactPoint* pt = (btContactPoint*) solveManifold.m_originalContactPoint;
			btAssert(pt);
			pt->m_appliedImpulse = solveManifold.m_appliedImpulse;
		//	float f = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
			//	printf("pt->m_appliedImpulseLateral1 = %f\n", f);
			pt->m_appliedImpulseLateral1 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
			//printf("pt->m_appliedImpulseLateral1 = %f\n", pt->m_appliedImpulseLateral1);
			if ((infoGlobal.m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS))
			{
				pt->m_appliedImpulseLateral2 = m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex+1].m_appliedImpulse;
			}
			//do a callback here?
		}
	}

	numPoolConstraints = m_tmpSolverNonContactConstraintPool.size();
	for (j=0;j<numPoolConstraints;j++)
	{
		const btSolverConstraint& solverConstr = m_tmpSolverNonContactConstraintPool[j];
		btTypedConstraint* constr = (btTypedConstraint*)solverConstr.m_originalContactPoint;
		btJointFeedback* fb = constr->getJointFeedback();
		if (fb)
		{
			fb->m_appliedForceBodyA += solverConstr.m_contactNormal*solverConstr.m_appliedImpulse*constr->getRigidBodyA().getLinearFactor()/infoGlobal.m_timeStep;
			fb->m_appliedForceBodyB += -solverConstr.m_contactNormal*solverConstr.m_appliedImpulse*constr->getRigidBodyB().getLinearFactor()/infoGlobal.m_timeStep;
			fb->m_appliedTorqueBodyA += solverConstr.m_relpos1CrossNormal* constr->getRigidBodyA().getAngularFactor()*solverConstr.m_appliedImpulse/infoGlobal.m_timeStep;
			fb->m_appliedTorqueBodyB += -solverConstr.m_relpos1CrossNormal* constr->getRigidBodyB().getAngularFactor()*solverConstr.m_appliedImpulse/infoGlobal.m_timeStep;
			
		}

		constr->internalSetAppliedImpulse(solverConstr.m_appliedImpulse);
		if (btFabs(solverConstr.m_appliedImpulse)>=constr->getBreakingImpulseThreshold())
		{
			constr->setEnabled(false);
		}
	}

	{
		BT_PROFILE("write back velocities and transforms");
		for ( i=0;i<m_tmpSolverBodyPool.size();i++)
		{
			int bodyIndex = m_tmpSolverBodyPool[i].m_originalBodyIndex;
			//btAssert(i==bodyIndex);

			btRigidBodyCL* body = &bodies[bodyIndex];
			if (body->getInvMass())
			{
				if (infoGlobal.m_splitImpulse)
					m_tmpSolverBodyPool[i].writebackVelocityAndTransform(infoGlobal.m_timeStep, infoGlobal.m_splitImpulseTurnErp);
				else
					m_tmpSolverBodyPool[i].writebackVelocity();

				if (m_usePgs)
				{
					body->m_linVel = m_tmpSolverBodyPool[i].m_linearVelocity;
					body->m_angVel = m_tmpSolverBodyPool[i].m_angularVelocity;
				} else
				{
					btScalar factor = 1.f/btScalar(m_bodyCount[bodyIndex]);

					btVector3 deltaLinVel = m_deltaLinearVelocities[bodyIndex]*factor;
					btVector3 deltaAngVel = m_deltaAngularVelocities[bodyIndex]*factor;
					//printf("body %d\n",bodyIndex);
					//printf("deltaLinVel = %f,%f,%f\n",deltaLinVel.getX(),deltaLinVel.getY(),deltaLinVel.getZ());
					//printf("deltaAngVel = %f,%f,%f\n",deltaAngVel.getX(),deltaAngVel.getY(),deltaAngVel.getZ());

					body->m_linVel += deltaLinVel;
					body->m_angVel += deltaAngVel;
				}
			
				if (infoGlobal.m_splitImpulse)
				{
					body->m_pos = m_tmpSolverBodyPool[i].m_worldTransform.getOrigin();
					btQuaternion orn;
					orn = m_tmpSolverBodyPool[i].m_worldTransform.getRotation();
					body->m_quat = orn;
				}
			}
		}
	}


	m_tmpSolverContactConstraintPool.resizeNoInitialize(0);
	m_tmpSolverNonContactConstraintPool.resizeNoInitialize(0);
	m_tmpSolverContactFrictionConstraintPool.resizeNoInitialize(0);
	m_tmpSolverContactRollingFrictionConstraintPool.resizeNoInitialize(0);

	m_tmpSolverBodyPool.resizeNoInitialize(0);
	return 0.f;
}



void	btPgsJacobiSolver::reset()
{
	m_btSeed2 = 0;
}