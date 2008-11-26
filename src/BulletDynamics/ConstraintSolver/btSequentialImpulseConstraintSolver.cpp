/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#define COMPUTE_IMPULSE_DENOM 1
//It is not necessary (redundant) to refresh contact manifolds, this refresh has been moved to the collision algorithms.

#include "btSequentialImpulseConstraintSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btContactConstraint.h"
#include "btSolve2LinearConstraint.h"
#include "btContactSolverInfo.h"
#include "LinearMath/btIDebugDraw.h"
#include "btJacobianEntry.h"
#include "LinearMath/btMinMax.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include <new>
#include "LinearMath/btStackAlloc.h"
#include "LinearMath/btQuickprof.h"
#include "btSolverBody.h"
#include "btSolverConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"


btSequentialImpulseConstraintSolver::btSequentialImpulseConstraintSolver()
:m_btSeed2(0)
{
}

btSequentialImpulseConstraintSolver::~btSequentialImpulseConstraintSolver()
{
}

// Project Gauss Seidel or the equivalent Sequential Impulse
void btSequentialImpulseConstraintSolver::resolveSingleConstraintRow(btSolverBody& body1,btSolverBody& body2,const btSolverConstraint& c)
{
	float deltaImpulse;
	deltaImpulse = c.m_rhs-c.m_appliedImpulse*c.m_cfm;
	btScalar deltaVel1Dotn	=	c.m_contactNormal.dot(body1.m_deltaLinearVelocity) 	+ c.m_relpos1CrossNormal.dot(body1.m_deltaAngularVelocity);
	btScalar deltaVel2Dotn	=	c.m_contactNormal.dot(body2.m_deltaLinearVelocity) 	+ c.m_relpos2CrossNormal.dot(body2.m_deltaAngularVelocity);
	btScalar delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
	deltaImpulse	-=	deltaVel1Dotn*c.m_jacDiagABInv;
	deltaImpulse	+=	deltaVel2Dotn*c.m_jacDiagABInv;

	btScalar sum = c.m_appliedImpulse + deltaImpulse;
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
	body1.applyImpulse(c.m_contactNormal*body1.m_invMass,c.m_angularComponentA,deltaImpulse);
	body2.applyImpulse(c.m_contactNormal*body2.m_invMass,c.m_angularComponentB,-deltaImpulse);
}



unsigned long btSequentialImpulseConstraintSolver::btRand2()
{
  m_btSeed2 = (1664525L*m_btSeed2 + 1013904223L) & 0xffffffff;
  return m_btSeed2;
}



//See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)
int btSequentialImpulseConstraintSolver::btRandInt2 (int n)
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








void	btSequentialImpulseConstraintSolver::initSolverBody(btSolverBody* solverBody, btCollisionObject* collisionObject)
{
	btRigidBody* rb = btRigidBody::upcast(collisionObject);

	solverBody->m_deltaLinearVelocity.setValue(0.f,0.f,0.f);
	solverBody->m_deltaAngularVelocity.setValue(0.f,0.f,0.f);

	if (rb)
	{
		solverBody->m_angularVelocity = rb->getAngularVelocity() ;
		solverBody->m_centerOfMassPosition = collisionObject->getWorldTransform().getOrigin();
		solverBody->m_friction = collisionObject->getFriction();
		solverBody->m_invMass = rb->getInvMass();
		solverBody->m_linearVelocity = rb->getLinearVelocity();
		solverBody->m_originalBody = rb;
		solverBody->m_angularFactor = rb->getAngularFactor();
	} else
	{
		solverBody->m_angularVelocity.setValue(0,0,0);
		solverBody->m_centerOfMassPosition = collisionObject->getWorldTransform().getOrigin();
		solverBody->m_friction = collisionObject->getFriction();
		solverBody->m_invMass = 0.f;
		solverBody->m_linearVelocity.setValue(0,0,0);
		solverBody->m_originalBody = 0;
		solverBody->m_angularFactor = 1.f;
	}
	solverBody->m_pushVelocity.setValue(0.f,0.f,0.f);
	solverBody->m_turnVelocity.setValue(0.f,0.f,0.f);
}


int		gNumSplitImpulseRecoveries = 0;

btScalar btSequentialImpulseConstraintSolver::restitutionCurve(btScalar rel_vel, btScalar restitution)
{
	btScalar rest = restitution * -rel_vel;
	return rest;
}

//SIMD_FORCE_INLINE
void	btSequentialImpulseConstraintSolver::resolveSplitPenetrationImpulseCacheFriendly(
        btSolverBody& body1,
        btSolverBody& body2,
        const btSolverConstraint& contactConstraint,
        const btContactSolverInfo& solverInfo)
{
        (void)solverInfo;

		if (contactConstraint.m_penetration < solverInfo.m_splitImpulsePenetrationThreshold)
        {

				gNumSplitImpulseRecoveries++;
                btScalar normalImpulse;

                //  Optimized version of projected relative velocity, use precomputed cross products with normal
                //      body1.getVelocityInLocalPoint(contactConstraint.m_rel_posA,vel1);
                //      body2.getVelocityInLocalPoint(contactConstraint.m_rel_posB,vel2);
                //      btVector3 vel = vel1 - vel2;
                //      btScalar  rel_vel = contactConstraint.m_contactNormal.dot(vel);

                btScalar rel_vel;
                btScalar vel1Dotn = contactConstraint.m_contactNormal.dot(body1.m_pushVelocity)
                + contactConstraint.m_relpos1CrossNormal.dot(body1.m_turnVelocity);
                btScalar vel2Dotn = contactConstraint.m_contactNormal.dot(body2.m_pushVelocity)
                + contactConstraint.m_relpos2CrossNormal.dot(body2.m_turnVelocity);

                rel_vel = vel1Dotn-vel2Dotn;


				btScalar positionalError = -contactConstraint.m_penetration * solverInfo.m_erp2/solverInfo.m_timeStep;
                //      btScalar positionalError = contactConstraint.m_penetration;

                btScalar velocityError = contactConstraint.m_restitution - rel_vel;// * damping;

                btScalar penetrationImpulse = positionalError * contactConstraint.m_jacDiagABInv;
                btScalar        velocityImpulse = velocityError * contactConstraint.m_jacDiagABInv;
                normalImpulse = penetrationImpulse+velocityImpulse;

                // See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
                btScalar oldNormalImpulse = contactConstraint.m_appliedPushImpulse;
                btScalar sum = oldNormalImpulse + normalImpulse;
                contactConstraint.m_appliedPushImpulse = btScalar(0.) > sum ? btScalar(0.): sum;

                normalImpulse = contactConstraint.m_appliedPushImpulse - oldNormalImpulse;

				body1.internalApplyPushImpulse(contactConstraint.m_contactNormal*body1.m_invMass, contactConstraint.m_angularComponentA,normalImpulse);
               
				body2.internalApplyPushImpulse(contactConstraint.m_contactNormal*body2.m_invMass, contactConstraint.m_angularComponentB,-normalImpulse);
               
        }

}





//SIMD_FORCE_INLINE 
void btSequentialImpulseConstraintSolver::resolveSingleFrictionCacheFriendly(
	btSolverBody& body1,
	btSolverBody& body2,
	const btSolverConstraint& contactConstraint,
	const btContactSolverInfo& solverInfo,
	btScalar appliedNormalImpulse)
{
	(void)solverInfo;

	const btScalar combinedFriction = contactConstraint.m_friction;
	
	const btScalar limit = appliedNormalImpulse * combinedFriction;
	
	if (appliedNormalImpulse>btScalar(0.))
	//friction
	{
		
		btScalar j1;
		{

#ifndef _USE_JACOBIAN
			btScalar rel_vel;
			const btScalar vel1Dotn = contactConstraint.m_contactNormal.dot(body1.m_linearVelocity) 
						+ contactConstraint.m_relpos1CrossNormal.dot(body1.m_angularVelocity);
			const btScalar vel2Dotn = contactConstraint.m_contactNormal.dot(body2.m_linearVelocity) 
				+ contactConstraint.m_relpos2CrossNormal.dot(body2.m_angularVelocity);

			rel_vel = vel1Dotn-vel2Dotn;
#else
			btScalar rel_vel = contactConstraint.m_jac.getRelativeVelocity(body1.m_linearVelocity1+body1.m_deltaLinearVelocity,body1.m_angularVelocity1+body1.m_deltaAngularVelocity,
			body2.m_linearVelocity1+body2.m_deltaLinearVelocity,body2.m_angularVelocity1+body2.m_deltaAngularVelocity);
#endif //_USE_JACOBIAN

			// calculate j that moves us to zero relative velocity
			j1 = -rel_vel * contactConstraint.m_jacDiagABInv;
#define CLAMP_ACCUMULATED_FRICTION_IMPULSE 1
#ifdef CLAMP_ACCUMULATED_FRICTION_IMPULSE
			btScalar oldTangentImpulse = contactConstraint.m_appliedImpulse;
			contactConstraint.m_appliedImpulse = oldTangentImpulse + j1;
			
			if (limit < contactConstraint.m_appliedImpulse)
			{
				contactConstraint.m_appliedImpulse = limit;
			} else
			{
				if (contactConstraint.m_appliedImpulse < -limit)
					contactConstraint.m_appliedImpulse = -limit;
			}
			j1 = contactConstraint.m_appliedImpulse - oldTangentImpulse;
#else
			if (limit < j1)
			{
				j1 = limit;
			} else
			{
				if (j1 < -limit)
					j1 = -limit;
			}

#endif //CLAMP_ACCUMULATED_FRICTION_IMPULSE

			//GEN_set_min(contactConstraint.m_appliedImpulse, limit);
			//GEN_set_max(contactConstraint.m_appliedImpulse, -limit);

			

		}
	
		body1.applyImpulse(contactConstraint.m_contactNormal*body1.m_invMass,contactConstraint.m_angularComponentA,j1);
		
		body2.applyImpulse(contactConstraint.m_contactNormal*body2.m_invMass,contactConstraint.m_angularComponentB,-j1);

	} 
}



btSolverConstraint&	btSequentialImpulseConstraintSolver::addFrictionConstraint(const btVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btManifoldPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,btCollisionObject* colObj0,btCollisionObject* colObj1, btScalar relaxation)
{

	btRigidBody* body0=btRigidBody::upcast(colObj0);
	btRigidBody* body1=btRigidBody::upcast(colObj1);

	btSolverConstraint& solverConstraint = m_tmpSolverFrictionConstraintPool.expand();
	solverConstraint.m_contactNormal = normalAxis;

	solverConstraint.m_solverBodyIdA = solverBodyIdA;
	solverConstraint.m_solverBodyIdB = solverBodyIdB;
	solverConstraint.m_constraintType = btSolverConstraint::BT_SOLVER_FRICTION_1D;
	solverConstraint.m_frictionIndex = frictionIndex;

	solverConstraint.m_friction = cp.m_combinedFriction;
	solverConstraint.m_originalContactPoint = 0;

	solverConstraint.m_appliedImpulse = btScalar(0.);
	solverConstraint.m_appliedPushImpulse = 0.f;
	solverConstraint.m_penetration = 0.f;
	{
		btVector3 ftorqueAxis1 = rel_pos1.cross(solverConstraint.m_contactNormal);
		solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentA = body0 ? body0->getInvInertiaTensorWorld()*ftorqueAxis1 : btVector3(0,0,0);
	}
	{
		btVector3 ftorqueAxis1 = rel_pos2.cross(solverConstraint.m_contactNormal);
		solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
		solverConstraint.m_angularComponentB = body1 ? body1->getInvInertiaTensorWorld()*ftorqueAxis1 : btVector3(0,0,0);
	}

#ifdef COMPUTE_IMPULSE_DENOM
	btScalar denom0 = rb0->computeImpulseDenominator(pos1,solverConstraint.m_contactNormal);
	btScalar denom1 = rb1->computeImpulseDenominator(pos2,solverConstraint.m_contactNormal);
#else
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
		vec = ( solverConstraint.m_angularComponentB).cross(rel_pos2);
		denom1 = body1->getInvMass() + normalAxis.dot(vec);
	}


#endif //COMPUTE_IMPULSE_DENOM
	btScalar denom = relaxation/(denom0+denom1);
	solverConstraint.m_jacDiagABInv = denom;

#ifdef _USE_JACOBIAN
	solverConstraint.m_jac =  btJacobianEntry (
								rel_pos1,rel_pos2,solverConstraint.m_contactNormal,
								body0->getInvInertiaDiagLocal(),
								body0->getInvMass(),
								body1->getInvInertiaDiagLocal(),
								body1->getInvMass());
#endif //_USE_JACOBIAN
	return solverConstraint;
}



btScalar btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup(btCollisionObject** /*bodies */,int /*numBodies */,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc)
{
	BT_PROFILE("solveGroupCacheFriendlySetup");
	(void)stackAlloc;
	(void)debugDrawer;


	if (!(numConstraints + numManifolds))
	{
//		printf("empty\n");
		return 0.f;
	}
	btPersistentManifold* manifold = 0;
	btCollisionObject* colObj0=0,*colObj1=0;

	//btRigidBody* rb0=0,*rb1=0;

	//if (1)
	{
		
		{
			int i;

			for (i=0;i<numManifolds;i++)
			{
				manifold = manifoldPtr[i];
				colObj0 = (btCollisionObject*)manifold->getBody0();
				colObj1 = (btCollisionObject*)manifold->getBody1();
			
				int solverBodyIdA=-1;
				int solverBodyIdB=-1;

				if (manifold->getNumContacts())
				{

					

					if (colObj0->getIslandTag() >= 0)
					{
						if (colObj0->getCompanionId() >= 0)
						{
							//body has already been converted
							solverBodyIdA = colObj0->getCompanionId();
						} else
						{
							solverBodyIdA = m_tmpSolverBodyPool.size();
							btSolverBody& solverBody = m_tmpSolverBodyPool.expand();
							initSolverBody(&solverBody,colObj0);
							colObj0->setCompanionId(solverBodyIdA);
						}
					} else
					{
						//create a static body
						solverBodyIdA = m_tmpSolverBodyPool.size();
						btSolverBody& solverBody = m_tmpSolverBodyPool.expand();
						initSolverBody(&solverBody,colObj0);
					}

					if (colObj1->getIslandTag() >= 0)
					{
						if (colObj1->getCompanionId() >= 0)
						{
							solverBodyIdB = colObj1->getCompanionId();
						} else
						{
							solverBodyIdB = m_tmpSolverBodyPool.size();
							btSolverBody& solverBody = m_tmpSolverBodyPool.expand();
							initSolverBody(&solverBody,colObj1);
							colObj1->setCompanionId(solverBodyIdB);
						}
					} else
					{
						//create a static body
						solverBodyIdB = m_tmpSolverBodyPool.size();
						btSolverBody& solverBody = m_tmpSolverBodyPool.expand();
						initSolverBody(&solverBody,colObj1);
					}
				}

				btVector3 rel_pos1;
				btVector3 rel_pos2;
				btScalar relaxation;

				for (int j=0;j<manifold->getNumContacts();j++)
				{
					
					btManifoldPoint& cp = manifold->getContactPoint(j);
					
					///this is a bad test and results in jitter -> always solve for those zero-distanc contacts! 
					///-> if (cp.getDistance() <= btScalar(0.))
					{
						
						const btVector3& pos1 = cp.getPositionWorldOnA();
						const btVector3& pos2 = cp.getPositionWorldOnB();

						 rel_pos1 = pos1 - colObj0->getWorldTransform().getOrigin(); 
						 rel_pos2 = pos2 - colObj1->getWorldTransform().getOrigin();

						
						relaxation = 1.f;
						btScalar rel_vel;
						btVector3 vel;

						int frictionIndex = m_tmpSolverConstraintPool.size();

						{
							btSolverConstraint& solverConstraint = m_tmpSolverConstraintPool.expand();
							btRigidBody* rb0 = btRigidBody::upcast(colObj0);
							btRigidBody* rb1 = btRigidBody::upcast(colObj1);

							solverConstraint.m_solverBodyIdA = solverBodyIdA;
							solverConstraint.m_solverBodyIdB = solverBodyIdB;
							solverConstraint.m_constraintType = btSolverConstraint::BT_SOLVER_CONTACT_1D;

							solverConstraint.m_originalContactPoint = &cp;

							btVector3 torqueAxis0 = rel_pos1.cross(cp.m_normalWorldOnB);
							solverConstraint.m_angularComponentA = rb0 ? rb0->getInvInertiaTensorWorld()*torqueAxis0 : btVector3(0,0,0);
							btVector3 torqueAxis1 = rel_pos2.cross(cp.m_normalWorldOnB);		
							solverConstraint.m_angularComponentB = rb1 ? rb1->getInvInertiaTensorWorld()*torqueAxis1 : btVector3(0,0,0);
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
									vec = ( solverConstraint.m_angularComponentB).cross(rel_pos2);
									denom1 = rb1->getInvMass() + cp.m_normalWorldOnB.dot(vec);
								}
#endif //COMPUTE_IMPULSE_DENOM		
								
								btScalar denom = relaxation/(denom0+denom1);
								solverConstraint.m_jacDiagABInv = denom;
							}

							solverConstraint.m_contactNormal = cp.m_normalWorldOnB;
							solverConstraint.m_relpos1CrossNormal = rel_pos1.cross(cp.m_normalWorldOnB);
							solverConstraint.m_relpos2CrossNormal = rel_pos2.cross(cp.m_normalWorldOnB);


							btVector3 vel1 = rb0 ? rb0->getVelocityInLocalPoint(rel_pos1) : btVector3(0,0,0);
							btVector3 vel2 = rb1 ? rb1->getVelocityInLocalPoint(rel_pos2) : btVector3(0,0,0);
				
							vel  = vel1 - vel2;
							
							rel_vel = cp.m_normalWorldOnB.dot(vel);
							
							solverConstraint.m_penetration = cp.getDistance()+infoGlobal.m_linearSlop;
							//solverConstraint.m_penetration = cp.getDistance();





						


							solverConstraint.m_friction = cp.m_combinedFriction;

							
							if (cp.m_lifeTime>infoGlobal.m_restingContactRestitutionThreshold)
							{
								solverConstraint.m_restitution = 0.f;
							} else
							{
								solverConstraint.m_restitution =  restitutionCurve(rel_vel, cp.m_combinedRestitution);
								if (solverConstraint.m_restitution <= btScalar(0.))
								{
									solverConstraint.m_restitution = 0.f;
								};
							}

							
							///warm starting (or zero if disabled)
							if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
							{
								solverConstraint.m_appliedImpulse = cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;
								if (rb0)
									m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdA].applyImpulse(solverConstraint.m_contactNormal*rb0->getInvMass(),solverConstraint.m_angularComponentA,solverConstraint.m_appliedImpulse);
								if (rb1)
									m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdB].applyImpulse(solverConstraint.m_contactNormal*rb1->getInvMass(),solverConstraint.m_angularComponentB,-solverConstraint.m_appliedImpulse);
							} else
							{
								solverConstraint.m_appliedImpulse = 0.f;
							}

							solverConstraint.m_appliedPushImpulse = 0.f;
							
							{
								btScalar rel_vel;
								btScalar vel1Dotn = solverConstraint.m_contactNormal.dot(rb0?rb0->getLinearVelocity():btVector3(0,0,0)) 
									+ solverConstraint.m_relpos1CrossNormal.dot(rb0?rb0->getAngularVelocity():btVector3(0,0,0));
								btScalar vel2Dotn = solverConstraint.m_contactNormal.dot(rb1?rb1->getLinearVelocity():btVector3(0,0,0)) 
									+ solverConstraint.m_relpos2CrossNormal.dot(rb1?rb1->getAngularVelocity():btVector3(0,0,0));

								rel_vel = vel1Dotn-vel2Dotn;

								btScalar positionalError = 0.f;
								positionalError = -solverConstraint.m_penetration * infoGlobal.m_erp/infoGlobal.m_timeStep;
								btScalar velocityError = solverConstraint.m_restitution - rel_vel;// * damping;

								btScalar penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
								btScalar	velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
								solverConstraint.m_rhs = (penetrationImpulse+velocityImpulse);
								solverConstraint.m_cfm = 0.f;
								solverConstraint.m_lowerLimit = 0;
								solverConstraint.m_upperLimit = 1e30f;
							}


#ifdef _USE_JACOBIAN
							solverConstraint.m_jac =  btJacobianEntry (
								rel_pos1,rel_pos2,cp.m_normalWorldOnB,
								rb0->getInvInertiaDiagLocal(),
								rb0->getInvMass(),
								rb1->getInvInertiaDiagLocal(),
								rb1->getInvMass());
#endif //_USE_JACOBIAN

							/////setup the friction constraints



							if (1)
							{
							solverConstraint.m_frictionIndex = m_tmpSolverFrictionConstraintPool.size();
							if (!cp.m_lateralFrictionInitialized)
							{
								cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
								btScalar lat_rel_vel = cp.m_lateralFrictionDir1.length2();
								if (lat_rel_vel > SIMD_EPSILON)//0.0f)
								{
									cp.m_lateralFrictionDir1 /= btSqrt(lat_rel_vel);
									addFrictionConstraint(cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
									if(infoGlobal.m_solverMode & SOLVER_USE_FRICTION_WARMSTARTING)
									{
										cp.m_lateralFrictionDir2 = cp.m_lateralFrictionDir1.cross(cp.m_normalWorldOnB);
										cp.m_lateralFrictionDir2.normalize();//??
										addFrictionConstraint(cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
										cp.m_lateralFrictionInitialized = true;
									}
								} else
								{
									//re-calculate friction direction every frame, todo: check if this is really needed
									btPlaneSpace1(cp.m_normalWorldOnB,cp.m_lateralFrictionDir1,cp.m_lateralFrictionDir2);
									addFrictionConstraint(cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
									addFrictionConstraint(cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
									if (infoGlobal.m_solverMode & SOLVER_USE_FRICTION_WARMSTARTING)
									{
										cp.m_lateralFrictionInitialized = true;
									}
								}
								
							} else
							{
								addFrictionConstraint(cp.m_lateralFrictionDir1,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
								if (infoGlobal.m_solverMode & SOLVER_USE_FRICTION_WARMSTARTING)
									addFrictionConstraint(cp.m_lateralFrictionDir2,solverBodyIdA,solverBodyIdB,frictionIndex,cp,rel_pos1,rel_pos2,colObj0,colObj1, relaxation);
							}

							if (infoGlobal.m_solverMode & SOLVER_USE_FRICTION_WARMSTARTING)
							{
								{
									btSolverConstraint& frictionConstraint1 = m_tmpSolverFrictionConstraintPool[solverConstraint.m_frictionIndex];
									if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
									{
										frictionConstraint1.m_appliedImpulse = cp.m_appliedImpulseLateral1 * infoGlobal.m_warmstartingFactor;
										if (rb0)
											m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdA].applyImpulse(frictionConstraint1.m_contactNormal*rb0->getInvMass(),frictionConstraint1.m_angularComponentA,frictionConstraint1.m_appliedImpulse);
										if (rb1)
											m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdB].applyImpulse(frictionConstraint1.m_contactNormal*rb1->getInvMass(),frictionConstraint1.m_angularComponentB,-frictionConstraint1.m_appliedImpulse);
									} else
									{
										frictionConstraint1.m_appliedImpulse = 0.f;
									}
								}
								{
									btSolverConstraint& frictionConstraint2 = m_tmpSolverFrictionConstraintPool[solverConstraint.m_frictionIndex+1];
									if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
									{
										frictionConstraint2.m_appliedImpulse = cp.m_appliedImpulseLateral2 * infoGlobal.m_warmstartingFactor;
										if (rb0)
											m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdA].applyImpulse(frictionConstraint2.m_contactNormal*rb0->getInvMass(),frictionConstraint2.m_angularComponentA,frictionConstraint2.m_appliedImpulse);
										if (rb1)
											m_tmpSolverBodyPool[solverConstraint.m_solverBodyIdB].applyImpulse(frictionConstraint2.m_contactNormal*rb1->getInvMass(),frictionConstraint2.m_angularComponentB,-frictionConstraint2.m_appliedImpulse);
									} else
									{
										frictionConstraint2.m_appliedImpulse = 0.f;
									}
								}
							}
							}
						}


					}
				}
			}
		}
	}
	
	btContactSolverInfo info = infoGlobal;

	{
		int j;
		for (j=0;j<numConstraints;j++)
		{
			btTypedConstraint* constraint = constraints[j];
			constraint->buildJacobian();
		}
	}
	
	int numConstraintPool = m_tmpSolverConstraintPool.size();
	int numFrictionPool = m_tmpSolverFrictionConstraintPool.size();

	///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
	m_orderTmpConstraintPool.resize(numConstraintPool);
	m_orderFrictionConstraintPool.resize(numFrictionPool);
	{
		int i;
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

btScalar btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyIterations(btCollisionObject** /*bodies */,int /*numBodies*/,btPersistentManifold** /*manifoldPtr*/, int /*numManifolds*/,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* /*debugDrawer*/,btStackAlloc* /*stackAlloc*/)
{
	BT_PROFILE("solveGroupCacheFriendlyIterations");
	int numConstraintPool = m_tmpSolverConstraintPool.size();
	int numFrictionPool = m_tmpSolverFrictionConstraintPool.size();

	//should traverse the contacts random order...
	int iteration;
	{
		for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
		{			

			int j;
			if (infoGlobal.m_solverMode & SOLVER_RANDMIZE_ORDER)
			{
				if ((iteration & 7) == 0) {
					for (j=0; j<numConstraintPool; ++j) {
						int tmp = m_orderTmpConstraintPool[j];
						int swapi = btRandInt2(j+1);
						m_orderTmpConstraintPool[j] = m_orderTmpConstraintPool[swapi];
						m_orderTmpConstraintPool[swapi] = tmp;
					}

					for (j=0; j<numFrictionPool; ++j) {
						int tmp = m_orderFrictionConstraintPool[j];
						int swapi = btRandInt2(j+1);
						m_orderFrictionConstraintPool[j] = m_orderFrictionConstraintPool[swapi];
						m_orderFrictionConstraintPool[swapi] = tmp;
					}
				}
			}

			for (j=0;j<numConstraints;j++)
			{
				btTypedConstraint* constraint = constraints[j];
				///todo: use solver bodies, so we don't need to copy from/to btRigidBody

				if ((constraint->getRigidBodyA().getIslandTag() >= 0) && (constraint->getRigidBodyA().getCompanionId() >= 0))
				{
					m_tmpSolverBodyPool[constraint->getRigidBodyA().getCompanionId()].writebackVelocity();
				}
				if ((constraint->getRigidBodyB().getIslandTag() >= 0) && (constraint->getRigidBodyB().getCompanionId() >= 0))
				{
					m_tmpSolverBodyPool[constraint->getRigidBodyB().getCompanionId()].writebackVelocity();
				}

				constraint->solveConstraint(infoGlobal.m_timeStep);

				if ((constraint->getRigidBodyA().getIslandTag() >= 0) && (constraint->getRigidBodyA().getCompanionId() >= 0))
				{
					m_tmpSolverBodyPool[constraint->getRigidBodyA().getCompanionId()].readVelocity();
				}
				if ((constraint->getRigidBodyB().getIslandTag() >= 0) && (constraint->getRigidBodyB().getCompanionId() >= 0))
				{
					m_tmpSolverBodyPool[constraint->getRigidBodyB().getCompanionId()].readVelocity();
				}

			}


			{
				int numPoolConstraints = m_tmpSolverConstraintPool.size();
				for (j=0;j<numPoolConstraints;j++)
				{
					const btSolverConstraint& solveManifold = m_tmpSolverConstraintPool[m_orderTmpConstraintPool[j]];
					resolveSingleConstraintRow(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold);
				}
			}

			{
				 int numFrictionPoolConstraints = m_tmpSolverFrictionConstraintPool.size();
				 for (j=0;j<numFrictionPoolConstraints;j++)
				{
					const btSolverConstraint& solveManifold = m_tmpSolverFrictionConstraintPool[m_orderFrictionConstraintPool[j]];
					btScalar totalImpulse = m_tmpSolverConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse+
								m_tmpSolverConstraintPool[solveManifold.m_frictionIndex].m_appliedPushImpulse;			
					resolveSingleFrictionCacheFriendly(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],
							m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold,infoGlobal,
							totalImpulse);
				}
			}
			


		}
	
		if (infoGlobal.m_splitImpulse)
		{
			
			for ( iteration = 0;iteration<infoGlobal.m_numIterations;iteration++)
			{
				{
					int numPoolConstraints = m_tmpSolverConstraintPool.size();
					int j;
					for (j=0;j<numPoolConstraints;j++)
					{
						const btSolverConstraint& solveManifold = m_tmpSolverConstraintPool[m_orderTmpConstraintPool[j]];

						resolveSplitPenetrationImpulseCacheFriendly(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA],
							m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB],solveManifold,infoGlobal);
					}
				}
			}

		}

	}

	return 0.f;
}



/// btSequentialImpulseConstraintSolver Sequentially applies impulses
btScalar btSequentialImpulseConstraintSolver::solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc,btDispatcher* /*dispatcher*/)
{
	BT_PROFILE("solveGroup");
	//we only implement SOLVER_CACHE_FRIENDLY now
	//you need to provide at least some bodies
	btAssert(bodies);
	btAssert(numBodies);

	int i;

	solveGroupCacheFriendlySetup( bodies, numBodies, manifoldPtr,  numManifolds,constraints, numConstraints,infoGlobal,debugDrawer, stackAlloc);
	solveGroupCacheFriendlyIterations(bodies, numBodies, manifoldPtr,  numManifolds,constraints, numConstraints,infoGlobal,debugDrawer, stackAlloc);

	int numPoolConstraints = m_tmpSolverConstraintPool.size();
	int j;
	for (j=0;j<numPoolConstraints;j++)
	{
		
		const btSolverConstraint& solveManifold = m_tmpSolverConstraintPool[j];
		btManifoldPoint* pt = (btManifoldPoint*) solveManifold.m_originalContactPoint;
		btAssert(pt);
		pt->m_appliedImpulse = solveManifold.m_appliedImpulse;
		if (infoGlobal.m_solverMode & SOLVER_USE_FRICTION_WARMSTARTING)
		{
			pt->m_appliedImpulseLateral1 = m_tmpSolverFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;
			pt->m_appliedImpulseLateral2 = m_tmpSolverFrictionConstraintPool[solveManifold.m_frictionIndex+1].m_appliedImpulse;
		}

		//do a callback here?

	}

	if (infoGlobal.m_splitImpulse)
	{		
		for ( i=0;i<m_tmpSolverBodyPool.size();i++)
		{
			m_tmpSolverBodyPool[i].writebackVelocity(infoGlobal.m_timeStep);
		}
	} else
	{
		for ( i=0;i<m_tmpSolverBodyPool.size();i++)
        {
                m_tmpSolverBodyPool[i].writebackVelocity();
        }
	}


	m_tmpSolverBodyPool.resize(0);
	m_tmpSolverConstraintPool.resize(0);
	m_tmpSolverFrictionConstraintPool.resize(0);

	return 0.f;
}









void	btSequentialImpulseConstraintSolver::reset()
{
	m_btSeed2 = 0;
}


