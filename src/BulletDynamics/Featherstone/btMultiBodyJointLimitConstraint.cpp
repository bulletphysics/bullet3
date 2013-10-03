
#include "btMultiBodyJointLimitConstraint.h"
#include "btMultiBody.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"


btMultiBodyJointLimitConstraint::btMultiBodyJointLimitConstraint(btMultiBody* body, int link, btScalar lower, btScalar upper)
	:btMultiBodyConstraint(body,body,link,link,2,true),
	m_lowerBound(lower),
	m_upperBound(upper)
{
	// the data.m_jacobians never change, so may as well
    // initialize them here
        
    // note: we rely on the fact that data.m_jacobians are
    // always initialized to zero by the Constraint ctor

    // row 0: the lower bound
    jacobianA(0)[6 + link] = 1;

    // row 1: the upper bound
    jacobianB(1)[6 + link] = -1;
}
btMultiBodyJointLimitConstraint::~btMultiBodyJointLimitConstraint()
{
}

int btMultiBodyJointLimitConstraint::getIslandIdA() const
{
	btMultiBodyLinkCollider* col = m_bodyA->getBaseCollider();
	if (col)
		return col->getIslandTag();
	for (int i=0;i<m_bodyA->getNumLinks();i++)
	{
		if (m_bodyA->getLink(i).m_collider)
			return m_bodyA->getLink(i).m_collider->getIslandTag();
	}
	return -1;
}

int btMultiBodyJointLimitConstraint::getIslandIdB() const
{
	btMultiBodyLinkCollider* col = m_bodyB->getBaseCollider();
	if (col)
		return col->getIslandTag();

	for (int i=0;i<m_bodyB->getNumLinks();i++)
	{
		col = m_bodyB->getLink(i).m_collider;
		if (col)
			return col->getIslandTag();
	}
	return -1;
}


void btMultiBodyJointLimitConstraint::createConstraintRows(btMultiBodyConstraintArray& constraintRows,
		btMultiBodyJacobianData& data,
		const btContactSolverInfo& infoGlobal)
{
    // only positions need to be updated -- data.m_jacobians and force
    // directions were set in the ctor and never change.
    
    // row 0: the lower bound
    setPosition(0, m_bodyA->getJointPos(m_linkA) - m_lowerBound);

    // row 1: the upper bound
    setPosition(1, m_upperBound - m_bodyA->getJointPos(m_linkA));

	for (int row=0;row<getNumRows();row++)
	{
		btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();
		constraintRow.m_multiBodyA = m_bodyA;
		constraintRow.m_multiBodyB = m_bodyB;
		btScalar penetration = getPosition(row);
		fillConstraintRow(constraintRow,data,jacobianA(row),jacobianB(row),penetration,0,0,infoGlobal);
	}

}
	
void btMultiBodyJointLimitConstraint::fillConstraintRow(btMultiBodySolverConstraint& constraintRow,
														btMultiBodyJacobianData& data,
														btScalar* jacOrgA,btScalar* jacOrgB,
														btScalar penetration,btScalar combinedFrictionCoeff, btScalar combinedRestitutionCoeff,
														const btContactSolverInfo& infoGlobal)

{
			
	
	
	btMultiBody* multiBodyA = constraintRow.m_multiBodyA;
	btMultiBody* multiBodyB = constraintRow.m_multiBodyB;

	if (multiBodyA)
	{
		
		const int ndofA  = multiBodyA->getNumLinks() + 6;

		constraintRow.m_deltaVelAindex = multiBodyA->getCompanionId();

		if (constraintRow.m_deltaVelAindex <0)
		{
			constraintRow.m_deltaVelAindex = data.m_deltaVelocities.size();
			multiBodyA->setCompanionId(constraintRow.m_deltaVelAindex);
			data.m_deltaVelocities.resize(data.m_deltaVelocities.size()+ndofA);
		} else
		{
			btAssert(data.m_deltaVelocities.size() >= constraintRow.m_deltaVelAindex+ndofA);
		}

		constraintRow.m_jacAindex = data.m_jacobians.size();
		data.m_jacobians.resize(data.m_jacobians.size()+ndofA);
		data.m_deltaVelocitiesUnitImpulse.resize(data.m_deltaVelocitiesUnitImpulse.size()+ndofA);
		btAssert(data.m_jacobians.size() == data.m_deltaVelocitiesUnitImpulse.size());
		for (int i=0;i<ndofA;i++)
			data.m_jacobians[constraintRow.m_jacAindex+i] = jacOrgA[i];
		
		float* delta = &data.m_deltaVelocitiesUnitImpulse[constraintRow.m_jacAindex];
		multiBodyA->calcAccelerationDeltas(&data.m_jacobians[constraintRow.m_jacAindex],delta,data.scratch_r, data.scratch_v);
	} 

	if (multiBodyB)
	{
		const int ndofB  = multiBodyB->getNumLinks() + 6;

		constraintRow.m_deltaVelBindex = multiBodyB->getCompanionId();
		if (constraintRow.m_deltaVelBindex <0)
		{
			constraintRow.m_deltaVelBindex = data.m_deltaVelocities.size();
			multiBodyB->setCompanionId(constraintRow.m_deltaVelBindex);
			data.m_deltaVelocities.resize(data.m_deltaVelocities.size()+ndofB);
		}

		constraintRow.m_jacBindex = data.m_jacobians.size();
		data.m_jacobians.resize(data.m_jacobians.size()+ndofB);

		for (int i=0;i<ndofB;i++)
			data.m_jacobians[constraintRow.m_jacBindex+i] = jacOrgB[i];

		data.m_deltaVelocitiesUnitImpulse.resize(data.m_deltaVelocitiesUnitImpulse.size()+ndofB);
		btAssert(data.m_jacobians.size() == data.m_deltaVelocitiesUnitImpulse.size());
		multiBodyB->calcAccelerationDeltas(&data.m_jacobians[constraintRow.m_jacBindex],&data.m_deltaVelocitiesUnitImpulse[constraintRow.m_jacBindex],data.scratch_r, data.scratch_v);
	} 
	{
						
		btVector3 vec;
		btScalar denom0 = 0.f;
		btScalar denom1 = 0.f;
		btScalar* jacB = 0;
		btScalar* jacA = 0;
		btScalar* lambdaA =0;
		btScalar* lambdaB =0;
		int ndofA  = 0;
		if (multiBodyA)
		{
			ndofA  = multiBodyA->getNumLinks() + 6;
			jacA = &data.m_jacobians[constraintRow.m_jacAindex];
			lambdaA = &data.m_deltaVelocitiesUnitImpulse[constraintRow.m_jacAindex];
			for (int i = 0; i < ndofA; ++i)
			{
				float j = jacA[i] ;
				float l =lambdaA[i];
				denom0 += j*l;
			}
		} 
		if (multiBodyB)
		{
			const int ndofB  = multiBodyB->getNumLinks() + 6;
			jacB = &data.m_jacobians[constraintRow.m_jacBindex];
			lambdaB = &data.m_deltaVelocitiesUnitImpulse[constraintRow.m_jacBindex];
			for (int i = 0; i < ndofB; ++i)
			{
				float j = jacB[i] ;
				float l =lambdaB[i];
				denom1 += j*l;
			}

		} 

		 if (multiBodyA && (multiBodyA==multiBodyB))
		 {
            // ndof1 == ndof2 in this case
            for (int i = 0; i < ndofA; ++i) 
			{
                denom1 += jacB[i] * lambdaA[i];
                denom1 += jacA[i] * lambdaB[i];
            }
        }

		 float d = denom0+denom1;
		 if (btFabs(d)>SIMD_EPSILON)
		 {
			 
			 constraintRow.m_jacDiagABInv = 1.f/(d);
		 } else
		 {
			constraintRow.m_jacDiagABInv  = 1.f;
		 }
		
	}

	
	//compute rhs and remaining constraintRow fields

	


	btScalar rel_vel = 0.f;
	int ndofA  = 0;
	int ndofB  = 0;
	{

		btVector3 vel1,vel2;
		if (multiBodyA)
		{
			ndofA  = multiBodyA->getNumLinks() + 6;
			btScalar* jacA = &data.m_jacobians[constraintRow.m_jacAindex];
			for (int i = 0; i < ndofA ; ++i) 
				rel_vel += multiBodyA->getVelocityVector()[i] * jacA[i];
		} 
		if (multiBodyB)
		{
			ndofB  = multiBodyB->getNumLinks() + 6;
			btScalar* jacB = &data.m_jacobians[constraintRow.m_jacBindex];
			for (int i = 0; i < ndofB ; ++i) 
				rel_vel += multiBodyB->getVelocityVector()[i] * jacB[i];

		}

		constraintRow.m_friction = combinedFrictionCoeff;

				
		
	}

	/*
	///warm starting (or zero if disabled)
	if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
	{
		constraintRow.m_appliedImpulse = isFriction ? 0 : cp.m_appliedImpulse * infoGlobal.m_warmstartingFactor;

		if (constraintRow.m_appliedImpulse)
		{
			if (multiBodyA)
			{
				btScalar impulse = constraintRow.m_appliedImpulse;
				btScalar* deltaV = &data.m_deltaVelocitiesUnitImpulse[constraintRow.m_jacAindex];
				multiBodyA->applyDeltaVee(deltaV,impulse);
				applyDeltaVee(deltaV,impulse,constraintRow.m_deltaVelAindex,ndofA);
			} 
			if (multiBodyB)
			{
				btScalar impulse = constraintRow.m_appliedImpulse;
				btScalar* deltaV = &data.m_deltaVelocitiesUnitImpulse[constraintRow.m_jacBindex];
				multiBodyB->applyDeltaVee(deltaV,impulse);
				applyDeltaVee(deltaV,impulse,constraintRow.m_deltaVelBindex,ndofB);
			} 
		}
	} 
	else
	*/
	{
		constraintRow.m_appliedImpulse = 0.f;
	}

	constraintRow.m_appliedPushImpulse = 0.f;

	{
		float desiredVelocity = -0.3;

		btScalar positionalError = 0.f;
		btScalar	velocityError =  - rel_vel;// * damping;
					

		btScalar erp = infoGlobal.m_erp2;
		if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
		{
			erp = infoGlobal.m_erp;
		}

		if (penetration>0)
		{
			positionalError = 0;
			velocityError = -penetration / infoGlobal.m_timeStep;
		} else
		{
			positionalError = -penetration * erp/infoGlobal.m_timeStep;
		}

		btScalar  penetrationImpulse = positionalError*constraintRow.m_jacDiagABInv;
		btScalar velocityImpulse = velocityError *constraintRow.m_jacDiagABInv;

		if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
		{
			//combine position and velocity into rhs
			constraintRow.m_rhs = penetrationImpulse+velocityImpulse;
			constraintRow.m_rhsPenetration = 0.f;

		} else
		{
			//split position and velocity into rhs and m_rhsPenetration
			constraintRow.m_rhs = velocityImpulse;
			constraintRow.m_rhsPenetration = penetrationImpulse;
		}

		
		

		constraintRow.m_cfm = 0.f;
		constraintRow.m_lowerLimit = 0;
		constraintRow.m_upperLimit = 1e10f;
	}

}
