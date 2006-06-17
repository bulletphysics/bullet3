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


#include "Generic6DofConstraint.h"
#include "Dynamics/RigidBody.h"
#include "Dynamics/MassProps.h"
#include "SimdTransformUtil.h"

static const SimdScalar kSign[] = { 1.0f, -1.0f, 1.0f };
static const int kAxisA[] = { 1, 0, 0 };
static const int kAxisB[] = { 2, 2, 1 };

Generic6DofConstraint::Generic6DofConstraint()
{
}

Generic6DofConstraint::Generic6DofConstraint(RigidBody& rbA, RigidBody& rbB, const SimdTransform& frameInA, const SimdTransform& frameInB )
: TypedConstraint(rbA, rbB)
, m_frameInA(frameInA)
, m_frameInB(frameInB)
{
	
}


void Generic6DofConstraint::BuildJacobian()
{
	SimdVector3	normal(0,0,0);

	const SimdVector3& pivotInA = m_frameInA.getOrigin();
	const SimdVector3& pivotInB = m_frameInB.getOrigin();

	//linear part
	for (int i=0;i<3;i++)
	{
		normal[i] = 1;

		new (&m_jac[i]) JacobianEntry(
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getCenterOfMassTransform()*pivotInA - m_rbA.getCenterOfMassPosition(),
			m_rbB.getCenterOfMassTransform()*pivotInB - m_rbB.getCenterOfMassPosition(),
			normal,
			m_rbA.getInvInertiaDiagLocal(),
			m_rbA.getInvMass(),
			m_rbB.getInvInertiaDiagLocal(),
			m_rbB.getInvMass());

		normal[i] = 0;
	}

	// angular part
	for (int i=0;i<3;i++)
	{
		#if 0

		SimdVector3 axisInA = m_frameInA.getBasis().getColumn(i);
		SimdVector3 axisInB = m_frameInB.getBasis().getColumn(i);

		new (&m_jacAng[i])	JacobianEntry(axisInA, axisInB,
			m_rbA.getInvInertiaDiagLocal(),
			m_rbB.getInvInertiaDiagLocal());

		#else

		SimdVector3 axisA = m_rbA.getCenterOfMassTransform().getBasis() * m_frameInA.getBasis().getColumn( kAxisA[i] );
		SimdVector3 axisB = m_rbB.getCenterOfMassTransform().getBasis() * m_frameInB.getBasis().getColumn( kAxisB[i] );

		// Dirk: This is IMO mathematically the correct way, but we should consider axisA and axisB being near parallel
		SimdVector3 axis = kSign[i] * axisA.cross(axisB);

		new (&m_jacAng[i])	JacobianEntry(axis,
		m_rbA.getCenterOfMassTransform().getBasis().transpose(),
		m_rbB.getCenterOfMassTransform().getBasis().transpose(),
		m_rbA.getInvInertiaDiagLocal(),
		m_rbB.getInvInertiaDiagLocal());

		#endif

	}
}

void	Generic6DofConstraint::SolveConstraint(SimdScalar	timeStep)
{
	SimdScalar tau = 0.1f;
	SimdScalar damping = 1.0f;

	SimdVector3 pivotAInW = m_rbA.getCenterOfMassTransform() * m_frameInA.getOrigin();
	SimdVector3 pivotBInW = m_rbB.getCenterOfMassTransform() * m_frameInB.getOrigin();

	SimdVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition(); 
	SimdVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();
	
	SimdVector3 normal(0,0,0);

	// linear
	for (int i=0;i<3;i++)
	{		
		SimdVector3 angvelA = m_rbA.getCenterOfMassTransform().getBasis().transpose() * m_rbA.getAngularVelocity();
		SimdVector3 angvelB = m_rbB.getCenterOfMassTransform().getBasis().transpose() * m_rbB.getAngularVelocity();
	

		normal[i] = 1;
		SimdScalar jacDiagABInv = 1.f / m_jac[i].getDiagonal();

		//velocity error (first order error)
		SimdScalar rel_vel = m_jac[i].getRelativeVelocity(m_rbA.getLinearVelocity(),angvelA, 
																m_rbB.getLinearVelocity(),angvelB);
	
		//positional error (zeroth order error)
		SimdScalar depth = -(pivotAInW - pivotBInW).dot(normal); 
		
		SimdScalar impulse = (tau*depth/timeStep - damping*rel_vel) * jacDiagABInv;

		SimdVector3 impulse_vector = normal * impulse;
		m_rbA.applyImpulse( impulse_vector, rel_pos1);
		m_rbB.applyImpulse(-impulse_vector, rel_pos2);
		
		normal[i] = 0;
	}

	// angular
	for (int i=0;i<3;i++)
	{
		SimdVector3 angvelA = m_rbA.getCenterOfMassTransform().getBasis().transpose() * m_rbA.getAngularVelocity();
		SimdVector3 angvelB = m_rbB.getCenterOfMassTransform().getBasis().transpose() * m_rbB.getAngularVelocity();
	
		SimdScalar jacDiagABInv = 1.f / m_jacAng[i].getDiagonal();
		
		//velocity error (first order error)
		SimdScalar rel_vel = m_jacAng[i].getRelativeVelocity(m_rbA.getLinearVelocity(),angvelA, 
																		m_rbB.getLinearVelocity(),angvelB);

		//positional error (zeroth order error)
		SimdVector3 axisA = m_rbA.getCenterOfMassTransform().getBasis() * m_frameInA.getBasis().getColumn( kAxisA[i] );
		SimdVector3 axisB = m_rbB.getCenterOfMassTransform().getBasis() * m_frameInB.getBasis().getColumn( kAxisB[i] );

		SimdScalar rel_pos = kSign[i] * axisA.dot(axisB);

		//impulse
		SimdScalar impulse = -(tau*rel_pos/timeStep + damping*rel_vel) * jacDiagABInv;
		
		// Dirk: Not needed - we could actually project onto Jacobian entry here (same as above)
		SimdVector3 axis = kSign[i] * axisA.cross(axisB);
		SimdVector3 impulse_vector = axis * impulse;

		m_rbA.applyTorqueImpulse( impulse_vector);
		m_rbB.applyTorqueImpulse(-impulse_vector);
	}
}

void	Generic6DofConstraint::UpdateRHS(SimdScalar	timeStep)
{

}

