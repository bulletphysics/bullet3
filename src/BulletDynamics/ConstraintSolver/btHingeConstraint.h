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

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

#ifndef HINGECONSTRAINT_H
#define HINGECONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btRigidBody;

/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/// axis defines the orientation of the hinge axis
ATTRIBUTE_ALIGNED16(class) btHingeConstraint : public btTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
	btJacobianEntry	m_jac[3]; //3 orthogonal linear constraints
	btJacobianEntry	m_jacAng[3]; //2 orthogonal angular constraints+ 1 for limit/motor

	btTransform m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btTransform m_rbBFrame;

	btScalar	m_motorTargetVelocity;
	btScalar	m_maxMotorImpulse;

	btScalar	m_limitSoftness; 
	btScalar	m_biasFactor; 
	btScalar    m_relaxationFactor; 

	btScalar    m_lowerLimit;	
	btScalar    m_upperLimit;	
	
	btScalar	m_kHinge;

	btScalar	m_limitSign;
	btScalar	m_correction;

	btScalar	m_accLimitImpulse;
	btScalar	m_hingeAngle;
	btScalar    m_referenceSign;

	bool		m_angularOnly;
	bool		m_enableAngularMotor;
	bool		m_solveLimit;
	bool		m_useSolveConstraintObsolete;
	bool		m_useReferenceFrameA;

	btScalar	m_accMotorImpulse;

	
public:

	btHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB, btVector3& axisInA,btVector3& axisInB, bool useReferenceFrameA = false);

	btHingeConstraint(btRigidBody& rbA,const btVector3& pivotInA,btVector3& axisInA, bool useReferenceFrameA = false);
	
	btHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btTransform& rbAFrame, const btTransform& rbBFrame, bool useReferenceFrameA = false);

	btHingeConstraint(btRigidBody& rbA,const btTransform& rbAFrame, bool useReferenceFrameA = false);

	btHingeConstraint();

	virtual void	buildJacobian();

	virtual void getInfo1 (btConstraintInfo1* info);

	void getInfo1NonVirtual(btConstraintInfo1* info);

	virtual void getInfo2 (btConstraintInfo2* info);

	void	getInfo2NonVirtual(btConstraintInfo2* info,const btTransform& transA,const btTransform& transB,const btVector3& angVelA,const btVector3& angVelB);

	void	getInfo2Internal(btConstraintInfo2* info,const btTransform& transA,const btTransform& transB,const btVector3& angVelA,const btVector3& angVelB);
		
	virtual	void	solveConstraintObsolete(btSolverBody& bodyA,btSolverBody& bodyB,btScalar	timeStep);

	void	updateRHS(btScalar	timeStep);

	const btRigidBody& getRigidBodyA() const
	{
		return m_rbA;
	}
	const btRigidBody& getRigidBodyB() const
	{
		return m_rbB;
	}

	btRigidBody& getRigidBodyA()	
	{		
		return m_rbA;	
	}	

	btRigidBody& getRigidBodyB()	
	{		
		return m_rbB;	
	}	
	
	void	setAngularOnly(bool angularOnly)
	{
		m_angularOnly = angularOnly;
	}

	void	enableAngularMotor(bool enableMotor,btScalar targetVelocity,btScalar maxMotorImpulse)
	{
		m_enableAngularMotor  = enableMotor;
		m_motorTargetVelocity = targetVelocity;
		m_maxMotorImpulse = maxMotorImpulse;
	}

	// extra motor API, including ability to set a target rotation (as opposed to angular velocity)
	// note: setMotorTarget sets angular velocity under the hood, so you must call it every tick to
	//       maintain a given angular target.
	void enableMotor(bool enableMotor) 	{ m_enableAngularMotor = enableMotor; }
	void setMaxMotorImpulse(btScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; }
	void setMotorTarget(const btQuaternion& qAinB, btScalar dt); // qAinB is rotation of body A wrt body B.
	void setMotorTarget(btScalar targetAngle, btScalar dt);


	void	setLimit(btScalar low,btScalar high,btScalar _softness = 0.9f, btScalar _biasFactor = 0.3f, btScalar _relaxationFactor = 1.0f)
	{
		m_lowerLimit = btNormalizeAngle(low);
		m_upperLimit = btNormalizeAngle(high);

		m_limitSoftness =  _softness;
		m_biasFactor = _biasFactor;
		m_relaxationFactor = _relaxationFactor;

	}

	void	setAxis(btVector3& axisInA)
	{
		btVector3 rbAxisA1, rbAxisA2;
		btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
		btVector3 pivotInA = m_rbAFrame.getOrigin();
//		m_rbAFrame.getOrigin() = pivotInA;
		m_rbAFrame.getBasis().setValue( rbAxisA1.getX(),rbAxisA2.getX(),axisInA.getX(),
										rbAxisA1.getY(),rbAxisA2.getY(),axisInA.getY(),
										rbAxisA1.getZ(),rbAxisA2.getZ(),axisInA.getZ() );

		btVector3 axisInB = m_rbA.getCenterOfMassTransform().getBasis() * axisInA;

		btQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
		btVector3 rbAxisB1 =  quatRotate(rotationArc,rbAxisA1);
		btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);


		m_rbBFrame.getOrigin() = m_rbA.getCenterOfMassTransform()(pivotInA);
		m_rbBFrame.getBasis().setValue( rbAxisB1.getX(),rbAxisB2.getX(),axisInB.getX(),
										rbAxisB1.getY(),rbAxisB2.getY(),axisInB.getY(),
										rbAxisB1.getZ(),rbAxisB2.getZ(),axisInB.getZ() );
	}

	btScalar	getLowerLimit() const
	{
		return m_lowerLimit;
	}

	btScalar	getUpperLimit() const
	{
		return m_upperLimit;
	}


	btScalar getHingeAngle();

	btScalar getHingeAngle(const btTransform& transA,const btTransform& transB);

	void testLimit(const btTransform& transA,const btTransform& transB);


	const btTransform& getAFrame() const { return m_rbAFrame; };	
	const btTransform& getBFrame() const { return m_rbBFrame; };

	btTransform& getAFrame() { return m_rbAFrame; };	
	btTransform& getBFrame() { return m_rbBFrame; };

	inline int getSolveLimit()
	{
		return m_solveLimit;
	}

	inline btScalar getLimitSign()
	{
		return m_limitSign;
	}

	inline bool getAngularOnly() 
	{ 
		return m_angularOnly; 
	}
	inline bool getEnableAngularMotor() 
	{ 
		return m_enableAngularMotor; 
	}
	inline btScalar getMotorTargetVelosity() 
	{ 
		return m_motorTargetVelocity; 
	}
	inline btScalar getMaxMotorImpulse() 
	{ 
		return m_maxMotorImpulse; 
	}

};

#endif //HINGECONSTRAINT_H
