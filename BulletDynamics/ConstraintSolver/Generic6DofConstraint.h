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

#ifndef GENERIC_6DOF_CONSTRAINT_H
#define GENERIC_6DOF_CONSTRAINT_H

#include "SimdVector3.h"

#include "ConstraintSolver/JacobianEntry.h"
#include "TypedConstraint.h"

class RigidBody;



/// Generic6DofConstraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/// Generic6DofConstraint can leave any of the 6 degree of freedom 'free' or 'locked'
/// Work in progress (is still a Hinge actually)
class Generic6DofConstraint : public TypedConstraint
{
	JacobianEntry	m_jac[3]; //3 orthogonal linear constraints
	JacobianEntry	m_jacAng[3]; //3 orthogonal angular constraints

	SimdVector3	m_pivotInA;
	SimdVector3	m_pivotInB;
	SimdVector3	m_axisInA;
	SimdVector3	m_axisInB;

		
public:

	Generic6DofConstraint(RigidBody& rbA,RigidBody& rbB, const SimdVector3& pivotInA,const SimdVector3& pivotInB,SimdVector3& axisInA,SimdVector3& axisInB);

	Generic6DofConstraint(RigidBody& rbA,const SimdVector3& pivotInA,SimdVector3& axisInA);

	Generic6DofConstraint();

	virtual void	BuildJacobian();

	virtual	void	SolveConstraint(SimdScalar	timeStep);

	void	UpdateRHS(SimdScalar	timeStep);

	const RigidBody& GetRigidBodyA() const
	{
		return m_rbA;
	}
	const RigidBody& GetRigidBodyB() const
	{
		return m_rbB;
	}
	

};

#endif //GENERIC_6DOF_CONSTRAINT_H
