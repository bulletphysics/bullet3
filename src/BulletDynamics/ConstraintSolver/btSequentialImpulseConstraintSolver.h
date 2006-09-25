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

#ifndef SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H
#define SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H

#include "btConstraintSolver.h"
class IDebugDraw;

#include "btContactConstraint.h"
	


/// SequentialImpulseConstraintSolver uses a Propagation Method and Sequentially applies impulses
/// The approach is the 3D version of Erin Catto's GDC 2006 tutorial. See http://www.gphysics.com
/// Although Sequential Impulse is more intuitive, it is mathematically equivalent to Projected Successive Overrelaxation (iterative LCP)
/// Applies impulses for combined restitution and penetration recovery and to simulate friction
class SequentialImpulseConstraintSolver : public ConstraintSolver
{
	float Solve(PersistentManifold* manifold, const ContactSolverInfo& info,int iter,IDebugDraw* debugDrawer);
	float SolveFriction(PersistentManifold* manifoldPtr, const ContactSolverInfo& info,int iter,IDebugDraw* debugDrawer);
	void  PrepareConstraints(PersistentManifold* manifoldPtr, const ContactSolverInfo& info,IDebugDraw* debugDrawer);

	ContactSolverFunc m_contactDispatch[MAX_CONTACT_SOLVER_TYPES][MAX_CONTACT_SOLVER_TYPES];
	ContactSolverFunc m_frictionDispatch[MAX_CONTACT_SOLVER_TYPES][MAX_CONTACT_SOLVER_TYPES];

public:

	SequentialImpulseConstraintSolver();

	///Advanced: Override the default contact solving function for contacts, for certain types of rigidbody
	///See RigidBody::m_contactSolverType and RigidBody::m_frictionSolverType
	void	SetContactSolverFunc(ContactSolverFunc func,int type0,int type1)
	{
		m_contactDispatch[type0][type1] = func;
	}
	
	///Advanced: Override the default friction solving function for contacts, for certain types of rigidbody
	///See RigidBody::m_contactSolverType and RigidBody::m_frictionSolverType
	void	SetFrictionSolverFunc(ContactSolverFunc func,int type0,int type1)
	{
		m_frictionDispatch[type0][type1] = func;
	}

	virtual ~SequentialImpulseConstraintSolver() {}
	
	virtual float SolveGroup(PersistentManifold** manifold,int numManifolds,const ContactSolverInfo& info, IDebugDraw* debugDrawer=0);

};

#endif //SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H

