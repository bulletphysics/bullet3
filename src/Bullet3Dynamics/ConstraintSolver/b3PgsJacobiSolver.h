#ifndef BT_PGS_JACOBI_SOLVER
#define BT_PGS_JACOBI_SOLVER


struct b3Contact4;
struct btContactPoint;


class btDispatcher;

#include "b3TypedConstraint.h"
#include "b3ContactSolverInfo.h"
#include "b3SolverBody.h"
#include "b3SolverConstraint.h"

struct b3RigidBodyCL;
struct btInertiaCL;

class b3PgsJacobiSolver
{

protected:
	b3AlignedObjectArray<b3SolverBody>      m_tmpSolverBodyPool;
	btConstraintArray			m_tmpSolverContactConstraintPool;
	btConstraintArray			m_tmpSolverNonContactConstraintPool;
	btConstraintArray			m_tmpSolverContactFrictionConstraintPool;
	btConstraintArray			m_tmpSolverContactRollingFrictionConstraintPool;

	b3AlignedObjectArray<int>	m_orderTmpConstraintPool;
	b3AlignedObjectArray<int>	m_orderNonContactConstraintPool;
	b3AlignedObjectArray<int>	m_orderFrictionConstraintPool;
	b3AlignedObjectArray<b3TypedConstraint::btConstraintInfo1> m_tmpConstraintSizesPool;
	
	b3AlignedObjectArray<int>		m_bodyCount;
	b3AlignedObjectArray<int>		m_bodyCountCheck;
	
	b3AlignedObjectArray<b3Vector3>	m_deltaLinearVelocities;
	b3AlignedObjectArray<b3Vector3>	m_deltaAngularVelocities;

	bool						m_usePgs;
	void						averageVelocities();

	int							m_maxOverrideNumSolverIterations;
	b3Scalar	getContactProcessingThreshold(b3Contact4* contact)
	{
		return 0.02f;
	}
	void setupFrictionConstraint(	b3RigidBodyCL* bodies,btInertiaCL* inertias, b3SolverConstraint& solverConstraint, const b3Vector3& normalAxis,int solverBodyIdA,int  solverBodyIdB,
									btContactPoint& cp,const b3Vector3& rel_pos1,const b3Vector3& rel_pos2,
									b3RigidBodyCL* colObj0,b3RigidBodyCL* colObj1, b3Scalar relaxation, 
									b3Scalar desiredVelocity=0., b3Scalar cfmSlip=0.);

	void setupRollingFrictionConstraint(b3RigidBodyCL* bodies,btInertiaCL* inertias,	b3SolverConstraint& solverConstraint, const b3Vector3& normalAxis,int solverBodyIdA,int  solverBodyIdB,
									btContactPoint& cp,const b3Vector3& rel_pos1,const b3Vector3& rel_pos2,
									b3RigidBodyCL* colObj0,b3RigidBodyCL* colObj1, b3Scalar relaxation, 
									b3Scalar desiredVelocity=0., b3Scalar cfmSlip=0.);

	b3SolverConstraint&	addFrictionConstraint(b3RigidBodyCL* bodies,btInertiaCL* inertias,const b3Vector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btContactPoint& cp,const b3Vector3& rel_pos1,const b3Vector3& rel_pos2,b3RigidBodyCL* colObj0,b3RigidBodyCL* colObj1, b3Scalar relaxation, b3Scalar desiredVelocity=0., b3Scalar cfmSlip=0.);
	b3SolverConstraint&	addRollingFrictionConstraint(b3RigidBodyCL* bodies,btInertiaCL* inertias,const b3Vector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btContactPoint& cp,const b3Vector3& rel_pos1,const b3Vector3& rel_pos2,b3RigidBodyCL* colObj0,b3RigidBodyCL* colObj1, b3Scalar relaxation, b3Scalar desiredVelocity=0, b3Scalar cfmSlip=0.f);


	void setupContactConstraint(b3RigidBodyCL* bodies, btInertiaCL* inertias,
								b3SolverConstraint& solverConstraint, int solverBodyIdA, int solverBodyIdB, btContactPoint& cp, 
								const b3ContactSolverInfo& infoGlobal, b3Vector3& vel, b3Scalar& rel_vel, b3Scalar& relaxation, 
								b3Vector3& rel_pos1, b3Vector3& rel_pos2);

	void setFrictionConstraintImpulse( b3RigidBodyCL* bodies, btInertiaCL* inertias,b3SolverConstraint& solverConstraint, int solverBodyIdA,int solverBodyIdB, 
										 btContactPoint& cp, const b3ContactSolverInfo& infoGlobal);

	///m_btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
	unsigned long	m_btSeed2;

	
	b3Scalar restitutionCurve(b3Scalar rel_vel, b3Scalar restitution);

	void	convertContact(b3RigidBodyCL* bodies, btInertiaCL* inertias,b3Contact4* manifold,const b3ContactSolverInfo& infoGlobal);


	void	resolveSplitPenetrationSIMD(
     b3SolverBody& bodyA,b3SolverBody& bodyB,
        const b3SolverConstraint& contactConstraint);

	void	resolveSplitPenetrationImpulseCacheFriendly(
       b3SolverBody& bodyA,b3SolverBody& bodyB,
        const b3SolverConstraint& contactConstraint);

	//internal method
	int		getOrInitSolverBody(int bodyIndex, b3RigidBodyCL* bodies,btInertiaCL* inertias);
	void	initSolverBody(int bodyIndex, b3SolverBody* solverBody, b3RigidBodyCL* collisionObject);

	void	resolveSingleConstraintRowGeneric(b3SolverBody& bodyA,b3SolverBody& bodyB,const b3SolverConstraint& contactConstraint);

	void	resolveSingleConstraintRowGenericSIMD(b3SolverBody& bodyA,b3SolverBody& bodyB,const b3SolverConstraint& contactConstraint);
	
	void	resolveSingleConstraintRowLowerLimit(b3SolverBody& bodyA,b3SolverBody& bodyB,const b3SolverConstraint& contactConstraint);
	
	void	resolveSingleConstraintRowLowerLimitSIMD(b3SolverBody& bodyA,b3SolverBody& bodyB,const b3SolverConstraint& contactConstraint);
		
protected:

	virtual b3Scalar solveGroupCacheFriendlySetup(b3RigidBodyCL* bodies, btInertiaCL* inertias,int numBodies,b3Contact4* manifoldPtr, int numManifolds,b3TypedConstraint** constraints,int numConstraints,const b3ContactSolverInfo& infoGlobal);


	virtual b3Scalar solveGroupCacheFriendlyIterations(b3TypedConstraint** constraints,int numConstraints,const b3ContactSolverInfo& infoGlobal);
	virtual void solveGroupCacheFriendlySplitImpulseIterations(b3TypedConstraint** constraints,int numConstraints,const b3ContactSolverInfo& infoGlobal);
	b3Scalar solveSingleIteration(int iteration, b3TypedConstraint** constraints,int numConstraints,const b3ContactSolverInfo& infoGlobal);


	virtual b3Scalar solveGroupCacheFriendlyFinish(b3RigidBodyCL* bodies, btInertiaCL* inertias,int numBodies,const b3ContactSolverInfo& infoGlobal);


public:

	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	b3PgsJacobiSolver();
	virtual ~b3PgsJacobiSolver();

	void	solveContacts(int numBodies, b3RigidBodyCL* bodies, btInertiaCL* inertias, int numContacts, b3Contact4* contacts);
	
	b3Scalar solveGroup(b3RigidBodyCL* bodies,btInertiaCL* inertias,int numBodies,b3Contact4* manifoldPtr, int numManifolds,b3TypedConstraint** constraints,int numConstraints,const b3ContactSolverInfo& infoGlobal);

	///clear internal cached data and reset random seed
	virtual	void	reset();
	
	unsigned long btRand2();

	int btRandInt2 (int n);

	void	setRandSeed(unsigned long seed)
	{
		m_btSeed2 = seed;
	}
	unsigned long	getRandSeed() const
	{
		return m_btSeed2;
	}




};

#endif //BT_PGS_JACOBI_SOLVER

