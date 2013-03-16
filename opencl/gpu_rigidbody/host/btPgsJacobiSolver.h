#ifndef BT_PGS_JACOBI_SOLVER
#define BT_PGS_JACOBI_SOLVER


struct btContact4;
struct btContactPoint;


class btDispatcher;

#include "btTypedConstraint.h"
#include "btContactSolverInfo.h"
#include "btSolverBody.h"
#include "btSolverConstraint.h"
#include "btConstraintSolver.h"
struct btRigidBodyCL;
struct btInertiaCL;

class btPgsJacobiSolver
{

protected:
	btAlignedObjectArray<btSolverBody>      m_tmpSolverBodyPool;
	btConstraintArray			m_tmpSolverContactConstraintPool;
	btConstraintArray			m_tmpSolverNonContactConstraintPool;
	btConstraintArray			m_tmpSolverContactFrictionConstraintPool;
	btConstraintArray			m_tmpSolverContactRollingFrictionConstraintPool;

	btAlignedObjectArray<int>	m_orderTmpConstraintPool;
	btAlignedObjectArray<int>	m_orderNonContactConstraintPool;
	btAlignedObjectArray<int>	m_orderFrictionConstraintPool;
	btAlignedObjectArray<btTypedConstraint::btConstraintInfo1> m_tmpConstraintSizesPool;
	
	btAlignedObjectArray<int>		m_bodyCount;
	btAlignedObjectArray<int>		m_bodyCountCheck;
	
	btAlignedObjectArray<btVector3>	m_deltaLinearVelocities;
	btAlignedObjectArray<btVector3>	m_deltaAngularVelocities;

	bool						m_usePgs;
	void						averageVelocities();

	int							m_maxOverrideNumSolverIterations;
	btScalar	getContactProcessingThreshold(btContact4* contact)
	{
		return 0.02f;
	}
	void setupFrictionConstraint(	btRigidBodyCL* bodies,btInertiaCL* inertias, btSolverConstraint& solverConstraint, const btVector3& normalAxis,int solverBodyIdA,int  solverBodyIdB,
									btContactPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,
									btRigidBodyCL* colObj0,btRigidBodyCL* colObj1, btScalar relaxation, 
									btScalar desiredVelocity=0., btScalar cfmSlip=0.);

	void setupRollingFrictionConstraint(btRigidBodyCL* bodies,btInertiaCL* inertias,	btSolverConstraint& solverConstraint, const btVector3& normalAxis,int solverBodyIdA,int  solverBodyIdB,
									btContactPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,
									btRigidBodyCL* colObj0,btRigidBodyCL* colObj1, btScalar relaxation, 
									btScalar desiredVelocity=0., btScalar cfmSlip=0.);

	btSolverConstraint&	addFrictionConstraint(btRigidBodyCL* bodies,btInertiaCL* inertias,const btVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btContactPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,btRigidBodyCL* colObj0,btRigidBodyCL* colObj1, btScalar relaxation, btScalar desiredVelocity=0., btScalar cfmSlip=0.);
	btSolverConstraint&	addRollingFrictionConstraint(btRigidBodyCL* bodies,btInertiaCL* inertias,const btVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btContactPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,btRigidBodyCL* colObj0,btRigidBodyCL* colObj1, btScalar relaxation, btScalar desiredVelocity=0, btScalar cfmSlip=0.f);


	void setupContactConstraint(btRigidBodyCL* bodies, btInertiaCL* inertias,
								btSolverConstraint& solverConstraint, int solverBodyIdA, int solverBodyIdB, btContactPoint& cp, 
								const btContactSolverInfo& infoGlobal, btVector3& vel, btScalar& rel_vel, btScalar& relaxation, 
								btVector3& rel_pos1, btVector3& rel_pos2);

	void setFrictionConstraintImpulse( btRigidBodyCL* bodies, btInertiaCL* inertias,btSolverConstraint& solverConstraint, int solverBodyIdA,int solverBodyIdB, 
										 btContactPoint& cp, const btContactSolverInfo& infoGlobal);

	///m_btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
	unsigned long	m_btSeed2;

	
	btScalar restitutionCurve(btScalar rel_vel, btScalar restitution);

	void	convertContact(btRigidBodyCL* bodies, btInertiaCL* inertias,btContact4* manifold,const btContactSolverInfo& infoGlobal);


	void	resolveSplitPenetrationSIMD(
     btSolverBody& bodyA,btSolverBody& bodyB,
        const btSolverConstraint& contactConstraint);

	void	resolveSplitPenetrationImpulseCacheFriendly(
       btSolverBody& bodyA,btSolverBody& bodyB,
        const btSolverConstraint& contactConstraint);

	//internal method
	int		getOrInitSolverBody(int bodyIndex, btRigidBodyCL* bodies,btInertiaCL* inertias);
	void	initSolverBody(int bodyIndex, btSolverBody* solverBody, btRigidBodyCL* collisionObject);

	void	resolveSingleConstraintRowGeneric(btSolverBody& bodyA,btSolverBody& bodyB,const btSolverConstraint& contactConstraint);

	void	resolveSingleConstraintRowGenericSIMD(btSolverBody& bodyA,btSolverBody& bodyB,const btSolverConstraint& contactConstraint);
	
	void	resolveSingleConstraintRowLowerLimit(btSolverBody& bodyA,btSolverBody& bodyB,const btSolverConstraint& contactConstraint);
	
	void	resolveSingleConstraintRowLowerLimitSIMD(btSolverBody& bodyA,btSolverBody& bodyB,const btSolverConstraint& contactConstraint);
		
protected:

	virtual btScalar solveGroupCacheFriendlySetup(btRigidBodyCL* bodies, btInertiaCL* inertias,int numBodies,btContact4* manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal);


	virtual btScalar solveGroupCacheFriendlyIterations(btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal);
	virtual void solveGroupCacheFriendlySplitImpulseIterations(btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal);
	btScalar solveSingleIteration(int iteration, btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal);


	virtual btScalar solveGroupCacheFriendlyFinish(btRigidBodyCL* bodies, btInertiaCL* inertias,int numBodies,const btContactSolverInfo& infoGlobal);


public:

	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btPgsJacobiSolver();
	virtual ~btPgsJacobiSolver();

	void	solveContacts(int numBodies, btRigidBodyCL* bodies, btInertiaCL* inertias, int numContacts, btContact4* contacts);
	
	btScalar solveGroup(btRigidBodyCL* bodies,btInertiaCL* inertias,int numBodies,btContact4* manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal);

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

