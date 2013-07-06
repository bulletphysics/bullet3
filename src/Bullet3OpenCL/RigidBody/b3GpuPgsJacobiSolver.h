#ifndef B3_GPU_PGS_JACOBI_SOLVER_H
#define B3_GPU_PGS_JACOBI_SOLVER_H

#include "Bullet3Dynamics/ConstraintSolver/b3PgsJacobiSolver.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"


class b3GpuPgsJacobiSolver : public b3PgsJacobiSolver
{
	int m_staticIdx;
	struct b3GpuPgsJacobiSolverInternalData* m_gpuData;

public:
	b3GpuPgsJacobiSolver (cl_context ctx, cl_device_id device, cl_command_queue queue,bool usePgs);
	virtual~b3GpuPgsJacobiSolver ();

	virtual b3Scalar solveGroupCacheFriendlyIterations(b3TypedConstraint** constraints,int numConstraints,const b3ContactSolverInfo& infoGlobal);
	virtual b3Scalar solveGroupCacheFriendlySetup(b3RigidBodyCL* bodies, b3InertiaCL* inertias, int numBodies, b3Contact4* manifoldPtr, int numManifolds,b3TypedConstraint** constraints,int numConstraints,const b3ContactSolverInfo& infoGlobal);


	int sortConstraintByBatch3( struct b3BatchConstraint* cs, int numConstraints, int simdWidth , int staticIdx, int numBodies);
};

#endif //B3_GPU_PGS_JACOBI_SOLVER_H
