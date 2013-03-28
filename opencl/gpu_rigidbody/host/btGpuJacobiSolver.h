
#ifndef BT_GPU_JACOBI_SOLVER_H
#define BT_GPU_JACOBI_SOLVER_H
#include "../../basic_initialize/btOpenCLUtils.h"

#include "../../gpu_sat/host/btRigidBodyCL.h"
#include "../../gpu_sat/host/btContact4.h"
#include "../../parallel_primitives/host/btOpenCLArray.h"

class btTypedConstraint;

struct btJacobiSolverInfo
{
	int m_fixedBodyIndex;

	float m_deltaTime;
	float m_positionDrift;
	float m_positionConstraintCoeff;
	int	m_numIterations;

	btJacobiSolverInfo()
		:m_fixedBodyIndex(0),
		m_deltaTime(1./60.f),
		m_positionDrift( 0.005f ), 
		m_positionConstraintCoeff( 0.99f ),
		m_numIterations(14)
	{
	}
};
class btGpuJacobiSolver
{
protected:

	struct btGpuJacobiSolverInternalData* m_data;

	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;

public:

	btGpuJacobiSolver(cl_context ctx, cl_device_id device, cl_command_queue queue, int pairCapacity);
	virtual ~btGpuJacobiSolver();



	void  solveGroupHost(btRigidBodyCL* bodies,btInertiaCL* inertias,int numBodies,btContact4* manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btJacobiSolverInfo& solverInfo);
	void  solveGroup(btOpenCLArray<btRigidBodyCL>* bodies,btOpenCLArray<btInertiaCL>* inertias,btOpenCLArray<btContact4>* manifoldPtr,const btJacobiSolverInfo& solverInfo);

};
#endif //BT_GPU_JACOBI_SOLVER_H

