#include "IKTrajectoryHelper.h"
#include "BussIK/Node.h"
#include "BussIK/Tree.h"
#include "BussIK/Jacobian.h"
#include "BussIK/VectorRn.h"
#include "Bullet3Common/b3AlignedObjectArray.h"


#define RADIAN(X)	((X)*RadiansToDegrees)


//use BussIK and Reflexxes to convert from Cartesian endeffector future target to
//joint space positions at each real-time (simulation) step
struct IKTrajectoryHelperInternalData
{
    VectorR3 m_endEffectorTargetPosition;
    
    Tree m_ikTree;
    b3AlignedObjectArray<Node*> m_ikNodes;
    Jacobian* m_ikJacobian;
    
    IKTrajectoryHelperInternalData()
    {
        m_endEffectorTargetPosition.SetZero();
    }
};


IKTrajectoryHelper::IKTrajectoryHelper()
{
    m_data = new IKTrajectoryHelperInternalData;
}

IKTrajectoryHelper::~IKTrajectoryHelper()
{
    delete m_data;
}


void IKTrajectoryHelper::createKukaIIWA()
{
    const VectorR3& unitx = VectorR3::UnitX;
    const VectorR3& unity = VectorR3::UnitY;
    const VectorR3& unitz = VectorR3::UnitZ;
    const VectorR3 unit1(sqrt(14.0) / 8.0, 1.0 / 8.0, 7.0 / 8.0);
    const VectorR3& zero = VectorR3::Zero;
    
    float minTheta = -4 * PI;
    float maxTheta = 4 * PI;
    
    m_data->m_ikNodes.resize(8);//7DOF+additional endeffector
    
    m_data->m_ikNodes[0] = new Node(VectorR3(0.100000, 0.000000, 0.087500), unitz, 0.08, JOINT, -1e30, 1e30, RADIAN(0.));
    m_data->m_ikTree.InsertRoot(m_data->m_ikNodes[0]);
    
    m_data->m_ikNodes[1] = new Node(VectorR3(0.100000, -0.000000, 0.290000), unity, 0.08, JOINT, -0.5, 0.4, RADIAN(0.));
    m_data->m_ikTree.InsertLeftChild(m_data->m_ikNodes[0], m_data->m_ikNodes[1]);
    
    m_data->m_ikNodes[2] = new Node(VectorR3(0.100000, -0.000000, 0.494500), unitz, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
    m_data->m_ikTree.InsertLeftChild(m_data->m_ikNodes[1], m_data->m_ikNodes[2]);
    
    m_data->m_ikNodes[3] = new Node(VectorR3(0.100000, 0.000000, 0.710000), -unity, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
    m_data->m_ikTree.InsertLeftChild(m_data->m_ikNodes[2], m_data->m_ikNodes[3]);
    
    m_data->m_ikNodes[4] = new Node(VectorR3(0.100000, 0.000000, 0.894500), unitz, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
    m_data->m_ikTree.InsertLeftChild(m_data->m_ikNodes[3], m_data->m_ikNodes[4]);
    
    m_data->m_ikNodes[5] = new Node(VectorR3(0.100000, 0.000000, 1.110000), unity, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
    m_data->m_ikTree.InsertLeftChild(m_data->m_ikNodes[4], m_data->m_ikNodes[5]);
    
    m_data->m_ikNodes[6] = new Node(VectorR3(0.100000, 0.000000, 1.191000), unitz, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
    m_data->m_ikTree.InsertLeftChild(m_data->m_ikNodes[5], m_data->m_ikNodes[6]);
    
    m_data->m_ikNodes[7] = new Node(VectorR3(0.100000, 0.000000, 1.20000), zero, 0.08, EFFECTOR);
    m_data->m_ikTree.InsertLeftChild(m_data->m_ikNodes[6], m_data->m_ikNodes[7]);
    
    m_data->m_ikJacobian = new Jacobian(&m_data->m_ikTree);
//    Reset(m_ikTree,m_ikJacobian);

    m_data->m_ikTree.Init();
    m_data->m_ikTree.Compute();
    m_data->m_ikJacobian->Reset();
    
    
}

bool IKTrajectoryHelper::computeIK(const double endEffectorTargetPosition[3],
               const double* q_current, int numQ,
               double* q_new, int ikMethod)
{

    if (numQ != 7)
    {
        return false;
    }
    
    for (int i=0;i<numQ;i++)
    {
        m_data->m_ikNodes[i]->SetTheta(q_current[i]);
    }
    bool UseJacobianTargets1 = false;
    
    if ( UseJacobianTargets1 ) {
        m_data->m_ikJacobian->SetJtargetActive();
    }
    else {
        m_data->m_ikJacobian->SetJendActive();
    }
    VectorR3 targets;
    targets.Set(endEffectorTargetPosition[0],endEffectorTargetPosition[1],endEffectorTargetPosition[2]);
    m_data->m_ikJacobian->ComputeJacobian(&targets);						// Set up Jacobian and deltaS vectors
    
    // Calculate the change in theta values
    switch (ikMethod) {
        case IK2_JACOB_TRANS:
            m_data->m_ikJacobian->CalcDeltaThetasTranspose();		// Jacobian transpose method
            break;
        case IK2_DLS:
            m_data->m_ikJacobian->CalcDeltaThetasDLS();			// Damped least squares method
            break;
        case IK2_DLS_SVD:
            m_data->m_ikJacobian->CalcDeltaThetasDLSwithSVD();
            break;
        case IK2_PURE_PSEUDO:
            m_data->m_ikJacobian->CalcDeltaThetasPseudoinverse();	// Pure pseudoinverse method
            break;
        case IK2_SDLS:
            m_data->m_ikJacobian->CalcDeltaThetasSDLS();			// Selectively damped least squares method
            break;
        default:
            m_data->m_ikJacobian->ZeroDeltaThetas();
            break;
    }
    
    m_data->m_ikJacobian->UpdateThetas();

    // Apply the change in the theta values
    m_data->m_ikJacobian->UpdatedSClampValue(&targets);
    
    for (int i=0;i<numQ;i++)
    {
        q_new[i] = m_data->m_ikNodes[i]->GetTheta();
    }
    return true;
}