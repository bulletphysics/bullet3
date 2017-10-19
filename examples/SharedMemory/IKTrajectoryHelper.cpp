#include "IKTrajectoryHelper.h"
#include "BussIK/Node.h"
#include "BussIK/Tree.h"
#include "BussIK/Jacobian.h"
#include "BussIK/VectorRn.h"
#include "BussIK/MatrixRmn.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"


#define RADIAN(X)	((X)*RadiansToDegrees)

//use BussIK and Reflexxes to convert from Cartesian endeffector future target to
//joint space positions at each real-time (simulation) step
struct IKTrajectoryHelperInternalData
{
    VectorR3 m_endEffectorTargetPosition;
    VectorRn m_nullSpaceVelocity;
    VectorRn m_dampingCoeff;
    
    b3AlignedObjectArray<Node*> m_ikNodes;
    
    IKTrajectoryHelperInternalData()
    {
        m_endEffectorTargetPosition.SetZero();
        m_nullSpaceVelocity.SetZero();
        m_dampingCoeff.SetZero();
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


bool IKTrajectoryHelper::computeIK(const double endEffectorTargetPosition[3],
                                   const double endEffectorTargetOrientation[4],
                                   const double endEffectorWorldPosition[3],
                                   const double endEffectorWorldOrientation[4],
               const double* q_current, int numQ,int endEffectorIndex,
               double* q_new, int ikMethod, const double* linear_jacobian, const double* angular_jacobian, int jacobian_size, const double dampIk[6])
{
	bool useAngularPart = (ikMethod==IK2_VEL_DLS_WITH_ORIENTATION || ikMethod==IK2_VEL_DLS_WITH_ORIENTATION_NULLSPACE
						   || ikMethod==IK2_VEL_SDLS_WITH_ORIENTATION) ? true : false;

    Jacobian ikJacobian(useAngularPart,numQ);

    ikJacobian.Reset();

    bool UseJacobianTargets1 = false;
    
    if ( UseJacobianTargets1 ) {
        ikJacobian.SetJtargetActive();
    }
    else {
        ikJacobian.SetJendActive();
    }
    VectorR3 targets;
    targets.Set(endEffectorTargetPosition[0],endEffectorTargetPosition[1],endEffectorTargetPosition[2]);
    ikJacobian.ComputeJacobian(&targets);						// Set up Jacobian and deltaS vectors
    
    // Set one end effector world position from Bullet
    VectorRn deltaS(3);
    for (int i = 0; i < 3; ++i)
    {
        deltaS.Set(i,dampIk[i]*(endEffectorTargetPosition[i]-endEffectorWorldPosition[i]));
    }
    
    // Set one end effector world orientation from Bullet
    VectorRn deltaR(3);
    if (useAngularPart)
    {
        btQuaternion startQ(endEffectorWorldOrientation[0],endEffectorWorldOrientation[1],endEffectorWorldOrientation[2],endEffectorWorldOrientation[3]);
        btQuaternion endQ(endEffectorTargetOrientation[0],endEffectorTargetOrientation[1],endEffectorTargetOrientation[2],endEffectorTargetOrientation[3]);
        btQuaternion deltaQ = endQ*startQ.inverse();
        float angle = deltaQ.getAngle();
        btVector3 axis = deltaQ.getAxis();
        if (angle > PI) {
            angle -= 2.0*PI;
        }
        else if (angle < -PI) {
            angle += 2.0*PI;
        }
        float angleDot = angle;
        btVector3 angularVel = angleDot*axis.normalize();
        for (int i = 0; i < 3; ++i)
        {
            deltaR.Set(i,dampIk[i+3]*angularVel[i]);
        }
    }
    
    {
        
		if (useAngularPart)
		{
			VectorRn deltaC(6);
			MatrixRmn completeJacobian(6,numQ);
			for (int i = 0; i < 3; ++i)
			{
				deltaC.Set(i,deltaS[i]);
				deltaC.Set(i+3,deltaR[i]);
				for (int j = 0; j < numQ; ++j)
				{
					completeJacobian.Set(i,j,linear_jacobian[i*numQ+j]);
					completeJacobian.Set(i+3,j,angular_jacobian[i*numQ+j]);
				}
			}
			ikJacobian.SetDeltaS(deltaC);
			ikJacobian.SetJendTrans(completeJacobian);
		} else
		{
			VectorRn deltaC(3);
			MatrixRmn completeJacobian(3,numQ);
			for (int i = 0; i < 3; ++i)
			{
				deltaC.Set(i,deltaS[i]);
				for (int j = 0; j < numQ; ++j)
				{
					completeJacobian.Set(i,j,linear_jacobian[i*numQ+j]);
				}
			}
			ikJacobian.SetDeltaS(deltaC);
			ikJacobian.SetJendTrans(completeJacobian);
		}
    }
    
    // Calculate the change in theta values
    switch (ikMethod) {
        case IK2_JACOB_TRANS:
            ikJacobian.CalcDeltaThetasTranspose();		// Jacobian transpose method
            break;
		case IK2_DLS:
		case IK2_VEL_DLS_WITH_ORIENTATION:
		case IK2_VEL_DLS:
            //ikJacobian.CalcDeltaThetasDLS();			// Damped least squares method
            assert(m_data->m_dampingCoeff.GetLength()==numQ);
            ikJacobian.CalcDeltaThetasDLS2(m_data->m_dampingCoeff);
            break;
        case IK2_VEL_DLS_WITH_NULLSPACE:
        case IK2_VEL_DLS_WITH_ORIENTATION_NULLSPACE:
            assert(m_data->m_nullSpaceVelocity.GetLength()==numQ);
            ikJacobian.CalcDeltaThetasDLSwithNullspace(m_data->m_nullSpaceVelocity);
            break;
        case IK2_DLS_SVD:
            ikJacobian.CalcDeltaThetasDLSwithSVD();
            break;
        case IK2_PURE_PSEUDO:
            ikJacobian.CalcDeltaThetasPseudoinverse();	// Pure pseudoinverse method
            break;
        case IK2_SDLS:
		case IK2_VEL_SDLS:
		case IK2_VEL_SDLS_WITH_ORIENTATION:
            ikJacobian.CalcDeltaThetasSDLS();			// Selectively damped least squares method
            break;
        default:
            ikJacobian.ZeroDeltaThetas();
            break;
    }
    
    // Use for velocity IK, update theta dot
    //ikJacobian.UpdateThetaDot();
    
    // Use for position IK, incrementally update theta
    //ikJacobian.UpdateThetas();
    
    // Apply the change in the theta values
    //ikJacobian.UpdatedSClampValue(&targets);
    
    for (int i=0;i<numQ;i++)
    {
        // Use for velocity IK
        q_new[i] = ikJacobian.dTheta[i] + q_current[i];
        
        // Use for position IK
        //q_new[i] = m_data->m_ikNodes[i]->GetTheta();
    }
    return true;
}

bool IKTrajectoryHelper::computeNullspaceVel(int numQ, const double* q_current, const double* lower_limit, const double* upper_limit, const double* joint_range, const double* rest_pose)
{
    m_data->m_nullSpaceVelocity.SetLength(numQ);
    m_data->m_nullSpaceVelocity.SetZero();
	// TODO: Expose the coefficents of the null space term so that the user can choose to balance the null space task and the IK target task.
	// Can also adaptively adjust the coefficients based on the residual of the null space velocity in the IK target task space.
    double stayCloseToZeroGain = 0.001;
    double stayAwayFromLimitsGain = 10.0;

    // Stay close to zero
    for (int i = 0; i < numQ; ++i)
    {
        m_data->m_nullSpaceVelocity[i] = stayCloseToZeroGain * (rest_pose[i]-q_current[i]);
    }

    // Stay away from joint limits
    for (int i = 0; i < numQ; ++i) {
        if (q_current[i] > upper_limit[i]) {
            m_data->m_nullSpaceVelocity[i] += stayAwayFromLimitsGain * (upper_limit[i] - q_current[i]) / joint_range[i];
        }
        if (q_current[i] < lower_limit[i]) {
            m_data->m_nullSpaceVelocity[i] += stayAwayFromLimitsGain * (lower_limit[i] - q_current[i]) / joint_range[i];
        }
    }
    return true;
}

bool IKTrajectoryHelper::setDampingCoeff(int numQ, const double* coeff)
{
    m_data->m_dampingCoeff.SetLength(numQ);
    m_data->m_dampingCoeff.SetZero();
    for (int i = 0; i < numQ; ++i)
    {
        m_data->m_dampingCoeff[i] = coeff[i];
    }
    return true;
}
