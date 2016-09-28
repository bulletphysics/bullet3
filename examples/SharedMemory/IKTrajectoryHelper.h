#ifndef IK_TRAJECTORY_HELPER_H
#define IK_TRAJECTORY_HELPER_H

enum IK2_Method
{
    IK2_JACOB_TRANS=0,
    IK2_PURE_PSEUDO,
    IK2_DLS,
    IK2_SDLS ,
    IK2_DLS_SVD,
    IK2_VEL_DLS,
	IK2_VEL_DLS_WITH_ORIENTATION,
};


class IKTrajectoryHelper
{
    struct IKTrajectoryHelperInternalData* m_data;
    
public:
    IKTrajectoryHelper();
    virtual ~IKTrajectoryHelper();
    
	bool computeIK(const double endEffectorTargetPosition[3],
		const double endEffectorTargetOrientation[4],
		const double endEffectorWorldPosition[3],
		const double endEffectorWorldOrientation[4],
		const double* q_old, int numQ, int endEffectorIndex,
		double* q_new, int ikMethod, const double* linear_jacobian, const double* angular_jacobian, int jacobian_size, const double dampIk[6]);
    
};
#endif //IK_TRAJECTORY_HELPER_H
