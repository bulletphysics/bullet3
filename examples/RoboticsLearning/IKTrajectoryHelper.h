#ifndef IK_TRAJECTORY_HELPER_H
#define IK_TRAJECTORY_HELPER_H

enum IK2_Method
{
    IK2_JACOB_TRANS=0,
    IK2_PURE_PSEUDO,
    IK2_DLS,
    IK2_SDLS ,
    IK2_DLS_SVD
};


class IKTrajectoryHelper
{
    struct IKTrajectoryHelperInternalData* m_data;
    
public:
    IKTrajectoryHelper();
    virtual ~IKTrajectoryHelper();
    
    ///todo: replace createKukaIIWA with a generic way of create an IK tree
    void createKukaIIWA();
    
    bool computeIK(const double endEffectorTargetPosition[3],
                   const double* q_old, int numQ,
                   double* q_new, int ikMethod);
    
};
#endif //IK_TRAJECTORY_HELPER_H
