#ifndef INVERSE_KINEMATICSEXAMPLE_H
#define INVERSE_KINEMATICSEXAMPLE_H

enum Method {IK_JACOB_TRANS=0, IK_PURE_PSEUDO, IK_DLS, IK_SDLS , IK_DLS_SVD};

class CommonExampleInterface*    InverseKinematicsExampleCreateFunc(struct CommonExampleOptions& options);

#endif //INVERSE_KINEMATICSEXAMPLE_H
