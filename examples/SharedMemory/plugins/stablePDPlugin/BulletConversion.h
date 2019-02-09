#ifndef BULLET_CONVERSION_H
#define BULLET_CONVERSION_H

class btMultiBody;
#include "MathUtil.h"
void btExtractJointBodyFromBullet(const btMultiBody* bulletMB, Eigen::MatrixXd& bodyDefs, Eigen::MatrixXd& jointMat);


#endif  //BULLET_CONVERSION_H