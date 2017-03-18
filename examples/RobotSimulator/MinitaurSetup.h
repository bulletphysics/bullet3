#ifndef MINITAUR_SIMULATION_SETUP_H
#define MINITAUR_SIMULATION_SETUP_H

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
class MinitaurSetup
{
	struct MinitaurSetupInternalData* m_data;
	void resetPose(class b3RobotSimulatorClientAPI* sim);

public:
	MinitaurSetup();
	virtual ~MinitaurSetup();

	int setupMinitaur(class b3RobotSimulatorClientAPI* sim, const class b3Vector3& startPos=b3MakeVector3(0,0,0), const class b3Quaternion& startOrn = b3Quaternion(0,0,0,1));

	void setDesiredMotorAngle(class b3RobotSimulatorClientAPI* sim, const char* motorName, double desiredAngle, double maxTorque=3,double kp=0.1, double kd=0.9);

};
#endif //MINITAUR_SIMULATION_SETUP_H
 