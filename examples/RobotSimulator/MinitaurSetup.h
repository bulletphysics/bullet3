#ifndef MINITAUR_SIMULATION_SETUP_H
#define MINITAUR_SIMULATION_SETUP_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
class MinitaurSetup
{
	struct MinitaurSetupInternalData* m_data;
	void resetPose(class b3RobotSimulatorClientAPI_NoGUI* sim);

public:
	MinitaurSetup();
	virtual ~MinitaurSetup();

	int setupMinitaur(class b3RobotSimulatorClientAPI_NoGUI* sim, const class btVector3& startPos=btVector3(0,0,0), const class btQuaternion& startOrn = btQuaternion(0,0,0,1));

	void setDesiredMotorAngle(class b3RobotSimulatorClientAPI_NoGUI* sim, const char* motorName, double desiredAngle, double maxTorque=3,double kp=0.1, double kd=0.9);

};
#endif //MINITAUR_SIMULATION_SETUP_H
 
