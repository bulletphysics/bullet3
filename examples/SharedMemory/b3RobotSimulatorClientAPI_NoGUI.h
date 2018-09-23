#ifndef B3_ROBOT_SIMULATOR_CLIENT_API_H
#define B3_ROBOT_SIMULATOR_CLIENT_API_H

#include "b3RobotSimulatorClientAPI_NoDirect.h"

///The b3RobotSimulatorClientAPI is pretty much the C++ version of pybullet
///as documented in the pybullet Quickstart Guide
///https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA
class b3RobotSimulatorClientAPI_NoGUI : public b3RobotSimulatorClientAPI_NoDirect
{
public:
	b3RobotSimulatorClientAPI_NoGUI();
	virtual ~b3RobotSimulatorClientAPI_NoGUI();

	bool connect(int mode, const std::string& hostName = "localhost", int portOrKey = -1);
};

#endif  //B3_ROBOT_SIMULATOR_CLIENT_API_H
