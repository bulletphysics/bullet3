#ifndef B3_ROBOT_SIMULATOR_CLIENT_API_GUI_H
#define B3_ROBOT_SIMULATOR_CLIENT_API_GUI_H

#include "../SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h"

///The b3RobotSimulatorClientAPI_GUI is pretty much the C++ version of pybullet
///as documented in the pybullet Quickstart Guide
///https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA
class b3RobotSimulatorClientAPI : public b3RobotSimulatorClientAPI_NoGUI
{
public:
	b3RobotSimulatorClientAPI();

	virtual ~b3RobotSimulatorClientAPI();

	virtual bool connect(int mode, const std::string& hostName = "localhost", int portOrKey = -1);

	virtual void renderScene();

	virtual void debugDraw(int debugDrawMode);

	virtual bool mouseMoveCallback(float x, float y);

	virtual bool mouseButtonCallback(int button, int state, float x, float y);
};

#endif  //B3_ROBOT_SIMULATOR_CLIENT_API_H
