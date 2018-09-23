#ifndef ROBOT_CONTROL_EXAMPLE_H
#define ROBOT_CONTROL_EXAMPLE_H

enum EnumRobotControls
{
	ROBOT_VELOCITY_CONTROL = 0,
	ROBOT_PD_CONTROL,
	ROBOT_PING_PONG_JOINT_FEEDBACK,
};

class CommonExampleInterface* RobotControlExampleCreateFunc(struct CommonExampleOptions& options);

#endif  //ROBOT_CONTROL_EXAMPLE_H
