#ifndef ROBOT_CONTROL_EXAMPLE_H
#define ROBOT_CONTROL_EXAMPLE_H

enum EnumRobotControls
{
	ROBOT_VELOCITY_CONTROL=0,
	ROBOT_PD_CONTROL=2,
};

class CommonExampleInterface*    RobotControlExampleCreateFunc(struct CommonExampleOptions& options);

#endif //ROBOT_CONTROL_EXAMPLE_H


