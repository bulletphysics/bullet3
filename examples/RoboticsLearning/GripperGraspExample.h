/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2016 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef GRIPPER_GRASP_EXAMPLE_H
#define GRIPPER_GRASP_EXAMPLE_H

enum GripperGraspExampleOptions
{
	eGRIPPER_GRASP = 1,
	eTWO_POINT_GRASP = 2,
	eONE_MOTOR_GRASP = 4,
	eGRASP_SOFT_BODY = 8,
	eSOFTBODY_MULTIBODY_COUPLING = 16,
};

class CommonExampleInterface* GripperGraspExampleCreateFunc(struct CommonExampleOptions& options);

#endif  //GRIPPER_GRASP_EXAMPLE_H
