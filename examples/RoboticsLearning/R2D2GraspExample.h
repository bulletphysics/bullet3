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

#ifndef R2D2_GRASP_EXAMPLE_H
#define R2D2_GRASP_EXAMPLE_H

enum RobotLearningExampleOptions
{
	eROBOTIC_LEARN_GRASP = 1,
	eROBOTIC_LEARN_COMPLIANT_CONTACT = 2,
	eROBOTIC_LEARN_ROLLING_FRICTION = 4
};

class CommonExampleInterface* R2D2GraspExampleCreateFunc(struct CommonExampleOptions& options);

#endif  //R2D2_GRASP_EXAMPLE_H
