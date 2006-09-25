/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef MY_MOTIONSTATE_H
#define MY_MOTIONSTATE_H

#include "PHY_IMotionState.h"
#include <LinearMath/SimdTransform.h>


class	MyMotionState : public PHY_IMotionState

{
	public:
		MyMotionState();

		virtual ~MyMotionState();

		virtual void	getWorldPosition(float& posX,float& posY,float& posZ);
		virtual void	getWorldScaling(float& scaleX,float& scaleY,float& scaleZ);
		virtual void	getWorldOrientation(float& quatIma0,float& quatIma1,float& quatIma2,float& quatReal);
		
		virtual void	setWorldPosition(float posX,float posY,float posZ);
		virtual	void	setWorldOrientation(float quatIma0,float quatIma1,float quatIma2,float quatReal);
		
		virtual	void	calculateWorldTransformations();
		
		SimdTransform	m_worldTransform;

};

#endif //MY_MOTIONSTATE_H
