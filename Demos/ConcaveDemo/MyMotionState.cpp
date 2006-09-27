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

#include "MyMotionState.h"
#include "LinearMath/btPoint3.h"

MyMotionState::MyMotionState()
{
	m_worldTransform.setIdentity();
}


MyMotionState::~MyMotionState()
{

}

void	MyMotionState::getWorldPosition(float& posX,float& posY,float& posZ)
{
	posX = m_worldTransform.getOrigin().x();
	posY = m_worldTransform.getOrigin().y();
	posZ = m_worldTransform.getOrigin().z();
}

void	MyMotionState::getWorldScaling(float& scaleX,float& scaleY,float& scaleZ)
{
	scaleX = 1.;
	scaleY = 1.;
	scaleZ = 1.;
}

void	MyMotionState::getWorldOrientation(float& quatIma0,float& quatIma1,float& quatIma2,float& quatReal)
{
	quatIma0 = m_worldTransform.getRotation().x();
	quatIma1 = m_worldTransform.getRotation().y();
	quatIma2 = m_worldTransform.getRotation().z();
	quatReal = m_worldTransform.getRotation()[3];
}
		
void	MyMotionState::setWorldPosition(float posX,float posY,float posZ)
{
	btPoint3 pos(posX,posY,posZ);
	m_worldTransform.setOrigin( pos );
}

void	MyMotionState::setWorldOrientation(float quatIma0,float quatIma1,float quatIma2,float quatReal)
{
	btQuaternion orn(quatIma0,quatIma1,quatIma2,quatReal);
	m_worldTransform.setRotation( orn );
}
		
void	MyMotionState::calculateWorldTransformations()
{

}
