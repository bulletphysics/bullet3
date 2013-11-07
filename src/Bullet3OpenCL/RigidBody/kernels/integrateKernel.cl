/*
Copyright (c) 2013 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans


#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"

float4 quatMult(float4 q1, float4 q2)
{
	float4 q;
	q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	q.y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
	q.z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;
	q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z; 
	return q;
}







__kernel void 
  integrateTransformsKernel( __global b3RigidBodyData_t* bodies,const int numNodes, float timeStep, float angularDamping, float4 gravityAcceleration)
{
	int nodeID = get_global_id(0);
	float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254f);
	if( nodeID < numNodes && (bodies[nodeID].m_invMass != 0.f))
	{
		//angular velocity
		{
			float4 axis;
			//add some hardcoded angular damping
			bodies[nodeID].m_angVel.x *= angularDamping;
			bodies[nodeID].m_angVel.y *= angularDamping;
			bodies[nodeID].m_angVel.z *= angularDamping;
			
			float4 angvel = bodies[nodeID].m_angVel;
			float fAngle = native_sqrt(dot(angvel, angvel));
			//limit the angular motion
			if(fAngle*timeStep > BT_GPU_ANGULAR_MOTION_THRESHOLD)
			{
				fAngle = BT_GPU_ANGULAR_MOTION_THRESHOLD / timeStep;
			}
			if(fAngle < 0.001f)
			{
				// use Taylor's expansions of sync function
				axis = angvel * (0.5f*timeStep-(timeStep*timeStep*timeStep)*0.020833333333f * fAngle * fAngle);
			}
			else
			{
				// sync(fAngle) = sin(c*fAngle)/t
				axis = angvel * ( native_sin(0.5f * fAngle * timeStep) / fAngle);
			}
			float4 dorn = axis;
			dorn.w = native_cos(fAngle * timeStep * 0.5f);
			float4 orn0 = bodies[nodeID].m_quat;

			float4 predictedOrn = quatMult(dorn, orn0);
			predictedOrn = b3QuatNormalize(predictedOrn);
			bodies[nodeID].m_quat=predictedOrn;
		}

		//linear velocity		
		bodies[nodeID].m_pos +=  bodies[nodeID].m_linVel * timeStep;
		
		//apply gravity
		bodies[nodeID].m_linVel += gravityAcceleration * timeStep;
		
	}
}
