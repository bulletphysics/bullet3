

#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"


inline void b3IntegrateTransform( b3RigidBodyData* body, float timeStep, float angularDamping, b3Float4ConstArg gravityAcceleration)
{
	float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254f);
	
	if( (body->m_invMass != 0.f))
	{
		//angular velocity
		{
			b3Float4 axis;
			//add some hardcoded angular damping
			body->m_angVel.x *= angularDamping;
			body->m_angVel.y *= angularDamping;
			body->m_angVel.z *= angularDamping;
			
			b3Float4 angvel = body->m_angVel;
			float fAngle = b3Sqrt(b3Dot(angvel, angvel));
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
				axis = angvel * ( b3Sin(0.5f * fAngle * timeStep) / fAngle);
			}
			b3Quat dorn;
			dorn.x = axis.x;
			dorn.y = axis.y;
			dorn.z = axis.z;
			dorn.w = b3Cos(fAngle * timeStep * 0.5f);
			b3Quat orn0 = body->m_quat;

			b3Quat predictedOrn = b3QuatMul(dorn, orn0);
			predictedOrn = b3QuatNormalized(predictedOrn);
			body->m_quat=predictedOrn;
		}

		//apply gravity
		body->m_linVel += gravityAcceleration * timeStep;

		//linear velocity		
		body->m_pos +=  body->m_linVel * timeStep;
		
	}
}
