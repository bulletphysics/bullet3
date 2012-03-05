MSTRINGIFY(

float4 quatMult(float4 q1, float4 q2)
{
	float4 q;
	q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	q.y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
	q.z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;
	q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z; 
	return q;
}

float4 quatNorm(float4 q)
{
	float len = native_sqrt(dot(q, q));
	if(len > 0.f)
	{
		q *= 1.f / len;
	}
	else
	{
		q.x = q.y = q.z = 0.f;
		q.w = 1.f;
	}
	return q;
}



__kernel void 
  interopKernel( const int startOffset, const int numNodes, __global float4 *g_vertexBuffer,
		   __global float4 *linVel,
		   __global float4 *pAngVel)
{
	int nodeID = get_global_id(0);
	float timeStep = 0.0166666;
	
	float BT_GPU_ANGULAR_MOTION_THRESHOLD = (0.25f * 3.14159254);
	
	if( nodeID < numNodes )
	{
		g_vertexBuffer[nodeID + startOffset/4] += linVel[nodeID]*timeStep;
		
		//		g_vertexBuffer[nodeID + startOffset/4+numNodes] += angVel[nodeID];

		float4 axis;
		float4 angvel = pAngVel[nodeID];
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
		float4 orn0 = g_vertexBuffer[nodeID + startOffset/4+numNodes];
		float4 predictedOrn = quatMult(dorn, orn0);
		predictedOrn = quatNorm(predictedOrn);
		g_vertexBuffer[nodeID + startOffset/4+numNodes]=predictedOrn;
	}
}

);