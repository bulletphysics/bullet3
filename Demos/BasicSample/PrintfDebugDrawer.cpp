#include "PrintfDebugDrawer.h"

#include <stdio.h> //printf debugging

PrintfDebugDrawer::PrintfDebugDrawer()
{
}
	

void	PrintfDebugDrawer::DrawLine(const SimdVector3& from,const SimdVector3& to,const SimdVector3& color)
{
	if (m_debugMode > 0)
	{
		printf("DrawLine: from(%f , %f . %f) , to(%f , %f . %f)\n",from.getX(),from.getY(),from.getZ(),to.getX(),to.getY(),to.getZ());
	}
}

void	PrintfDebugDrawer::DrawContactPoint(const SimdVector3& pointOnB,const SimdVector3& normalOnB,float distance,int lifeTime,const SimdVector3& color)
{
	if (m_debugMode & IDebugDraw::DBG_DrawContactPoints)
	{
		SimdVector3 to=pointOnB+normalOnB*distance;
		const SimdVector3&from = pointOnB;
		printf("DrawContactPoint (%d) from(%f , %f . %f) , to(%f , %f . %f)\n",lifeTime, from.getX(),from.getY(),from.getZ(),to.getX(),to.getY(),to.getZ());
	}
}

void	PrintfDebugDrawer::SetDebugMode(int debugMode)
{
		m_debugMode = debugMode;
}
