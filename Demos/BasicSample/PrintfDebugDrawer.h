#ifndef PRINTF_DEBUG_DRAWER_H
#define PRINTF_DEBUG_DRAWER_H

#include "IDebugDraw.h"




class PrintfDebugDrawer : public IDebugDraw
{
	int m_debugMode;

public:

	PrintfDebugDrawer();

	virtual void	DrawLine(const SimdVector3& from,const SimdVector3& to,const SimdVector3& color);

	virtual void	DrawContactPoint(const SimdVector3& PointOnB,const SimdVector3& normalOnB,float distance,int lifeTime,const SimdVector3& color);

	virtual void	SetDebugMode(int debugMode);

	virtual int		GetDebugMode() const { return m_debugMode;}

};

#endif//PRINTF_DEBUG_DRAWER_H
