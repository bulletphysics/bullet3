#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "IDebugDraw.h"



class GLDebugDrawer : public IDebugDraw
{
	int m_debugMode;

public:

	GLDebugDrawer();

	virtual void	DrawLine(const SimdVector3& from,const SimdVector3& to,const SimdVector3& color);

	virtual void	DrawContactPoint(const SimdVector3& PointOnB,const SimdVector3& normalOnB,float distance,int lifeTime,const SimdVector3& color);

	virtual void	SetDebugMode(int debugMode);

	virtual int		GetDebugMode() const { return m_debugMode;}

};

#endif//GL_DEBUG_DRAWER_H
