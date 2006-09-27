#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "LinearMath/btIDebugDraw.h"



class GLDebugDrawer : public btIDebugDraw
{
	int m_debugMode;

public:

	GLDebugDrawer();

	virtual void	DrawLine(const btVector3& from,const btVector3& to,const btVector3& color);

	virtual void	DrawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,float distance,int lifeTime,const btVector3& color);

	virtual void	SetDebugMode(int debugMode);

	virtual int		GetDebugMode() const { return m_debugMode;}

};

#endif//GL_DEBUG_DRAWER_H
