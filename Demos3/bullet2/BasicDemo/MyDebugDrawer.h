#ifndef MY_DEBUG_DRAWER_H
#define MY_DEBUG_DRAWER_H

#include "LinearMath/btIDebugDraw.h"

class MyDebugDrawer : public btIDebugDraw
{
	SimpleOpenGL3App* m_glApp;
	int m_debugMode;
public:

	MyDebugDrawer(SimpleOpenGL3App* app)
		: m_glApp(app)
		,m_debugMode(btIDebugDraw::DBG_DrawWireframe|btIDebugDraw::DBG_DrawAabb)
	{

	}
	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
	{
		m_glApp->m_instancingRenderer->drawLine(from,to,color);
	}

	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
	{
	}

	virtual void	reportErrorWarning(const char* warningString)
	{
	}

	virtual void	draw3dText(const btVector3& location,const char* textString)
	{
	}
	
	virtual void	setDebugMode(int debugMode)
	{
		m_debugMode = debugMode;
	}
	
	virtual int		getDebugMode() const
	{
		return m_debugMode;
	}


};

#endif //MY_DEBUG_DRAWER_H
