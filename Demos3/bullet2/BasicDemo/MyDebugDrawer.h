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
	virtual void	drawLine(const btVector3& from1,const btVector3& to1,const btVector3& color1)
	{
        float from[4] = {from1[0],from1[1],from1[2],from1[3]};
        float to[4] = {to1[0],to1[1],to1[2],to1[3]};
        float color[4] = {color1[0],color1[1],color1[2],color1[3]};
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
