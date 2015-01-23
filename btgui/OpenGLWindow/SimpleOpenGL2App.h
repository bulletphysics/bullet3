#ifndef SIMPLE_OPENGL2_APP_H
#define SIMPLE_OPENGL2_APP_H

#include "OpenGLWindow/CommonGraphicsApp.h"

class SimpleOpenGL2App : public CommonGraphicsApp
{
protected:
	struct SimpleOpenGL2AppInternalData*	m_data;

public:
	SimpleOpenGL2App(const char* title, int width, int height);
	virtual ~SimpleOpenGL2App();

	virtual void drawGrid(DrawGridData data=DrawGridData());
	virtual void setUpAxis(int axis);
	virtual int getUpAxis() const;
	
	virtual void swapBuffer();
	virtual void drawText( const char* txt, int posX, int posY);

	virtual int	registerCubeShape(float halfExtentsX,float halfExtentsY, float halfExtentsZ)
	{
		return 0;
	}
	virtual int	registerGraphicsSphereShape(float radius, bool usePointSprites, int largeSphereThreshold, int mediumSphereThreshold)
	{
		return 0;
	}
};
#endif //SIMPLE_OPENGL2_APP_H