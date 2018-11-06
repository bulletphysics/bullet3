#ifndef SIMPLE_OPENGL2_APP_H
#define SIMPLE_OPENGL2_APP_H

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"

class SimpleOpenGL2App : public CommonGraphicsApp
{
protected:
	struct SimpleOpenGL2AppInternalData* m_data;

public:
	SimpleOpenGL2App(const char* title, int width, int height);
	virtual ~SimpleOpenGL2App();

	virtual void drawGrid(DrawGridData data = DrawGridData());
	virtual void setUpAxis(int axis);
	virtual int getUpAxis() const;

	virtual void swapBuffer();
	virtual void drawText(const char* txt, int posX, int posY, float size, float colorRGBA[4]);

	virtual void drawTexturedRect(float x0, float y0, float x1, float y1, float color[4], float u0, float v0, float u1, float v1, int useRGBA){};
	virtual void setBackgroundColor(float red, float green, float blue);
	virtual int registerCubeShape(float halfExtentsX, float halfExtentsY, float halfExtentsZ, int textureIndex = -1, float textureScaling = 1);

	virtual int registerGraphicsUnitSphereShape(EnumSphereLevelOfDetail lod, int textureId = -1);
	virtual void drawText3D(const char* txt, float posX, float posZY, float posZ, float size);
	virtual void drawText3D(const char* txt, float position[3], float orientation[4], float color[4], float size, int optionFlag);

	virtual void registerGrid(int xres, int yres, float color0[4], float color1[4]);
};
#endif  //SIMPLE_OPENGL2_APP_H