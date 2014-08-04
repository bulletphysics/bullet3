#ifndef SIMPLE_OPENGL3_APP_H
#define SIMPLE_OPENGL3_APP_H

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "OpenGLWindow/b3gWindowInterface.h"

struct DrawGridData
{
    int gridSize;
    float upOffset;
    int upAxis;
    float gridColor[4];

    DrawGridData()
    :gridSize(10),
    upOffset(0.001f),
    upAxis(1)
    {
        gridColor[0] = 0.6f;
        gridColor[1] = 0.6f;
        gridColor[2] = 0.6f;
        gridColor[3] = 1.f;
    }
};

struct SimpleOpenGL3App
{
	struct SimpleInternalData* m_data;

	class b3gWindowInterface*	m_window;
	class GLPrimitiveRenderer*	m_primRenderer;
	class GLInstancingRenderer* m_instancingRenderer;
	struct CommonParameterInterface*	m_parameterInterface;

	SimpleOpenGL3App(const char* title, int width,int height);
	virtual ~SimpleOpenGL3App();

	int	registerCubeShape(float halfExtentsX=1.f,float halfExtentsY=1.f, float halfExtentsZ = 1.f);
	int	registerGraphicsSphereShape(float radius, bool usePointSprites=true, int largeSphereThreshold=100, int mediumSphereThreshold=10);

    void dumpNextFrameToPng(const char* pngFilename);
    void dumpFramesToVideo(const char* mp4Filename);
    
	void drawGrid(DrawGridData data=DrawGridData());
	void setUpAxis(int axis);
	int getUpAxis() const;
	
	void swapBuffer();
	void drawText( const char* txt, int posX, int posY);
	struct sth_stash* getFontStash();


};

#endif //SIMPLE_OPENGL3_APP_H
