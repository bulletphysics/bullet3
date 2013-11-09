#ifndef SIMPLE_OPENGL3_APP_H
#define SIMPLE_OPENGL3_APP_H

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "OpenGLWindow/b3gWindowInterface.h"

struct SimpleOpenGL3App
{
	struct SimpleInternalData* m_data;

	class b3gWindowInterface*	m_window;
	class GLPrimitiveRenderer*	m_primRenderer;
	class GLInstancingRenderer* m_instancingRenderer;

	SimpleOpenGL3App(const char* title, int width,int height);
	virtual ~SimpleOpenGL3App();
	
	void drawGrid(int gridSize=10);
	void swapBuffer();
	void drawText( const char* txt, int posX, int posY);

};

#endif //SIMPLE_OPENGL3_APP_H
