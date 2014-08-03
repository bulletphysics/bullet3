#include "stan/vecmath.h"
#include "stan/Cloth.h"
#include "Bullet3Common/b3Vector3.h"
#include "../../btgui/OpenGLWindow/SimpleOpenGL3App.h"

Cloth* cloth = 0;

int numX = 60, numY=60;
const size_t total_points = (numX)*(numY);

void drawCloth(class GLInstancingRenderer* renderer)
{
	GLint err = glGetError();
    assert(err==GL_NO_ERROR);
	
	
	err = glGetError();
    assert(err==GL_NO_ERROR);
	
	b3AlignedObjectArray<unsigned int> indices;

	for (int i=0;i<cloth->springs.count;i++)
	{
		indices.push_back(cloth->springs[i].a);
		indices.push_back(cloth->springs[i].b);
	}
	float lineColor[4]={0.4,0.4,1.0,1};
	renderer->drawLines(&cloth->X[0].x,lineColor,total_points,sizeof(float3),&indices[0],indices.size(),1);
	err = glGetError();
    assert(err==GL_NO_ERROR);
	float pointColor[4]={1,0.4,0.4,1};

	glDisable(GL_DEPTH_TEST);
	renderer->drawPoints(&cloth->X[0].x,pointColor,total_points,sizeof(float3),3);
	glEnable(GL_DEPTH_TEST);

	err = glGetError();
    assert(err==GL_NO_ERROR);
	
	
}

int main(int argc, char* argv[])
{
	float size=10;
	cloth = ClothCreate(numX,numY,size);
	
	float dt = 1./120.f;
	
	SimpleOpenGL3App* app = new SimpleOpenGL3App("title",1024,768);
	app->m_instancingRenderer->setCameraDistance(13);
	app->m_instancingRenderer->setCameraPitch(0);
	app->m_instancingRenderer->setCameraTargetPosition(b3MakeVector3(0,0,0));

	GLint err = glGetError();
    assert(err==GL_NO_ERROR);
	
	do
	{
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera();
		
		app->drawGrid();
		app->drawText("Cloth simulation using implicit integration, by Stan Melax",10,10);
		cloth->Simulate(dt);
		cloth->cloth_gravity.y = -9.8;//-9.8;//-9.8;//-9.8;//0;//-9.8;//0;//-9.8;//0;//-9.8;
		cloth->cloth_gravity.z =-9.8;//0;//-9.8;//0;//-9.8;

		cloth->spring_struct=10000000.0f;
		cloth->spring_shear=10000000.0f;

		//cloth->spring_struct=1000000.0f;
		//cloth->spring_shear=1000000.0f;

		cloth->spring_damp = 0;//100;

		drawCloth(app->m_instancingRenderer);

		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	delete cloth;
	delete app;
	return 0;
}
