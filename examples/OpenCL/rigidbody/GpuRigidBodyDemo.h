#ifndef GPU_RIGID_BODY_DEMO_H
#define GPU_RIGID_BODY_DEMO_H

#include "Bullet3Common/b3Vector3.h"
#include "../CommonOpenCL/CommonOpenCLBase.h"

class GpuRigidBodyDemo : public CommonOpenCLBase
{
protected:
	class GLInstancingRenderer* m_instancingRenderer;
	class GLPrimitiveRenderer* m_primRenderer;
	class CommonWindowInterface* m_window;

	struct GpuRigidBodyDemoInternalData* m_data;

public:
	GpuRigidBodyDemo(GUIHelperInterface* helper);
	virtual ~GpuRigidBodyDemo();

	virtual void initPhysics();

	virtual void setupScene();

	virtual void destroyScene(){};

	virtual void exitPhysics();

	virtual void renderScene();

	void resetCamera();

	virtual void stepSimulation(float deltaTime);

	//for picking
	b3Vector3 getRayTo(int x, int y);
	virtual bool mouseMoveCallback(float x, float y);
	virtual bool mouseButtonCallback(int button, int state, float x, float y);
	virtual bool keyboardCallback(int key, int state);

	unsigned char* loadImage(const char* fileName, int& width, int& height, int& n);
};

#endif  //GPU_RIGID_BODY_DEMO_H
