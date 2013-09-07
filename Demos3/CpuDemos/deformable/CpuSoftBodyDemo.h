#ifndef CPU_SOFT_BODY_DEMO_H
#define CPU_SOFT_BODY_DEMO_H

#include "../CpuDemo.h"

class CpuSoftBodyDemo : public CpuDemo
{
protected:
	class GLInstancingRenderer* m_instancingRenderer;
	class b3gWindowInterface*	m_window;

	struct CpuSoftBodyDemoInternalData*	m_data;

public:
	
	CpuSoftBodyDemo();
	virtual ~CpuSoftBodyDemo();

	virtual void	initPhysics(const ConstructionInfo& ci);

	virtual void	setupScene(const ConstructionInfo& ci);

	virtual void	destroyScene(){};

	virtual void	exitPhysics();
	
	virtual const char* getName()
	{
		return "CPUSOFT";
	}
	static CpuDemo* MyCreateFunc()
	{
		CpuDemo* demo = new CpuSoftBodyDemo;
		return demo;
	}
	
	virtual void renderScene();
	
	

	virtual void clientMoveAndDisplay();

	
};

class CpuSoftClothDemo : public CpuSoftBodyDemo
{
protected:

	struct CpuSoftClothDemoInternalData*	m_clothData;

public:
		CpuSoftClothDemo();
		virtual ~CpuSoftClothDemo();
		
	unsigned char* loadImage(const char* fileName, int& width, int& height, int& n);

	virtual void	setupScene(const ConstructionInfo& ci);
			
	virtual void	renderScene();

	virtual const char* getName()
	{
		return "CpuSoftCloth";
	}
	
	
	virtual void clientMoveAndDisplay();

	static CpuDemo* MyCreateFunc()
	{
		CpuDemo* demo = new CpuSoftClothDemo;
		return demo;
	}
	
};

#endif //CPU_SOFT_BODY_DEMO_H

