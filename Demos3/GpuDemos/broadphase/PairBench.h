#ifndef PAIR_BENCH_H
#define PAIR_BENCH_H

#include "../GpuDemo.h"

class PairBench : public GpuDemo
{
	
	class GLInstancingRenderer* m_instancingRenderer;
	class b3gWindowInterface*	m_window;

	struct PairBenchInternalData*	m_data;



public:
	
	PairBench();
	virtual ~PairBench();

	virtual void	initPhysics(const ConstructionInfo& ci);
	virtual void	exitPhysics();
	

	void	createBroadphase(int xdim, int ydim, int zdim);
	void	deleteBroadphase();

	virtual const char* getName()
	{
		return "PairBench";
	}
	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new PairBench;
		return demo;
	}
	
	
	
	virtual void renderScene();
	
	virtual void clientMoveAndDisplay();

	
};

#endif

