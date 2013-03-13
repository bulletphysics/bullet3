#ifndef PAIR_BENCH_H
#define PAIR_BENCH_H

#include "../GpuDemo.h"

class PairBench : public GpuDemo
{
public:
	
	virtual void	initPhysics(const ConstructionInfo& ci);
	
	virtual void	exitPhysics();
	
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

