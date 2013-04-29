#ifndef BULLET2_FILE_DEMO_H
#define BULLET2_FILE_DEMO_H



#include "GpuRigidBodyDemo.h"

class Bullet2FileDemo : public GpuRigidBodyDemo
{
	class b3BulletDataExtractor* m_loader;

public:

	Bullet2FileDemo();
	virtual ~Bullet2FileDemo();
	virtual const char* getName()
	{
		return "Bullet2File";
	}

	static GpuDemo* MyCreateFunc()
	{
		GpuDemo* demo = new Bullet2FileDemo;
		return demo;
	}

	virtual void setupScene(const ConstructionInfo& ci);

};

#endif//BULLET2_FILE_DEMO_H

