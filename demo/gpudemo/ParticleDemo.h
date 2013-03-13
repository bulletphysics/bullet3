#ifndef PARTICLE_DEMO_H
#define PARTICLE_DEMO_H

//#include "GpuDemo.h"
struct GLInstancingRenderer;
class ParticleDemo;

class ParticleDemo //: public GpuDemo
{
public:

	typedef class ParticleDemo* (CreateFunc)();
	



	struct ConstructionInfo
	{
		bool useOpenCL;
		int preferredOpenCLPlatformIndex;
		int preferredOpenCLDeviceIndex;
		int arraySizeX;
		int arraySizeY;
		int arraySizeZ;
		bool m_useConcaveMesh;
		float gapX;
		float gapY;
		float gapZ;
		GLInstancingRenderer*	m_instancingRenderer;
		ConstructionInfo()
			:useOpenCL(false),//true),
			preferredOpenCLPlatformIndex(-1),
			preferredOpenCLDeviceIndex(-1),
			arraySizeX(10),
			arraySizeY(10 ),
			arraySizeZ(10),
			m_useConcaveMesh(false),
			gapX(4.3),
			gapY(4.0),
			gapZ(4.3),
			m_instancingRenderer(0)
		{
		}
	};

	protected:
	struct ParticleInternalData*	m_data;

	GLInstancingRenderer*	m_instancingRenderer;

	void initCL(int preferredDeviceIndex, int preferredPlatformIndex);
	void exitCL();

public:

	ParticleDemo();

	virtual ~ParticleDemo();

	virtual void setupScene(const ConstructionInfo& ci);

	virtual void	initPhysics(const ConstructionInfo& ci);

	virtual void	exitPhysics();

	virtual const char* getName()
	{
		return "ParticleDemo";
	}
	static ParticleDemo* MyCreateFunc()
	{
		ParticleDemo* demo = new ParticleDemo;
		return demo;
	}



	virtual void renderScene();

	virtual void clientMoveAndDisplay();
};

#endif //PARTICLE_DEMO_H
