#ifndef GPU_DEMO_H
#define GPU_DEMO_H
class GLInstancingRenderer;




class GpuDemo
{
protected:
	
	struct GpuDemoInternalData*	m_clData;

	
	virtual void initCL(int preferredDeviceIndex, int preferredPlatformIndex);
	virtual void exitCL();
public:
	
	typedef class GpuDemo* (CreateFunc)();

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
            GLInstancingRenderer*   m_instancingRenderer;
			class btgWindowInterface*	m_window;

            ConstructionInfo()
                    :useOpenCL(true),
                    preferredOpenCLPlatformIndex(-1),
                    preferredOpenCLDeviceIndex(-1),
					arraySizeX(30),
		arraySizeY(20 ),
		arraySizeZ(30),
		m_useConcaveMesh(false),
		gapX(14.3),
		gapY(14.0),
		gapZ(14.3),
                    m_instancingRenderer(0),
					m_window(0)
            {
            }
    };

	GpuDemo();
	virtual ~GpuDemo();
	
	virtual const char* getName()=0;
	
	virtual void    initPhysics(const ConstructionInfo& ci)=0;
	
	virtual void    exitPhysics()=0;
	
	virtual void renderScene()=0;
	
	virtual void clientMoveAndDisplay()=0;


};

#endif

