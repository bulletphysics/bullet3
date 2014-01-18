#ifndef GPU_DEMO_H
#define GPU_DEMO_H
class GLInstancingRenderer;
class GLPrimitiveRenderer;




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
			bool m_useInstancedCollisionShapes;
            GLInstancingRenderer*   m_instancingRenderer;
			GLPrimitiveRenderer*	m_primRenderer;
			
			class b3gWindowInterface*	m_window;
			class GwenUserInterface*	m_gui;
			
            ConstructionInfo()
                    :useOpenCL(true),
                    preferredOpenCLPlatformIndex(-1),
                    preferredOpenCLDeviceIndex(-1),
	#ifdef __APPLE__
	arraySizeX(10),
	arraySizeY(10),
	arraySizeZ(10),
	#else

		arraySizeX(30),
		arraySizeY(30),
		arraySizeZ(30),
#endif
		m_useConcaveMesh(false),
		gapX(16.3),
		gapY(6.3),
		gapZ(16.3),
		m_useInstancedCollisionShapes(true),
                    m_instancingRenderer(0),
					m_window(0),
					m_gui(0)
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

	unsigned char* loadImage(const char* fileName, int& width, int& height, int& n);

	int	registerGraphicsSphereShape(const ConstructionInfo& ci, float radius, bool usePointSprites=true, int largeSphereThreshold=100, int mediumSphereThreshold=10);

	struct GpuDemoInternalData*	getInternalData();
	
	virtual bool	mouseMoveCallback(float x,float y)
	{
		return false;
	}
	virtual bool	mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}
	virtual bool	keyboardCallback(int key, int state)
	{
		return false;
	}
};

#endif
