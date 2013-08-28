
#ifndef CPU_DEMO_H
#define CPU_DEMO_H

struct GLInstancingRenderer;
struct GwenUserInterface;

struct CpuDemo
{

	struct ConstructionInfo
    {
    		bool m_useInstancedCollisionShapes;
            GLInstancingRenderer*   m_instancingRenderer;
			struct GLPrimitiveRenderer*	m_primRenderer;
			
			class b3gWindowInterface*	m_window;
			class GwenUserInterface*	m_gui;
			
            ConstructionInfo()
				:m_useInstancedCollisionShapes(true),
                    m_instancingRenderer(0),
					m_window(0),
					m_gui(0)
            {
            }
    };


	virtual void    initPhysics(const ConstructionInfo& ci)=0;
	
	virtual void    exitPhysics()=0;
	
	virtual void renderScene()=0;
	
	virtual void clientMoveAndDisplay()=0;


	virtual const char* getName() {
		return "";
	}
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

	typedef class CpuDemo* (CreateFunc)();
};

#endif //CPU_DEMO_H

