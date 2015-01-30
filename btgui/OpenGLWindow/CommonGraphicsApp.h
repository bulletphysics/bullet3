#ifndef COMMON_GRAPHICS_APP_H
#define COMMON_GRAPHICS_APP_H

struct DrawGridData
{
    int gridSize;
    float upOffset;
    int upAxis;
    float gridColor[4];

    DrawGridData()
    :gridSize(10),
    upOffset(0.001f),
    upAxis(1)
    {
        gridColor[0] = 0.6f;
        gridColor[1] = 0.6f;
        gridColor[2] = 0.6f;
        gridColor[3] = 1.f;
    }
};

struct CommonGraphicsApp
{
	CommonGraphicsApp()
		:m_window(0),
		m_renderer(0),
		m_parameterInterface(0)
	{
	}
	virtual ~CommonGraphicsApp()
	{
	}
	
	class b3gWindowInterface*	m_window;
	struct CommonRenderInterface*	m_renderer;
	struct CommonParameterInterface*	m_parameterInterface;

	virtual void drawGrid(DrawGridData data=DrawGridData()) = 0;
	virtual void setUpAxis(int axis) = 0;
	virtual int getUpAxis() const = 0;
	
	virtual void swapBuffer() = 0;
	virtual void drawText( const char* txt, int posX, int posY) = 0;
	virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size)=0;
	virtual int	registerCubeShape(float halfExtentsX,float halfExtentsY, float halfExtentsZ)=0;
	virtual int	registerGraphicsSphereShape(float radius, bool usePointSprites=true, int largeSphereThreshold=100, int mediumSphereThreshold=10)=0;
	virtual void registerGrid(int xres, int yres, float color0[4], float color1[4])=0;
};


#endif //COMMON_GRAPHICS_APP_H
