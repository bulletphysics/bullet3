#ifndef COMMON_GRAPHICS_APP_H
#define COMMON_GRAPHICS_APP_H



#include "Bullet3Common/b3Vector3.h"
#include "CommonRenderInterface.h"
#include "CommonWindowInterface.h"
#include "CommonCameraInterface.h"

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
	class CommonWindowInterface*	m_window;
	struct CommonRenderInterface*	m_renderer;
	struct CommonParameterInterface*	m_parameterInterface;
	struct Common2dCanvasInterface*	m_2dCanvasInterface;

	bool	m_leftMouseButton;
	bool	m_middleMouseButton;
	bool	m_rightMouseButton;
	float m_wheelMultiplier;
	float m_mouseMoveMultiplier;
	float	m_mouseXpos;
	float	m_mouseYpos;
	bool	m_mouseInitialized;

	CommonGraphicsApp()
		:m_window(0),
		m_renderer(0),
		m_parameterInterface(0),
		m_2dCanvasInterface(0),
		m_leftMouseButton(false),
		m_middleMouseButton(false),
		m_rightMouseButton(false),
		m_wheelMultiplier(0.01f),
		m_mouseMoveMultiplier(0.4f),
		m_mouseXpos(0.f),
		m_mouseYpos(0.f),
		m_mouseInitialized(false)
	{
	}
	virtual ~CommonGraphicsApp()
	{
	}

	virtual void drawGrid(DrawGridData data=DrawGridData()) = 0;
	virtual void setUpAxis(int axis) = 0;
	virtual int getUpAxis() const = 0;
	
	virtual void swapBuffer() = 0;
	virtual void drawText( const char* txt, int posX, int posY) = 0;
	virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size)=0;
	virtual int	registerCubeShape(float halfExtentsX,float halfExtentsY, float halfExtentsZ)=0;
	virtual int	registerGraphicsSphereShape(float radius, bool usePointSprites=true, int largeSphereThreshold=100, int mediumSphereThreshold=10)=0;
	virtual void registerGrid(int xres, int yres, float color0[4], float color1[4])=0;

	void defaultMouseButtonCallback( int button, int state, float x, float y)
	{
		if (button==0)
			m_leftMouseButton= (state==1);
		if (button==1)
			m_middleMouseButton= (state==1);

		if (button==2)
			m_rightMouseButton=	(state==1);

		m_mouseXpos = x;
		m_mouseYpos = y;
		m_mouseInitialized = true;
	}
	void defaultMouseMoveCallback(  float x, float y)
	{
		
		if (m_window && m_renderer)
		{
			CommonCameraInterface* camera = m_renderer->getActiveCamera();

			bool isAltPressed = m_window->isModifierKeyPressed(B3G_ALT);
			bool isControlPressed = m_window->isModifierKeyPressed(B3G_CONTROL);
			
			
			if (isAltPressed || isControlPressed)
			{
				float xDelta = x-m_mouseXpos;
				float yDelta = y-m_mouseYpos;
				float cameraDistance = camera->getCameraDistance();
				float pitch = camera->getCameraPitch();
				float yaw = camera->getCameraYaw();

				float targPos[3];
				float	camPos[3];

				camera->getCameraTargetPosition(targPos);
				camera->getCameraPosition(camPos);
			
				b3Vector3 cameraPosition = b3MakeVector3(b3Scalar(camPos[0]),
														b3Scalar(camPos[1]),
														b3Scalar(camPos[2]));

				b3Vector3 cameraTargetPosition = b3MakeVector3(	b3Scalar(targPos[0]),
																b3Scalar(targPos[1]),
																b3Scalar(targPos[2]));
				b3Vector3 cameraUp = b3MakeVector3(0,0,0);
				cameraUp[camera->getCameraUpAxis()] = 1.f;
							
				if (m_leftMouseButton)
				{
		//			if (b3Fabs(xDelta)>b3Fabs(yDelta))
		//			{
						pitch -= xDelta*m_mouseMoveMultiplier;
		//			} else
		//			{
						yaw += yDelta*m_mouseMoveMultiplier;
		//			}
				}

				if (m_middleMouseButton)
				{
					cameraTargetPosition += cameraUp * yDelta*0.01;


					b3Vector3 fwd = cameraTargetPosition-cameraPosition;
					b3Vector3 side = cameraUp.cross(fwd);
					side.normalize();
					cameraTargetPosition += side * xDelta*0.01;

				}
				if (m_rightMouseButton)
				{
						cameraDistance -= xDelta*0.01f;
						cameraDistance -= yDelta*0.01f;
						if (cameraDistance<1)
							cameraDistance=1;
						if (cameraDistance>1000)
							cameraDistance=1000;
				}
				camera->setCameraDistance(cameraDistance);
				camera->setCameraPitch(pitch);
				camera->setCameraYaw(yaw);
				camera->setCameraTargetPosition(cameraTargetPosition[0],cameraTargetPosition[1],cameraTargetPosition[2]);
				
			}	

		}//m_window &&  m_renderer
		
		m_mouseXpos = x;
		m_mouseYpos = y;
		m_mouseInitialized = true;
	}
//	void defaultKeyboardCallback(int key, int state)
//	{
//	}
	void defaultWheelCallback( float deltax, float deltay)
	{

		if (m_renderer)
		{
			b3Vector3 cameraTargetPosition, cameraPosition, cameraUp = b3MakeVector3(0,0,0);
			cameraUp[getUpAxis()] = 1;
			CommonCameraInterface* camera = m_renderer->getActiveCamera();
			
			camera->getCameraPosition(cameraPosition);
			camera->getCameraTargetPosition(cameraTargetPosition);
			
			if (!m_leftMouseButton)
			{

				float cameraDistance = 	camera->getCameraDistance();
				if (deltay<0 || cameraDistance>1)
				{
					cameraDistance -= deltay*0.1f;
					if (cameraDistance<1)
						cameraDistance=1;
					camera->setCameraDistance(cameraDistance);

				} else
				{
					
					b3Vector3 fwd = cameraTargetPosition-cameraPosition;
					fwd.normalize();
					cameraTargetPosition += fwd*deltay*m_wheelMultiplier;//todo: expose it in the GUI?
				}
			} else
			{
				if (b3Fabs(deltax)>b3Fabs(deltay))
				{
					b3Vector3 fwd = cameraTargetPosition-cameraPosition;
					b3Vector3 side = cameraUp.cross(fwd);
					side.normalize();
					cameraTargetPosition += side * deltax*m_wheelMultiplier;

				} else
				{
					cameraTargetPosition -= cameraUp * deltay*m_wheelMultiplier;

				}
			}

			camera->setCameraTargetPosition(cameraTargetPosition[0],cameraTargetPosition[1],cameraTargetPosition[2]);
		}

	}


};


#endif //COMMON_GRAPHICS_APP_H
