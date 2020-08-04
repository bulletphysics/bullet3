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

	DrawGridData(int upAx = 1)
		: gridSize(10),
		upOffset(0.001f),
		upAxis(upAx)
	{
		gridColor[0] = 0.6f;
		gridColor[1] = 0.6f;
		gridColor[2] = 0.6f;
		gridColor[3] = 1.f;
	}
};

enum EnumSphereLevelOfDetail
{
	SPHERE_LOD_POINT_SPRITE = 0,
	SPHERE_LOD_LOW,
	SPHERE_LOD_MEDIUM,
	SPHERE_LOD_HIGH,

};
struct CommonGraphicsApp
{
	enum drawText3DOption
	{
		eDrawText3D_OrtogonalFaceCamera = 1,
		eDrawText3D_TrueType = 2,
		eDrawText3D_TrackObject = 4,
	};
	class CommonWindowInterface* m_window;
	struct CommonRenderInterface* m_renderer;
	struct CommonParameterInterface* m_parameterInterface;
	struct Common2dCanvasInterface* m_2dCanvasInterface;

	bool m_leftMouseButton;
	bool m_middleMouseButton;
	bool m_rightMouseButton;
	float m_wheelMultiplier;
	float m_mouseMoveMultiplier;
	float m_mouseXpos;
	float m_mouseYpos;
	bool m_mouseInitialized;
	float m_backgroundColorRGB[3];

	CommonGraphicsApp()
		: m_window(0),
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
		m_backgroundColorRGB[0] = 0.7;
		m_backgroundColorRGB[1] = 0.7;
		m_backgroundColorRGB[2] = 0.8;
	}
	virtual ~CommonGraphicsApp()
	{
	}

	virtual void dumpNextFrameToPng(const char* pngFilename) {}
	virtual void dumpFramesToVideo(const char* mp4Filename) {}

	virtual void getScreenPixels(unsigned char* rgbaBuffer, int bufferSizeInBytes, float* depthBuffer, int depthBufferSizeInBytes) {}
	virtual void setViewport(int width, int height) {}

	virtual void getBackgroundColor(float* red, float* green, float* blue) const
	{
		if (red)
			*red = m_backgroundColorRGB[0];
		if (green)
			*green = m_backgroundColorRGB[1];
		if (blue)
			*blue = m_backgroundColorRGB[2];
	}
	virtual void setMp4Fps(int fps) {}
	virtual void setBackgroundColor(float red, float green, float blue)
	{
		m_backgroundColorRGB[0] = red;
		m_backgroundColorRGB[1] = green;
		m_backgroundColorRGB[2] = blue;
	}
	virtual void setMouseWheelMultiplier(float mult)
	{
		m_wheelMultiplier = mult;
	}
	virtual float getMouseWheelMultiplier() const
	{
		return m_wheelMultiplier;
	}

	virtual void setMouseMoveMultiplier(float mult)
	{
		m_mouseMoveMultiplier = mult;
	}

	virtual float getMouseMoveMultiplier() const
	{
		return m_mouseMoveMultiplier;
	}

	virtual void drawGrid(DrawGridData data = DrawGridData()) = 0;
	virtual void setUpAxis(int axis) = 0;
	virtual int getUpAxis() const = 0;

	virtual void swapBuffer() = 0;
	virtual void drawText(const char* txt, int posX, int posY)
	{
		float size = 1;
		float colorRGBA[4] = { 0, 0, 0, 1 };
		drawText(txt, posX, posY, size, colorRGBA);
	}

	virtual void drawText(const char* txt, int posX, int posY, float size)
	{
		float colorRGBA[4] = { 0, 0, 0, 1 };
		drawText(txt, posX, posY, size, colorRGBA);
	}
	virtual void drawText(const char* txt, int posX, int posY, float size, float colorRGBA[4]) = 0;
	virtual void drawText3D(const char* txt, float posX, float posY, float posZ, float size) = 0;
	virtual void drawText3D(const char* txt, float position[3], float orientation[4], float color[4], float size, int optionFlag) = 0;
	virtual void drawTexturedRect(float x0, float y0, float x1, float y1, float color[4], float u0, float v0, float u1, float v1, int useRGBA) = 0;
	virtual int registerCubeShape(float halfExtentsX, float halfExtentsY, float halfExtentsZ, int textureIndex = -1, float textureScaling = 1) = 0;
	virtual int registerGraphicsUnitSphereShape(EnumSphereLevelOfDetail lod, int textureId = -1) = 0;

	virtual void registerGrid(int xres, int yres, float color0[4], float color1[4]) = 0;

	void defaultMouseButtonCallback(int button, int state, float x, float y)
	{
		if (button == 0)
			m_leftMouseButton = (state == 1);
		if (button == 1)
			m_middleMouseButton = (state == 1);

		if (button == 2)
			m_rightMouseButton = (state == 1);

		m_mouseXpos = x;
		m_mouseYpos = y;
		m_mouseInitialized = true;
	}
	void defaultMouseMoveCallback(float x, float y)
	{
		if (m_window && m_renderer)
		{
			CommonCameraInterface* camera = m_renderer->getActiveCamera();

			bool isAltPressed = m_window->isModifierKeyPressed(B3G_ALT);
			bool isControlPressed = m_window->isModifierKeyPressed(B3G_CONTROL);

			if (isAltPressed || isControlPressed)
			{
				float xDelta = x - m_mouseXpos;
				float yDelta = y - m_mouseYpos;
				float cameraDistance = camera->getCameraDistance();
				float pitch = camera->getCameraPitch();
				float yaw = camera->getCameraYaw();

				float targPos[3];
				float camPos[3];

				camera->getCameraTargetPosition(targPos);
				camera->getCameraPosition(camPos);

				b3Vector3 cameraPosition = b3MakeVector3(b3Scalar(camPos[0]),
					b3Scalar(camPos[1]),
					b3Scalar(camPos[2]));

				b3Vector3 cameraTargetPosition = b3MakeVector3(b3Scalar(targPos[0]),
					b3Scalar(targPos[1]),
					b3Scalar(targPos[2]));
				b3Vector3 cameraUp = b3MakeVector3(0, 0, 0);
				cameraUp[camera->getCameraUpAxis()] = 1.f;

				if (m_leftMouseButton)
				{
					//			if (b3Fabs(xDelta)>b3Fabs(yDelta))
					//			{
					pitch -= yDelta * m_mouseMoveMultiplier;
					//			} else
					//			{
					yaw -= xDelta * m_mouseMoveMultiplier;
					//			}
				}

				if (m_middleMouseButton)
				{
					cameraTargetPosition += cameraUp * yDelta *m_mouseMoveMultiplier* 0.01;

					b3Vector3 fwd = cameraTargetPosition - cameraPosition;
					b3Vector3 side = cameraUp.cross(fwd);
					side.normalize();
					cameraTargetPosition += side * xDelta *m_mouseMoveMultiplier* 0.01;
				}
				if (m_rightMouseButton)
				{
					cameraDistance -= xDelta * m_mouseMoveMultiplier*0.01f;
					cameraDistance -= yDelta * m_mouseMoveMultiplier*0.01f;
					if (cameraDistance < 1)
						cameraDistance = 1;
					if (cameraDistance > 1000)
						cameraDistance = 1000;
				}
				camera->setCameraDistance(cameraDistance);
				camera->setCameraPitch(pitch);
				camera->setCameraYaw(yaw);
				camera->setCameraTargetPosition(cameraTargetPosition[0], cameraTargetPosition[1], cameraTargetPosition[2]);
			}

		}  //m_window &&  m_renderer

		m_mouseXpos = x;
		m_mouseYpos = y;
		m_mouseInitialized = true;
	}
	//	void defaultKeyboardCallback(int key, int state)
	//	{
	//	}
	void defaultWheelCallback(float deltax, float deltay)
	{
		if (m_renderer)
		{
			b3Vector3 cameraTargetPosition, cameraPosition, cameraUp = b3MakeVector3(0, 0, 0);
			cameraUp[getUpAxis()] = 1;
			CommonCameraInterface* camera = m_renderer->getActiveCamera();

			camera->getCameraPosition(cameraPosition);
			camera->getCameraTargetPosition(cameraTargetPosition);

			if (!m_leftMouseButton)
			{
				float cameraDistance = camera->getCameraDistance();
				if (deltay < 0 || cameraDistance > 1)
				{
					cameraDistance -= deltay*m_wheelMultiplier;
					if (cameraDistance < 1)
						cameraDistance = 1;
					camera->setCameraDistance(cameraDistance);
				}
				else
				{
					b3Vector3 fwd = cameraTargetPosition - cameraPosition;
					fwd.normalize();
					cameraTargetPosition += fwd * deltay * m_wheelMultiplier;  //todo: expose it in the GUI?
				}
			}
			else
			{
				if (b3Fabs(deltax) > b3Fabs(deltay))
				{
					b3Vector3 fwd = cameraTargetPosition - cameraPosition;
					b3Vector3 side = cameraUp.cross(fwd);
					side.normalize();
					cameraTargetPosition += side * deltax * m_wheelMultiplier;
				}
				else
				{
					cameraTargetPosition -= cameraUp * deltay * m_wheelMultiplier;
				}
			}

			camera->setCameraTargetPosition(cameraTargetPosition[0], cameraTargetPosition[1], cameraTargetPosition[2]);
		}
	}
};

#endif  //COMMON_GRAPHICS_APP_H
